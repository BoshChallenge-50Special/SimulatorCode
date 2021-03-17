#!/usr/bin/env python

from threading import Thread

import numpy as np
import cv2
import os
from matplotlib import pyplot as plt
import math
from math import sqrt
from os import listdir
import argparse
import time
import json
# local modules
from classification_sk import training, getLabel, load_model
import sys
sys.settrace
import traceback
sys.path.insert(0, './src/startup_package/src/')
from bfmclib.gps_s import Gps
from bfmclib.bno055_s import BNO055
from bfmclib.camera_s import CameraHandler
from bfmclib.controller_p import Controller
from bfmclib.trafficlight_s import TLColor, TLLabel, TrafficLight
import rospy
from std_msgs.msg import String
from SimulatorCode.templates import Producer
from SimulatorCode.LineTracking.image_processing import ImageProcessing

#import sift

IS_TEST_ENVIRONMENT = False

import random
#Parameter
SIZE = 32
CLASS_NUMBER = 11


SIGNS = ["ERROR",           #0
        "STOP",             #1
        "PARKING",          #2
        "PRIORITY",         #3
        "CROSSWALK SIGN",   #4
        "HIGHWAY ENTRANCE", #5
        "ONE WAY",          #6
        "HIGHWAY EXIT",     #7
        "ROUNDABOUT",       #8
        "NO-ENTRY",         #9
        "OTHER"]            #10

# Clean all previous file
def clean_images():
	file_list = os.listdir('/home/ebllaei/traffic/shape/Traffic-Sign-Detection/')
	for file_name in file_list:
		if '.png' in file_name:
			os.remove(file_name)


class SignDetector(Producer):

    def __init__(self, get_image_function, model_classification, verbose=False):
        super(SignDetector, self).__init__("SignDetector")
        '''
        :)
        '''
        self.set_publisher("Sign")
        self.model_classification = model_classification
        self.verbose=verbose

        self.winSize = (50, 50)
        self.blockSize = (10, 10)
        self.blockStride = (5, 5)
        self.cellSize = (5, 5)
        self.nbins = 9
        self.derivAperture = 1
        self.winSigma = -1.
        self.histogramNormType = 0
        self.L2HysThreshold = 0.2
        self.gammaCorrection = 1
        self.nlevels = 64
        self.signedGradients = True
        self.current_sign_type = 0
        # This is used.
        self.hog = cv2.HOGDescriptor(self.winSize, self.blockSize, self.blockStride, self.cellSize, self.nbins,
                                    self.derivAperture, self.winSigma, self.histogramNormType,
                                self.L2HysThreshold, self.gammaCorrection, self.nlevels, self.signedGradients)

        self.params = cv2.SimpleBlobDetector_Params()
        self.params.minDistBetweenBlobs = 40
        self.params.minThreshold = 253
        self.params.maxThreshold = 255
        self.params.thresholdStep = 1

        self.params.filterByArea = True
        self.params.minArea = 400
        self.params.maxArea = 3500

        self.params.filterByCircularity = True
        self.params.filterByConvexity = False
        self.params.filterByColor = False
        self.params.filterByInertia = False
        # As is this.
        self.detector = cv2.SimpleBlobDetector_create(self.params)

        if(get_image_function != None):
            self.get_image_function = get_image_function
        else:
            self.get_image_function_test_load()
            self.get_image_function = self.get_image_function_test

    #====================================================================
    def withinBoundsX(self, coord, img):
        '''
        #Checking to see if the X coordinate is not out of image's bounds.
        '''
        if(coord < 0 or coord > img.shape[1]):
            return 0
        return 1

    def withinBoundsY(self, coord, img):
        '''
        #Checking to see if the X coordinate is not out of image's bounds.
        '''
        if(coord < 0 or coord > img.shape[0]):
            return 0
        return 1
    #=====================================================================

    def detectColorAndCenters(self, img):
        '''
        It detects the color, then uses a simple blob goodness (basically countour finding, but somewhat automated)
        to find regions of interest based on color. There are 3 masks, 1 red, 1 yellow and 1 blue.
        On each mask I find blobs of interest then I store the centers of all of those in an array, if you want to know
        specifics, ask me since I can't write that much here. xoxoxo
        :param img: the fucking image
        :return: centers of all blobs of interest
        '''
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        imgHSV = cv2.GaussianBlur(imgHSV, (5, 5), 0)
        # Range for lower RED
        mask1 = cv2.inRange(imgHSV, (0,50, 50), (10,255,255))
        # Range for upper range
        mask2 = cv2.inRange(imgHSV, (160,60,50), (180,255,255))
        red_det = mask1 + mask2
        # Yellow mask
        yellow_det = cv2.inRange(imgHSV, (15, 90, 20), (35, 255, 255))
        # Blue mask
        blue_det = cv2.inRange(imgHSV, (95, 90, 20), (140, 250, 250))
        # green mask
        green_det = cv2.inRange( imgHSV, (70,50,50), (85,255,255))
        #finalMask = red_det + yellow_det + blue_det

        # Bitwise-AND mask and original image
        ##### ATTENZIONE----> res non sembra mai utilizzato#################################################
        #res = cv2.bitwise_and(imgHSV,imgHSV, mask= (red_det))
        #cv2.imshow('res',res)
        #==============================================================================
        #==============================================================================

        #other_keypoints = cv2.bitwise_and(imgHSV,imgHSV, mask= (red_det))

        keypoints_red = self.detector.detect(red_det)
        keypoints_yellow = self.detector.detect(yellow_det)
        keypoints_blue = self.detector.detect(blue_det)
        keypoints_green = self.detector.detect(green_det)

        red_image = cv2.drawKeypoints(red_det, keypoints_red, np.array([]), (0, 0, 255),
                                     cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        yellow_image = cv2.drawKeypoints(yellow_det, keypoints_yellow, np.array([]), (0, 255, 0),
                                     cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        blue_image = cv2.drawKeypoints(blue_det, keypoints_blue, np.array([]), (255, 0, 0),
                                     cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        green_image = cv2.drawKeypoints(green_det, keypoints_green, np.array([]), (0, 128, 0),
                                     cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        #print("***********************************")
        #print(" X : ", keypoints_red[0].pt[0], " Y : ", keypoints_red[0].pt[1], " SIZE : ", keypoints_red[0].size)
        #print(red_det)
        finalImage = red_image + yellow_image + blue_image + green_image
        if(self.verbose):
            cv2.imshow("Image after applied the masks", finalImage)
        for point in keypoints_yellow:
            point.size = point.size + 30  #TO find the precedence

        keypoints_all = keypoints_red + keypoints_yellow + keypoints_blue + keypoints_green

        new_keypoints=[]
        for p in range(0,len(keypoints_all)):
            if keypoints_all[p].size > 30:
                new_keypoints.append(keypoints_all[p])
            #point.size = point.size + 30  #TO find the precedence

        cv2.drawKeypoints( finalImage, new_keypoints, finalImage, (0, 0, 255))
        if(self.verbose):
            cv2.imshow("finalImage + IMAGE plus keypoints", finalImage)

        return (new_keypoints)
    # ===================================================================================


    def detectSign(self, img, watch, keypoints, y_offset=0):
        # img used for detection
        # watch used for showing the results
        coordinates = []
        termination = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)
        roiBox = None
        roiHist = None
        position = []
        coordinate=None
        signsFound=[]
        for c, point in enumerate(keypoints):

            radiusOfsign = int(point.size/2) + 4

            x1 = int(point.pt[0]) - radiusOfsign #- 35
            y1 = int(point.pt[1]) - radiusOfsign #- 35
            x2 = int(point.pt[0]) + radiusOfsign #+ 35
            y2 = int(point.pt[1]) + radiusOfsign #+ 35
            for disp in (0,):#, 4):
                if (not self.withinBoundsX(x1, img) or  not self.withinBoundsY(y1+disp, img) or not self.withinBoundsX(x2, img) or not self.withinBoundsY(y2+disp, img)):

                    break

                #The window to check using the HOG.
                roi = img[y1+disp:y2+disp, x1:x2]
                sign = roi#cv2.resize(roi, (40, 40), interpolation=cv2.INTER_AREA)
                if(self.verbose):
                    cv2.imshow("roi "+str(c), sign)

                #If the SVM gives a positive response (i.e. it is a sign) we show the image and draw a rectangle around the sign
                #we further process it with the classifier.
                sign_type = -1
                i = 0
                #continue

                if sign is not None:
                    #sign_type =sift.get_label_sift(sign)
                    sign_type = getLabel(self.model_classification, sign)
                    sign_type = sign_type if sign_type <= 11 else 11
                    text = SIGNS[sign_type]
                    cv2.rectangle(watch, (x1, y1 + disp+y_offset), (x2, y2 + disp+y_offset), (255, 255, 255), 2)
                    coordinate=None
                    coordinate = (x1, y1+disp),(x2, y2+disp)


                    # Uncomment to save images for dataset
                    #cv2.imwrite(str(self.count)+'_'+str(random.randint(0,20))+'.png', sign)
                    #if sign_type != 0:
                    #   print(self.count, text)

                if sign_type > 0 and sign_type != self.current_sign_type:
                    font = cv2.FONT_HERSHEY_PLAIN
                    cv2.putText(watch, text,(x1, y1+y_offset), font, 1,(0,0,255),2, cv2.LINE_4)
                    cv2.rectangle(watch, (x1, y1 + disp+y_offset), (x2, y2 + disp+y_offset), (0, 0, 255), 2)
                    signsFound.append(SIGNS[sign_type])

                if False and sign_type > 0 and (not self.current_sign or sign_type != self.current_sign):
                    self.current_sign = sign_type
                    self.current_text = text

                    top = int(coordinate[0][1]*1.05)
                    left = int(coordinate[0][0]*1.05)
                    bottom = int(coordinate[1][1]*0.95)
                    right = int(coordinate[1][0]*0.95)

                    position = [self.count, sign_type if sign_type <= 8 else 8, coordinate[0][0], coordinate[0][1], coordinate[1][0], coordinate[1][1]]
                    cv2.rectangle(img, coordinate[0],coordinate[1], (0, 255, 0), 1)
                    font = cv2.FONT_HERSHEY_PLAIN
                    cv2.putText(img,text,(coordinate[0][0], coordinate[0][1] -15), font, 1,(0,0,255),2,cv2.LINE_4)

                    tl = [left, top]
                    br = [right,bottom]
                    self.current_size = math.sqrt(math.pow((tl[0]-br[0]),2) + math.pow((tl[1]-br[1]),2))
                    # grab the ROI for the bounding box and convert it
                    # to the HSV color space
                    roi = watch[tl[1]:br[1], tl[0]:br[0]]
                    roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                    #roi = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
                    # compute a HSV histogram for the ROI and store the
                    # bounding box
                    roiHist = cv2.calcHist([roi], [0], None, [16], [0, 180])
                    roiHist = cv2.normalize(roiHist, roiHist, 0, 255, cv2.NORM_MINMAX)
                    roiBox = (tl[0], tl[1], br[0], br[1])
                elif False and((self.current_sign != None) and (roiHist is not None)):
                    hsv = cv2.cvtColor(watch, cv2.COLOR_BGR2HSV)
                    backProj = cv2.calcBackProject([hsv], [0], roiHist, [0, 180], 1)

                    # apply cam shift to the back projection, convert the
                    # points to a bounding box, and then draw them
                    (r, roiBox) = cv2.CamShift(backProj, roiBox, termination)
                    pts = np.int0(cv2.boxPoints(r))
                    s = pts.sum(axis = 1)
                    tl = pts[np.argmin(s)]
                    br = pts[np.argmax(s)]
                    size = math.sqrt(pow((tl[0]-br[0]),2) +pow((tl[1]-br[1]),2))

                    if  self.current_size < 1 or size < 1 or size / self.current_size > 30 or math.fabs((tl[0]-br[0])/(tl[1]-br[1])) > 2 or math.fabs((tl[0]-br[0])/(tl[1]-br[1])) < 0.5:
                        self.current_sign = None
                        #print("Stop tracking")
                    else:
                        self.current_size = size

                    if sign_type > 0:
                        top = int(coordinate[0][1])
                        left = int(coordinate[0][0])
                        bottom = int(coordinate[1][1])
                        right = int(coordinate[1][0])

                        position = [self.count, sign_type if sign_type <= 8 else 8, left, top, right, bottom]
                        cv2.rectangle(watch, coordinate[0],coordinate[1], (0, 255, 0), 1)
                        font = cv2.FONT_HERSHEY_PLAIN
                        cv2.putText(watch,text,(coordinate[0][0], coordinate[0][1] -15), font, 1,(0,0,255),2,cv2.LINE_4)
                    elif self.current_sign:
                        position = [self.count, sign_type if sign_type <= 8 else 8, tl[0], tl[1], br[0], br[1]]
                        print("rectangle")
                        cv2.rectangle(watch, (tl[0], tl[1]),(br[0], br[1]), (0, 255, 0), 1)
                        font = cv2.FONT_HERSHEY_PLAIN
                        cv2.putText(watch,self.current_text,(tl[0], tl[1] -15), font, 1,(0,0,255),2,cv2.LINE_4)

                if self.current_sign:
                    self.sign_count += 1
                    coordinates.append(position)

        cv2.imshow('Result detectSign', watch)
        return signsFound
        #Write to video
        #out.write(img)
        #if cv2.waitKey(1) & 0xFF == ord('q'):
        #    break

        #if self.clf02.predict(pca_values) == 1:
        '''
        Succesive detections imeplemented on the receiving end, maybe?
        '''


    #====================================================================================

    def run(self):
        self.current_sign = None
        self.current_text = ""
        self.current_size = 0
        self.sign_count = 0
        self.count = 0
        imageUtility = ImageProcessing()
        #Clean previous image
        #clean_images()
        #Training phase

        #while success:
        # Uncomment elisa

        # uncomment simulator:
        while not rospy.is_shutdown():
            #print("TRAFFIC")
            '''
            victim - the image I actually do the processing on
            watch - this is where I draw the rectangles and whatnot so it can be tested in practice
            keypoints - the keypoints of the regions of interest
            '''
            # Uncomment one when Eisa:
            #watch = cv2.imread("/home/ebllaei/Downloads/dataset/0973.png") #0816 1723
            #watch = cv2.imread("/home/morettini18/Downloads/Image_2.png")
            #watch = cv2.imread("/home/ebllaei/dataset_bosch/4/img_159.png")
            # Uncomment when simulator:
            watch = self.get_image_function()
            cut_top=80
            watch = np.copy(watch)
            watch_copy = np.copy(watch)
            victim =  imageUtility.crop_image(watch_copy, 0, cut_top, 640, 240)  # x_top_left, y_top_left, x_bottom_right, y_bottom_right
            #victim = watch[0:(int)(watch.shape[0]/2), (int)(watch.shape[1]/2):watch.shape[1]]
            #victim = cv2.copyMakeBorder(watch, 0, 0, 0, 32, cv2.BORDER_REPLICATE)
            #victim = cv2.copyMakeBorder(watch, 0, 0, 0, 0, cv2.BORDER_REPLICATE)

            img = np.int16(victim)
            contrast = 10
            brightness = 50
            img = img * (contrast/127+1) - contrast + brightness
            img = np.clip(img, 0, 255)
            victim = np.uint8(img)

            #victim = np.copy(watch)
            # Get the keypoints.
            keypoints = self.detectColorAndCenters(victim)
            # Detect and classify the signs.
            signs = self.detectSign(victim, watch, keypoints, y_offset=cut_top)
            #self.detectSign(watch, watch, keypoints)

            self.count = self.count + 1
            #cv2.imshow("Input image to detection sign", watch)
            #self.outP.send(0)
            #print(0)
            signs_json = json.dumps(signs)
            self.pub.publish(signs_json)
            time.sleep(0.1)
            #success,watch = vidcap.read()
            if cv2.waitKey(1) == 27:
                break

        cv2.destroyAllWindows()

    def get_image_function_test_load(self):
        folder = '/home/ebllaei/Downloads/dataset'
        self.image_files = os.listdir(folder)
        self.index_file = -1

    def get_image_function_test(self):
        self.index_file = self.index_file+1
        return cv2.imread(os.path.join(folder,filename))

if __name__ == '__main__':
    try:
        model = None
        #model = training()
        #model.save('./src/startup_package/src/SimulatorCode/TrafficSignDetection/model_svm.plk')
        sg = None
        if(IS_TEST_ENVIRONMENT):
            model = load_model('./src/startup_package/src/SimulatorCode/TrafficSignDetection/model_svm.plk')
            sg = SignDetector(None, model, verbose=False)
        else:
            cam = CameraHandler()
            model=load_model('./src/startup_package/src/SimulatorCode/TrafficSignDetection/model_svm.plk')
            sg = SignDetector(cam.getImage, model, verbose=False)
        #print(cv2.__version__)
        sg.run()
    except Exception as e:
        print("Error in TrafficSignDetection")
        print(e)
        print(traceback.print_exc())
