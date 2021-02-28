from threading import Thread

import cv2
import joblib
import numpy as np
import sklearn
import os
from matplotlib import pyplot as plt
from math import sqrt
from os import listdir
from skimage.feature import blob_dog, blob_log, blob_doh
import imutils
import argparse
import time
# local modules
from common import mosaic


#Parameter
SIZE = 32
CLASS_NUMBER = 10


SIGNS = ["ERROR",
        "STOP",
        "PARKING",
        "PRIORITY",
        "CROSSWALK SIGN",
        "HIGHWAY ENTRANCE",
        "ONE WAY",
        "HIGHWAY EXIT",
        "ROUNDABOUT",
        "NO-ENTRY"]


# Clean all previous file
def clean_images():
	file_list = os.listdir('/home/ebllaei/traffic/shape/Traffic-Sign-Detection/')
	for file_name in file_list:
		if '.png' in file_name:
			os.remove(file_name)

def load_traffic_dataset():
    dataset = []
    labels = []
    for sign_type in range(CLASS_NUMBER):
        sign_list = listdir("/home/ebllaei/traffic/shape/Traffic-Sign-Detection/dataset/{}".format(sign_type))
        for sign_file in sign_list:
            if '.png' in sign_file:
                path = "/home/ebllaei/traffic/shape/Traffic-Sign-Detection/dataset/{}/{}".format(sign_type,sign_file)
                print(path)
                img = cv2.imread(path,0)
                img = cv2.resize(img, (SIZE, SIZE))
                img = np.reshape(img, [SIZE, SIZE])
                dataset.append(img)
                labels.append(sign_type)
    return np.array(dataset), np.array(labels)


def deskew(img):
    m = cv2.moments(img)
    if abs(m['mu02']) < 1e-2:
        return img.copy()
    skew = m['mu11']/m['mu02']
    M = np.float32([[1, skew, -0.5*SIZE*skew], [0, 1, 0]])
    img = cv2.warpAffine(img, M, (SIZE, SIZE), flags=cv2.WARP_INVERSE_MAP | cv2.INTER_LINEAR)
    return img

class StatModel(object):
    def load(self, fn):
        self.model.load(fn)  # Known bug: https://github.com/opencv/opencv/issues/4969
    def save(self, fn):
        self.model.save(fn)

class SVM(StatModel):
    def __init__(self, C = 12.5, gamma = 0.50625):
        self.model = cv2.ml.SVM_create()
        self.model.setGamma(gamma)
        self.model.setC(C)
        self.model.setKernel(cv2.ml.SVM_RBF)
        self.model.setType(cv2.ml.SVM_C_SVC)

    def train(self, samples, responses):
        self.model.train(samples, cv2.ml.ROW_SAMPLE, responses)

    def predict(self, samples):

        return self.model.predict(samples)[1].ravel()


def evaluate_model(model, data, samples, labels):
    resp = model.predict(samples)
    print(resp)
    err = (labels != resp).mean()
    print('Accuracy: %.2f %%' % ((1 - err)*100))

    confusion = np.zeros((10, 10), np.int32)
    for i, j in zip(labels, resp):
        confusion[int(i), int(j)] += 1
    print('confusion matrix:')
    print(confusion)

    vis = []
    for img, flag in zip(data, resp == labels):
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        if not flag:
            img[...,:2] = 0
        
        vis.append(img)
    return mosaic(16, vis)

def preprocess_simple(data):
    return np.float32(data).reshape(-1, SIZE*SIZE) / 255.0


def get_hog() : 
    winSize = (20,20)
    blockSize = (10,10)
    blockStride = (5,5)
    cellSize = (10,10)
    nbins = 9
    derivAperture = 1
    winSigma = -1.
    histogramNormType = 0
    L2HysThreshold = 0.2
    gammaCorrection = 1
    nlevels = 64
    signedGradient = True

    hog = cv2.HOGDescriptor(winSize,blockSize,blockStride,cellSize,nbins,derivAperture,winSigma,histogramNormType,L2HysThreshold,gammaCorrection,nlevels, signedGradient)

    return hog
    affine_flags = cv2.WARP_INVERSE_MAP|cv2.INTER_LINEAR


def training():
    print('Loading data from data.png ... ')
    # Load data.
    #data, labels = load_data('data.png')
    data, labels = load_traffic_dataset()
    print(data.shape)
    print('Shuffle data ... ')
    # Shuffle data
    rand = np.random.RandomState(10)
    shuffle = rand.permutation(len(data))
    data, labels = data[shuffle], labels[shuffle]
    
    print('Deskew images ... ')
    data_deskewed = list(map(deskew, data))
    
    print('Defining HoG parameters ...')
    # HoG feature descriptor
    hog = get_hog()

    print('Calculating HoG descriptor for every image ... ')
    hog_descriptors = []
    for img in data_deskewed:
        hog_descriptors.append(hog.compute(img))
    hog_descriptors = np.squeeze(hog_descriptors)

    print('Spliting data into training (90%) and test set (10%)... ')
    train_n=int(0.9*len(hog_descriptors))
    data_train, data_test = np.split(data_deskewed, [train_n])
    hog_descriptors_train, hog_descriptors_test = np.split(hog_descriptors, [train_n])
    labels_train, labels_test = np.split(labels, [train_n])
    
    
    print('Training SVM model ...')
    model = SVM()
    model.train(hog_descriptors_train, labels_train)

    print('Saving SVM model ...')
    model.save('data_svm.dat')
    return model

def getLabel(model, data):
    gray = cv2.cvtColor(data, cv2.COLOR_BGR2GRAY)
    img = [cv2.resize(gray,(SIZE,SIZE))]
    #print(np.array(img).shape)
    img_deskewed = list(map(deskew, img))
    hog = get_hog()
    hog_descriptors = np.array([hog.compute(img_deskewed[0])])
    hog_descriptors = np.reshape(hog_descriptors, [-1, hog_descriptors.shape[1]])
    return int(model.predict(hog_descriptors)[0])




class SignDetector(Thread):

    def __init__(self, inP, outP):
        '''
        :)
        '''
        super(SignDetector, self).__init__()
        self.inP = inP
        self.outP = outP

        self.winSize = (40, 40)
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
        self.params.minArea = 300
        self.params.maxArea = 3500

        self.params.filterByCircularity = False
        self.params.filterByConvexity = False
        self.params.filterByColor = False
        self.params.filterByInertia = False
        # As is this.
        self.detector = cv2.SimpleBlobDetector_create(self.params)
        # And these three.
        #self.clf02 = joblib.load("classifierSVM.joblib")
        #self.clf = joblib.load("LDA.joblib")
        #self.pca = joblib.load("pca.joblib")    

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
        res = cv2.bitwise_and(imgHSV,imgHSV, mask= (red_det))
        cv2.imshow('res',res)
        #==============================================================================
        #==============================================================================

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

        finalImage = red_image + yellow_image + blue_image + green_image
        cv2.imshow("watch1", finalImage)
        keypoints_all = keypoints_red + keypoints_yellow + keypoints_blue + keypoints_green



        cv2.imshow("finalImage", finalImage)

        return (keypoints_all)
    # ===================================================================================


    def detectSign(self, img, watch, centers,model):
        for points in centers:
            x1 = int(points.pt[0]) - 35
            y1 = int(points.pt[1]) - 35
            x2 = int(points.pt[0]) + 35
            y2 = int(points.pt[1]) + 35

            for disp in (0, 4):
                if (not self.withinBoundsX(x1, img) or  not self.withinBoundsY(y1+disp, img) or not self.withinBoundsX(x2, img) or not self.withinBoundsY(y2+disp, img)):
                    break
                #The window to check using the HOG.
                roi = img[y1+disp:y2+disp, x1:x2]
                sign = cv2.resize(roi, (40, 40), interpolation=cv2.INTER_AREA)
                #Computing the HOG and making it so we can give it to the SVM, with PCA.
                #descriptor = self.hog.compute(roi)
                #descriptor = np.array(descriptor)
                #descriptor = descriptor.transpose()


                #pca_values = self.pca.transform(descriptor)
                #If the SVM gives a positive response (i.e. it is a sign) we show the image and draw a rectangle around the sign
                #we further process it with the classifier.
                sign_type = -1
                i = 0

                if sign is not None:
                    sign_type = getLabel(model, sign)
                    sign_type = sign_type if sign_type <= 8 else 8
                    text = SIGNS[sign_type]
                    cv2.rectangle(watch, (x1, y1 + disp), (x2, y2 + disp), (255, 255, 255), 2)
                    #cv2.imwrite(str(count)+'_'+text+'.png', sign)
                    print(text)

                if sign_type > 0 and sign_type != self.current_sign_type:        
                    cv2.rectangle(watch, (x1, y1 + disp), (x2, y2 + disp), (0, 0, 255), 2) 
                    cv2.putText(original_image,text,(x1, x2 -15), font, 1,(0,0,255),2,cv2.LINE_4)
                                       
                if sign_type > 0 and (not self.current_sign or sign_type != current_sign):
                    self.current_sign = sign_type
                    self.current_text = text
                    top = int(coordinate[0][1]*1.05)
                    left = int(coordinate[0][0]*1.05)
                    bottom = int(coordinate[1][1]*0.95)
                    right = int(coordinate[1][0]*0.95)

                    position = [count, sign_type if sign_type <= 8 else 8, coordinate[0][0], coordinate[0][1], coordinate[1][0], coordinate[1][1]]
                    cv2.rectangle(img, coordinate[0],coordinate[1], (0, 255, 0), 1)
                    font = cv2.FONT_HERSHEY_PLAIN
                    cv2.putText(img,text,(coordinate[0][0], coordinate[0][1] -15), font, 1,(0,0,255),2,cv2.LINE_4)

                    tl = [left, top]
                    br = [right,bottom]
                    print(tl, br)
                    self.current_size = math.sqrt(math.pow((tl[0]-br[0]),2) + math.pow((tl[1]-br[1]),2))
                    # grab the ROI for the bounding box and convert it
                    # to the HSV color space
                    roi = frame[tl[1]:br[1], tl[0]:br[0]]
                    roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                    #roi = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)

                    # compute a HSV histogram for the ROI and store the
                    # bounding box
                    roiHist = cv2.calcHist([roi], [0], None, [16], [0, 180])
                    roiHist = cv2.normalize(roiHist, roiHist, 0, 255, cv2.NORM_MINMAX)
                    roiBox = (tl[0], tl[1], br[0], br[1])

                elif self.current_sign:
                    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                    backProj = cv2.calcBackProject([hsv], [0], roiHist, [0, 180], 1)

                    # apply cam shift to the back projection, convert the
                    # points to a bounding box, and then draw them
                    (r, roiBox) = cv2.CamShift(backProj, roiBox, termination)
                    pts = np.int0(cv2.boxPoints(r))
                    s = pts.sum(axis = 1)
                    tl = pts[np.argmin(s)]
                    br = pts[np.argmax(s)]
                    size = math.sqrt(pow((tl[0]-br[0]),2) +pow((tl[1]-br[1]),2))
                    print(size)

                    if  self.current_size < 1 or size < 1 or size / current_size > 30 or math.fabs((tl[0]-br[0])/(tl[1]-br[1])) > 2 or math.fabs((tl[0]-br[0])/(tl[1]-br[1])) < 0.5:
                        self.current_sign = None
                        print("Stop tracking")
                    else:
                        self.current_size = size

                    if sign_type > 0:
                        top = int(coordinate[0][1])
                        left = int(coordinate[0][0])
                        bottom = int(coordinate[1][1])
                        right = int(coordinate[1][0])

                        position = [count, sign_type if sign_type <= 8 else 8, left, top, right, bottom]
                        cv2.rectangle(img, coordinate[0],coordinate[1], (0, 255, 0), 1)
                        font = cv2.FONT_HERSHEY_PLAIN
                        cv2.putText(img,text,(coordinate[0][0], coordinate[0][1] -15), font, 1,(0,0,255),2,cv2.LINE_4)
                    elif self.current_sign:
                        position = [count, sign_type if sign_type <= 8 else 8, tl[0], tl[1], br[0], br[1]]
                        cv2.rectangle(img, (tl[0], tl[1]),(br[0], br[1]), (0, 255, 0), 1)
                        font = cv2.FONT_HERSHEY_PLAIN
                        cv2.putText(img,self.current_text,(tl[0], tl[1] -15), font, 1,(0,0,255),2,cv2.LINE_4)

                if self.current_sign:
                    self.sign_count += 1
                    coordinates.append(position)

                cv2.imshow('Result', img)
                self.count = self.count + 1
                #Write to video
                #out.write(img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                #if self.clf02.predict(pca_values) == 1:
                    '''
                    Succesive detections imeplemented on the receiving end, maybe?
                    '''
                 #   if(self.clf.predict(descriptor) == 1):
                  #      self.outP.send(1)
                   #     cv2.rectangle(watch, (x1, y1 + disp), (x2, y2 + disp), (255, 0, 0), 2)

                    #elif (self.clf.predict(descriptor) == 2):
                    #    self.outP.send(2)
                    #    cv2.rectangle(watch, (x1, y1 + disp), (x2, y2 + disp), (255, 255, 255), 2)

#                    elif (self.clf.predict(descriptor) == 3):
 #                       self.outP.send(3)
  #                      cv2.rectangle(watch, (x1, y1 + disp), (x2, y2 + disp), (0, 255, 0), 2)
#
 #                   elif (self.clf.predict(descriptor) == 4):
  #                      self.outP.send(4)
   #                     cv2.rectangle(watch, (x1, y1 + disp), (x2, y2 + disp), (0, 0, 255), 2)
    #                print("O intrat")
     #              
      #          else:
       #            # self.outP.send(0)
        #           print(0)

    #====================================================================================

    def run(self):
        self.current_sign = None
        self.current_text = ""
        self.current_size = 0
        self.sign_count = 0
        self.count = 0
        	#Clean previous image    
        #clean_images()
        #Training phase
        model = training()

        #vidcap = cv2.VideoCapture('/home/ebllaei/Downloads/video.mp4')
        #vidcap = cv2.VideoCapture('/home/ebllaei/traffic_signs.mp4')
        #vidcap = cv2.VideoCapture('/home/ebllaei/traffic/shape/Traffic-Sign-Detection/MVI_1049.avi')
        #success,watch = vidcap.read()
        
        #while success:


        folder = '/home/ebllaei/Downloads/dataset'

        for filename in sorted(os.listdir(folder)):
            '''
            victim - the image I actually do the processing on
            watch - this is where I draw the rectangles and whatnot so it can be tested in practice
            centers - the centers of the regions of interest
            '''
            print(filename)
            # A dirty drick, unsure if still necessary, but I will leave it here.
            watch = cv2.imread(os.path.join(folder,filename))
            #watch = cv2.imread('/home/ebllaei/Downloads/dataset/img_426.png')
            victim = watch[0:(int)(watch.shape[0]/2), (int)(watch.shape[1]/2):watch.shape[1]]
            victim = cv2.copyMakeBorder(victim, 0, 0, 0, 32, cv2.BORDER_REPLICATE)
            
            img = np.int16(watch)
            contrast = 10
            brightness = 50
            img = img * (contrast/127+1) - contrast + brightness
            img = np.clip(img, 0, 255)
            watch = np.uint8(img)
            
            # Get the centers.
            centers = self.detectColorAndCenters(watch)
            # Detect and classify the signs.
            self.detectSign(victim, watch, centers,model)
            cv2.imshow("watch", watch)
            #self.outP.send(0)
            print(0)
            time.sleep(0.1)
            #success,watch = vidcap.read()
            if cv2.waitKey(1) == 27:
                break
            
        cv2.destroyAllWindows()

sg = SignDetector(None,None)
sg.run()



