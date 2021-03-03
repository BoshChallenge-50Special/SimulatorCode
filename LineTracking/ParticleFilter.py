# -*- coding: utf-8 -*-
import cv2 as cv
import numpy as np
from random import seed
from random import randint
import random
from copy import deepcopy
import math
import os, os.path
import sys
import time
import timeit
import csv

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

'''
Image progessing library import:
- Read from camera
- BW mask
- Blurry
- Crop Image

Particle Object
- Create many particles (Lines)
- Each line is associated with a weight

Utils:
- draw_particles
- order points          | --> for IPM transformation
- four points transform |
'''

from image_processing import ImageProcessing
from particle import Particle
from utils import Utils

class ParticleFilter(object):

    def __init__(self, N, order, N_points, Interpolation_points, h_image, w_image, type_approximation=None, threshold_reset=None, parameters={}):

        self.particles      = []
        self.cumweights     = []
        self.N              = N
        self.h_image        = h_image
        self.w_image        = w_image

        self.failing_count  = 0
        self.approximation_to_show  = True
        self.steps_good_approximation  = 0

        self.order          = order
        self.N_points       = N_points

        self.Interpolation_points = Interpolation_points

        self.approximation  = Particle(self.N_points, self.Interpolation_points, self.order )

        if(type_approximation==None):
            self.type_approximation="max"
        else:
            self.type_approximation=type_approximation

        if(threshold_reset==None):
            self.threshold_reset=5
        else:
            self.threshold_reset=threshold_reset

        return

    def initialization(self, verbose=False):
        # Distribute hundreds of lines over the image
        # Larger the image is covered and more are the chance of find
        # the true line
        h = self.h_image
        w = self.w_image
        self.particles=[]

        # mean and standard deviation for spline generation
        mu, sigma  = w/2, w/3

        for _ in range(0, self.N):

            p = Particle(self.N_points, self.Interpolation_points, self.order)

            # Points distribution, prom the bottom to the top, all the picture is covered
            # Alto mu alto perchè gira, con il giro della ruota possiamo aumentare il mu quando gira
            for j in range(self.N_points):

                random_numbers = np.random.normal(mu, sigma, 1).astype(int)
                p.points[j] = [abs(random_numbers[0]), int(j * h/(self.N_points - 1)) ]

            p.w = 1/self.N
            self.particles.append(p)

        self.steps_good_approximation = 0
        return self.particles

    def sampling(self, verbose=False):
        # It is the Prediction -> not use an Encoder, only noise (diffusion step)
        # From the previus step the like marking is maybe changed, we need to follow this change
        # to keep the traking of the line.

        # What we do: we move a random amount to the x coordinate and of start point and
        # and of the end point of the line. the random amount is sampled from a gaussian distribution
        # N(mu, sigma): mu -> current value of x at start or end, sigma

        sigma = [None] * self.N_points

        for i in range(self.N_points):
            sigma[self.N_points - 1 - i] = ((self.N_points-1) - i) * 20/self.N_points + 5

        for p in self.particles:

            if(verbose):
                print("p1_First " + str(p.points))

            for i, point in enumerate(p.points):

                # Variation in X
                rand_norm_1 = np.random.normal(point[0], sigma[i], 1).astype(int)[0]

                # Variation in Y, little
                #rand_norm_2 = np.random.normal(point[1], 1, 1).astype(int)[0]

                point[0] = rand_norm_1
                #point[1] = rand_norm_2

            if(verbose):
                print("p1_After "   + str(p.points))

        return

    def weighting(self, pdf, verbose=False, debug_data=None):
        utils = Utils()
        self.cumweights=[]
        pdf_n=np.array(pdf)

        if(debug_data != None):
            if("particles" in debug_data):
                self.particles=debug_data["particles"]

        tot_w = 0

        for p in self.particles:

            w=0
            p.generateSpline()

            for point_s in p.spline:
                # Each spline point is taken to be weighted
                x, y = int(point_s[0]),int(point_s[1])
                w   += np.sum(pdf_n[y-1:y+2 , x-1:x+2]) # Square around spline point for comparing the numbers

            p.w_original = w
            tot_w       += w


        if(verbose):
            print("BEFORE NORMALIZE WEIGHT")
            for p in self.particles:
                print(p.toString())

        tot_tmp=0

        for p in self.particles:
            if(tot_w==0):
                p.w=1.0/self.N
            else:
                p.w=p.w_original/tot_w
            tot_tmp+=p.w

            # cdf: Cumulative distribution function
            self.cumweights.append(tot_tmp)

        if(verbose):
            print("AFTER NORMALIZE WEIGHT")
            for p in self.particles:
                print(p.toString())

        return

    def resampling(self, verbose=False):

        # It does the resampling using the systematic resampling
        if(verbose):
            print("BEFORE RESAMPLING")
            for p in self.particles:
                print(p.toString())

        new_particles = []
        r_0 = random.uniform(0.0, 1.0/self.N)

        for m in range(0, self.N):
            i = next(k for k, value in enumerate(self.cumweights)if value > r_0 + (float(m)-1.0)/float(self.N))
            new_particles.append(deepcopy(self.particles[i]))

        self.particles=new_particles
        self.doApproximation()
        # print(str(self.approximation.w_original) +"    <"+ str(0.2*self.Interpolation_points*(9*255)))
        if(self.approximation.w_original < 0.1*self.Interpolation_points*(9*255)): # Line weight < 20% of max weights - 255 max value for the pixel
            self.failing_count+=1
            self.approximation_to_show =False
            if(self.failing_count >= self.threshold_reset):
                    self.failing_count=0
                    self.initialization()
                    #print("reset")
        else:
            self.failing_count=0
            self.approximation_to_show =True
            self.steps_good_approximation=self.steps_good_approximation+1

        if(verbose):
            print("AFTER RESAMPLING")
            for p in self.particles:
                print(p.toString())
        return

    def doApproximation(self):
        # Return a particle that approximate the particle distributions
        # It return the average of the particles or the partichle with higher weight
        # type can be "max" or "average"

        if(self.type_approximation=="average"):
            p=deepcopy(self.particles[0])
            for part in self.particles:
                for index, coo in enumerate(p.points):
                    coo[0] += part.points[index][0]
                    coo[1] += part.points[index][1]
            for coo in p.points:
                coo[0] = int(coo[0]/len(self.particles))
                coo[1] = int(coo[1]/len(self.particles))
            if(isinstance(p, Particle)):
                p.generateSpline(Interpolation_points=20)
            self.approximation=p
        elif(self.type_approximation=="max" or True):
            self.approximation = max(self.particles, key=lambda p: p.w)


"""
def filter_usage(N_Particles, Interpolation_points, order=2, N_points=3, dataset_number=1, Images_print=False, blur=7, threshold_reset=4, pts=[]):

    # pts = np.array([(0, 120), (600, 86), (-148, 278), (780, 279)]) # Up-left - - -

    # Function used for creating and running the particle filter

    utils = Utils()

    # path = "../datasets/dataset_"+str(dataset_number)+"/"
    path = "../Dataset/"

    save_images   = False
    testImageCrop = False

    type_approximation = "max"
    threshold_reset    = threshold_reset

    crop_points, approximationPF1, approximationPF2 = [], [], []

    # crop_points = [0, 200, 640, 480]

    # First image upload (one frame) in order to find shapes
    # image.shape[0] -> height
    # image.shape[1] -> width

    pts = np.array([(190, 30), (0, 280), (490, 30), (640, 280)]) # Total pic: [(0, 0), (0, 280), (640, 0), (640, 280)] -> UL, DL, UR, DR

    N_STEP = len([path+name for name in os.listdir(path) if (os.path.isfile(path+name) and ".png" in name) ])-2

    # crop = 0

    for step in range(0, N_STEP):

        #if(crop == 1 and number < 3):
        #    crop_points = np.array( ( crop_points[0], crop_points[1]/(3/2), crop_points[2], crop_points[3]) )
        #    crop_points = crop_points.astype(int)
        #    pts = np.array([(100, 0), (0, 400 - crop_points[1]), (500, 0), (640, 400 - crop_points[1])])
        #    print(crop_points)
        #else:
        #    pts = np.array([(190, 30), (0, 280), (490, 30), (640, 280)])
        #    crop_points = [0, 200, 640, 480]
        #    number = 0
        #    crop = 0

        if(step == 0 or crop == 1):
            ip    = ImageProcessing(pts, path=path, crop_points=crop_points, kernel_size=blur)
            pdf, pdf1, pdf2, image, image1, image2 = ip.get_lines_pdf()

            if(testImageCrop):
                # Temporarly, when decided the size of image it can be put fixed
                ip    = ImageProcessing(pts)
                pdf, pdf1, pdf2, image, image1, image2 = ip.get_lines_pdf()
                cv.imshow("pdf1", pdf1)
                k = cv.waitKey(0)
                cv.imshow("pdf2", pdf2)
                k = cv.waitKey(0)
                cv.imshow("image1", image1)
                k = cv.waitKey(0)
                cv.imshow("image2", image2)
                k = cv.waitKey(0)

            pf1 = ParticleFilter(N_Particles, order, N_points, Interpolation_points, image1.shape[0], image1.shape[1], type_approximation=type_approximation, threshold_reset=threshold_reset)
            pf2 = ParticleFilter(N_Particles, order, N_points, Interpolation_points, image2.shape[0], image2.shape[1], type_approximation=type_approximation, threshold_reset=threshold_reset)

            ip = ImageProcessing(pts, path, crop_points=crop_points, kernel_size=blur)

            pf1.initialization()
            pf2.initialization()

        pf1.sampling()
        pf2.sampling()

        pdf, pdf1, pdf2, image, image1, image2 = ip.get_lines_pdf()

        pf1.weighting(pdf1)
        pf2.weighting(pdf2)

        pf1.resampling()
        pf2.resampling()

        approximationPF1.append(pf1.approximation)
        approximationPF2.append(pf2.approximation)

        best_particles, offset_Approximation = [], []

        if(pf1.approximation_to_show):
            best_particles.append(pf1.approximation)
            offset_Approximation.append(0)

        if(pf2.approximation_to_show):
            best_particles.append(pf2.approximation)
            offset_Approximation.append(int(image.shape[1]/2 ))

        print('pf1 = ' + str(pf1.approximation.points))
        print('pf2 = ' + str(pf2.approximation.points))

        if( abs(pf1.approximation.points[0][0] - pf1.approximation.points[1][0]) > 200 or abs(pf2.approximation.points[0][0] - pf2.approximation.points[1][0]) > 200 ):
            crop = 1
            print('crop rezised')
            number = number + 1

        if(Images_print):
            image_color  = cv.cvtColor(pdf, cv.COLOR_GRAY2RGB)  # Image with color

            # Print single filter image (right or left image size)
            # image_color1 = cv.cvtColor(pdf1, cv.COLOR_GRAY2RGB) # Image with color
            # image_color2 = cv.cvtColor(pdf2, cv.COLOR_GRAY2RGB) # Image with color
            # utils.draw_particles(image_color1, [], "Reampling_PDF1", [pf1.approximation])
            # utils.draw_particles(image_color2, [], "Reampling_PDF2", [pf2.approximation])

            # Print total image
            res_1 = utils.draw_particles(image, [], "Resampling", best_particles, offset=[0]*len(pf1.particles)+[int(image.shape[1]/2 )]*len(pf2.particles), offsetApproximation=offset_Approximation)
            res_2 = utils.draw_particles(image_color, pf1.particles + pf2.particles, "Resampling_PDF", best_particles, offset=[0]*len(pf1.particles)+[int(image.shape[1]/2 )]*len(pf2.particles), offsetApproximation=offset_Approximation)

            if(save_images ):
                cv.imwrite('../outputs/particles_output/img_' + str(step) + '.png', res_2)
                cv.imwrite('../outputs/result_output/img' + str(step) + '.png', res_1)

    return approximationPF1, approximationPF2, N_STEP
"""
def filter_usage_BOSH(N_Particles, Interpolation_points, get_image_function=None, order=1, N_points=2, Images_print=True, blur=7, threshold_reset=7, pts=[], data_queue=None):
    # Function used for creating and running the particle filter

    utils = Utils()

    #path = "../datasets/dataset_"+str(dataset_number)+"/"

    save_images   = False
    testImageCrop = False

    type_approximation = "max"
    threshold_reset    = threshold_reset
    crop_points, approximationPF1, approximationPF2 = [], [], []

    #if(dataset_number==1):
    x_up, y_up, x_down, y_down=0, 300, 640, 470
    crop_points = [x_up, y_up, x_down, y_down]
    modifier=150
    #crop_points = [0, 200, 640, 480]
    pts = np.array([(0, 0), (-modifier, y_down-y_up), (x_down, 0), (x_down+modifier, y_down-y_up)]) # Total pic: [(0, 0), (0, 280), (640, 0), (640, 280)] -> UL, DL, UR, DR

    #else:
    #    crop_points = [0, 200, 640, 480]

    # First image upload (one frame) in order to find shapes
    #ip    = ImageProcessing(pts, path=path, crop_points=crop_points, kernel_size=blur)
    #pdf, pdf1, pdf2, image, image1, image2 = ip.get_lines_pdf()

    #if(testImageCrop):
        # Temporarly, when decided the size of image it can be put fixed
    #    ip    = ImageProcessing(pts)
    #    pdf, pdf1, pdf2, image, image1, image2 = ip.get_lines_pdf()
    #    cv.imshow("pdf1", pdf1)
    #    k = cv.waitKey(0)
    #    cv.imshow("pdf2", pdf2)
    #    k = cv.waitKey(0)
    #    cv.imshow("image1", image1)
    #    k = cv.waitKey(0)
    #    cv.imshow("image2", image2)
    #    k = cv.waitKey(0)

    ip = ImageProcessing(pts, "", crop_points=crop_points, kernel_size=blur, get_image=get_image_function)
    pdf, pdf1, pdf2, image, image1, image2 = ip.get_lines_pdf()

    pf1 = ParticleFilter(N_Particles, order, N_points, Interpolation_points, image1.shape[0], image1.shape[1], type_approximation=type_approximation, threshold_reset=threshold_reset)
    pf2 = ParticleFilter(N_Particles, order, N_points, Interpolation_points, image2.shape[0], image2.shape[1], type_approximation=type_approximation, threshold_reset=threshold_reset)

    pf1.initialization()
    pf2.initialization()

    while(1):
        pf1.sampling()
        pf2.sampling()

        pdf, pdf1, pdf2, image, image1, image2 = ip.get_lines_pdf()

        pf1.weighting(pdf1)
        pf2.weighting(pdf2)


        pf1.resampling()
        pf2.resampling()

        #approximationPF1.append(pf1.approximation)
        #approximationPF2.append(pf2.approximation)

        best_particles, offset_Approximation, lines = [], [], [None, None]

        if(pf1.approximation_to_show):
            best_particles.append(pf1.approximation)
            offset_Approximation.append(0)
            lines[0]=pf1.approximation

        if(pf2.approximation_to_show):
            best_particles.append(pf2.approximation)
            offset_Approximation.append(int(image.shape[1]/2 ))
            lines[1]=pf2.approximation
        ##########  TO AVOID THE MERGE OF 2 LINES
        if(pf1.approximation_to_show & pf2.approximation_to_show):
            too_near = False
            acc_dist=0
            for i in range(1, len(lines[0].points)-1):
                #print(abs(dist[0][i])+abs(dist[1][i]))
                #acc_dist = acc_dist + abs(lines[1].spline[i][0] - lines[0].spline[i][0])
                #image1.shape[0] is for adding offset since the image start in the "middle"
                print(image1.shape[0]  + lines[1].points[i][0] - lines[0].points[i][0])
                if(image1.shape[0]  +  lines[1].points[i][0] - lines[0].points[i][0] <20):
                    too_near=True
                #print(abs(lines[1].spline[i][0] - lines[0].spline[i][0]))
            #too_near = acc_dist/len(lines[0].spline) < 640/6
            if(too_near):
                print("too near")
                if(pf1.steps_good_approximation > pf2.steps_good_approximation):
                    pf2.initialization()
                    lines[1] = None
                else:
                    pf1.initialization()
                    lines[0] = None

        data_queue.put(lines)
        if(Images_print):
            image_color  = cv.cvtColor(pdf, cv.COLOR_GRAY2RGB)  # Image with color

            # Print single filter image (right or left image size)
            #image_color1 = cv.cvtColor(pdf1, cv.COLOR_GRAY2RGB) # Image with color
            #image_color2 = cv.cvtColor(pdf2, cv.COLOR_GRAY2RGB) # Image with color
            #utils.draw_particles(image_color1, [], "Reampling_PDF1", [pf1.approximation])
            #utils.draw_particles(image_color2, [], "Reampling_PDF2", [pf2.approximation])

            # Print total image
            res_1 = utils.draw_particles(image, [], "Resampling", best_particles, offset=[0]*len(pf1.particles)+[int(image.shape[1]/2 )]*len(pf2.particles), offsetApproximation=offset_Approximation)
            res_2 = utils.draw_particles(image_color, pf1.particles + pf2.particles, "Resampling_PDF", best_particles, offset=[0]*len(pf1.particles)+[int(image.shape[1]/2 )]*len(pf2.particles), offsetApproximation=offset_Approximation)

            if(save_images ):
                cv.imwrite('../outputs/particles_output/img_' + str(step) + '.png', res_2)
                cv.imwrite('../outputs/result_output/img' + str(step) + '.png', res_1)

    #return approximationPF1, approximationPF2, N_STEP


if __name__ == '__main__':

    N_particles           = 50  #100 # Particles used in the filter
    Interpolation_points  = 17  #25  # Interpolation points used for the spline
    order                 = 2        # Spline order
    N_c                   = 3        # Number of spline control points
    dataset_number        = 2        # Dataset number -> available 1/2/3/4

    # Dataset one has a different camera orientation with respect the others. Thus for this reason the points chosen for the IPM changes

    # pts = np.array([(0, 176-70), (500+100, 186-100), (-148, 345), (639+148, 350)])

    approximationPF1, approximationPF2, N_frame      = filter_usage(N_particles, Interpolation_points, order, N_c, dataset_number, Images_print=True)
