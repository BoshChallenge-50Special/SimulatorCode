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
from scipy.spatial import distance as dist

class Utils(object):

    def draw_particles(self, image, particles, name, approximations=None, offset=[], offsetApproximation=[], start_coo=[]):
        image= image.copy()
        line_thickness=1
        for i, p in enumerate(particles):

            if(len(offset)>0 and offset[i]!=0 and len(p.spline)>0):
                p.generateSpline(offset[i])

            cv.polylines(image, np.int32([p.spline]), False, (0,0,255), thickness=line_thickness)

        if(approximations!=None):
            for i, approximation in enumerate(approximations):
                if(approximation!=None):
                    if(len(offsetApproximation)>0 and offsetApproximation[i]!=0):
                        approximation.generateSpline(offset_x=offsetApproximation[i])
                    if(len(start_coo)>0):
                        approximation.generateSpline(offset_x=start_coo[i][0], offset_y=start_coo[i][1])

                    cv.polylines(image, np.int32([approximation.spline]), False, (0,254,0), thickness=line_thickness+1)

        cv.imshow(name, image)
        k = cv.waitKey(1)

        return image

    def order_points(self, pts):
        # sort the points based on their x-coordinates
        # [[145 176]  ---> [[  0 345]
        #  [500 186]  --->  [145 176]
        #  [  0 345]  --->  [500 186]
        #  [639 350]] --->  [639 350]]

        xSorted = pts[np.argsort(pts[:, 0]), :]

        # grab the left-most and right-most points from the sorted
        # x-roodinate points
        leftMost  = xSorted[:2, :]
        rightMost = xSorted[2:, :]

        # now, sort the left-most coordinates according to their
        # y-coordinates so we can grab the top-left and bottom-left
        # points, respectively
        leftMost = leftMost[np.argsort(leftMost[:, 1]), :]
        (tl, bl) = leftMost

        # now that we have the top-left coordinate, use it as an
        # anchor to calculate the Euclidean distance between the
        # top-left and right-most points; by the Pythagorean
        # theorem, the point with the largest distance will be
        # our bottom-right point
        D = dist.cdist(tl[np.newaxis], rightMost, "euclidean")[0]
        (br, tr) = rightMost[np.argsort(D)[::-1], :]

        # return the coordinates in top-left, top-right,
        # bottom-right, and bottom-left order
        return np.array([tl, tr, br, bl], dtype="float32")

    def make_matrix_transform_four_point(self, pts):
        # obtain a consistent order of the points and unpack them
        # individually
        rect = self.order_points(pts)
        (tl, tr, br, bl) = rect

        # compute the width of the new image, which will be the
        # maximum distance between bottom-right and bottom-left
        # x-coordiates or the top-right and top-left x-coordinates
        widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
        widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
        maxWidth = max(int(widthA), int(widthB))

        # compute the height of the new image, which will be the
        # maximum distance between the top-right and bottom-right
        # y-coordinates or the top-left and bottom-left y-coordinates
        heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
        heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
        maxHeight = max(int(heightA), int(heightB))

        # now that we have the dimensions of the new image, construct
        # the set of destination points to obtain a "birds eye view",
        # (i.e. top-down view) of the image, again specifying points
        # in the top-left, top-right, bottom-right, and bottom-left
        # order
        dst = np.array([
            [0, 0],
            [maxWidth - 1, 0],
            [maxWidth - 1, maxHeight - 1],
            [0, maxHeight - 1]], dtype="float32")

        # compute the perspective transform matrix and then apply it
        M = cv.getPerspectiveTransform(rect, dst)

        return M, maxWidth, maxHeight

    def four_point_transform(self, image, pts):

        M, maxWidth, maxHeight = self.make_matrix_transform_four_point(pts)
        warped = cv.warpPerspective(image, M, (maxWidth, maxHeight))

        # return the warped image
        return warped

    def four_point_transform_inverse(self, points, pts,  add_half_image=False):

        M, maxWidth, maxHeight = self.make_matrix_transform_four_point(pts)
        if(add_half_image==True):
            for i in range(0, len(points)):
                points[i][0] = points[i][0] + maxWidth/2
        M_inv = np.linalg.inv(M)

        pointsOut = cv.perspectiveTransform(np.array([points]), M_inv)[0]

        #M_inv = np.linalg.inv(M)
        #transformed_points = pointsOut = cv2.perspectiveTransform(a, h) cv2.warpPerspective(points, M, table_image_size, cv2.WARP_INVERSE_MAP)

        # return transformed points
        return pointsOut

    def evaluate(self, particle, pdf, verbose=False):

        pdf_n=np.array(pdf)
        w=0
        w_=0
        particle.generateSpline(Interpolation_Points=100)

        for point_s in particle.spline:
            # Each spline point is taken to be weighted
            x, y = int(point_s[0]),int(point_s[1])
            if(y<pdf.shape[0] and x<pdf.shape[1] and x > 0 and y > 0 and pdf_n[y , x]>220 ):
                w+=1

        return w/100
