#!/usr/bin/env python

__author__ = "Kevin Trejos Vargas"
__email__  = "kevin.trejosvargas@ucr.ac.cr"

"""
MIT License

Copyright (c) 2022-2023 Kevin Trejos Vargas

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import rospy, cv2, math, os, PIL
import numpy as np
from PIL import Image 

Image.MAX_IMAGE_PIXELS = 60000000*60000000

class MapAccuracy:
    def __init__(self):
        self.GTMapImage = ''

    def set_ground_truth_map(self, GTMapPath):
        self.GTMapImage = GTMapPath
    
    def set_slam_generated_map(self, SLAMMapPath):
        self.SLAMMapImage = SLAMMapPath
        
    def pre_process_slam_generated_map(self):
        # -- Reading the SLAM-generated map as black and white
        self.SLAMMapImage = cv2.imread(self.SLAMMapImage, cv2.IMREAD_GRAYSCALE)
        self.SLAMMapImage = (
            cv2.threshold(self.SLAMMapImage, 200, 255, cv2.THRESH_BINARY)[1]
        )

        # -- Converting the image to a binary array
        self.SLAMMapImage = np.asarray(self.SLAMMapImage)
        lowestCol         = 0
        highestCol        = self.SLAMMapImage.shape[1]
        lowestRow         = 0
        highestRow        = self.SLAMMapImage.shape[0]
        bLowColFound      = False
        bLowRowFound      = False

        # -- Getting bounds to cut the SLAM generated map
        for col in range(self.SLAMMapImage.shape[1]):
            if (
                np.any(self.SLAMMapImage[:,col] < 25) and
                bLowColFound == False
            ):
                lowestCol = col
                bLowColFound = True
            elif (
                np.any(self.SLAMMapImage[:,col] < 25) and
                bLowColFound != False
            ):
                highestCol = col
        for row in range(self.SLAMMapImage.shape[0]):
            if (
                np.any(self.SLAMMapImage[row,:] < 25) and
                bLowRowFound == False
            ):
                lowestRow = row
                bLowRowFound = True
            elif (
                np.any(self.SLAMMapImage[row,:] < 25) and
                bLowRowFound != False
            ):
                highestRow = row
        self.SLAMMapImage = (
            self.SLAMMapImage[lowestRow:highestRow, lowestCol:highestCol]
        )                                                                       # Stores the map output cropped
        self.SLAMMapImage = cv2.resize(self.SLAMMapImage, self.tSizeScale)

    def pre_process_ground_truth_map(self):
        # -- Converts the ground truth image to a binary array
        self.GTMapImage = Image.open(self.GTMapImage)
        fn              = lambda x : 255 if x > 200 else 0                      # Decision threshold settled to 200
        self.GTMapImage = np.asarray(
            self.GTMapImage.convert('L').point(fn, mode='1')
        )                                                                       # Converts the ground truth image to an array
        self.GTMapImage = (self.GTMapImage * 255).astype(np.uint8)
        self.tSizeScale = (self.GTMapImage.shape[1], self.GTMapImage.shape[0])

    def compute_map_error(self):
        # -- Compute auxiliar variables
        fSLAMMapImage    = self.SLAMMapImage.astype("float")
        fGTImage         = self.GTMapImage.astype("float")
        iSLAMImageHeight = self.SLAMMapImage.shape[0]
        iSLAMImageWidth  = self.SLAMMapImage.shape[1]

        # -- Compares the ground truth with the SLAM generated map
        fMapError        = np.sum((fSLAMMapImage - fGTImage) ** 2)
        fMapError       /= float(iSLAMImageHeight * iSLAMImageWidth)
        fMapError        = math.sqrt(fMapError)

        return fMapError


################################################################################
# --                               Main script                              -- #
################################################################################

PACKAGE_PATH = "/home/cerlabrob/catkin_ws/src/slam_auto_calibrator/"

# -- Get the paths of the maps
mmv = open(PACKAGE_PATH + "src/MapMetricVariables.txt", "r")
for line in mmv.readlines():
    if "GTMapPath" in line:
        GTMapPath = line.split("=")[1].replace("\n","")
    elif "SLAMMapPath" in line:
        SLAMMapPath = line.split("=")[1].replace("\n","")
mmv.close()

# -- Pre-process the maps
MapMetric = MapAccuracy()
MapMetric.set_ground_truth_map(GTMapPath)
MapMetric.set_slam_generated_map(SLAMMapPath)
MapMetric.pre_process_ground_truth_map()
MapMetric.pre_process_slam_generated_map()

error = MapMetric.compute_map_error()

# -- Save the map error into the output file
mmv = open(PACKAGE_PATH + "src/MapMetricVariables.txt", "w")
mmv.write("MapError={}\n".format(error))
mmv.close()