#!/usr/bin/env python
# Copyright 2022 Kevin Trejos Vargas <kevin.trejosvargas@ucr.ac.cr>

# -- Imports

# -- Step 1: Rotate the generated map to the same orientation of the Ground Truth.
# -- Step 2: Try to match, using a correlation study, looking for a correlation beyond 95% (empirical guess).
# -- Step 2.1: If the correlation is not met, move the image along the ground truth looking for condition to be met.
# -- Clue: Starting with a kind of coarse search may be a good idea, or using a pattern recognition algorithm.
# -- Step 3: Compute the metric for map error (using knn if pixels to meters conversion is well known).

# from PIL import Image
import cv2
import numpy as np
import math
import os

# from skimage import io, img_as_float
# import matplotlib.pyplot as plt
# import numpy as np

class MapAccuracy:
    def __init__(self):
        self.GTMapPath = ''

    def setGroundTruthMap(self, GTMapPath):
        self.GTMapPath = GTMapPath

    def compute(self, ActualMap):
        # -- SLAM map to BW and cropped
        SLAMMapImage = cv2.imread(ActualMap)
        gray_image = cv2.cvtColor(SLAMMapImage, cv2.COLOR_BGR2GRAY)
        (thresh, SLAMMapImage) = cv2.threshold(gray_image, 125, 255, cv2.THRESH_BINARY)
        lowestCol    = 0
        highestCol   = SLAMMapImage.shape[1]
        lowestRow    = 0
        highestRow   = SLAMMapImage.shape[0]
        bLowColFound = False
        bLowRowFound = False
        # -- Getting bounds to cut the SLAM generated map
        for col in range(SLAMMapImage.shape[1]):
            if np.any(SLAMMapImage[:,col] < 25) and bLowColFound == False:
                lowestCol = col
                bLowColFound = True
            elif np.any(SLAMMapImage[:,col] < 25) and bLowColFound != False:
                highestCol = col
        for row in range(SLAMMapImage.shape[0]):
            if np.any(SLAMMapImage[row,:] < 25) and bLowRowFound == False:
                lowestRow = row
                bLowRowFound = True
            elif np.any(SLAMMapImage[row,:] < 25) and bLowRowFound != False:
                highestRow = row
        SLAMMapImage = SLAMMapImage[lowestRow:highestRow, lowestCol:highestCol] # Map in BW and cropped
        GTImage = cv2.imread(self.GTMapPath)                                    # GT map
        GTImage = cv2.cvtColor(GTImage, cv2.COLOR_BGR2GRAY)
        tSizeScale = (GTImage.shape[1],GTImage.shape[0])
        SLAMMapImage = cv2.resize(SLAMMapImage, tSizeScale)                     # SLAM map image resized
        # -- Computing the map error metric as RMSE - Try using https://stackoverflow.com/questions/52576498/find-nearest-neighbor-to-each-pixel-in-a-map
        fMapError = np.sum((SLAMMapImage.astype("float") - GTImage.astype("float")) ** 2)
        fMapError /= float(SLAMMapImage.shape[0] * SLAMMapImage.shape[1])
        fMapError = math.sqrt(fMapError)
        return fMapError
    
# MapMetric = MapAccuracy()
# MapMetric.setGroundTruthMap('/home/cerlabrob/catkin_ws/src/slam_auto_calibrator/Maps/Ground_Truth_arena_entrenamiento.png')
# error = MapMetric.compute('/home/cerlabrob/catkin_ws/src/slam_auto_calibrator/Maps/gmapping_Trial_0_RobotsQty_3_Map_2022_10_11-11:19:49_PM.pgm')
# print("error is {}".format(error))
# print(os.getcwd())