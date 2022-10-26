#!/usr/bin/env python
# Copyright 2022 Kevin Trejos Vargas <kevin.trejosvargas@ucr.ac.cr>

# -- Imports

# -- Step 1: Rotate the generated map to the same orientation of the Ground Truth.
# -- Step 2: Try to match, using a correlation study, looking for a correlation beyond 95% (empirical guess).
# -- Step 2.1: If the correlation is not met, move the image along the ground truth looking for condition to be met.
# -- Clue: Starting with a kind of coarse search may be a good idea, or using a pattern recognition algorithm.
# -- Step 3: Compute the metric for map error (using knn if pixels to meters conversion is well known).

# from PIL import Image
import rospy
import cv2
import numpy as np
import math
import os
import PIL
from PIL import Image 

# from skimage import io, img_as_float
# import matplotlib.pyplot as plt
# import numpy as np

class MapAccuracy:
    def __init__(self):
        self.GTMapPath = ''

    def setGroundTruthMap(self, GTMapPath):
        self.GTMapPath = GTMapPath

    def compute(self, ActualMap):
        SLAMMapImage0 = Image.open(ActualMap)
        print(SLAMMapImage0.mode)
        print(SLAMMapImage0.size)
        thresh = 200
        fn = lambda x : 255 if x > thresh else 0
        SLAMMapImage = SLAMMapImage0.convert('L').point(fn, mode='1')
        print(SLAMMapImage.mode)
        print(SLAMMapImage.size)
        SLAMMapImage = np.asarray(SLAMMapImage)
        SLAMMapImage = (SLAMMapImage * 255).astype(np.uint8)
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
        SLAMMapImage = Image.fromarray(np.uint8(SLAMMapImage)).convert('L')
        # SLAMMapImage.save('Prueba0.png')
        GTImage0 = Image.open(self.GTMapPath)
        print(GTImage0.mode)
        thresh = 200
        fn = lambda x : 255 if x > thresh else 0
        GTImage = GTImage0.convert('L').point(fn, mode='1')
        print(GTImage.mode)
        GTImage = np.asarray(GTImage)
        GTImage = (GTImage * 255).astype(np.uint8)
        tSizeScale = (GTImage.shape[1], GTImage.shape[0])
        SLAMMapImage = SLAMMapImage.resize(tSizeScale)
        thresh = 200
        fn = lambda x : 255 if x > thresh else 0
        SLAMMapImage = SLAMMapImage.point(fn, mode='1')
        SLAMMapImage = np.asarray(SLAMMapImage)
        SLAMMapImage = (SLAMMapImage * 255).astype(np.uint8)
        # SLAMMapImage.save('Prueba1.png')
        # # -- Computing the map error metric as RMSE - Try using https://stackoverflow.com/questions/52576498/find-nearest-neighbor-to-each-pixel-in-a-map
        fMapError = np.sum((SLAMMapImage.astype("float") - GTImage.astype("float")) ** 2)
        fMapError /= float(SLAMMapImage.shape[0] * SLAMMapImage.shape[1])
        fMapError = math.sqrt(fMapError)
        return fMapError

mmv = open(os.getcwd().replace('.ros','') + "catkin_ws/src/slam_auto_calibrator/src/MapMetricVariables.txt", "r")
for line in mmv.readlines():
    if "GTMapPath" in line:
        GTMapPath = line.split("=")[1].replace("\n","")
    elif "SLAMMapPath" in line:
        SLAMMapPath = line.split("=")[1].replace("\n","")
mmv.close()
MapMetric = MapAccuracy()
MapMetric.setGroundTruthMap(GTMapPath)
error = MapMetric.compute(SLAMMapPath)
mmv = open(os.getcwd().replace('.ros','') + "catkin_ws/src/slam_auto_calibrator/src/MapMetricVariables.txt", "w")
mmv.write("MapError={}\n".format(error))
mmv.close()