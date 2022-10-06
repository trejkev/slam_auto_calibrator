# Copyright 2022 Kevin Trejos Vargas <kevin.trejosvargas@ucr.ac.cr>

# -- Imports

# -- Step 1: Rotate the generated map to the same orientation of the Ground Truth.
# -- Step 2: Try to match, using a correlation study, looking for a correlation beyond 95% (empirical guess).
# -- Step 2.1: If the correlation is not met, move the image along the ground truth looking for condition to be met.
# -- Clue: Starting with a kind of coarse search may be a good idea, or using a pattern recognition algorithm.
# -- Step 3: Compute the metric for map error (using knn if pixels to meters conversion is well known).

class MapAccuracy(object):
    def __init__(self, GroundTruthMap):
        self.setGroundTruthMap(GroundTruthMap)

    def setGroundTruthMap(self, GTMap):
        self.GTMap = GTMap

    def compute(self, ActualMap):
        fMapError = 73.59
        return fMapError
