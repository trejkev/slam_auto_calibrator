#! /usr/bin/env python

__author__ = "Kevin Trejos Vargas"
__email__  = "kevin.trejosvargas@ucr.ac.cr"

class ParamsGenerator(object):

    def __init__(self, sParametersPath):
        self.dParams     = {}
        self.sParamsPath = ""
        self.iMapError   = 9999999
        self.lPoseError  = []
        self._setParametersPath(sParametersPath)

    def _setParametersPath(self, sParametersPath):
        self.sParamsPath = sParametersPath
        self._getParameters()
    
    def _getParameters(self):
        fParamsFile = open(self.sParamsPath, 'r')
        for line in fParamsFile:
            # If the line contains a parameter
            if line.split(":")[0] != "":
                param = line.split(":")[0]
                value = line.split(":")[1].replace(" ", "").split("#")[0]
                type  = line.split("#")[1]
                self.dParams.update({param: [value, type]})
        fParamsFile.close()

    def computeNewParameters(self, iMapError, lPoseError):
        self.iMapError  = iMapError
        self.lPoseError = lPoseError
        # HERE GOES THE AI ALGORITHM THAT UPGRADES THE PARAMS
        self._setParameters()
        return self.dParams
            
    def _setParameters(self):
        fParamsFile = open(self.sParamsPath, 'r+')
        fParamsFile.truncate(0)                                                 # Removing all contents
        for param in list(self.dParams.keys()):
            value = self.dParams[param][0]
            type = self.dParams[param][1]
            fParamsFile.write("{}: {} #{}".format(param, value, type))
        fParamsFile.close()
