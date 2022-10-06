# Copyright 2022 Kevin Trejos Vargas <kevin.trejosvargas@ucr.ac.cr>

class ParamsGenerator(object):
    """docstring for ."""

    def __init__(self, arg):
        super(, self).__init__()
        self.arg = arg
        self.dParams = {}

    def setInitialParameters(self, dParameters):
        self.dParams = dParameters

    def computeNewParameters(self, MapError , XPoseError, YPoseError, RotPoseError):
        return self.dParams
