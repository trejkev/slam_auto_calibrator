# Copyright 2022 Kevin Trejos Vargas <kevin.trejosvargas@ucr.ac.cr>

class PoseAccuracy(object):
    """Pose accuracy metric for robot x, y, and rotational errors"""

    def __init__(self, arg):
        super(, self).__init__()
        self.arg = arg

    def compute(self, GroundTruthPose, ActualPose):
        fXPoseError   = 0.2319
        fYPoseError   = 0.9874
        fRotPoseError = 12
        return fXPoseError, fYPoseError, fRotPoseError
