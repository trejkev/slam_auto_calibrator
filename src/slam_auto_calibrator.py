#! /usr/bin/env python

__author__ = "Kevin Trejos Vargas"
__email__  = "kevin.trejosvargas@ucr.ac.cr"

# -- Imports -- #
import rospy                                                                    # Needed for ros and python interaction
import subprocess, time, os                                                     # Needed to interact with ros terminal
from datetime import datetime
import logging as log

# from MapAccuracy     import MapAccuracy     as MapMetric
# from PoseAccuracy    import PoseAccuracy    as PoseMetric
# from ParamsGenerator import ParamsGenerator as Params

# MapMetric  = MapMetric(GroundTruthMap)
# PoseMetric = PoseMetric()
# Params     = Params()

# -- Global variables -- #
iTrainingCycles    = rospy.get_param("/TrainingCycles"    )                     # Number of cycles to run the AI calibration strategy
sSLAMName          = rospy.get_param("/SLAMName"          )
iRobotsQty         = rospy.get_param("/RobotsQty"         )
# -- Robots launching variables
sRobotsLauchName   = rospy.get_param("/RobotsLauchName"   )
# -- SLAM launching variables: Contains SLAM, map merger, and rviz stuff
sSLAMPackageName   = rospy.get_param("/SLAMPackageName"   )
sSLAMLaunchName    = rospy.get_param("/SLAMLaunchName"    )
# -- Parameters files
sParamsDictName    = rospy.get_param("/ParamsDictName"    )
dParams            = {}                                                         # Stores the algorithm parameters

rospy.init_node('slam_auto_calibrator')                                         # SLAM auto calibrator node creation

subprocess.Popen("killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient", shell = True)
time.sleep(2)
# -- Open the arena with three robots
subprocess.Popen("roslaunch slam_auto_calibrator {lf}".format(lf = sRobotsLauchName), shell = True)
time.sleep(10)

# -- Set the initial parameters
# Params.setInitialParameters(dParameters = dParams)

# Set the initial parameters to SLAM running

for iActualCycle in range(iTrainingCycles):
    # -- Use each SLAM algorithm until they complete their route
    subprocess.Popen("roslaunch {pn} {lf}".format(pn=sSLAMPackageName, lf=sSLAMLaunchName), shell = True)
    time.sleep(10)
    bRunCompleted = False
    while bRunCompleted == False:
        nodes = os.popen("rosnode list").readlines()
        for index in range(len(nodes)):
            nodes[index] = nodes[index].replace("\n", "")
        iNodeCount = 0
        for node in nodes:
            iNodeCount += 1
            if "speed_controller" in node:
                break
            if iNodeCount == len(nodes):
                bRunCompleted = True
        time.sleep(10)
            
    # -- Generate the info needed by map metric
    date = datetime.now().strftime("%Y_%m_%d-%I:%M:%S_%p")
    MapName = "{sn}_Trial_{t}_RobotsQty_{rc}_Map_{d}".format(sn = sSLAMName, t = iActualCycle, rc = iRobotsQty, d = date)
    subprocess.Popen("rosrun map_server map_saver -f ~/Documents/Maps/{MapName}".format(MapName = MapName), shell = True)
    time.sleep(2)
    # -- Generate the info needed by pose metric

    # -- Kill the SLAM algorithms and their related nodes
    nodes = os.popen("rosnode list").readlines()
    for index in range(len(nodes)):
        nodes[index] = nodes[index].replace("\n", "")
    for node in nodes:
        if 'rviz'                in node \
            or 'map_merge'       in node \
            or 'map_saver'       in node \
            or 'turtlebot3_slam' in node:
            os.system("rosnode kill " + node)
    # -- Compute the metrics
    # fActualMapError  = MapMetric.compute(ActualMap)
    # fActualXPoseError, fActualYPoseError, fActualRotPoseError = PoseMetric.compute(GroundTruthPose, ActualPose)
    # if iActualCycle == 0:
    #     log.debug("Initial map error is {} meters".format(fActualMapError))
    #     log.debug("Initial pose error is {} meters".format(fActualPoseError))
    # -- Feed the AI to generate the new set of parameters
    # dParams = Params.computeNewParameters(MapError     = fActualMapError    ,
    #                                       XPoseError   = fActualXPoseError  ,
    #                                       YPoseError   = fActualYPoseError  ,
    #                                       RotPoseError = fActualRotPoseError)
    # -- Modify the SLAM parameters into their launch files

# log.debug("Final map error is {} meters".format(fActualMapError))
# log.debug("Final pose error is {} meters".format(fActualPoseError))
# -- Kill the gazebo related nodes
nodes = os.popen("rosnode list").readlines()
for index in range(len(nodes)):
    nodes[index] = nodes[index].replace("\n", "")
for node in nodes:
    if 'rosout' not in node and 'slam_auto_calibrator' not in node:
        os.system("rosnode kill " + node)
subprocess.Popen("killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient", shell = True)
