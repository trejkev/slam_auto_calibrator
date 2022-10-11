#! /usr/bin/env python

__author__ = "Kevin Trejos Vargas"
__email__  = "kevin.trejosvargas@ucr.ac.cr"

# -- Imports -- #
import rospy                                                                    # Needed for ros and python interaction
import subprocess, time, os                                                     # Needed to interact with ros terminal

from datetime        import datetime
# from std_msgs.msg    import Float64                                             # Needed for APE publishing

from slam_auto_calibrator.msg import APE
# from MapAccuracy     import MapAccuracy     as MapMetric
# from PoseAccuracy    import PoseAccuracy    as PoseMetric
# from ParamsGenerator import ParamsGenerator as Params

rospy.init_node('slam_auto_calibrator') 

# MapMetric  = MapMetric(GroundTruthMap)
# PoseMetric = PoseMetric()
# Params     = Params()

# message = APE()
def APEReading(data):
    global RobotsNamespaceBase
    rospy.loginfo("Actual frame id is " + data.frame_id)
    if data.frame_id == RobotsNamespaceBase + "0":
        pass
    elif data.frame_id == RobotsNamespaceBase + "1":
        pass
    elif data.frame_id == RobotsNamespaceBase + "2":
        pass

################################################################################
############################ -- ASSIGNING PARAMS -- ############################
################################################################################

iTrainingCycles             = 1000
sSLAMName                   = "gmapping"
iRobotsQty                  = 3
sRobotsLauchName            = "multi_robot_in_world.launch"
sSLAMPackageName            = "slam_auto_calibrator"
sSLAMLaunchName             = "slam_launcher.launch"
sParamsDictName             = "Parameters_Robot"
APETranslationTopicName     = "APETranslation"
APERotationTopicName        = "APERotation"
APETopicName                = "APE"
APETranslationTopicReadings = list(range(3))
APERotationTopicReadings    = list(range(3))
RobotsNamespaceBase         = "tb3_"
dParams                     = {}                                                    # Stores the algorithm parameters

if rospy.has_param("/TrainingCycles"):
    iTrainingCycles = rospy.get_param("/TrainingCycles")                        # Number of cycles to run the AI calibration strategy
if rospy.has_param("/SLAMName"):
    sSLAMName = rospy.get_param("/SLAMName")
if rospy.has_param("/RobotsQty"):
    iRobotsQty = rospy.get_param("/RobotsQty")
# -- Robots launching variables
if rospy.has_param("/RobotsLauchName"):
    sRobotsLauchName = rospy.get_param("/RobotsLauchName")
# -- SLAM launching variables: Contains SLAM, map merger, and rviz stuff
if rospy.has_param("/SLAMPackageName"):
    sSLAMPackageName = rospy.get_param("/SLAMPackageName")
if rospy.has_param("/SLAMLaunchName"):
    sSLAMLaunchName    = rospy.get_param("/SLAMLaunchName")
# -- Parameters files
if rospy.has_param("/RobotsQty"):
    sParamsDictName = rospy.get_param("/RobotsQty")
# -- APE topics publishing translation and rotation APE mean
if rospy.has_param("/APE_Translation_Topic_Name"):
    APETranslationTopicName = rospy.get_param("/APE_Translation_Topic_Name")
if rospy.has_param("/APE_Rotation_Topic_Name"):
    APERotationTopicName = rospy.get_param("/APE_Rotation_Topic_Name")
if rospy.has_param("/APE_Topic_Name"):
    APETopicName = rospy.get_param("/APE_Topic_Name")
# -- Robots namespace base naming convention
if rospy.has_param("/Robots_Namespace_Base"):
    RobotsNamespaceBase = rospy.get_param("/Robots_Namespace_Base")

# -- Killing all gazebo processes that may be open
subprocess.Popen("killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient", shell = True)
time.sleep(2)
# -- Open the arena with three robots
subprocess.Popen("roslaunch slam_auto_calibrator {lf}".format(lf = sRobotsLauchName), shell = True)
time.sleep(10)

# -- Set the initial parameters
# Params.setInitialParameters(dParameters = dParams)

# -- Subscribe to each APE publisher robot topic
APETranslationTopics = list(range(iRobotsQty))
APERotationTopics    = list(range(iRobotsQty))
for iRobot in range(iRobotsQty):
    # APETranslationTopics[iRobot] = rospy.Subscriber("/" + RobotsNamespaceBase + str(iRobot) + "/" + APETranslationTopicName, APE, APEReading)
    # APERotationTopics[iRobot]    = rospy.Subscriber("/" + RobotsNamespaceBase + str(iRobot) + "/" + APERotationTopicName   , APE, APEReading)
    APERotationTopics[iRobot] = rospy.Subscriber("/" + RobotsNamespaceBase + str(iRobot) + "/" + APETopicName, APE, APEReading)
    

for iActualCycle in range(iTrainingCycles):
    # -- Use each SLAM algorithm until they complete their route
    subprocess.Popen("roslaunch {} {}".format(sSLAMPackageName, sSLAMLaunchName), shell = True)
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
    subprocess.Popen("rosrun map_server map_saver -f ~/catkin_ws/src/slam_auto_calibrator/Maps/{}".format(MapName), shell = True)
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
