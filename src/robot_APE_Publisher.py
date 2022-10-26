#! /usr/bin/env python

__author__      = "Kevin Trejos Vargas"
__email__       = "kevin.trejosvargas@ucr.ac.cr"

"""
    Description:
        Robot APE publisher, for position and rotation. It takes
        the tf topic and robotPosePublisher topics, digs for specific transforms
        and recursively publishes the corresponding pose and rotation mean errors
        through {robot_ID}/APE[Position, Rotation] topic, at a rate defined by 
        samplingRate.
"""

import rospy                                                                    # Needed for ros and python interaction
import math                                                                     # Needed to compute sqrt for pose APE
from tf2_msgs.msg      import TFMessage                                         # Needed to get the bot estimated pose
from geometry_msgs.msg import PoseStamped                                       # Needed to get the bot GT pose
from std_msgs.msg      import Header                                            # Needed to publish the APE message header
from std_msgs.msg      import Float64                                           # Needed for APE publishing
from datetime          import datetime

from slam_auto_calibrator.msg import APE

bTFReadingSet            = False
bGTReadingSet            = False
APEOngoing               = False
bOdomTFReadingSet        = False
bBaseFootprintReadingSet = False
lGTPosition              = list(range(3))
lTFPosition              = list(range(9))
APETranslation           = 0
APERotation              = 0
iSamplesCounter          = 1


def TFReader(data):
    ''' Description: This method is called when there is a message published into
    tf topic, it reads x and y translation, and z rotation, both from odom and
    base_footprint, and then computes the addition of them to get the map to
    base_footprint pose estimation and rotation estimation '''
    global bTFReadingSet
    global APEOngoing
    global bOdomTFReadingSet
    global bBaseFootprintReadingSet
    global lTFPosition
    if APEOngoing == False:
        if bTFReadingSet == False and "odom" in data.transforms[0].child_frame_id:
            lTFPosition[0] = data.transforms[0].transform.translation.x
            lTFPosition[1] = data.transforms[0].transform.translation.y
            lTFPosition[2] = data.transforms[0].transform.rotation.z
            bOdomTFReadingSet = True
        elif bTFReadingSet == False and "base_footprint" in data.transforms[0].child_frame_id:
            lTFPosition[3] = data.transforms[0].transform.translation.x
            lTFPosition[4] = data.transforms[0].transform.translation.y
            lTFPosition[5] = data.transforms[0].transform.rotation.z
            bBaseFootprintReadingSet = True
        if bOdomTFReadingSet == True and bBaseFootprintReadingSet == True:
            lTFPosition[6] = lTFPosition[0] + lTFPosition[3]
            lTFPosition[7] = lTFPosition[1] + lTFPosition[4]
            lTFPosition[8] = lTFPosition[2] + lTFPosition[5]
            bOdomTFReadingSet        = False
            bBaseFootprintReadingSet = False
            bTFReadingSet            = True
    
def GTReader(data):
    ''' Description: This method is called when there is a message published into
    robotPosePublisher topic, it reads x and y translation, and z rotation, for 
    the robot in Gazebo, which means that these values are considered Ground
    Truth for translation and rotation errors '''
    global bGTReadingSet
    if bGTReadingSet == False and data.header.frame_id == robotModelName:
        lGTPosition[0] = data.pose.position.x
        lGTPosition[1] = data.pose.position.y
        lGTPosition[2] = data.pose.orientation.z
        bGTReadingSet  = True

rospy.init_node('robotAPEPublisher')  

################################################################################
############################ -- ASSIGNING PARAMS -- ############################
################################################################################

robotModelName           = ''
APETranslationTopicName  = 'APETranslation'
APERotationTopicName     = 'APERotation'
GTSubscribedTopicName    = 'robotGTPosePublisher'
APETopicName             = 'APE'

if rospy.has_param("~robot_name"):
    robotModelName  = rospy.get_param("~robot_name")
if rospy.has_param("~APE_Translation_Topic_Name"):
    APETranslationTopicName  = rospy.get_param("~APE_Translation_Topic_Name")
if rospy.has_param("~APE_Rotation_Topic_Name"):
    APERotationTopicName  = rospy.get_param("~APE_Rotation_Topic_Name")
if rospy.has_param("~APE_Topic_Name"):
    APETopicName  = rospy.get_param("~APE_Topic_Name")
if rospy.has_param("~GT_Subscribed_Topic_Name"):
    GTSubscribedTopicName  = rospy.get_param("~GT_Subscribed_Topic_Name")

################################################################################
############################ -- LAUNCHING TOPICS -- ############################
################################################################################

# -- Creates topics to publish APE for translation and rotation
robotAPEPublisher = rospy.Publisher(APETopicName, APE, queue_size=10)

# -- Subscribing to the ground truth pose publisher
robotGTPoseListener = rospy.Subscriber(GTSubscribedTopicName, PoseStamped, GTReader)

# -- Subscribing to tf to get the estimated translation and rotation
robotTFPoseListener = rospy.Subscriber('/tf', TFMessage, TFReader)              # Slash is needed to get topics out of my namespace

message = APE()
message.frame_id = robotModelName

while not rospy.is_shutdown():
    if bTFReadingSet == True and bGTReadingSet == True:
        message.datetime = datetime.now().strftime("%Y_%m_%d-%I:%M:%S_%p")
        
        # -- Computing and publishing APE mean for translation
        iXDiff = lGTPosition[0] - lTFPosition[6]
        iYDiff = lGTPosition[1] - lTFPosition[7]
        APETranslation += math.sqrt((iXDiff)**2 + (iYDiff)**2)
        message.translation_error_mean = APETranslation/iSamplesCounter
        # -- Computing and publishing APE mean for rotation
        APERotation += abs(lGTPosition[2] - lTFPosition[8])
        message.rotation_error_mean = APERotation/iSamplesCounter
        
        robotAPEPublisher.publish(message)
        
        # -- Updating variables
        bTFReadingSet = False
        bGTReadingSet = False
        iSamplesCounter += 1