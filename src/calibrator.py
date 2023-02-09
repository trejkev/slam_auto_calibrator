#! /usr/bin/env python

__author__ = "Kevin Trejos Vargas"
__email__  = "kevin.trejosvargas@ucr.ac.cr"

# -- Imports -- #
import rospy, subprocess, time, os                                              
from datetime                 import datetime
from slam_auto_calibrator.msg import APE
from hyperopt import tpe, hp, fmin
from hyperopt.pyll import scope
import numpy as np


class Calibrator(object):

    def __init__(self):
        self.readLaunchParams()
        self.iActualCycle     = 0
        self.dParams          = {}
        self.space            = {}
        self.APETopicReadings = list(range(2*self.iRobotsQty))
        self.APETopics        = list(range(self.iRobotsQty))
        self.GTMapPath        = self.GTpath + self.GTName
        self.nodeInitialization()
        self.getParamsFromYaml()
        self.setSearchSpace()
        
    def readLaunchParams(self):
        # -- Defines the quantity of iterations for training the algorithm
        if rospy.has_param("/TrainingCycles"):
            self.iTrainingCycles = rospy.get_param("/TrainingCycles")
        # Defines the SLAM method under test
        if rospy.has_param("/SLAMName"):
            self.sSLAMName = rospy.get_param("/SLAMName")
        # -- Defines the robots quantity
        if rospy.has_param("/RobotsQty"):
            self.iRobotsQty = rospy.get_param("/RobotsQty")
        # -- Robots launching variables
        if rospy.has_param("/RobotsLauchName"):
            self.sRobotsLauchName = rospy.get_param("/RobotsLauchName")
        # -- SLAM launching variables: Contains SLAM, map merger, and rviz stuff
        if rospy.has_param("/SLAMPackageName"):
            self.sSLAMPackageName = rospy.get_param("/SLAMPackageName")
        if rospy.has_param("/SLAMLaunchName"):
            self.sSLAMLaunchName    = rospy.get_param("/SLAMLaunchName")
        # -- APE topic publishing translation and rotation APE mean
        if rospy.has_param("/APE_Topic_Name"):
            self.APETopicName = rospy.get_param("/APE_Topic_Name")
        # -- Robots namespace base naming convention
        if rospy.has_param("/Robots_Namespace_Base"):
            self.RobotsNamespaceBase = rospy.get_param("/Robots_Namespace_Base")
        # -- Ground truth map file name
        if rospy.has_param("/Ground_Truth_Filename"):
            self.GTName = rospy.get_param("/Ground_Truth_Filename")
        # -- This node src path
        if rospy.has_param("/This_Node_src_Path"):
            self.sSourcePath = rospy.get_param("/This_Node_src_Path")
        # -- Ground truth map path
        if rospy.has_param("/Maps_Path"):
            self.GTpath = rospy.get_param("/Maps_Path")
        # -- Parameters file path
        if rospy.has_param("/Params_File_Path"):
            self.sParamsFilePath = rospy.get_param("/Params_File_Path")

    def nodeInitialization(self):
        self.AllNodesKiller()                                                   # Making sure we have a fresh start without unwanted nodes
        rospy.init_node('slam_auto_calibrator')
        # -- Open the arena with three robots
        subprocess.Popen("roslaunch slam_auto_calibrator {}".format(self.sRobotsLauchName), shell = True)
        time.sleep(60)
        
    def getParamsFromYaml(self):
        fParamsFile = open(self.sParamsFilePath, 'r')
        for line in fParamsFile:
            # If the line contains a parameter
            if line != "" and line != "\n":
                param   = line.split(":")[0]
                typeVar = line.split("#")[1].replace(" ", "")
                if typeVar == "int":
                    value  = int(line.split(":")[1].replace(" ", "").split("#")[0])
                    minVal = int(line.split("#")[2].replace(" ", "").replace("min=", "").replace("\n", ""))
                    maxVal = int(line.split("#")[3].replace(" ", "").replace("max=", "").replace("\n", ""))
                elif typeVar == "float":
                    value  = float(line.split(":")[1].replace(" ", "").split("#")[0])
                    minVal = float(line.split("#")[2].replace(" ", "").replace("min=", "").replace("\n", ""))
                    maxVal = float(line.split("#")[3].replace(" ", "").replace("max=", "").replace("\n", ""))
                else:
                    value  = line.split(":")[1].replace(" ", "").split("#")[0]
                    minVal = line.split("#")[2].replace(" ", "").replace("min=", "").replace("\n", "")
                    maxVal = line.split("#")[3].replace(" ", "").replace("max=", "").replace("\n", "")
                self.dParams.update({param: [value, typeVar, minVal, maxVal]})
        fParamsFile.close()
    
    def setParamsOnYaml(self):
        fParamsFile = open(self.sParamsFilePath, 'r+')
        fParamsFile.truncate(0)                                                 # Removing all contents
        for param in list(self.dParams.keys()):
            value   = self.dParams[param][0]
            typeVar = self.dParams[param][1]
            minVal  = self.dParams[param][2]
            maxVal  = self.dParams[param][3]
            fParamsFile.write("{}: {} #{} #min={} #max={}\n".format(param, value, typeVar, minVal, maxVal))
        fParamsFile.close()
        
    def setSearchSpace(self):
        for sParamName in list(self.dParams.keys()):
            if self.dParams[sParamName][1].lower() == "int":
                self.space[sParamName] = hp.choice(sParamName, list(range(self.dParams[sParamName][2], self.dParams[sParamName][3] + 1)))
            elif self.dParams[sParamName][1].lower() == "float":
                self.space[sParamName] = hp.uniform(sParamName, self.dParams[sParamName][2], self.dParams[sParamName][3])
            elif self.dParams[sParamName][1].lower() == "bool":
                self.space[sParamName] = hp.choice(sParamName, [self.dParams[sParamName][2], self.dParams[sParamName][3]])

    def APEReader(self, data):
        if data.frame_id == self.RobotsNamespaceBase + "0":
            self.APETopicReadings[0] = data.translation_error_mean
            self.APETopicReadings[0 + self.iRobotsQty] = data.rotation_error_mean
        elif data.frame_id == self.RobotsNamespaceBase + "1":
            self.APETopicReadings[1] = data.translation_error_mean
            self.APETopicReadings[1 + self.iRobotsQty] = data.rotation_error_mean
        elif data.frame_id == self.RobotsNamespaceBase + "2":
            self.APETopicReadings[2] = data.translation_error_mean
            self.APETopicReadings[2 + self.iRobotsQty] = data.rotation_error_mean
        
    def AllNodesKiller(self):
        # -- Killing all gazebo processes that may be open
        process = subprocess.Popen("killall -9 gazebo & killall -9 gzserver & killall -9 gzclient", shell = True)
        process.wait()
        # -- Kill all nodes for a clean start
        nodes = os.popen("rosnode list").readlines()
        for index in range(len(nodes)):
            nodes[index] = nodes[index].replace("\n", "")
        for node in nodes:
            if 'rosout' not in node and 'slam_auto_calibrator' not in node:
                os.system("rosnode kill " + node)
        time.sleep(10)
    
    def NonGazeboNodesKiller(self):
        # -- Kill the SLAM algorithms and their related nodes
        nodes = os.popen("rosnode list").readlines()
        for index in range(len(nodes)):
            nodes[index] = nodes[index].replace("\n", "")
        for node in nodes:
            if 'rviz'                in node \
                or 'map_merge'       in node \
                or 'map_saver'       in node \
                or 'turtlebot3_slam' in node \
                or 'APE'             in node:
                os.system("rosnode kill " + node)
                rospy.loginfo("Cycle {} completed: rosnode kill {}".format(self.iActualCycle, node))
            
    def ErrorsRecorder(self):
        rospy.loginfo("ME_C{}: {}".format(self.iActualCycle, self.fActualMapError))
        for robot in range(self.iRobotsQty):
            rospy.loginfo("Rob{}_TE_C{}: {}".format(robot, self.iActualCycle, self.APETopicReadings[robot]))
            rospy.loginfo("Rob{}_RE_C{}: {}".format(robot, self.iActualCycle, self.APETopicReadings[robot + self.iRobotsQty]))
    
    def MapMetricComputation(self):
        self.sSLAMMapPath = "{}{}.pgm".format(self.GTpath, self.MapName)
        # --- Sending ground truth and slam maps paths to the error calculator
        with open("{}MapMetricVariables.txt".format(self.sSourcePath), "w") as mmv:
            mmv.write("GTMapPath={}\n".format(self.GTMapPath))
            mmv.write("SLAMMapPath={}\n".format(self.sSLAMMapPath))
            mmv.close()
        # --- Running the error calculator
        self.sMapMetricFile = "{}MapAccuracy.py".format(self.sSourcePath)
        process = subprocess.Popen("'{}'".format(self.sMapMetricFile), shell = True)
        process.wait()
        # --- Reading the error
        try:
            with open("{}MapMetricVariables.txt".format(self.sSourcePath), "r") as mmv:
                for line in mmv.readlines():
                    self.fActualMapError = float(line.split("=")[1])
            return self.fActualMapError
        except:
            return "NA"                                                         # SLAM map file too large

    def CycleCompletionWatchdog(self):
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
            time.sleep(5)
        
    def SLAMMapGenerator(self):
        date = datetime.now().strftime("%Y_%m_%d-%I:%M:%S_%p").replace(":","_")
        self.MapName = "{}_Trial_{}_RobotsQty_{}_Map_{}".format(self.sSLAMName, self.iActualCycle, self.iRobotsQty, date)
        sPath = "{}{}".format(self.GTpath, self.MapName)
        process = subprocess.Popen("rosrun map_server map_saver -f {}".format(sPath), shell = True)
        process.wait()
        return self.MapName

    def CycleRunner(self):
        # -- Launch the SLAM algorithms and the automatic navigator
        subprocess.Popen("roslaunch {} {}".format(self.sSLAMPackageName, self.sSLAMLaunchName), shell = True)
        time.sleep(10)
        for iRobot in range(self.iRobotsQty):
            sTopic = "/{}{}/{}".format(self.RobotsNamespaceBase, str(iRobot), self.APETopicName)
            self.APETopics[iRobot] = rospy.Subscriber(sTopic, APE, self.APEReader)
            rospy.loginfo("Subscribed to {}".format(sTopic))
            
        self.CycleCompletionWatchdog()                                          # Wait for the robots to complete their travel
        self.SLAMMapGenerator()                                                 # Generate the SLAM map as pgm image
        self.MapMetricComputation()                                             # Call an external script to compute the map metric
        self.ErrorsRecorder()                                                   # Record the errors into log files
        self.NonGazeboNodesKiller()
        self.iActualCycle += 1
        # return self.fActualMapError, self.APETopicReadings
        return self.fActualMapError
    
    def TargetFunction(self, args):
        for param in list(self.dParams.keys()):
            self.dParams.update({param: [args[param], self.dParams[param][1], self.dParams[param][2], self.dParams[param][3]]})
        rospy.loginfo(self.dParams)
        self.setParamsOnYaml()
        return self.CycleRunner()
    
    def paramsOptimizer(self):
        best = fmin(self.TargetFunction, self.space, algo = tpe.suggest, max_evals = self.iTrainingCycles)
        rospy.loginfo("---- BEST SETUP IS ----")
        rospy.loginfo(best)
        rospy.loginfo("---- BEST SETUP IS ----")
    
    
    
    ############################################################################
    ############################################################################
    # --        Validation methods, not part of the functional code         -- #
    ############################################################################
    ############################################################################
    
    def validationTrials(self, iTrialsQty = 30):
        # -- Run optimized params
        rospy.loginfo("OPTIMIZED PARAMS RUNS")
        self.dParams.update({'lasamplestep': [0.006365372021281653, self.dParams['lasamplestep'][1], self.dParams['lasamplestep'][2], self.dParams['lasamplestep'][3]]})
        self.dParams.update({'lasamplerange': [0.0036290800590042358, self.dParams['lasamplerange'][1], self.dParams['lasamplerange'][2], self.dParams['lasamplerange'][3]]})
        self.dParams.update({'llsamplestep': [0.010989209780321061, self.dParams['llsamplestep'][1], self.dParams['llsamplestep'][2], self.dParams['llsamplestep'][3]]})
        self.dParams.update({'lstep': [0.06169138956104462, self.dParams['lstep'][1], self.dParams['lstep'][2], self.dParams['lstep'][3]]})
        self.dParams.update({'llsamplerange': [0.007014004148778169, self.dParams['llsamplerange'][1], self.dParams['llsamplerange'][2], self.dParams['llsamplerange'][3]]})
        self.dParams.update({'map_update_interval': [7.298784369818808, self.dParams['map_update_interval'][1], self.dParams['map_update_interval'][2], self.dParams['map_update_interval'][3]]})
        self.dParams.update({'lsigma': [0.0901361970689364, self.dParams['lsigma'][1], self.dParams['lsigma'][2], self.dParams['lsigma'][3]]})
        self.dParams.update({'astep': [0.03022028311165237, self.dParams['astep'][1], self.dParams['astep'][2], self.dParams['astep'][3]]})
        self.dParams.update({'particles': [18, self.dParams['particles'][1], self.dParams['particles'][2], self.dParams['particles'][3]]})
        self.dParams.update({'throttle_scans': [4, self.dParams['throttle_scans'][1], self.dParams['throttle_scans'][2], self.dParams['throttle_scans'][3]]}) 
        self.dParams.update({'angularUpdate': [0.6423697727104153, self.dParams['angularUpdate'][1], self.dParams['angularUpdate'][2], self.dParams['angularUpdate'][3]]})
        self.dParams.update({'maxUrange': [71.08889795230735, self.dParams['maxUrange'][1], self.dParams['maxUrange'][2], self.dParams['maxUrange'][3]]})
        self.dParams.update({'linearUpdate': [0.8538667411625122, self.dParams['linearUpdate'][1], self.dParams['linearUpdate'][2], self.dParams['linearUpdate'][3]]})
        self.dParams.update({'iterations': [2, self.dParams['iterations'][1], self.dParams['iterations'][2], self.dParams['iterations'][3]]})
        self.dParams.update({'ogain': [3.6455524873631235, self.dParams['ogain'][1], self.dParams['ogain'][2], self.dParams['ogain'][3]]})
        self.dParams.update({'delta': [0.030041051212017597, self.dParams['delta'][1], self.dParams['delta'][2], self.dParams['delta'][3]]})
        self.dParams.update({'resampleThreshold': [0.5626372407098409, self.dParams['resampleThreshold'][1], self.dParams['resampleThreshold'][2], self.dParams['resampleThreshold'][3]]})
        self.dParams.update({'sigma': [0.06773553594065067, self.dParams['sigma'][1], self.dParams['sigma'][2], self.dParams['sigma'][3]]})
        self.dParams.update({'minimumScore': [3.827622107765204, self.dParams['minimumScore'][1], self.dParams['minimumScore'][2], self.dParams['minimumScore'][3]]})
        rospy.loginfo(self.dParams)
        self.setParamsOnYaml()
        for trial in range(iTrialsQty):
            self.CycleRunner()
        # -- Run default params
        rospy.loginfo("DEFAULT PARAMS RUNS")
        self.dParams.update({'lasamplestep': [0.005, self.dParams['lasamplestep'][1], self.dParams['lasamplestep'][2], self.dParams['lasamplestep'][3]]})
        self.dParams.update({'lasamplerange': [0.005, self.dParams['lasamplerange'][1], self.dParams['lasamplerange'][2], self.dParams['lasamplerange'][3]]})
        self.dParams.update({'llsamplestep': [0.01, self.dParams['llsamplestep'][1], self.dParams['llsamplestep'][2], self.dParams['llsamplestep'][3]]})
        self.dParams.update({'lstep': [0.05, self.dParams['lstep'][1], self.dParams['lstep'][2], self.dParams['lstep'][3]]})
        self.dParams.update({'llsamplerange': [0.01, self.dParams['llsamplerange'][1], self.dParams['llsamplerange'][2], self.dParams['llsamplerange'][3]]})
        self.dParams.update({'map_update_interval': [5.0, self.dParams['map_update_interval'][1], self.dParams['map_update_interval'][2], self.dParams['map_update_interval'][3]]})
        self.dParams.update({'lsigma': [0.075, self.dParams['lsigma'][1], self.dParams['lsigma'][2], self.dParams['lsigma'][3]]})
        self.dParams.update({'astep': [0.05, self.dParams['astep'][1], self.dParams['astep'][2], self.dParams['astep'][3]]})
        self.dParams.update({'particles': [30, self.dParams['particles'][1], self.dParams['particles'][2], self.dParams['particles'][3]]})
        self.dParams.update({'throttle_scans': [1, self.dParams['throttle_scans'][1], self.dParams['throttle_scans'][2], self.dParams['throttle_scans'][3]]}) 
        self.dParams.update({'angularUpdate': [0.5, self.dParams['angularUpdate'][1], self.dParams['angularUpdate'][2], self.dParams['angularUpdate'][3]]})
        self.dParams.update({'maxUrange': [80.0, self.dParams['maxUrange'][1], self.dParams['maxUrange'][2], self.dParams['maxUrange'][3]]})
        self.dParams.update({'linearUpdate': [1.0, self.dParams['linearUpdate'][1], self.dParams['linearUpdate'][2], self.dParams['linearUpdate'][3]]})
        self.dParams.update({'iterations': [5, self.dParams['iterations'][1], self.dParams['iterations'][2], self.dParams['iterations'][3]]})
        self.dParams.update({'ogain': [3.0, self.dParams['ogain'][1], self.dParams['ogain'][2], self.dParams['ogain'][3]]})
        self.dParams.update({'delta': [0.05, self.dParams['delta'][1], self.dParams['delta'][2], self.dParams['delta'][3]]})
        self.dParams.update({'resampleThreshold': [0.5, self.dParams['resampleThreshold'][1], self.dParams['resampleThreshold'][2], self.dParams['resampleThreshold'][3]]})
        self.dParams.update({'sigma': [0.05, self.dParams['sigma'][1], self.dParams['sigma'][2], self.dParams['sigma'][3]]})
        self.dParams.update({'minimumScore': [0.0, self.dParams['minimumScore'][1], self.dParams['minimumScore'][2], self.dParams['minimumScore'][3]]})
        rospy.loginfo(self.dParams)
        self.setParamsOnYaml()
        for trial in range(iTrialsQty):
            self.CycleRunner()
    
    def OptReviewTrialsVer2(self, iTrialsQty = 16):
        rospy.loginfo("200 TIMES PARAMS RUNS")
        self.dParams.update({'lasamplestep': [0.003007914478503184, self.dParams['lasamplestep'][1], self.dParams['lasamplestep'][2], self.dParams['lasamplestep'][3]]})
        self.dParams.update({'lasamplerange': [0.0038359543315193536, self.dParams['lasamplerange'][1], self.dParams['lasamplerange'][2], self.dParams['lasamplerange'][3]]})
        self.dParams.update({'llsamplestep': [0.013637398940780539, self.dParams['llsamplestep'][1], self.dParams['llsamplestep'][2], self.dParams['llsamplestep'][3]]})
        self.dParams.update({'lstep': [0.06965953136587595, self.dParams['lstep'][1], self.dParams['lstep'][2], self.dParams['lstep'][3]]})
        self.dParams.update({'llsamplerange': [0.0052006405330092065, self.dParams['llsamplerange'][1], self.dParams['llsamplerange'][2], self.dParams['llsamplerange'][3]]})
        self.dParams.update({'map_update_interval': [4.553564036658955, self.dParams['map_update_interval'][1], self.dParams['map_update_interval'][2], self.dParams['map_update_interval'][3]]})
        self.dParams.update({'lsigma': [0.07979527539202991, self.dParams['lsigma'][1], self.dParams['lsigma'][2], self.dParams['lsigma'][3]]})
        self.dParams.update({'astep': [0.03295152841339962, self.dParams['astep'][1], self.dParams['astep'][2], self.dParams['astep'][3]]})
        self.dParams.update({'particles': [16, self.dParams['particles'][1], self.dParams['particles'][2], self.dParams['particles'][3]]})
        self.dParams.update({'throttle_scans': [4, self.dParams['throttle_scans'][1], self.dParams['throttle_scans'][2], self.dParams['throttle_scans'][3]]}) 
        self.dParams.update({'angularUpdate': [0.6252325042386583, self.dParams['angularUpdate'][1], self.dParams['angularUpdate'][2], self.dParams['angularUpdate'][3]]})
        self.dParams.update({'maxUrange': [73.72032793849412, self.dParams['maxUrange'][1], self.dParams['maxUrange'][2], self.dParams['maxUrange'][3]]})
        self.dParams.update({'linearUpdate': [0.9440511343184271, self.dParams['linearUpdate'][1], self.dParams['linearUpdate'][2], self.dParams['linearUpdate'][3]]})
        self.dParams.update({'iterations': [3, self.dParams['iterations'][1], self.dParams['iterations'][2], self.dParams['iterations'][3]]})
        self.dParams.update({'ogain': [2.089006392255016, self.dParams['ogain'][1], self.dParams['ogain'][2], self.dParams['ogain'][3]]})
        self.dParams.update({'delta': [0.030090086346347418, self.dParams['delta'][1], self.dParams['delta'][2], self.dParams['delta'][3]]})
        self.dParams.update({'resampleThreshold': [0.5491136438232976, self.dParams['resampleThreshold'][1], self.dParams['resampleThreshold'][2], self.dParams['resampleThreshold'][3]]})
        self.dParams.update({'sigma': [0.058313499281896795, self.dParams['sigma'][1], self.dParams['sigma'][2], self.dParams['sigma'][3]]})
        self.dParams.update({'minimumScore': [1.5826387504423072, self.dParams['minimumScore'][1], self.dParams['minimumScore'][2], self.dParams['minimumScore'][3]]})
        rospy.loginfo(self.dParams)
        self.setParamsOnYaml()
        for trial in range(iTrialsQty):
            self.CycleRunner()
        rospy.loginfo("100 TIMES PARAMS RUNS")
        self.dParams.update({'lasamplestep': [0.003632033648581685, self.dParams['lasamplestep'][1], self.dParams['lasamplestep'][2], self.dParams['lasamplestep'][3]]})
        self.dParams.update({'lasamplerange': [0.005872984205421812, self.dParams['lasamplerange'][1], self.dParams['lasamplerange'][2], self.dParams['lasamplerange'][3]]})
        self.dParams.update({'llsamplestep': [0.007359893813202421, self.dParams['llsamplestep'][1], self.dParams['llsamplestep'][2], self.dParams['llsamplestep'][3]]})
        self.dParams.update({'lstep': [0.05226470938753834, self.dParams['lstep'][1], self.dParams['lstep'][2], self.dParams['lstep'][3]]})
        self.dParams.update({'llsamplerange': [0.009401480413120054, self.dParams['llsamplerange'][1], self.dParams['llsamplerange'][2], self.dParams['llsamplerange'][3]]})
        self.dParams.update({'map_update_interval': [1.869112578756949, self.dParams['map_update_interval'][1], self.dParams['map_update_interval'][2], self.dParams['map_update_interval'][3]]})
        self.dParams.update({'lsigma': [0.08156101216303907, self.dParams['lsigma'][1], self.dParams['lsigma'][2], self.dParams['lsigma'][3]]})
        self.dParams.update({'astep': [0.03921716395615649, self.dParams['astep'][1], self.dParams['astep'][2], self.dParams['astep'][3]]})
        self.dParams.update({'particles': [4, self.dParams['particles'][1], self.dParams['particles'][2], self.dParams['particles'][3]]})
        self.dParams.update({'throttle_scans': [1, self.dParams['throttle_scans'][1], self.dParams['throttle_scans'][2], self.dParams['throttle_scans'][3]]}) 
        self.dParams.update({'angularUpdate': [0.5391493636343758, self.dParams['angularUpdate'][1], self.dParams['angularUpdate'][2], self.dParams['angularUpdate'][3]]})
        self.dParams.update({'maxUrange': [76.51319240518663, self.dParams['maxUrange'][1], self.dParams['maxUrange'][2], self.dParams['maxUrange'][3]]})
        self.dParams.update({'linearUpdate': [1.495146418510496, self.dParams['linearUpdate'][1], self.dParams['linearUpdate'][2], self.dParams['linearUpdate'][3]]})
        self.dParams.update({'iterations': [2, self.dParams['iterations'][1], self.dParams['iterations'][2], self.dParams['iterations'][3]]})
        self.dParams.update({'ogain': [3.436491254492076, self.dParams['ogain'][1], self.dParams['ogain'][2], self.dParams['ogain'][3]]})
        self.dParams.update({'delta': [0.03000843009132362, self.dParams['delta'][1], self.dParams['delta'][2], self.dParams['delta'][3]]})
        self.dParams.update({'resampleThreshold': [0.5889282445232121, self.dParams['resampleThreshold'][1], self.dParams['resampleThreshold'][2], self.dParams['resampleThreshold'][3]]})
        self.dParams.update({'sigma': [0.06874326008678544, self.dParams['sigma'][1], self.dParams['sigma'][2], self.dParams['sigma'][3]]})
        self.dParams.update({'minimumScore': [3.283669648172889, self.dParams['minimumScore'][1], self.dParams['minimumScore'][2], self.dParams['minimumScore'][3]]})
        rospy.loginfo(self.dParams)
        self.setParamsOnYaml()
        for trial in range(iTrialsQty):
            self.CycleRunner()
        rospy.loginfo("10 TIMES PARAMS RUNS")
        self.dParams.update({'lasamplestep': [0.005970996960046699, self.dParams['lasamplestep'][1], self.dParams['lasamplestep'][2], self.dParams['lasamplestep'][3]]})
        self.dParams.update({'lasamplerange': [0.0040029166972444, self.dParams['lasamplerange'][1], self.dParams['lasamplerange'][2], self.dParams['lasamplerange'][3]]})
        self.dParams.update({'llsamplestep': [0.04387652788786439, self.dParams['llsamplestep'][1], self.dParams['llsamplestep'][2], self.dParams['llsamplestep'][3]]})
        self.dParams.update({'lstep': [0.05556745260775601, self.dParams['lstep'][1], self.dParams['lstep'][2], self.dParams['lstep'][3]]})
        self.dParams.update({'llsamplerange': [0.03303935015706494, self.dParams['llsamplerange'][1], self.dParams['llsamplerange'][2], self.dParams['llsamplerange'][3]]})
        self.dParams.update({'map_update_interval': [2.8588357060331093, self.dParams['map_update_interval'][1], self.dParams['map_update_interval'][2], self.dParams['map_update_interval'][3]]})
        self.dParams.update({'lsigma': [0.05980455258486999, self.dParams['lsigma'][1], self.dParams['lsigma'][2], self.dParams['lsigma'][3]]})
        self.dParams.update({'astep': [0.03457222951002627, self.dParams['astep'][1], self.dParams['astep'][2], self.dParams['astep'][3]]})
        self.dParams.update({'particles': [21, self.dParams['particles'][1], self.dParams['particles'][2], self.dParams['particles'][3]]})
        self.dParams.update({'throttle_scans': [1, self.dParams['throttle_scans'][1], self.dParams['throttle_scans'][2], self.dParams['throttle_scans'][3]]}) 
        self.dParams.update({'angularUpdate': [0.3420335148163475, self.dParams['angularUpdate'][1], self.dParams['angularUpdate'][2], self.dParams['angularUpdate'][3]]})
        self.dParams.update({'maxUrange': [87.28132830734856, self.dParams['maxUrange'][1], self.dParams['maxUrange'][2], self.dParams['maxUrange'][3]]})
        self.dParams.update({'linearUpdate': [0.9099934746437636, self.dParams['linearUpdate'][1], self.dParams['linearUpdate'][2], self.dParams['linearUpdate'][3]]})
        self.dParams.update({'iterations': [6, self.dParams['iterations'][1], self.dParams['iterations'][2], self.dParams['iterations'][3]]})
        self.dParams.update({'ogain': [1.1128626504367944, self.dParams['ogain'][1], self.dParams['ogain'][2], self.dParams['ogain'][3]]})
        self.dParams.update({'delta': [0.031450137974610444, self.dParams['delta'][1], self.dParams['delta'][2], self.dParams['delta'][3]]})
        self.dParams.update({'resampleThreshold': [0.5735065430677528, self.dParams['resampleThreshold'][1], self.dParams['resampleThreshold'][2], self.dParams['resampleThreshold'][3]]})
        self.dParams.update({'sigma': [0.03462754633865353, self.dParams['sigma'][1], self.dParams['sigma'][2], self.dParams['sigma'][3]]})
        self.dParams.update({'minimumScore': [2.332935277784088, self.dParams['minimumScore'][1], self.dParams['minimumScore'][2], self.dParams['minimumScore'][3]]})
        rospy.loginfo(self.dParams)
        self.setParamsOnYaml()
        for trial in range(iTrialsQty):
            self.CycleRunner()
    
    def OptComparisonTrials(self, iTrialsQty = 16):
        rospy.loginfo("200 TIMES PARAMS RUNS")
        self.dParams.update({'lasamplestep': [0.00495233677585376, self.dParams['lasamplestep'][1], self.dParams['lasamplestep'][2], self.dParams['lasamplestep'][3]]})
        self.dParams.update({'lasamplerange': [0.0054377921630375005, self.dParams['lasamplerange'][1], self.dParams['lasamplerange'][2], self.dParams['lasamplerange'][3]]})
        self.dParams.update({'llsamplestep': [0.005290734732866759, self.dParams['llsamplestep'][1], self.dParams['llsamplestep'][2], self.dParams['llsamplestep'][3]]})
        self.dParams.update({'lstep': [0.04227198559224202, self.dParams['lstep'][1], self.dParams['lstep'][2], self.dParams['lstep'][3]]})
        self.dParams.update({'llsamplerange': [0.006221148233139982, self.dParams['llsamplerange'][1], self.dParams['llsamplerange'][2], self.dParams['llsamplerange'][3]]})
        self.dParams.update({'map_update_interval': [1.813598711221583, self.dParams['map_update_interval'][1], self.dParams['map_update_interval'][2], self.dParams['map_update_interval'][3]]})
        self.dParams.update({'lsigma': [0.0886672294507992, self.dParams['lsigma'][1], self.dParams['lsigma'][2], self.dParams['lsigma'][3]]})
        self.dParams.update({'astep': [0.05657238420955355, self.dParams['astep'][1], self.dParams['astep'][2], self.dParams['astep'][3]]})
        self.dParams.update({'particles': [9, self.dParams['particles'][1], self.dParams['particles'][2], self.dParams['particles'][3]]})
        self.dParams.update({'throttle_scans': [2, self.dParams['throttle_scans'][1], self.dParams['throttle_scans'][2], self.dParams['throttle_scans'][3]]}) 
        self.dParams.update({'angularUpdate': [0.48701375174371564, self.dParams['angularUpdate'][1], self.dParams['angularUpdate'][2], self.dParams['angularUpdate'][3]]})
        self.dParams.update({'maxUrange': [81.77346395724913, self.dParams['maxUrange'][1], self.dParams['maxUrange'][2], self.dParams['maxUrange'][3]]})
        self.dParams.update({'linearUpdate': [0.9449037073959806, self.dParams['linearUpdate'][1], self.dParams['linearUpdate'][2], self.dParams['linearUpdate'][3]]})
        self.dParams.update({'iterations': [0, self.dParams['iterations'][1], self.dParams['iterations'][2], self.dParams['iterations'][3]]})
        self.dParams.update({'ogain': [2.1241121539269807, self.dParams['ogain'][1], self.dParams['ogain'][2], self.dParams['ogain'][3]]})
        self.dParams.update({'delta': [0.032043882708133346, self.dParams['delta'][1], self.dParams['delta'][2], self.dParams['delta'][3]]})
        self.dParams.update({'resampleThreshold': [0.5055213731286743, self.dParams['resampleThreshold'][1], self.dParams['resampleThreshold'][2], self.dParams['resampleThreshold'][3]]})
        self.dParams.update({'sigma': [0.04847341281267477, self.dParams['sigma'][1], self.dParams['sigma'][2], self.dParams['sigma'][3]]})
        self.dParams.update({'minimumScore': [0.035624478932486214, self.dParams['minimumScore'][1], self.dParams['minimumScore'][2], self.dParams['minimumScore'][3]]})
        rospy.loginfo(self.dParams)
        self.setParamsOnYaml()
        for trial in range(iTrialsQty):
            self.CycleRunner()
        rospy.loginfo("100 TIMES PARAMS RUNS")
        self.dParams.update({'lasamplestep': [0.003632033648581685, self.dParams['lasamplestep'][1], self.dParams['lasamplestep'][2], self.dParams['lasamplestep'][3]]})
        self.dParams.update({'lasamplerange': [0.005872984205421812, self.dParams['lasamplerange'][1], self.dParams['lasamplerange'][2], self.dParams['lasamplerange'][3]]})
        self.dParams.update({'llsamplestep': [0.007359893813202421, self.dParams['llsamplestep'][1], self.dParams['llsamplestep'][2], self.dParams['llsamplestep'][3]]})
        self.dParams.update({'lstep': [0.05226470938753834, self.dParams['lstep'][1], self.dParams['lstep'][2], self.dParams['lstep'][3]]})
        self.dParams.update({'llsamplerange': [0.009401480413120054, self.dParams['llsamplerange'][1], self.dParams['llsamplerange'][2], self.dParams['llsamplerange'][3]]})
        self.dParams.update({'map_update_interval': [1.869112578756949, self.dParams['map_update_interval'][1], self.dParams['map_update_interval'][2], self.dParams['map_update_interval'][3]]})
        self.dParams.update({'lsigma': [0.08156101216303907, self.dParams['lsigma'][1], self.dParams['lsigma'][2], self.dParams['lsigma'][3]]})
        self.dParams.update({'astep': [0.03921716395615649, self.dParams['astep'][1], self.dParams['astep'][2], self.dParams['astep'][3]]})
        self.dParams.update({'particles': [4, self.dParams['particles'][1], self.dParams['particles'][2], self.dParams['particles'][3]]})
        self.dParams.update({'throttle_scans': [1, self.dParams['throttle_scans'][1], self.dParams['throttle_scans'][2], self.dParams['throttle_scans'][3]]}) 
        self.dParams.update({'angularUpdate': [0.5391493636343758, self.dParams['angularUpdate'][1], self.dParams['angularUpdate'][2], self.dParams['angularUpdate'][3]]})
        self.dParams.update({'maxUrange': [76.51319240518663, self.dParams['maxUrange'][1], self.dParams['maxUrange'][2], self.dParams['maxUrange'][3]]})
        self.dParams.update({'linearUpdate': [1.495146418510496, self.dParams['linearUpdate'][1], self.dParams['linearUpdate'][2], self.dParams['linearUpdate'][3]]})
        self.dParams.update({'iterations': [2, self.dParams['iterations'][1], self.dParams['iterations'][2], self.dParams['iterations'][3]]})
        self.dParams.update({'ogain': [3.436491254492076, self.dParams['ogain'][1], self.dParams['ogain'][2], self.dParams['ogain'][3]]})
        self.dParams.update({'delta': [0.03000843009132362, self.dParams['delta'][1], self.dParams['delta'][2], self.dParams['delta'][3]]})
        self.dParams.update({'resampleThreshold': [0.5889282445232121, self.dParams['resampleThreshold'][1], self.dParams['resampleThreshold'][2], self.dParams['resampleThreshold'][3]]})
        self.dParams.update({'sigma': [0.06874326008678544, self.dParams['sigma'][1], self.dParams['sigma'][2], self.dParams['sigma'][3]]})
        self.dParams.update({'minimumScore': [3.283669648172889, self.dParams['minimumScore'][1], self.dParams['minimumScore'][2], self.dParams['minimumScore'][3]]})
        rospy.loginfo(self.dParams)
        self.setParamsOnYaml()
        for trial in range(iTrialsQty):
            self.CycleRunner()
        rospy.loginfo("10 TIMES PARAMS RUNS")
        self.dParams.update({'lasamplestep': [0.005970996960046699, self.dParams['lasamplestep'][1], self.dParams['lasamplestep'][2], self.dParams['lasamplestep'][3]]})
        self.dParams.update({'lasamplerange': [0.0040029166972444, self.dParams['lasamplerange'][1], self.dParams['lasamplerange'][2], self.dParams['lasamplerange'][3]]})
        self.dParams.update({'llsamplestep': [0.04387652788786439, self.dParams['llsamplestep'][1], self.dParams['llsamplestep'][2], self.dParams['llsamplestep'][3]]})
        self.dParams.update({'lstep': [0.05556745260775601, self.dParams['lstep'][1], self.dParams['lstep'][2], self.dParams['lstep'][3]]})
        self.dParams.update({'llsamplerange': [0.03303935015706494, self.dParams['llsamplerange'][1], self.dParams['llsamplerange'][2], self.dParams['llsamplerange'][3]]})
        self.dParams.update({'map_update_interval': [2.8588357060331093, self.dParams['map_update_interval'][1], self.dParams['map_update_interval'][2], self.dParams['map_update_interval'][3]]})
        self.dParams.update({'lsigma': [0.05980455258486999, self.dParams['lsigma'][1], self.dParams['lsigma'][2], self.dParams['lsigma'][3]]})
        self.dParams.update({'astep': [0.03457222951002627, self.dParams['astep'][1], self.dParams['astep'][2], self.dParams['astep'][3]]})
        self.dParams.update({'particles': [21, self.dParams['particles'][1], self.dParams['particles'][2], self.dParams['particles'][3]]})
        self.dParams.update({'throttle_scans': [1, self.dParams['throttle_scans'][1], self.dParams['throttle_scans'][2], self.dParams['throttle_scans'][3]]}) 
        self.dParams.update({'angularUpdate': [0.3420335148163475, self.dParams['angularUpdate'][1], self.dParams['angularUpdate'][2], self.dParams['angularUpdate'][3]]})
        self.dParams.update({'maxUrange': [87.28132830734856, self.dParams['maxUrange'][1], self.dParams['maxUrange'][2], self.dParams['maxUrange'][3]]})
        self.dParams.update({'linearUpdate': [0.9099934746437636, self.dParams['linearUpdate'][1], self.dParams['linearUpdate'][2], self.dParams['linearUpdate'][3]]})
        self.dParams.update({'iterations': [6, self.dParams['iterations'][1], self.dParams['iterations'][2], self.dParams['iterations'][3]]})
        self.dParams.update({'ogain': [1.1128626504367944, self.dParams['ogain'][1], self.dParams['ogain'][2], self.dParams['ogain'][3]]})
        self.dParams.update({'delta': [0.031450137974610444, self.dParams['delta'][1], self.dParams['delta'][2], self.dParams['delta'][3]]})
        self.dParams.update({'resampleThreshold': [0.5735065430677528, self.dParams['resampleThreshold'][1], self.dParams['resampleThreshold'][2], self.dParams['resampleThreshold'][3]]})
        self.dParams.update({'sigma': [0.03462754633865353, self.dParams['sigma'][1], self.dParams['sigma'][2], self.dParams['sigma'][3]]})
        self.dParams.update({'minimumScore': [2.332935277784088, self.dParams['minimumScore'][1], self.dParams['minimumScore'][2], self.dParams['minimumScore'][3]]})
        rospy.loginfo(self.dParams)
        self.setParamsOnYaml()
        for trial in range(iTrialsQty):
            self.CycleRunner()
        

calibrator = Calibrator()
calibrator.paramsOptimizer()
# calibrator.OptReviewTrialsVer2()
# calibrator.validationTrials()
# calibrator.OptComparisonTrials()

calibrator.AllNodesKiller()      
os.system("rosnode kill slam_auto_calibrator")
