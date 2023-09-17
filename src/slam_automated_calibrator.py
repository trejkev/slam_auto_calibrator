#!/usr/bin/env python

__author__ = "Kevin Trejos Vargas"
__email__  = "kevin.trejosvargas@ucr.ac.cr"

"""
MIT License

Copyright (c) 2022-2023 Kevin Trejos Vargas

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

# -- Imports -- #
import rospy, subprocess, time, os, numpy as np
from datetime                 import datetime
from slam_auto_calibrator.msg import APE
from hyperopt                 import tpe, hp, fmin
from hyperopt.pyll            import scope


class Calibrator(object):

    def __init__(self):
        self.iActualCycle = 0
        self.dParams      = {}
        self.dSpace       = {}

        self.readLaunchParams()

        self.APETopicReadings = list(range(2*self.iRobotsQty))
        self.APETopics        = list(range(self.iRobotsQty))
        self.GTMapPath        = self.GTpath + self.GTName

        self.nodeInitialization()
        self.getParamsFromYaml()
        self.setSearchSpace()


    def readLaunchParams(self):
        if rospy.has_param("/TrainingCycles"       ):
            self.iTrainingCycles     = rospy.get_param("/TrainingCycles"     )
        if rospy.has_param("/SLAMName"             ):
            self.sSLAMName           = rospy.get_param("/SLAMName"           )
        if rospy.has_param("/RobotsQty"            ):
            self.iRobotsQty          = rospy.get_param("/RobotsQty"          )
        if rospy.has_param("/RobotsLauchName"      ):
            self.sRobotsLauchName    = rospy.get_param("/RobotsLauchName"    )  # Launch file where robots and world description are
        if rospy.has_param("/SelfPackageName"      ):
            self.sSelfPackageName    = rospy.get_param("/SelfPackageName"    )  # Launch file where SLAM, map merger, and rviz are called
        if rospy.has_param("/SLAMLaunchName"       ):
            self.sSLAMLaunchName     = rospy.get_param("/SLAMLaunchName"     )
        if rospy.has_param("/APETopicName"         ):
            self.APETopicName        = rospy.get_param("/APETopicName"       )  # Publisher topic for traslation and rotation APE mean
        if rospy.has_param("/RobotsNamespaceBase"  ):
            self.RobotsNamespaceBase = rospy.get_param("/RobotsNamespaceBase")  # Prefix for robots
        if rospy.has_param("/GroundTruthFilename"  ):
            self.GTName              = rospy.get_param("/GroundTruthFilename")
        if rospy.has_param("/ThisNodeSrcPath"      ):
            self.sSourcePath         = rospy.get_param("/ThisNodeSrcPath"    )
        if rospy.has_param("/MapsPath"             ):
            self.GTpath              = rospy.get_param("/MapsPath"           )
        if rospy.has_param("/ParamsFilePath"       ):
            self.sParamsFilePath     = rospy.get_param("/ParamsFilePath"     )  # SLAM params file location


    def nodeInitialization(self):
        self.AllNodesKiller()                                                   # Making sure we have a fresh start without unwanted nodes
        rospy.init_node('slam_auto_calibrator')
        # -- Open the arena with three robots
        subprocess.Popen(
            "roslaunch slam_auto_calibrator {}"
            .format(self.sRobotsLauchName),
            shell = True
        )
        time.sleep(60)


    def getParamsFromYaml(self):
        fParamsFile = open(self.sParamsFilePath, 'r')
        for line in fParamsFile:
            # If the line contains a parameter
            if line != "" and line != "\n":
                param   = line.split(":")[0]
                typeVar = line.split("#")[1].replace(" ", "")
                value   = (
                    line.split(":")[1].replace(" ", "").split("#")[0]
                )
                minVal = (
                    line.split("#")[2]
                    .replace(" ", "")
                    .replace("min=", "")
                    .replace("\n", "")
                )
                maxVal = (
                    line.split("#")[3]
                    .replace(" ", "")
                    .replace("max=", "")
                    .replace("\n", "")
                )
                if typeVar == "int":
                    value  = int(value)
                    minVal = int(minVal)
                    maxVal = int(maxVal)
                elif typeVar == "float":
                    value  = float(value)
                    minVal = float(minVal)
                    maxVal = float(maxVal)
                else:
                    value  = value
                    minVal = minVal
                    maxVal = maxVal
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
            fParamsFile.write(
                "{}: {} #{} #min={} #max={}\n"
                .format(param, value, typeVar, minVal, maxVal)
            )
        fParamsFile.close()


    def setSearchSpace(self):
        for sParamName in list(self.dParams.keys()):
            iMin = self.dParams[sParamName][2]
            iMax = self.dParams[sParamName][3]
            if self.dParams[sParamName][1].lower() == "int":
                self.dSpace[sParamName] = hp.choice(
                    sParamName, list(range(iMin, iMax + 1))
                )
            elif self.dParams[sParamName][1].lower() == "float":
                self.dSpace[sParamName] = hp.uniform(sParamName, iMin, iMax)
            elif self.dParams[sParamName][1].lower() == "bool":
                self.dSpace[sParamName] = hp.choice(sParamName, [iMin, iMax])
            else:
                rospy.logerr(
                    "Parameter type {} not supported"
                    .format(self.dParams[sParamName][1].lower())
                )


    def APEReader(self, data):
        if data.frame_id == self.RobotsNamespaceBase + "0":
            self.APETopicReadings[0] = data.translation_error_mean
            self.APETopicReadings[0 + self.iRobotsQty] = (
                data.rotation_error_mean
            )
        elif data.frame_id == self.RobotsNamespaceBase + "1":
            self.APETopicReadings[1] = data.translation_error_mean
            self.APETopicReadings[1 + self.iRobotsQty] = (
                data.rotation_error_mean
            )
        elif data.frame_id == self.RobotsNamespaceBase + "2":
            self.APETopicReadings[2] = data.translation_error_mean
            self.APETopicReadings[2 + self.iRobotsQty] = (
                data.rotation_error_mean
            )


    def AllNodesKiller(self):
        # -- Killing all gazebo processes that may be open
        for procToKill in ["gazebo", "gzserver", "gzclient"]:
            process = subprocess.Popen(
                "killall -9 {}".format(procToKill), shell = True
            )
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
                rospy.loginfo(
                    "Cycle {} completed: rosnode kill {}"
                    .format(self.iActualCycle, node)
                )
                time.sleep(10)


    def ErrorsRecorder(self):
        rospy.loginfo(
            "ME_C{}: {}"
            .format(self.iActualCycle, self.fActualMapError)
        )
        for robot in range(self.iRobotsQty):
            rospy.loginfo(
                "Rob{}_TE_C{}: {}"
                .format(robot, self.iActualCycle, self.APETopicReadings[robot])
            )
            rospy.loginfo(
                "Rob{}_RE_C{}: {}"
                .format(
                    robot,
                    self.iActualCycle,
                    self.APETopicReadings[robot + self.iRobotsQty]
                )
            )


    def MapMetricComputation(self):
        self.sSLAMMapPath = "{}{}.pgm".format(self.GTpath, self.MapName)

        # --- Sending ground truth and slam maps paths to the error calculator
        with open(
            "{}MapMetricVariables.txt".format(self.sSourcePath), "w") as mmv:
            mmv.write("GTMapPath={}\n".format(self.GTMapPath))
            mmv.write("SLAMMapPath={}\n".format(self.sSLAMMapPath))
            mmv.close()

        # --- Running the error calculator
        self.sMapMetricFile = "{}MapAccuracy.py".format(self.sSourcePath)
        process = subprocess.Popen(
            "'{}'".format(self.sMapMetricFile), shell = True
        )
        process.wait()

        # --- Reading the error
        try:
            with open(
                "{}MapMetricVariables.txt"
                .format(self.sSourcePath), "r") as mmv:
                for line in mmv.readlines():
                    self.fActualMapError = float(line.split("=")[1])
            return self.fActualMapError
        except:
            rospy.logerr("Map file {} too large".format(self.iActualCycle))
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
        date = (
            datetime.now().strftime("%Y_%m_%d-%I:%M:%S_%p").replace(":","_")
        )
        self.MapName = (
            "{}_Trial_{}_RobotsQty_{}_Map_{}"
            .format(self.sSLAMName, self.iActualCycle, self.iRobotsQty, date)
        )
        sPath = "{}{}".format(self.GTpath, self.MapName)
        process = (
            subprocess.Popen(
                "rosrun map_server map_saver -f {}".format(sPath), shell = True
            )
        )
        process.wait()
        return self.MapName


    def CycleRunner(self):
        time.sleep(32)
        # -- Launch the SLAM algorithms and the automatic navigator
        subprocess.Popen(
            "roslaunch {} {}"
            .format(self.sSelfPackageName, self.sSLAMLaunchName), shell = True
        )
        time.sleep(10)
        for iRobot in range(self.iRobotsQty):
            sTopic = (
                "/{}{}/{}"
                .format(
                    self.RobotsNamespaceBase,
                    str(iRobot),
                    self.APETopicName
                )
            )
            self.APETopics[iRobot] = rospy.Subscriber(
                sTopic, APE, self.APEReader
            )
            rospy.loginfo("Subscribed to {}".format(sTopic))
        rospy.loginfo("Starting lap {}".format(self.iActualCycle))    
        self.CycleCompletionWatchdog()                                          # Wait for the robots to complete their lap
        rospy.loginfo("Completed lap {}".format(self.iActualCycle))  
        sCurrDate = datetime.now().strftime("%Y_%m_%d-%I:%M:%S_%p")
        rospy.loginfo(
            "Running map generator for cycle {}".format(self.iActualCycle)
        )
        self.SLAMMapGenerator()                                                 # Generate the SLAM map as pgm image
        sCurrDate = datetime.now().strftime("%Y_%m_%d-%I:%M:%S_%p")
        rospy.loginfo(
            "Running map metric computation for cycle {}"
            .format(self.iActualCycle)
        )  
        self.MapMetricComputation()                                             # Call an external script to compute the map metric
        sCurrDate = datetime.now().strftime("%Y_%m_%d-%I:%M:%S_%p")
        rospy.loginfo(
            "Running errors recorder for cycle {}".format(self.iActualCycle)
        )
        self.ErrorsRecorder()                                                   # Record the errors into log files
        self.NonGazeboNodesKiller()
        sCurrDate = datetime.now().strftime("%Y_%m_%d-%I:%M:%S_%p")
        rospy.loginfo(
            "Killing all non-gazebo nodes for cycle {}"
            .format(self.iActualCycle)
        )
        self.iActualCycle += 1
        return self.fActualMapError # , self.APETopicReadings


    def TargetFunction(self, args):
        for param in list(self.dParams.keys()):
            self.dParams.update({param: [args[param]            ,
                                        self.dParams[param][1]  ,
                                        self.dParams[param][2]  ,
                                        self.dParams[param][3]]})
        rospy.loginfo(self.dParams)
        self.setParamsOnYaml()
        return self.CycleRunner()


    def paramsOptimizer(self):
        best = fmin(
            self.TargetFunction,
            self.dSpace,
            algo = tpe.suggest,
            max_evals = self.iTrainingCycles
        )
        rospy.loginfo("---- BEST SETUP IS ----")
        rospy.loginfo(best)
        rospy.loginfo("---- BEST SETUP IS ----")


    def paramsValidator(self, iTrialsQty):
        # -- Run optimized params iTrialsQty times
        rospy.loginfo(
            "Running with optimized parameters {} times".format(iTrialsQty)
        )
        self.getParamsFromYaml()
        rospy.loginfo(self.dParams)
        for trial in range(iTrialsQty):
            self.CycleRunner()


################################################################################
# --                              Main script                              --  #
################################################################################
if __name__ == "__main__":
    calibrator = Calibrator()

    # -- Detect if will perform optimization or validation
    sRunType = "optimization"
    if rospy.has_param("/RunType"):
        sRunType = rospy.get_param("/RunType")

    # -- Perform parameters optimization
    if sRunType.lower() == "optimization":
        rospy.loginfo("-- RUNNING OPTIMIZATION --")
        calibrator.paramsOptimizer()

    # -- Perform parameters validation
    elif sRunType.lower() == "validation":
        rospy.loginfo("-- RUNNING VALIDATION --")
        iValTrials = 30
        if rospy.has_param("ValidationTrialsQty"):
            iValTrials = rospy.get_param("/ValidationTrialsQty")
        calibrator.paramsValidator(iValTrials)

    # -- Clean background jobs
    calibrator.AllNodesKiller()      
    os.system("rosnode kill slam_auto_calibrator")
