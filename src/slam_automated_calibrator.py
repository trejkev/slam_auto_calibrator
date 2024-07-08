#!/usr/bin/env python

__author__ = "Kevin Trejos Vargas"
__email__  = "kevin.trejosvargas@ucr.ac.cr"

"""
MIT License

Copyright (c) 2022-2024 Kevin Trejos Vargas

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

# General libraries
import rospy
import subprocess
import time
import os
import random
import numpy as np
from datetime import datetime

# Local libraries
from slam_auto_calibrator.msg import APE

# Genetic Algorithm libraries
from deap import algorithms
from deap import base
from deap import creator
from deap import tools

# Global Variables
NODE_INITIALIZATION_WAIT_TIME_SEC           = 60  # Wait time for node initialization
NODES_KILLER_WAIT_TIME_SEC                  = 10  # Wait time for killing nodes
WATCHDOG_WAIT_TIME_SEC                      = 5   # Wait time for watchdog
CYCLE_WAIT_TIME                             = 32  # Wait time per cycle
NAVIGATION_ALGORITHMS_WAKE_UP_WAIT_TIME_SEC = 10  # Wait time for navigation algorithms wake up

# Params into slam_auto_calibrator.launch
lParamsList = [
    "/TrainingCycles"  , "/SLAMName"       , "/RobotsQty"          ,
    "/RobotsLaunchName", "/SelfPackageName", "/SLAMLaunchName"     ,
    "/APETopicName"    , "/RobotsPronoun"  , "/GroundTruthFilename",
    "/ThisNodeSrcPath" , "/MapsPath"       , "/ParamsFilePath"     ,
    "/Population_Size" , "/Generations_Qty"
]

class Calibrator(object):
    """Calibrator class for optimizing SLAM algorithms using genetic algorithms."""

    def __init__(self):
        self.iActualCycle = 0
        self.dParams = {}

        # Read launch parameters
        self.launchParams = {
            param.replace("/",""): rospy.get_param(param)
            for param in lParamsList if rospy.has_param(param)
        }

        # Create the lists that will store the APE results
        robotsQty = self.launchParams["RobotsQty"]
        self.lAPETopicReadings = list(range(2 * robotsQty))
        self.lAPETopics = list(range(robotsQty))

        # Path to the ground truth file
        self.sGTMapPath = self.launchParams["MapsPath"] + \
                          self.launchParams["GroundTruthFilename"]

        self.node_initialization()



    def node_initialization(self):
        """Initialize ROS node and launch the required processes."""
        self.kill_all_nodes()  # Ensure we have a fresh start without unwanted nodes
        rospy.init_node('slam_auto_calibrator')

        rospy.loginfo("Launch params are: {}".format(self.launchParams))

        # Open the arena with the robot(s)
        subprocess.Popen(
            "roslaunch slam_auto_calibrator {}".format(self.launchParams["RobotsLaunchName"]),
            shell = True
        )
        time.sleep(NODE_INITIALIZATION_WAIT_TIME_SEC)


    def get_parameters_from_yaml(self):
        """Read parameters from YAML file and store them in a dictionary."""
        fParamsFile = open(self.launchParams["ParamsFilePath"], 'r')
        with open(self.launchParams["ParamsFilePath"], 'r') as fParamsFile:
            for line in fParamsFile:
                # If the line contains a parameter
                if line.strip():
                    param, rest = line.split(":", 1)
                    typeVar = rest.split("#")[1].strip()
                    value   = line.split(":")[1].split("#")[0].strip()
                    minVal  = rest.split("#")[2].replace("min=", "").strip()
                    maxVal  = rest.split("#")[3].replace("max=", "").strip()

                    if typeVar == "int":
                        value  = int(value)
                        minVal = int(minVal)
                        maxVal = int(maxVal)
                    elif typeVar == "float":
                        value  = float(value)
                        minVal = float(minVal)
                        maxVal = float(maxVal)
                    elif typeVar == "bool":
                        value  = value
                        minVal = False
                        maxVal = True
                    else:
                        rospy.logerr("Parameter type {} not supported".format(typeVar))
                        continue

                    self.dParams.update({param: [value, typeVar, minVal, maxVal]})
 

    def set_parameters_on_yaml(self):
        """Write parameters from the dictionary to the YAML file."""
        with open(self.launchParams["ParamsFilePath"], 'w') as fParamsFile:
            for param, contents in self.dParams.items():
                value, typeVar, minVal, maxVal = contents
                fParamsFile.write(
                    "{}: {} #{} #min={} #max={}\n".format(param, value, typeVar, minVal, maxVal)
                )


    def set_search_space(self, toolbox):
        """Define the search space for the genetic algorithm."""
        for sParamName, values in self.dParams.items():
            param_type = values[1].lower()
            iMin, iMax = values[2], values[3]
            if param_type == "int":
                toolbox.register("attr_{}".format(sParamName), random.randint, iMin, iMax)
            elif param_type == "float":
                toolbox.register("attr_{}".format(sParamName), random.uniform, iMin, iMax)
            elif param_type == "bool":
                toolbox.register("attr_{}".format(sParamName), random.choice, [False, True])
            else:
                rospy.logerr("Parameter type {} not supported".format(param_type))
                return
            rospy.loginfo("Sarch space parameter {} as {}".format(sParamName, param_type))


    def ape_reader(self, data):
        """Read APE data and update topic readings."""
        prefix    = self.launchParams["RobotsPronoun"]
        robotsQty = self.launchParams["RobotsQty"]
        
        for i in range(robotsQty):
            if data.frame_id == "{}{}".format(prefix, i):
                self.lAPETopicReadings[i] = data.translation_error_mean
                self.lAPETopicReadings[i + robotsQty] = data.rotation_error_mean
                break  # Exit loop once data is processed for the correct robot


    def kill_all_nodes(self):
        """Kill all Gazebo and ROS nodes for a clean start."""
        # Killing all gazebo processes that may be open
        for procToKill in ["gazebo", "gzserver", "gzclient"]:
            process = subprocess.Popen("killall -9 {}".format(procToKill), shell = True)
            process.wait()

        # Kill all nodes except for rosout and this node for a clean start
        nodes = os.popen("rosnode list").readlines()
        nodes = [node.strip() for node in nodes]
        for node in nodes:
            if 'rosout' not in node and 'slam_auto_calibrator' not in node:
                os.system("rosnode kill {}".format(node))

        time.sleep(NODES_KILLER_WAIT_TIME_SEC)


    def kill_all_non_gazebo_nodes(self):
        """Kill the SLAM algorithms and their related nodes."""
        nodes = os.popen("rosnode list").readlines()
        nodes = [node.strip() for node in nodes]
        for node in nodes:
            if any(substring in node for substring in [
                'rviz', 'map_merge', 'map_saver', 'turtlebot3_slam', 'APE']):
                os.system("rosnode kill {}".format(node))
                rospy.loginfo(
                    "Cycle {} completed: rosnode kill {}"
                    .format(self.iActualCycle, node)
                )
                time.sleep(NODES_KILLER_WAIT_TIME_SEC)


    def record_errors(self):
        """Log the errors for map and robot APE readings."""
        rospy.loginfo("ME_C{}: {}".format(self.iActualCycle, self.fActualMapError))
        for robot in range(self.launchParams["RobotsQty"]):
            rospy.loginfo(
                "Rob{}_TE_C{}: {}".format(
                    robot, self.iActualCycle, self.lAPETopicReadings[robot]
                )
            )
            readingIndex = robot + self.launchParams["RobotsQty"]
            rospy.loginfo(
                "Rob{}_RE_C{}: {}".format(
                    robot, self.iActualCycle, self.lAPETopicReadings[readingIndex]
                )
            )


    def compute_map_metric(self):
        """Compute the map metric by comparing SLAM and ground truth maps."""
        mapsPath          = self.launchParams["MapsPath"]
        self.sSLAMMapPath = "{}{}.pgm".format(mapsPath, self.MapName)

        # Sending ground truth and slam maps paths to the error calculator
        filePath = "{}MapMetricVariables.txt".format(mapsPath)
        with open(filePath, "w") as mmv:
            mmv.write("GTMapPath={}\n".format(self.sGTMapPath))
            mmv.write("SLAMMapPath={}\n".format(self.sSLAMMapPath))

        # Running the error calculator
        srcPath             = self.launchParams["ThisNodeSrcPath"]
        self.sMapMetricFile = "{}map_accuracy.py".format(srcPath)
        process = subprocess.Popen("'{}'".format(self.sMapMetricFile), shell = True)
        process.wait()

        # Reading the error
        try:
            filePath = "{}MapMetricVariables.txt".format(mapsPath)
            with open(filePath, "r") as mmv:
                for line in mmv.readlines():
                    self.fActualMapError = float(line.split("=")[1])
            return self.fActualMapError
        except:
            rospy.logerr("Map file {} too large".format(self.iActualCycle))
            return "NA"  # SLAM map file too large


    def cycle_completion_watchdog(self):
        """Watchdog to monitor completion of ROS nodes."""
        bRunCompleted = False
        while not bRunCompleted:
            nodes = os.popen("rosnode list").readlines()
            nodes = [node.strip() for node in nodes]
            bFoundSpeedController = False
            for node in nodes:
                if "speed_controller" in node:
                    bFoundSpeedController = True
                    break
            if not bFoundSpeedController:
                bRunCompleted = True
            time.sleep(WATCHDOG_WAIT_TIME_SEC)


    def generate_map(self):
        """Generate a map with a timestamped filename."""
        date = (datetime.now().strftime("%Y_%m_%d-%I:%M:%S_%p").replace(":","_"))
        self.MapName = "{}_Trial_{}_RobotsQty_{}_Map_{}".format(
            self.launchParams["SLAMName"],
            self.iActualCycle,
            self.launchParams["RobotsQty"],
            date
        )
        sPath = "{}{}".format(self.launchParams["MapsPath"], self.MapName)
        process = subprocess.Popen(
            "rosrun map_server map_saver -f {}".format(sPath), shell = True
        )
        process.wait()
        return self.MapName


    def run_cycle(self):
        """Run a cycle of SLAM calibration and map generation."""
        time.sleep(CYCLE_WAIT_TIME)

        # Launch the SLAM algorithms and the automatic navigator
        subprocess.Popen(
            "roslaunch {} {}".format(
                self.launchParams["SelfPackageName"],
                self.launchParams["SLAMLaunchName"]
            ),
            shell = True
        )
        time.sleep(NAVIGATION_ALGORITHMS_WAKE_UP_WAIT_TIME_SEC)

        # Subscribe to APE topics for each robot
        for iRobot in range(self.launchParams["RobotsQty"]):
            sTopic = "/{}{}/{}".format(
                self.launchParams["RobotsPronoun"],
                str(iRobot),
                self.launchParams["APETopicName"]
            )
            self.lAPETopics[iRobot] = rospy.Subscriber(
                sTopic, APE, self.ape_reader
            )
            rospy.loginfo("Subscribed to {}".format(sTopic))

        rospy.loginfo("Starting lap {}".format(self.iActualCycle))    
        self.cycle_completion_watchdog()  # Wait for the robots to complete their lap

        rospy.loginfo("Completed lap {}".format(self.iActualCycle))
        rospy.loginfo("Running map generator for cycle {}".format(self.iActualCycle))
        self.generate_map()  # Generate the SLAM map as pgm image

        rospy.loginfo("Running map metric computation for cycle {}".format(self.iActualCycle))  
        self.compute_map_metric()  # Call an external script to compute the map metric

        rospy.loginfo("Running errors recorder for cycle {}".format(self.iActualCycle))
        self.record_errors()  # Record the errors into log files

        rospy.loginfo("Killing all non-gazebo nodes for cycle {}".format(self.iActualCycle))
        self.kill_all_non_gazebo_nodes()

        self.iActualCycle += 1
        return self.fActualMapError  # Return the map error for optimization


    def target_function(self, individual):
        """Target function for the genetic algorithm optimization."""
        # Update parameters based on individual
        params = {
            param: individual[i] for i, param in enumerate(self.dParams.keys())
        }
        for param in list(self.dParams.keys()):
            value = params[param]
            # Handle params out of bounds because of mutation post processing
            if self.dParams[param][1].lower() == 'int':
                value = int(max(min(value, self.dParams[param][3]), self.dParams[param][2]))
            elif self.dParams[param][1].lower() == 'float':
                value = max(min(value, self.dParams[param][3]), self.dParams[param][2])
            elif self.dParams[param][1].lower() == 'bool':
                value = bool(value)
            self.dParams.update({param: [value                  ,
                                        self.dParams[param][1]  ,
                                        self.dParams[param][2]  ,
                                        self.dParams[param][3]]})

        rospy.loginfo("Current run params: {}".format(self.dParams))
        self.set_parameters_on_yaml()
        return self.run_cycle(),


    def mutate_individual(self, individual, indpb):
        """Mutate an individual for evolutionary optimization."""
        rospy.loginfo("MUTATION INIT: Individual is: {}".format(individual))
        for i, param in enumerate(self.dParams.keys()):
            if self.dParams[param][1].lower() == 'float':
                individual[i] += random.gauss(0, 0.1)
                # Ensure the value stays within the bounds
                individual[i] = max(
                    min(individual[i], self.dParams[param][3]), self.dParams[param][2]
                )
                rospy.loginfo(
                    'Float mutation: {} param - {} contents - {} mut value'
                    .format(param, self.dParams[param], individual[i])
                )
            elif self.dParams[param][1].lower() == 'int':
                if random.random() < indpb:
                    individual[i] += random.randint(-1, 1)
                    # Ensure the value stays within the bounds
                    individual[i] = int(
                        max(min(individual[i], self.dParams[param][3]), self.dParams[param][2])
                    )
                    rospy.loginfo(
                        'Integer mutation: {} param - {} contents - {} mut value'
                        .format(param, self.dParams[param], individual[i])
                    )
                else:
                    rospy.loginfo(
                        "Integer mutation: Param {} - {} - not muted {}"
                        .format(param, self.dParams[param], individual[i])
                    )
            elif self.dParams[param][1].lower() == 'bool':
                if random.random() < indpb:
                    individual[i] = not individual[i]
                    rospy.loginfo(
                        'Boolean mutation: {} param - {} contents - {} mut value'
                        .format(param, self.dParams[param], individual[i])
                    )
                else:
                    rospy.loginfo(
                        "Boolean mutation: Param {} - {} - not muted {}"
                        .format(param, self.dParams[param], individual[i])
                    )
        rospy.loginfo("MUTATION END: Individual is: {}".format(individual))
        return individual,


    def optimize_parameters(self):
        """Optimize SLMM parameters using an evolutionary algorithm."""
        # Define the fitness function as minimizing
        creator.create("FitnessMin", base.Fitness, weights = (-1.0,))

        # Define the individual as a list with the fitness attribute
        creator.create("Individual", list, fitness = creator.FitnessMin)

        self.get_parameters_from_yaml()

        toolbox = base.Toolbox()

        self.set_search_space(toolbox = toolbox)

        # Collect attribute names dynamically
        attributes = [getattr(toolbox, "attr_{}".format(param)) for param in self.dParams.keys()]

        # Define the structure of an individual and the population
        toolbox.register("individual", tools.initCycle, creator.Individual, attributes, n = 1)
        toolbox.register("population", tools.initRepeat, list, toolbox.individual)

        # Define the target function
        toolbox.register("evaluate", self.target_function)

        # Define the recombination method for parent individuals
        toolbox.register("mate", tools.cxBlend, alpha = 0.5)

        # Define the mutation algorithms
        toolbox.register("mutate", self.mutate_individual, indpb = 0.2)

        # Define the selection criteria by tournament of 3 individuals
        toolbox.register("select", tools.selTournament, tournsize = 3)

        population = toolbox.population(n = self.launchParams["Population_Size"])

        # Define the probabilities of mating and mutation
        cxpb, mutpb = 0.5, 0.2

        # Statistics to gather during the evolution
        stats = tools.Statistics(lambda ind: ind.fitness.values)
        stats.register("avg", np.mean)
        stats.register("std", np.std)
        stats.register("min", np.min)
        stats.register("max", np.max)

        # Hall of Fame to store the best individuals
        hof = tools.HallOfFame(1)

        # Run the evolutionary algorithm
        algorithms.eaSimple(
            population,
            toolbox,
            cxpb,
            mutpb,
            self.launchParams["Generations_Qty"],
            stats = stats,
            halloffame = hof,
            verbose = True
        )

        # Print the best solution found
        best_individual = hof[0]
        best_params = {
            param: best_individual[i] for i, param in enumerate(self.dParams.keys())
        }
        best_fitness = best_individual.fitness.values[0]
        rospy.loginfo("Best individual parameters:")
        for param, value in best_params.items():
            rospy.loginfo("{}: {}".format(param, value))
        rospy.loginfo("With fitness: {}".format(best_fitness))
        rospy.loginfo("PARAMS VALIDATION FOR 30 TRIALS")
        for _ in range(30):
            self.target_function(individual = hof[0])


    def validate_parameters(self, iTrialsQty):
        """Validate optimized parameters through multiple trials."""
        # Run optimized params iTrialsQty times
        rospy.loginfo("Running with optimized parameters {} times".format(iTrialsQty))
        self.get_parameters_from_yaml()
        rospy.loginfo(self.dParams)
        for _ in range(iTrialsQty):
            self.run_cycle()


"""
Main script for parameter optimization or validation using the Calibrator class

This script initializes a Calibrator object and performs either optimization or
validation of parameters based on a specified run type ('optimization' or
'validation').

Parameters:
    /RunType: Specifies the type of run ('optimization' or 'validation').
        Default is 'optimization'.
    /ValidationTrialsQty: Number of validation trials to perform when running
        in 'validation' mode. Default is 30.

"""
if __name__ == "__main__":
    calibrator = Calibrator()

    # Detect if will perform optimization or validation
    sRunType = "optimization"
    if rospy.has_param("/RunType"):
        sRunType = rospy.get_param("/RunType")

    # Perform parameters optimization
    if sRunType.lower() == "optimization":
        rospy.loginfo("-- RUNNING OPTIMIZATION --")
        calibrator.optimize_parameters()

    # Perform parameters validation
    elif sRunType.lower() == "validation":
        rospy.loginfo("-- RUNNING VALIDATION --")
        iValTrials = 30
        if rospy.has_param("ValidationTrialsQty"):
            iValTrials = rospy.get_param("/ValidationTrialsQty")
        calibrator.validate_parameters(iValTrials)

    # Clean background jobs
    calibrator.kill_all_nodes()      
    os.system("rosnode kill slam_auto_calibrator")
