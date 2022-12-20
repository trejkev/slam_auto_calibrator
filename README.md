# SLAM Automatic Calibrator
Rosnode capable of performing automatic calibration of a SLAM algorithm being managed by a set of robots collaborating between them, and using the humongous power that federated learning provides.

## Steps to use the node:
1. Into slam_auto_calibrator.launch set the variables needed to run, all of them are mandatory.
2. Into multi_robot_in_world.launch, set the environment with the robots to be used.
3. Into slam launcher, set the variables needed to run the SLAM algorithms desired.
4. Make sure that all the different variables match between them, otherwise the code will not work correctly.
5. Start roscore and then run the node by using the command roslaunch slam_auto_calibrator slam_auto_calibrator.launch