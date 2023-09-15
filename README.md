# SLAM Automatic Calibrator
This ROS node is capable of performing automated external calibration of a SLAM algorithm, being managed by a set of robots collaborating between them, or a single robot (it will depend on the .launch files setup), and using the humongous power that Bayesian optimization provides.

The tool makes use of hyperopt, a Python library implementing Tree-Structured Parzen Estimators (which is a statistical variant that Bayesian optimization could use), to try to converge to the set of params that maximizes an output function result.

## Steps to use the node:
1. Into _slam_auto_calibrator.launch_, set the variables needed to run the optimization or validation algorithm, most of them are mandatory.
2. Into _multi_robot_in_world.launch_, set the environment with the robots to be used.
3. Into the _slam_launcher.launch_, set the variables needed to run the SLAM algorithms desired per robot, and the navigation algorithm to be used.
4. Start _roscore_ in a shell, and then, run the node in another shell by using the command _roslaunch slam_auto_calibrator slam_auto_calibrator.launch_.
