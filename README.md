#I.Overview

Implementation of a maze solver algorithm (the robot always follows the wall on its right). Works with [ros_fastsim](https://github.com/jbmouret/ros_fastsim) from [jbmouret](https://github.com/jbmouret).

#II.Setup

First you need to install [libfastsim](https://github.com/jbmouret/libfastsim) and [ros_fastsim](https://github.com/jbmouret/ros_fastsim).
Then you can create a map (an black and white image with file extension .pbm) and place it in the envs folder of ros_fastsim node, or use one that is already in that folder. Then just modify the .xml and .launch file in the launch folder of the wall_follower node to meet your expectations.