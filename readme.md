# MSP Humanoid Project IDA ROS Workspace

Welcome to the ROS workspace for the MSP Humanoid Project IDA. This workspace encompasses a suite of nodes and packages tailored for voice activation, object recognition, and precise motion control.


## Runing the Project 
To activate the project and run all nodes, execute the command:
```ros2 launch camera_pkg launch.launch.py```

# Nodes & Packages
![Node Map](rosgraph.png)
In this project, we have five nodes in total. One for Chatbot, two for Camera and two for Arm.

# Mic Package

## Mic Node

### Publishing Data Type
Robot Status

### Function
This node is the entry point for IDA. This node is responsible for keyword activation, TTS and STT. Once IDA is activated, it will publish the Robot Status.

# Camera Package

## Bottle Recognition Node

### Publishing Data Type
Position, color and text of every bottle. (The format is TBD)

### Function
This node will identify the colors, text on the bottle and position of them.

## Pouring Accuracy Node

### Publishing Data Type
The percentage of liquid in the bottle.

### Function
This node will read in the percentage of liquid from the bottle.

# Motion Control Package

## Arm Control Node

### Publishing Data Type
A flag variable to determine if the arm arrives at the end point.

### Function
This node is subscribing all the relevant bottle information published by the bottle recognition Node, and then move the arm to the end point.

## Pouring Node

### Function
This node will control the pouring, once the liquid is beyond the threshold. It will stop.
