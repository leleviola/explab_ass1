# EXPERIMENTAL ROBOTICS ASSIGNMENT 1
This code is a simulation of a mobile robot, that, whith a camera, analyzes 5 aruco markers, placed in a circle around it, and detect their ID order from the lowest to the highest. For each marker, it publishes an image with the marker circled in the custom topic /img_markers.
There are 2 different nodes:
- marker_analysis1: the robot look around by rotating itself;
- marker_analysis2: the robot look around by rotating the camera.
This program runs in ROS2 and is made with Python scripts.
## Instructions
1. Clone the repository in your ROS2 workspace
   ```bash
   git clone
   ``` 
