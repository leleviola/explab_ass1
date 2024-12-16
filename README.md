# EXPERIMENTAL ROBOTICS ASSIGNMENT 1
This code is a simulation of a mobile robot, that, whith a camera, analyzes 5 aruco markers, placed in a circle around it, and detect their ID order from the lowest to the highest. For each marker, it publishes an image with the marker circled in the custom topic /img_markers.
There are 2 different nodes:
- marker_analysis1: the robot look around by rotating itself;
- marker_analysis2: the robot look around by rotating the camera.
This program runs in ROS2 and is made with Python scripts.
## Usage Instructions
1. Clone the repository in your ROS2 workspace
   ```bash
   git clone https://github.com/leleviola/explab_ass1.git
   ```

2. Make sure you have the Ros aruco package in your ros2 repository (if not you can find it on github https://github.com/CarmineD8/ros2_aruco)
3. After having builded the workspace, you can go in the src folder and run the environment from terminal using
   ```bash
   ros2 launch robot_urdf gazebo_aruco.launch.py
   ```
4. Then, in a new terminal, run the aruco node
   ```bash
   ros2 run ros2_aruco aruco_node
   ```
5. Finally, to make robot start moving and detect aruco marker, run
```bash
   ros2 run robot_urdf marker_analysis1
```
or
```bash
   ros2 run robot_urdf marker_analysis2
   ```

