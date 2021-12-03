# RGB + Depth ROS package (Dual Hikvision and Basler Setup)

Steps to make it work

First download and install the Hikvision and Basler Blaze-101 sdks. 

- Declare a Catkin Workspace.
- clone this repo into the src folder.
- run the catkin_build command
- source the src directory with the current ROS version
- In 2 separate terminals fire up the rgb camera and 3D camera nodes
- In a 3rd terminal run the Calibration node to start Calibrating both the cameras
- Once finished with calibration, run the Fusion_node to see the RGB-D integrated point cloud.

