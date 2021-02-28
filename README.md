# Robot-Kinematics

WIP

## Setup

Currently, this repo is not published to ROS package, but it can be treated similar as ROS packages.
As a stopgap measures, you can setup this as below:

1. Create package by using `catkin_create_package` with dependencies: `rospy`, `std_msgs`, `visualization_msgs`, `geometry_msgs`.
2. Now, directory structure under `catkin_ws` will be look like:
    ```
    src
    ├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
    ├── LICENSE
    ├── README.md
    ├── assets
    ├── interactive_marker_tutorials
    ├── robot_kinematics # <- HERE
    ├── using_markers
    └── viz_engine
    ```
3. Copy all contents in this repo under `robot_kinematics` folder.
4. Run `catkin_make` to compile it.


## Brief Description of Package

* `rk`: `RobotObject` in `robot_config.py` manages kinematics. Kinematic chain is defined by a convention used in *Introduction to Humanoid Robotics by Shuuji Kajita*.
* `test`: This is for development. You can easily check intermediate values through this. `pytest` is used.
* `viz`: This controls visualization. It is compatible with RVIZ. `rk_api.py` serves to communicate between RVIZ and kinematics tool through ROS. `viz_manager.py` spawns RVIZ markers. Implementations are strongly based on [RVIZ tutorials](http://wiki.ros.org/rviz/Tutorials#Text-based_Tutorials), so if you are interested in visualization with RVIZ, please refer to the tutorial first.
