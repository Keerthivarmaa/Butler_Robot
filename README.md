# Butler_Robot

This project is a ROS2-based navigation system for a Butler Robot in a simulated environment. The robot can take orders to deliver items to specific tables, moving between predefined locations such as the home position, kitchen, and tables. It uses `nav2_simple_commander` for handling navigation tasks.

# Prerequisites

Ensure the following packages and dependencies are installed before setting up the workspace.

- **[ROS2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)**

- **NAV2 Stack**

        sudo apt install ros-humble-navigation2

        sudo apt install ros-humble-nav2-bringup

- **Slam-toolbox**

        sudo apt install ros-humble-slam-toolbox

# Workspace setup

- Create a workspace

        mkdir -p ~/robot_ws/src

- Clone the repository
        
        cd ~/robot_ws/src

        git clone https://github.com/Keerthivarmaa/Butler_Robot.git

- Build and source the workspace

        cd ~/robot_ws

        colcon build

        source install/setup.bash

# Robot Simulation

All the commands should run in the `robot_ws` directory and sourcing the workspace is mandatory

- Launch gazebo

        ros2 launch butler gazebo.launch.py world:=src/Butler_Robot/world/cafe.world

- Launch rviz2

        ros2 run rviz2 rviz2 -d ./src/Butler_Robot/config/map.rviz

- Launch Map 

        ros2 launch butler online_async_launch.py

- Launch Navigation_stack

        ros2 launch butler navigation_launch.py

**Current features enabled for the robot:**

- To navigate the robot to the user defined location , in our case **`Home`,`Kitchen`,`Tables 1,2,3`** 

        ros2 run butler move

- To accept the order from the kitchen and deliver to each tables sequentially

        ros2 run butler queue

- To deliver multiple orders:`The robot will accept multiple orders from the kitchen and deliver them to the designated tables. If any order is canceled, the robot will skip the corresponding table(s), return the canceled order to the kitchen, and then go back to its home position. If all orders are accepted, the robot will deliver to all tables and return to home`.

        ros2 run butler multi_order





