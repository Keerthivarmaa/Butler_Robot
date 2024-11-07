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



