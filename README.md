# TurtleBot3 Navigation Setup Guide

This guide provides detailed instructions for setting up TurtleBot3 navigation on Ubuntu 20.04. It includes steps for installing TurtleBot3 packages, creating a map, and launching navigation.

## Install TurtleBot3 Packages

1. **Install TurtleBot3 packages**

    ```sh
    sudo apt update
    sudo apt install ros-noetic-turtlebot3
    ```

2. **Install TurtleBot3 simulation packages**

    ```bash
    sudo apt install ros-noetic-turtlebot3-simulations
    ```

## Setup TurtleBot3 Environment Variables

1. **Add the following environment variables to your `.bashrc` file:**

    ```sh
    echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

## Launch TurtleBot3 Simulation

1. **Launch the TurtleBot3 World:**

    ```sh
    roslaunch turtlebot3_gazebo turtlebot3_world.launch
    ```
    ![Gazebo model](https://github.com/user-attachments/assets/bfdf7d90-3abb-4c8e-8dde-d23d8ae613e0)


## Create Map and Launch Navigation

1. **Launch the SLAM node:**

    ```sh
    roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
    ```
    ![gmapping](https://github.com/user-attachments/assets/6ce2cdbd-c2e2-4697-90ea-712dac8f025f)


2. **Run the teleoperation node to control the robot and create a map:**

    ```sh
    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
    ```
    Use the keyboard to move the robot around and create the map.


    ![control the robot](https://github.com/user-attachments/assets/f89ad127-83c8-43d4-97b9-986d485c8f92)



3. **Save the map once you have finished mapping:**

    ```sh
    rosrun map_server map_saver -f ~/map
    ```
    ![moving around to complete the map](https://github.com/user-attachments/assets/86078cc2-b57a-45c0-8248-8b17fbaa64e4)

4. **Launch the Navigation node with the saved map:**

    ```sh
    roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
    ```
    ![saved map](https://github.com/user-attachments/assets/2262482b-e6d4-4310-baaa-060012b7b7ec)
