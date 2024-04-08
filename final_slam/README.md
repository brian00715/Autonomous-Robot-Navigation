# ME5413 Final SLAM
The ROS package '`final_slam`' contains SLAM algorithms utilized in the ME5413 Final Project by Group 8.

## Install Cartographer 
Please open a terminal in your workspace and execute the following commands to install Cartographer.

### Install Required Tools

    ```shell
    sudo apt-get update
    # sudo apt-get install -y  python3-rosdep stow
    sudo apt-get install -y python3-wstool python3-rosdep ninja-build stow
    ```

### Install Cartographer Dependencies

    ```shell
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
    ```

### Install Abseil-cpp Library

Cartographer requires the abseil-cpp library, which needs to be manually installed using the provided script:

Run the `install_abseil.sh` script:

    ```
    src/ME5413_Final_Project/final_slam/cartographer/cartographer/scripts/install_abseil.sh
    ```

### Build and Install

Build Cartographer ROS and install it:

    ```
    catkin build
    ```

After completing these steps, Cartographer ROS should be successfully installed on your system. You can activate it by running:

```shell
roslaunch final_slam mapping_carto.launch
```




