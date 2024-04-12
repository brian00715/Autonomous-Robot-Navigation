# ME5413 Final Project - Group 8

Demo:
![](./docs/30s.gif)

For the original simulation environment that this project relies on, please refer to [ME5413_Final_Project](https://github.com/NUS-Advanced-Robotics-Centre/ME5413_Final_Project).

## Structure

```shell
.
├── final_pnc # Navigation package
├── final_percep # Perception package
├── final_slam # SLAM package
├── final_fsm # Finite State Machine package
├── interactive_tools
├── jackal_description
└── me5413_world
```

## Installation

### System dependencies

- Ubuntu 20.04
- ROS Noetic
- C++11 and above

### ROS dependencies
```shell
rosdep install --from-paths src --ignore-src -r -y
sudo apt install ros-noetic-rviz-imu-plugin ros-noetic-move-base ros-noetic-navfn tmux python3-catkin-tools zsh
```

### Python dependencies

```shell
python -m pip install Pillow markupsafe==1.1.1 ipdb
```

### Build

```shell
mkdir -p ~/me5413_final_ws/src
cd ~/me5413_final_ws/
git clone https://github.com/brian00715/ME5413_Final_Project src

catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release

catkin build cartographer* final_slam final_pnc final_percep final_fsm jackal* interactive_tools me5413_world
```

### Perception using conda

Please install `conda` first.

```shell
conda create -n me5413 python=3.8
conda activate me5413
conda install pytorch==2.1.1 torchvision==0.16.1 pytorch-cuda=12.1 -c pytorch -c nvidia
conda install -c conda-forge opencv rosdep rospkg easyocr decorator pexpect numpy defusedxml ipdb
export PYTHONPATH=$PYTHONPATH:/usr/lib/python3.8/dist-packages
```

## Running

### One-click launch

```shell
rosrun final_fsm start.sh
```

### Step-by-step launch

- mapping

```shell
roslaunch me5413_world world.launch
roslaunch final_slam mapping_carto.launch
```

- localization

```shell
roslaunch final_slam localization_carto.launch
```

- navigation(with finate state machine)

```shell
roslaunch me5413_world world.launch
roslaunch final_fsm fsm.launch
```

- navigation

```shell
roslaunch final_pnc navigation.launch
```


# Acknowledgement

We would like to thank the following open-source projects:

- [ME5413_Final_Project](https://github.com/NUS-Advanced-Robotics-Centre/ME5413_Final_Project)
- [Cartographer](https://github.com/cartographer-project/cartographer)
- [Fast-LIO](https://github.com/hku-mars/FAST_LIO)
- [ros_motion_planning](https://github.com/ai-winter/ros_motion_planning)
- [pcd2pgm_package](https://github.com/Hinson-A/pcd2pgm_package)
- [PA-DMPC-UAV-Ad-Hoc](https://github.com/brian00715/PA-DMPC-UAV-Ad-Hoc)
- [EasyOCR](https://github.com/JaidedAI/EasyOCR)
- [occ_grid_mapping](https://github.com/ydsf16/occ_grid_mapping/)
---


