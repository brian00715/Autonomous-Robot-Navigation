
#!/bin/zsh
rosbag record /amcl_pose /final_slam/odom /gazebo/ground_truth/state -O /home/simon/nus_ws/src/final_slam/data/odom
