#!/bin/zsh
#  /gazebo/ground_truth/state /amcl_pose /final_slam/odom
bag_path=$(rospack find final_slam)/data
echo "bag_path: $bag_path"
# evo_ape bag $bag_path/odom.bag /gazebo/ground_truth/state /amcl_pose -p --plot_mode xy --save_results $bag_path/amcl.zip
# evo_ape bag $bag_path/odom.bag /gazebo/ground_truth/state /final_slam/odom -p --plot_mode xy --save_results $bag_path/carto.zip

# evo_res $bag_path/*.zip -p #--save_table $bag_path/table.csv 