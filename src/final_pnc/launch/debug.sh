#!/bin/bash -e
echo "Start a Tmux session."
tmux start-server

uid=$(whoami)

# Create a Tmux session "robot_runtime"
tmux new-session -d -s robot_runtime

# roscore
# > window: main
tmux new-window -n main
tmux split-window -h -t robot_runtime:main
tmux split-window -v -t robot_runtime:main.1
tmux split-window -v -t robot_runtime:main.3
tmux send -t robot_runtime:main.1 'roslaunch me5413_world world.launch' ENTER
tmux send -t robot_runtime:main.2 'sleep 4' ENTER
tmux send -t robot_runtime:main.2 "roslaunch final_slam localization_carto.launch" ENTER
tmux send -t robot_runtime:main.3 'sleep 4' ENTER
tmux send -t robot_runtime:main.3 "roslaunch final_pnc pnc.launch" ENTER
# tmux send -t robot_runtime:main.3 'sleep 8' ENTER
# tmux send -t robot_runtime:main.3 "rosrun final_pnc eval_pnc.py" ENTER
# tmux send -t robot_runtime:main.4 "rosrun final_pnc pub_reach.py "

# tmux new-window -n debug
tmux attach-session -t robot_runtime
