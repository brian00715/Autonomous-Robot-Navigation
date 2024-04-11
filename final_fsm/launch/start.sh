#!/bin/zsh -e
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
tmux send -t robot_runtime:main.2 "roscd final_percep/src; conda activate test" ENTER
tmux send -t robot_runtime:main.2 'sleep 4' ENTER
tmux send -t robot_runtime:main.2 "python visual.py" ENTER
tmux send -t robot_runtime:main.3 'sleep 4' ENTER
tmux send -t robot_runtime:main.3 "roslaunch final_pnc slam_pnc.launch" ENTER
tmux send -t robot_runtime:main.4 'sleep 7' ENTER
tmux send -t robot_runtime:main.4 "roslaunch final_fsm fsm.launch" ENTER

# tmux new-window -n debug
tmux attach-session -t robot_runtime
