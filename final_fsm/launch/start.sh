#!/bin/bash -e
echo "Start a Tmux session."
tmux start-server

uid=$(whoami)

# Create a Tmux session "robot_runtime"
tmux new-session -d -s robot_runtime

tmux split-window -h -t robot_runtime:zsh

# roscore
# > window: main
tmux new-window -n main
tmux split-window -h -t robot_runtime:main
tmux split-window -v -t robot_runtime:main.1
tmux split-window -v -t robot_runtime:main.3
tmux split-window -h -t robot_runtime:main.4
tmux split-window -h -t robot_runtime:main.4

tmux send -t robot_runtime:main.1 'roslaunch me5413_world world.launch' ENTER
tmux send -t robot_runtime:main.2 "roscd final_percep/src; conda activate test" ENTER
tmux send -t robot_runtime:main.2 "sleep 4 && python visual.py" ENTER
tmux send -t robot_runtime:main.3 "sleep 4 && roslaunch final_pnc slam_pnc.launch" ENTER
tmux send -t robot_runtime:main.4 "sleep 7 && roslaunch final_fsm fsm.launch" ENTER

tmux send -t robot_runtime:main.5 'roscd final_fsm/launch' ENTER
tmux send -t robot_runtime:main.5 'sleep 7 && ./screen_record.sh' ENTER

# tmux new-window -n debug
tmux attach-session -t robot_runtime
