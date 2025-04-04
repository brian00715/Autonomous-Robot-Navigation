sudo apt install -y ros-noetic-rviz-imu-plugin ros-noetic-move-base ros-noetic-navfn tmux python3-catkin-tools python3-wstool python3-rosdep ninja-build stow ffmpeg lua5.2 liblua5.2-dev libceres-dev ros-noetic-ros-control ros-noetic-navigation ros-noetic-jsk-rviz-plugins htop ros-noetic-velodyne-description ros-noetic-flir-camera-description ros-noetic-lms1xx ros-noetic-pointgrey-camera-description ros-noetic-sick-tim ros-noetic-interactive-marker-twist-server ros-noetic-diff-drive-controller ros-noetic-joint-state-controller ros-noetic-joy ros-noetic-robot-localization ros-noetic-teleop-twist-joy ros-noetic-topic-tools ros-noetic-twist-mux xclip

sudo apt-get remove -y ros-${ROS_DISTRO}-abseil-cpp
python -m pip install Pillow markupsafe==2.0.1 jinja2 ipdb scipy casadi

git clone https://github.com/osrf/gazebo_models.git ~/.gazebo/models

echo 'shopt -s nocaseglob' >>~/.bashrc
touch ~/.inputrc
echo 'set completion-ignore-case on
"\e[A": history-search-backward
"\e[B": history-search-forward' >>~/.inputrc

cd ~
git clone https://github.com/gpakosz/.tmux.git
ln -s -f .tmux/.tmux.conf
cp .tmux/.tmux.conf.local .
tmux source-file ~/.tmux.conf
echo "set -g mouse on" >>~/.tmux.conf
echo "tmux_conf_copy_to_os_clipboard=true" >>~/.tmux.conf

echo 'alias rosk="rosnode kill -a ; killall -9 roscore rosmaster gzserver gazebo rviz rqt rqt_tf_tree rqt_graph rqt_reconfigure"
alias gazebok="pkill -P $(pgrep -f gazebo.launch) ; pkill -9 gzserver ; pkill -9 gzclient"
alias rqtg="rosrun rqt_graph rqt_graph"
alias rqttf="rosrun rqt_tf_tree rqt_tf_tree"
alias rqtrecon="rosrun rqt_reconfigure rqt_reconfigure"' >>~/.bashrc

echo "Installation complete. "
