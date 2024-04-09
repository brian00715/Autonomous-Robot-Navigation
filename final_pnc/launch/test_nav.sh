#!/bin/zsh
xpanes -d -e "roslaunch me5413_world world.launch" "sleep 3;roslaunch final_slam localization_carto.launch" "sleep 5;roslaunch final_pnc navigation.launch"
