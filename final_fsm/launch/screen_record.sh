#!/bin/bash

timestamp=$(date +"%Y-%m-%d_%H-%M-%S")

ffmpeg -f x11grab -video_size 1920x1080 -i :0.0 -c:v h264_nvenc -framerate 60 -r 60 -b:v 3000k -draw_mouse 1 \
"/home/simon/Videos/screenrecord_$timestamp.mp4"
