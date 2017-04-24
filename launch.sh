#!/bin/bash

sudo modprobe bcm2835-v4l2
v4l2-ctl -d /dev/video2 -c vertical_flip=1
v4l2-ctl -d /dev/video1 -c vertical_flip=1
/home/pi/CORE2017-Vision/build/CORE2017-Vision
