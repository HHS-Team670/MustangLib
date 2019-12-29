#!/bin/bash

CAMERA_FRONT="/dev/video0"
CAMERA_BACK="/dev/video1"
PORT_FOR_FRONT=5800
PORT_FOR_BACK=5801
DS_IP_ADDR=10.6.70.5
SELF_IP_ADDR=0.0.0.0

while true
do
	ping -c 1 -w 1 ${DS_IP_ADDR}
	if [ $? -eq 0 ]; then
		echo "pinged ds"
		if [ -e ${CAMERA_FRONT} ]; then
			echo "launching"
			v4l2-ctl -d ${CAMERA_FRONT} -c brightness=100 -c contrast=50 -c white_balance_temperature_auto=1 -c backlight_compensation=1 -c exposure_auto=1 -c exposure_absolute=20
		        gst-launch-1.0 -v v4l2src device=${CAMERA_FRONT} ! "video/x-raw,width=320,height=240,framerate=30/1" ! avenc_mpeg4 ! rtpmp4vpay config-interval=3 ! udpsink host=${DS_IP_ADDR} port=${PORT_FOR_FRONT} &
			echo "camera 1"
		fi
		if [ -e ${CAMERA_BACK} ]; then
			v4l2-ctl -d /dev/video1 -c brightness=100 -c contrast=50 -c white_balance_temperature_auto=1 -c backlight_compensation=1 -c exposure_auto=1 -c exposure_absolute=20
			gst-launch-1.0 -v v4l2src device=${CAMERA_BACK} ! "video/x-raw,width=320,height=240,framerate=30/1" ! avenc_mpeg4 ! rtpmp4vpay config-interval=3 ! udpsink host=${DS_IP_ADDR} port=${PORT_FOR_BACK} &
		fi
		exit
	fi
	sleep 1
done
