# Instructions for setting up Camera Streaming

## Update and install tools
* run `sudo apt update` & `sudo apt upgrade`
* run `sudo apt-get instal gstreamer1.0-tools`

## Create a Gstreamer Startup script
* `CAMERA_FRONT` & `CAMERA_BACK` = Ports for front and back usb cameras (Ex: /dev/video0)
* `PORT_FOR_FRONT` & `PORT_FOR_BACK` = Server ports on which the camera streaming for each camera has to run
  * For a list of available ports, refer to: 
* `DS_IP_ADDR` = The ip address of your driverstation
  * Set the Driver Station IP as a static ip in Control Panel > Network & Internet > Network & Sharing Center > Change Adapter Settings > Properties of WiFi > Internet Protocol Version 4 (TCP/IPv4)
* To set the properties of the webcam, `v4l2-ctl -d ${CAMERA} -c brightness=100 -c contrast=50 -c white_balance_temperature_auto=1 -c backlight_compensation=1 -c exposure_auto=1 -c exposure_absolute=20`
* To launch, `gst-launch-1.0 -v v4l2src device=${CAMERA} ! "video/x-raw,width=320,height=420,framerate=30/1 ! avenc_mpeg4 ! rtpmp4vpay  config_interval=3 ! udpsink host=${DS_IP_ADDR} port=${PORT_FOR_FRONT} &`

## Create a Startup Script
* Sleep for 10ms & run gstreamer_script.sh in the background in this script
* `sudo bash gstreamer_script.sh &`

## Add Startup script to rc.local to make it autostart
* `sudo nano /etc/rc.local`
* Add this to the end of the script: `sudo bash /home/pi/startup.sh &`

