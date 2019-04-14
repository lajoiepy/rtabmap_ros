docker build . --tag realsense_ros

docker run -it --privileged -v /dev/bus/usb:/dev/bus/usb realsense_ros

roslaunch realsense2_camera rs_camera.launch