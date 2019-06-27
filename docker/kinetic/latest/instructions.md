docker build . --tag realsense_ros

docker run -it --privileged -v /dev/bus/usb:/dev/bus/usb realsense_ros

roslaunch realsense2_camera rs_camera.launch

sudo x11docker --hostdisplay --hostnet --user=RETAIN -- --privileged -v /dev/bus/usb:/dev/bus/usb -v /home/lajoiepy/Documents/masters/research_code/robust_distributed_slam/frontend/bags:/home/bags -- realsense_ros

rosbag record /camera/infra1/camera_info /camera/infra2/camera_info /camera/infra1/image_rect_raw/compressed /camera/infra2/image_rect_raw/compressed /camera/color/image_raw/compressed /camera/color/camera_info /camera/infra1/image_rect_raw /camera/infra2/image_rect_raw /camera/color/image_raw