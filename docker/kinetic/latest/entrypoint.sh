#!/bin/bash

export ROS_IP=$(echo $(hostname -I) | cut -d' ' -f 2)
export ROS_MASTER_URI=http://192.168.12.$1:11311

source "/opt/ros/kinetic/setup.bash"

case "$3" in
        rviz)
            rviz
            ;;
         
        record)
            cd /home/bags && \
            rosbag record -o /home/bags/ /robot_$2/camera/color/camera_info /robot_$2/camera/color/image_raw \
                          /robot_$2/camera/infra1/camera_info /robot_$2/camera/infra1/image_rect_raw \
                          /robot_$2/camera/infra2/camera_info /robot_$2/camera/infra2/image_rect_raw \
                          /tf /tf_static
            ;;
	trigger)
	    rosservice call /robot_0/start_optimization "{}"
	    rosservice call /robot_1/start_optimization "{}"
	    rosservice call /robot_2/start_optimization "{}"
	    ;;

        bash)
            /bin/bash
            ;;
        *)
            echo $"Usage: $3 {rviz|record|bash}"
            exit 1
 
esac
