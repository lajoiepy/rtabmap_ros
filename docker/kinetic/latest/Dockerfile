FROM ros:kinetic-perception
# install rtabmap packages
RUN apt-get update && apt-get install -y \
    ros-kinetic-rtabmap-ros \
    && apt-get remove -y \
    ros-kinetic-rtabmap \
    && rm -rf /var/lib/apt/lists/

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

WORKDIR /root/

ARG CACHE_DATE=2016-01-01

RUN git clone https://github.com/lajoiepy/rtabmap.git

# Install Intel Realsense packages
RUN apt-get update && apt-get install -y software-properties-common
RUN apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
RUN add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
RUN apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev
RUN apt-get install  ros-kinetic-diagnostic-updater ros-kinetic-ddynamic-reconfigure

RUN source /ros_entrypoint.sh && \
    mkdir -p ~/catkin_ws/src &&\
    cd ~/catkin_ws/src &&\
    git clone https://github.com/intel-ros/realsense.git && \
    catkin_init_workspace && \
    cd .. &&\
    catkin_make clean &&\
    catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release &&\
    catkin_make install &&\
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc &&\
    source ~/.bashrc

# Build RTAB-Map project
RUN source /ros_entrypoint.sh && \
    cd rtabmap/build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    cd ../.. && \
    rm -rf rtabmap && \
    ldconfig

RUN source /ros_entrypoint.sh && \
    mkdir -p catkin_ws/src && \
    cd catkin_ws/src && \
    git clone https://github.com/lajoiepy/rtabmap_ros.git && \
    cd .. && \
    catkin_make -j1 -DCMAKE_INSTALL_PREFIX=/opt/ros/kinetic install


RUN mkdir -p bags

COPY entrypoint.sh /tmp/entrypoint.sh
RUN chmod +x /tmp/entrypoint.sh
