FROM ros:humble-ros-base-jammy as builder
ENV ROS_WS=/root/ros_ws
#WORKDIR /home/orangepi/ros2_ws
WORKDIR $ROS_WS

COPY ./ src/lidar_odom_from_scratch

RUN bash -c "apt update && \
    apt-get install -y build-essential cmake libgsl0-dev libcairo2-dev libboost-all-dev libpcl-dev ros-humble-pcl-ros ros-humble-laser-geometry && \
    source /opt/ros/humble/setup.bash && \
    mkdir /git && cd /git &&\
    git clone https://github.com/AndreaCensi/csm.git && \
    cd csm && \
    mkdir build && cd build && cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr/local .. && make && make install && \ 
    cd $ROS_WS && colcon build && \
    rm -rf /git && \
    apt-get clean && rm -rf /var/lib/apt/lists/* && \
    mkdir -p ${ROS_WS}/log/runtime/container" 
    
ENV LD_LIBRARY_PATH=/usr/local/lib
ENV LOG_DIR=${ROS_WS}/log/runtime/container

CMD [ "/bin/bash" ,"-c", "source ${ROS_WS}/install/setup.bash && \
                          export ROS_DOMAIN_ID=30 && \
                          $DATE=$(date '+%Y-%m-%d_%H-%M-%S') && \
                          echo $(date '+%Y-%m-%d %H:%M:%S.%N') >> ${LOG_DIR}/lidar_odom_cont_$DATE.log && \
                          ros2 run lidar_odom_from_scratch lidar_odom_from_scratch_node >> ${LOG_DIR}/lidar_odom_cont_$DATE.log 2>&1" ]
