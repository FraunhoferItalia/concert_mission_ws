FROM fhi-git.fraunhofer.it:5050/rise-libs/ro/mi/fhi_mission_stack:release

ENV ROS_WS_DIR="/concert_mission_ws"
WORKDIR ${ROS_WS_DIR}

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# for rviz_plugins
RUN apt-get -y update && apt-get -y install \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rviz-common \
    ros-${ROS_DISTRO}-rviz-default-plugins \
    ros-${ROS_DISTRO}-rviz-visual-tools \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp 

# Visual studio configuration, update it and register aliases
COPY "/.vscode" "${ROS_WS_DIR}/.vscode"

RUN echo "source ${ROS_WS_DIR}/install/setup.bash" >> ~/.bashrc && \
    echo "source /mission_stack_ws/install/setup.bash" >> ~/.bashrc && \
    echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc && \
    echo "alias rbuild='(cd ${ROS_WS_DIR} && colcon build --cmake-args -DBUILD_TESTING=OFF && source install/setup.sh)'" >> ~/.bash_aliases && \
    echo "alias rclean='(cd ${ROS_WS_DIR} && rm -rf install/ log/ build/)'"  >> ~/.bash_aliases

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]