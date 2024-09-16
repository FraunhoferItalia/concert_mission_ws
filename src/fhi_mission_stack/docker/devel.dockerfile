FROM fhi-git.fraunhofer.it:5050/behavior-tree/behavior_tree_stack:release

ENV ROS_WS_DIR="/mission_stack_ws"
ENV ROS_DISTRO="${ROS_DISTRO}"

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR ${ROS_WS_DIR}

SHELL ["/bin/bash", "-c"]

RUN apt-get update -qq \
  && apt-get -y install ros-${ROS_DISTRO}-nav2-msgs \
  # installing rqt plot inorder to prevent to call --force-discover
  && apt-get -y install ros-${ROS_DISTRO}-rqt-plot \
  && apt-get -y install ros-${ROS_DISTRO}-moveit-msgs \
  && apt-get -y install ros-${ROS_DISTRO}-controller-manager-msgs \
  && apt-get -y install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

ENV DEBIAN_FRONTEND=dialog

COPY ".vscode" "${ROS_WS_DIR}/.vscode"
RUN sed -i -E "s/(ROS_DISTRO=\").*(\")/\1${ROS_DISTRO}\2/" ${ROS_WS_DIR}/.vscode/ros_source.sh
# source the workspace where the behaviortree stack was build in
RUN echo "source /behavior_tree_stack_ws/install/setup.bash" >> ~/.bashrc
RUN echo "source install/setup.bash" >> ~/.bashrc
RUN echo "alias rsource=' source ${ROS_WS_DIR}/.vscode/ros_source.sh'" >> ~/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

