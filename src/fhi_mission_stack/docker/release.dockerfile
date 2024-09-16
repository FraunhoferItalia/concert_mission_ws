FROM fhi-git.fraunhofer.it:5050/rise-libs/ro/mi/fhi_mission_stack:devel

ENV ROS_WS_DIR="/mission_stack_ws"
ENV ROS_DISTRO="${ROS_DISTRO}"


ENV DEBIAN_FRONTEND=noninteractive

WORKDIR ${ROS_WS_DIR}

SHELL ["/bin/bash", "-c"]

# build the workspce, let BUILDKIT remove the source
RUN --mount=type=bind,source=/,target=mission_stack_ws/src/ cd /mission_stack_ws \
    && . /opt/ros/$ROS_DISTRO/setup.sh \
    && . /behavior_tree_stack_ws/install/setup.sh \
    && colcon build --event-handlers console_direct+

ENV DEBIAN_FRONTEND=dialog

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

