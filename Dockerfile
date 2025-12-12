# build depends on ros noetic image
FROM osrf/ros:noetic-desktop-full
# install nano
RUN apt-get update && apt-get install nano -y && rm -rf /var/lib/apt/lists/*
# install git
RUN apt-get update && apt-get install git -y && rm -rf /var/lib/apt/lists/*
# create a non-root user linked to system's default user
ARG USERNAME=ros-noetic
RUN groupadd --gid 1000 ${USERNAME} \
    && useradd -s /bin/bash --uid 1000 --gid 1000 -m ${USERNAME} \
    && mkdir /home/${USERNAME}/.config && chown 1000:1000 /home/${USERNAME}/.config
# set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME}\
  && chmod 0440 /etc/sudoers.d/${USERNAME} \
  && rm -rf /var/lib/apt/lists/*
# install ros package dependencies
RUN apt-get update && apt-get install -y \
  ros-noetic-ros-control \
  ros-noetic-ros-controllers \
  ros-noetic-teleop-twist-keyboard \
  ros-noetic-moveit-simple-controller-manager \
  ros-noetic-moveit-fake-controller-manager \
  ros-noetic-moveit-setup-assistant \
  ros-noetic-moveit-planners \
  ros-noetic-moveit-ros-control-interface \
  ros-noetic-robot-pose-ekf \
  ros-noetic-gmapping \
  ros-noetic-map-server \
  ros-noetic-global-planner \
  ros-noetic-dwa-local-planner \
  ros-noetic-amcl \
  ros-noetic-move-base \
  ros-noetic-explore-lite \
  ros-noetic-rtabmap \
  ros-noetic-find-object-2d \
  && rm -rf /var/lib/apt/lists/*
# set up the entrypoint and bashrc
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc
ENTRYPOINT ["/bin/bash" , "/entrypoint.sh"]
WORKDIR /home/${USERNAME}
CMD ["/bin/bash"]