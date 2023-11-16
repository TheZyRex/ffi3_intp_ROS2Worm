# syntax=docker/dockerfile:1
FROM osrf/ros:humble-desktop
ARG USERNAME=ffi3
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV ROS_DOMAIN_ID=24
ENV ROS_LOCALHOST_ONLY=0
ENV SHELL [ "/bin/bash" ]

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip python3-rosdep python3-colcon-common-extensions libncurses5-dev libncursesw5-dev

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]

WORKDIR /home/$USERNAME
RUN git clone https://github.com/TheZyRex/ffi3_intp_ROS2Worm.git -b main
WORKDIR /home/$USERNAME/ffi3_intp_ROS2Worm

RUN rosdep fix-permissions\ 
&& rosdep update\
&& rosdep install -i --from-path src --rosdistro humble -y

COPY ros2_worm_entrypoint.sh /home/$USERNAME/.
ENTRYPOINT [ "/home/$USERNAME/ros2_worm_entrypoint.sh" ]