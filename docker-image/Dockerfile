# syntax=docker/dockerfile:1
FROM ros:humble
ARG USERNAME=ffi3
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG ROS2_INSTALL_DIR="/opt/ros/humble/"

ENV SHELL [ "/bin/bash" ]

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y git python3-pip python3-rosdep python3-colcon-common-extensions libncurses5-dev libncursesw5-dev

# ********************************************************

# All following actions will be executed as USERNAME
USER $USERNAME

WORKDIR /home/$USERNAME
RUN git clone https://github.com/TheZyRex/ffi3_intp_ROS2Worm.git -b main
WORKDIR /home/$USERNAME/ffi3_intp_ROS2Worm

RUN if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then sudo rosdep init; fi

RUN rosdep fix-permissions \ 
	&& rosdep update \
	&& rosdep install -i --from-path src --rosdistro humble -y

RUN /bin/bash -c "if [ -d /opt/ros/humble ]; then . /opt/ros/humble/setup.bash; colcon build; fi"

COPY ros2_worm_entrypoint.sh /home/$USERNAME/.
RUN sudo chown $USERNAME:$USERNAME /home/$USERNAME/ros2_worm_entrypoint.sh
RUN sudo chmod +x /home/$USERNAME/ros2_worm_entrypoint.sh
ENTRYPOINT [ "/home/ffi3/ros2_worm_entrypoint.sh" ]

CMD ["/bin/bash"]