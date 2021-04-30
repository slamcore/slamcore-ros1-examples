FROM osrf/ros:melodic-desktop-full as RosBase
ARG VER=melodic

# install packages -------------------------------------------------------------
# you can get the list of packages for each one of the ROS packages that you'll
# be installing from source using rosdep, use a command like the following:
# rosdep install --simulate --from-path src/<name-of-ros-package> -r -y
RUN \
    apt-get update && \
    # bug in genpy < 0.6.13 - need to upgrade otherwise won't connect to
    # /dev/tty* port
    apt-get upgrade -y && \
    apt-get install --no-install-recommends --assume-yes \
    software-properties-common \
    ros-${VER}-joy \
    ros-${VER}-teleop-twist-joy \
    ros-${VER}-teleop-twist-keyboard \
    ros-${VER}-depthimage-to-laserscan \
    ros-${VER}-rosserial-arduino \
    ros-${VER}-rosserial-python \
    ros-${VER}-rosserial-server \
    ros-${VER}-rosserial-client \
    ros-${VER}-rosserial-msgs \
    ros-${VER}-map-server \
    ros-${VER}-move-base \
    ros-${VER}-urdf \
    ros-${VER}-xacro \
    ros-${VER}-compressed-image-transport \
    ros-${VER}-rqt-image-view \
    ros-${VER}-navigation \
    ros-${VER}-interactive-markers \
    ros-${VER}-teb-local-planner \
    ros-${VER}-joy-teleop \
    # kobuki dependencies
    ros-${VER}-kobuki-ftdi \
    ros-${VER}-ecl-sigslots \
    ros-${VER}-ecl-exceptions \
    ros-${VER}-capabilities \
    ros-${VER}-yocs-controllers \
    ros-${VER}-ecl-time \
    ros-${VER}-kobuki-driver \
    ros-${VER}-kobuki-msgs \
    ros-${VER}-ecl-threads \
    ros-${VER}-ecl-streams \
    ros-${VER}-yocs-cmd-vel-mux \
    ros-${VER}-ecl-geometry \
    ros-${VER}-kobuki-dock-drive \
    ros-${VER}-std-capabilities \
    ros-${VER}-ecl-linear-algebra \
    ros-${VER}-yocs-velocity-smoother \
    ros-${VER}-ddynamic-reconfigure \
    udev \
    # dev packages
    vim neovim git tmux curl man ssh iproute2 gdb cmake-gui cmake-curses-gui \
    psmisc  htop wget sxiv rsync python3-pip python3-setuptools ccache  \
    iputils-ping ntp ntpdate \
    && rm -rf /var/lib/apt/lists/*

# slamcore_deps/position dependencies
RUN apt-get update && \
    apt-get --assume-yes --no-install-recommends install \
    cmake build-essential git sudo gnupg2 fakeroot python3 libgl-dev libglew-dev \
    libusb-1.0-0-dev lsb-release libsnappy-dev libcereal-dev \
    build-essential libavahi-client-dev alien libcxsparse3 \
    libjpeg-turbo8-dev libcurl4-openssl-dev libpng-dev && \
    apt-get install -y python3-snappy ninja-build \
    && rm -rf /var/lib/apt/lists/*

# nvidia stuff
RUN apt-get update && apt-get install -y --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libgles2 \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# pip3 dependencies
RUN pip3 install wheel && \
    pip3 install pyyaml rospkg toml && \
    pip3 install "git+https://github.com/dirk-thomas/vcstool"

# SLAMcore dependencies --------------------------------------------------------
RUN ./install-debs.sh

FROM RosBase as UserStuff

# don't run as root ------------------------------------------------------------
ARG USERNAME
ARG UID
ARG GID
ARG HOME=/home/$USERNAME
ARG ROS_DIR=$HOME/ros_ws
ENV HOME=$HOME
ENV VER=$VER
ENV ROS_DIR=$ROS_DIR

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:-all}

RUN groupadd -g $GID -o $USERNAME
RUN useradd \
    --create-home \
    --uid $UID \
    --gid $GID \
    --groups sudo \
    --password $(openssl passwd -1 $USERNAME) $USERNAME \
    --home-dir $HOME
RUN usermod -aG sudo $USERNAME
RUN echo "$USERNAME ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/$USERNAME
RUN cat /etc/sudoers.d/$USERNAME

# additional rules for accessing hw
RUN usermod -aG dialout,plugdev $USERNAME

COPY config/entrypoint.sh /
RUN chown $USERNAME:$USERNAME /entrypoint.sh

# vvvv Regular user vvvv -------------------------------------------------------

USER $USERNAME

# setup ROS workspace ----------------------------------------------------------
# add these as symbolic links so that we're able to edit them during the docker
# runtime
RUN ln -s "$ROS_DIR/.sctest/ubuntu18.04-amd64/ros_demos_ws/config/bashrc.local" $HOME/.bashrc.local
RUN ln -s "$ROS_DIR/.sctest/ubuntu18.04-amd64/ros_demos_ws/config/rosconfig.conf" $HOME/.rosconfig.conf
RUN echo "source \$HOME/.bashrc.local" >> ~/.bashrc

RUN rosdep update

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

WORKDIR $ROS_DIR
