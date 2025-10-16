# ====================================================================
# Base image
# ====================================================================
FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    LANG=C.UTF-8 LC_ALL=C.UTF-8 \
    ROS_DISTRO=jazzy \
    XDG_RUNTIME_DIR=/tmp/runtime-root

# ====================================================================
# Create non-root user matching host UID/GID (for volume access)
# ====================================================================
ARG USERNAME=root
ARG USER_UID=1000
ARG USER_GID=1000

# Create a user only if not root
RUN if [ "$USERNAME" != "root" ]; then \
      if ! getent group ${USER_GID} >/dev/null; then \
        groupadd --gid ${USER_GID} ${USERNAME}; \
      else \
        echo "GID ${USER_GID} already exists, reusing existing group"; \
        group_name=$(getent group ${USER_GID} | cut -d: -f1); \
        usermod --gid ${USER_GID} ${USERNAME} 2>/dev/null || true; \
      fi && \
      useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
      apt-get update && apt-get install -y sudo && \
      echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
      chmod 0440 /etc/sudoers.d/$USERNAME; \
    fi

# ====================================================================
# Install system dependencies
# ====================================================================
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
    gcc-10 g++-10 \
    build-essential cmake \
    x11-apps mesa-utils libgl1 \
    xterm vim gdb bash bash-completion wget curl unzip git tree \
    ffmpeg \
    libpcl-dev libgoogle-glog-dev libgflags-dev \
    libblas-dev liblapack-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev \
    libopencv-dev libboost-dev libboost-filesystem-dev \
    libcanberra-gtk-module libcanberra-gtk3-module \
    python3-pip \
    python3-dev \
    python3-setuptools \
    python3-wheel \
    build-essential \
    python3-matplotlib \
    python3-numpy \
    python3-psutil \
    python3-tk \
    python3-colcon-common-extensions \
    ros-$ROS_DISTRO-pcl-ros \
    ros-$ROS_DISTRO-tf2-sensor-msgs \
    ros-$ROS_DISTRO-compressed-image-transport \
    ros-$ROS_DISTRO-foxglove-bridge && \
    rm -rf /var/lib/apt/lists/*

# ====================================================================
# Set GCC/G++ alternatives
# ====================================================================
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 100 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-10 100

# ====================================================================
# Create workspace and clone repo
# ====================================================================
WORKDIR /ws
RUN chown -R $USER_UID:$USER_GID /ws
USER $USERNAME

RUN git clone https://github.com/Alexander-guo/gopro_ros2.git src/gopro_ros2

# ====================================================================
# Build the ROS2 workspace
# ====================================================================
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --packages-select gopro_ros2 --symlink-install \
    --cmake-args -DBUILD_GOPRO_TO_ASL=OFF"

# ====================================================================
# Shell environment setup
# ====================================================================
#RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc && \
#    echo "source /ws/install/setup.bash" >> /etc/bash.bashrc && \
#    echo "alias SOURCE_WS='source /ws/install/setup.bash'" >> /etc/bash.bashrc && \
#    echo 'alias ROS_BUILD_RELEASE="colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=1"' >> /etc/bash.bashrc

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> $HOME/.bashrc && \
    echo "source /ws/install/setup.bash" >> $HOME/.bashrc && \
    echo "alias SOURCE_WS='source /ws/install/setup.bash'" >> $HOME/.bashrc && \
    echo 'alias ROS_BUILD_RELEASE="colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=1"' >> $HOME/.bashrc

# ====================================================================
# Default command
# ====================================================================
CMD ["bash"]

