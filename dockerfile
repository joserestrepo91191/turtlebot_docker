#FROM --platform=linux/arm64 osrf/ros:humble-desktop-full
FROM arm64v8/ros:humble-ros-base
# upgrade os and install python with setuptools
RUN apt-get update && apt-get -y upgrade && \
  apt-get install -y python3-pip && \
  pip install setuptools==58.2.0 && \
  rm -rf /var/lib/apt/lists/*

#paths for pkgs
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' \
&& curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -


#  Limpiar posibles fuentes conflictivas de ROS2
RUN rm -f /etc/apt/sources.list.d/ros2.list \
    && rm -f /usr/share/keyrings/ros-archive-keyring.gpg

# Usar ros2-apt-source para configurar repositorios y claves ROS2 automáticamente
RUN apt-get update && apt-get install -y curl \
    && export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') \
    && curl -L -o /tmp/ros2-apt-source.deb \
       "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && dpkg -i /tmp/ros2-apt-source.deb

# Ahora sí actualizar todo el sistema
RUN apt-get update && apt-get -y upgrade
#installing ROS2 pkgs
RUN apt-get install -y \
  ros-humble-ros-testing \
  ros-humble-laser-proc \
  ros-humble-urg-c \
  ros-humble-urg-node-msgs \
  ros-humble-joint-state-publisher \
  ros-humble-ament-cmake \
  ament-cmake \
  ros-humble-diagnostic-updater \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws/src

# Cloning pkg

RUN git clone https://github.com/kobuki-base/kobuki_core.git && \
  git clone https://github.com/kobuki-base/kobuki_ros_interfaces.git&& \
  git clone https://github.com/stonier/ecl_lite.git && \  
  git clone https://github.com/stonier/ecl_tools.git && \
  git clone https://github.com/stonier/ecl_core.git && \
  git clone -b release/1.2.x https://github.com/stonier/sophus.git && \
  git clone https://github.com/kobuki-base/cmd_vel_mux.git && \
  git clone https://git.kcir.pwr.edu.pl/irak/turtlebot2.git && \
  git clone -b ros2-devel https://github.com/igrak34/urg_node.git && \
  git clone https://github.com/igrak34/kobuki_ros.git

RUN  apt-get update && apt-get -y upgrade
RUN rm -rf /etc/ros/rosdep/sources.list.d/20-default.list 
RUN sudo rosdep init
RUN rosdep update --include-eol-distros
RUN rosdep install --from-paths . --ignore-src -r -y
RUN  apt-get update && apt-get -y upgrade

WORKDIR /ros2_ws

RUN /bin/bash -c "source /opt/ros/humble/setup.bash; colcon build --symlink-install"
RUN apt-get update && apt-get -y upgrade 
RUN apt-get update && apt-get install -y network-manager dbus
RUN rm -rf /etc/ros/rosdep/sources.list.d/20-default.list


WORKDIR  /ros2_ws/src/kobuki_core/
RUN sudo cp 60-kobuki.rules /etc/udev/rules.d/
RUN sudo service udev reload
RUN sudo service udev restart

WORKDIR /ros2_ws
RUN /lib/systemd/systemd-udevd --daemon && udevadm control --reload-rules


