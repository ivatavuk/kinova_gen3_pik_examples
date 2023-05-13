FROM ubuntu:20.04

ENV TZ=Europe/Zagreb

# Setup timezone (fix interactive package installation)
RUN ln -snf /usr/share/zoneinfo/${TZ} /etc/localtime && echo ${TZ} > /etc/timezone

# Install dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    cmake \
    git \
    python3-dev \
    python3-pip \
    python3-venv

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y keyboard-configuration

# Install ROS Noetic
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-get update && apt-get install -y ros-noetic-desktop-full

# Set up ROS environment
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# Install colcon-common-extensions
RUN pip3 install colcon-common-extensions

# Create a catkin workspace
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'

# Set up the catkin workspace environment
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# Install additional ROS packages
RUN apt-get update && apt-get install -y \
    ros-noetic-rosbash \
    ros-noetic-tf2 \
    ros-noetic-joy \
    ros-noetic-robot-state-publisher \
    ros-noetic-xacro \
    ros-noetic-moveit \
    ros-noetic-rqt \
    ros-noetic-rviz \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-plugins \
    ros-noetic-gazebo-msgs \
    ros-noetic-gazebo-dev

RUN apt-get install -y libeigen3-dev

#install osqp from source
WORKDIR /root
RUN git clone --recursive https://github.com/osqp/osqp
WORKDIR /root/osqp
RUN mkdir build 
WORKDIR /root/osqp/build
RUN cmake -G "Unix Makefiles" ..
RUN cmake --build . 
RUN cmake --build . --target install
    
#install osqp-eigen from source
WORKDIR /root
RUN git clone https://github.com/robotology/osqp-eigen.git
WORKDIR /root/osqp-eigen
RUN mkdir build
WORKDIR /root/osqp-eigen/build
RUN cmake .. 
RUN make 
RUN make install

#install ptsc-eigen from source
WORKDIR /root
RUN git clone https://github.com/ivatavuk/ptsc_eigen.git
WORKDIR /root/ptsc_eigen
RUN mkdir build
WORKDIR /root/ptsc_eigen/build
RUN cmake .. 
RUN make 
RUN make install
RUN ctest

#install pik-ros
WORKDIR /root/catkin_ws

#install ros_kortex
RUN apt-get update && \
    apt-get install -y python3-rosdep && \
    rosdep init && \
    rosdep update
RUN apt install python3 python3-pip
RUN python3 -m pip install conan==1.59
RUN conan config set general.revisions_enabled=1
RUN conan profile new default --detect > /dev/null
RUN conan profile update settings.compiler.libcxx=libstdc++11 default
#For now its private - so you need ssh key
#RUN git clone git@github.com:ivatavuk/pik_ros.git
WORKDIR /root/catkin_ws/src
RUN git clone -b noetic-devel https://github.com/Kinovarobotics/ros_kortex.git
WORKDIR /root/catkin_ws
#RUN rosdep install --from-paths src --ignore-src -y
#RUN catkin_make
RUN bash -c "source /opt/ros/noetic/setup.bash"
ENV PATH="/opt/ros/noetic/bin:${PATH}"


RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'

RUN apt install -y tmux
RUN apt install -y ros-noetic-ros-controllers
RUN apt install -y gitg
RUN apt install -y git-gui
RUN apt install -y htop
# Clean up
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

CMD ["/bin/bash"]
