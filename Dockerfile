# This Dockerfile enables installation of ROS and this package in a VNC-exposed environment.
#
# More specifically, ROS, f1tenth_simulator and f1tenth_gym will get installed as part of this docker environment, as
# as this package will get cloned from the remote and configured.
#
# Then the bash scripts present in the "scripts" folder can be used to launch specific parts of the system.
#
# As part of the environment, a "workspace" folder will hold all relevant information, including the "setup-workspace"
# file which must be sourced before executing any ROS-specific commands.

# Use Ubuntu 18 exposed via the VNC interface
FROM dorowu/ubuntu-desktop-lxde-vnc:bionic
RUN apt-get update

# This is where all relevant files will be stored
ARG WORKSPACE=/root/workspace
RUN mkdir -p $WORKSPACE

# Need dirmngr to run apt-key commands
RUN apt-get install dirmngr -y

# Need pip bootstrap file to install pip and then to install this application with pip
RUN cd $WORKSPACE \
    && curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py

# Install Python3.8 and make python3 point to it for ease of use, also remove the bootstrap file once installed pip
RUN apt-get install python3.8 python3.8-dev python3.8-distutils build-essential -y \
    && cd $WORKSPACE \
    && python3.8 get-pip.py \
    && rm get-pip.py \
    && update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 1

# Add ROS sources and install ros-melodic (full desktop for ease of use)
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && apt-get update \
    && apt-get install ros-melodic-desktop-full -y

# Install F1/10 dependencies, the simulator, and build
RUN apt-get install git -y \
    && apt-get install ros-melodic-tf2-geometry-msgs ros-melodic-ackermann-msgs ros-melodic-joy ros-melodic-map-server -y \
    && python3.8 -m pip install catkin_pkg \
    && mkdir -p $WORKSPACE/simulator/src \
    && cd $WORKSPACE/simulator/src \
    && git clone https://github.com/f1tenth/f1tenth_simulator.git \
    && bash -c "source /opt/ros/melodic/setup.bash && cd $WORKSPACE/simulator && catkin_make"

# Install the application
RUN mkdir $WORKSPACE/code \
    && cd $WORKSPACE/code \
    && git clone https://89447c1d8e792ef3e72f5a02585cad4dc5bc686d@github.com/TheCodeSummoner/f1tenth-racing-algorithms.git \
    && cd $WORKSPACE/code/f1tenth-racing-algorithms \
    && python3.8 -m pip install .

# Finally, put all relevant source commands into a single setup-workspace file
RUN touch $WORKSPACE/setup-workspace.sh \
    && echo "source /opt/ros/melodic/setup.bash" >> $WORKSPACE/setup-workspace.sh \
    && echo "source $WORKSPACE/simulator/devel/setup.bash" >> $WORKSPACE/setup-workspace.sh \
    && echo "export PYTHONPATH=/opt/ros/melodic/lib/python2.7/dist-packages"
