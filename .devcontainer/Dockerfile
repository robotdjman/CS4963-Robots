FROM osrf/ros:noetic-desktop-full

# Add vscode user with same UID and GID as your host system
# (copied from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG DEBIAN_FRONTEND=noninteractive
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

#Create setup folder
RUN sudo mkdir /setup


# Install Deps and Python 3.11
RUN sudo apt-get install software-properties-common -y
RUN sudo apt install wget build-essential libncursesw5-dev libssl-dev \
    libsqlite3-dev tk-dev libgdbm-dev libc6-dev libbz2-dev libffi-dev zlib1g-dev -y
RUN sudo add-apt-repository ppa:deadsnakes/ppa
RUN sudo apt install python3.11 -y
RUN sudo apt-get -y install python3-pip

#Install Conda
RUN sudo apt-get install wget
RUN sudo wget https://repo.anaconda.com/archive/Anaconda3-2023.07-2-Linux-x86_64.sh -O /setup/anaconda.sh
RUN sudo bash /setup/anaconda.sh -b -p /home/vscode/anaconda3
RUN sudo rm /setup/anaconda.sh
RUN sudo chown -R vscode /home/vscode/anaconda3

ENV PATH /home/vscode/anaconda3/bin:$PATH
RUN conda update conda -y
# RUN conda update anaconda -y
# Set Solver to libmamba
RUN conda update --all -y
RUN conda install -n base conda-libmamba-solver
RUN conda config --set solver libmamba 
RUN conda init

# Setup Conda environment
COPY foundations.yaml /home/vscode/foundations.yaml
RUN conda env create -f /home/vscode/foundations.yaml


#Install catkin
RUN pip3 install -U catkin_tools

# Install Git
RUN sudo apt install -y git

# Rosdep update
RUN rosdep update

RUN mkdir /home/vscode/setup

# --------------------------- MuSHR Deps ---------------------------
# Install git, tkinter, wget, g++, vim, tmux, networking stuff, apt-add-repository
RUN sudo apt-get install -y git-all python3-tk wget g++ vim tmux net-tools iputils-ping software-properties-common
RUN sudo apt-get update
RUN sudo apt-get install -y python3-vcstool python3-pip

# Install extra ROS packages
RUN sudo apt-get install -y ros-noetic-ackermann-msgs ros-noetic-map-server ros-noetic-urg-node ros-noetic-robot-state-publisher ros-noetic-xacro ros-noetic-joy ros-noetic-ddynamic-reconfigure ros-noetic-fake-localization ros-noetic-gmapping ros-noetic-rosbridge-suite ros-noetic-sbpl

# Install Cython, PyTorch 1.10 at least!
RUN sudo pip3 install Cython torch torchvision torchaudio numpy scipy networkx progress --upgrade

# Create OpenCV symbolic link
RUN sudo ln -s /usr/include/opencv4 /usr/include/opencv

# Auto source this workspace on terminal startup
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Install rangelibc
RUN python3 /CS4963-Robots/catkin_ws/src/range_libc/pywrapper/setup.py install
RUN rm -rf /CS4963-Robots/catkin_ws/src/range_libc

# Create default RVIZ setup
RUN mkdir ~/.rviz
RUN cp /catkin_ws/src/mushr/mushr_utils/rviz/default.rviz ~/.rviz/

# MuSHR Hardware drivers
# Hardware drivers (Only needed on physical robot)
#COPY mushr_install_hw_drivers.bash /home/vscode/setup/mushr_install_hw_drivers.bash
#RUN if [ "$REAL" = 1 ] ; then chmod +x /home/vscode/setup/mushr_install_hw_drivers.bash && /home/vscode/setup/mushr_install_hw_drivers.bash ; fi

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
