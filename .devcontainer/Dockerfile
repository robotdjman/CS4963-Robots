FROM osrf/ros:noetic-desktop-full

# Add vscode user with same UID and GID as your host system
# (copied from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID
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

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
