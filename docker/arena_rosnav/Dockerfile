FROM ros:melodic

RUN apt update
RUN apt -y upgrade

RUN apt-get -y install curl
RUN apt-get -y install python2.7 python-dev
RUN curl https://bootstrap.pypa.io/pip/2.7/get-pip.py -o get-pip.py
RUN python get-pip.py
RUN pip2 install tensorflow==1.4.0

# Install Jupyter notebook
RUN pip install ipython==5.7 ipykernel==4.10 jupyter
RUN pip install matplotlib==2.2.3

SHELL ["/bin/bash", "-c", "-l"]

WORKDIR /home/catkin_ws
RUN pip install pyyaml
RUN apt-get install -y python-tk
COPY ./docker/cadrl_ros_docker_entrypoint.sh /home
RUN ["chmod", "+x", "/home/cadrl_ros_docker_entrypoint.sh"]
RUN mkdir src
COPY ./src/hkust_rgd_msgs src/hkust_rgd_msgs
COPY ./src/cadrl_ros src/cadrl_ros
RUN source "/opt/ros/$ROS_DISTRO/setup.bash" && catkin_make

RUN ls /home

RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN sudo apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
RUN source ~/.bashrc && rosdep update
RUN sudo apt-get update && sudo apt-get install -y \
libqt4-dev \
libopencv-dev \
liblua5.2-dev \
screen \
python3.6 \
python3.6-dev \
libpython3.6-dev \
python3-catkin-pkg-modules \
python3-rospkg-modules \
python3-empy \
python3-setuptools \
ros-melodic-navigation \
ros-melodic-teb-local-planner \
ros-melodic-mpc-local-planner \
libarmadillo-dev \
ros-melodic-nlopt

RUN source ~/.bashrc && sudo apt update && sudo apt install -y rviz ros-melodic-rviz ros-melodic-pcl-conversions ros-melodic-pcl-ros python3-pip
RUN sudo pip3 install --upgrade pip && sudo pip3 install virtualenv && sudo pip3 install virtualenvwrapper && cd $HOME && mkdir python_env \
&& echo "export WORKON_HOME=$HOME/python_env   #path to your venv folder \
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3   #path to your python3 \
export VIRTUALENVWRAPPER_VIRTUALENV=/usr/local/bin/virtualenv \
source /usr/local/bin/virtualenvwrapper.sh" >> ~/.bashrc

RUN mkvirtualenv --python=python3.6 rosnav && workon rosnav

ENTRYPOINT ["/home/cadrl_ros_docker_entrypoint.sh"]