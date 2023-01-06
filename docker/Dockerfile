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

ENTRYPOINT ["/home/cadrl_ros_docker_entrypoint.sh"]