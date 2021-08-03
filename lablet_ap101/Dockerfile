FROM osrf/ros:melodic-desktop-full
USER root
RUN apt-get -y update && \
    apt-get install -y wget && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list' && \
        wget http://packages.ros.org/ros.key -O - | sudo apt-key add - && \
        apt-get -y update && \
        apt-get install -y tree python-catkin-tools

# Setting up 'artpark' user
RUN groupadd -g 1001 artpark && \
    useradd -d /home/artpark -s /bin/bash -m artpark -u 1001 -g 1001
USER artpark
