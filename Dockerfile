FROM osrf/ros:kinetic-desktop

MAINTAINER Yosuke Matsusaka <yosuke.matsusaka@gmail.com>

RUN useradd -m developer

# need to renew the key for some reason
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# workaround to enable bash completion for apt-get
# see: https://github.com/tianon/docker-brew-ubuntu-core/issues/75
RUN rm /etc/apt/apt.conf.d/docker-clean

# OSRF distribution is better for gazebo
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    curl -L http://packages.osrfoundation.org/gazebo.key | apt-key add -

RUN apt-get update && \
    apt-get install -y bash-completion less wget dos2unix vim-tiny clang clang-format python-pip python3-pip openjdk-8-jdk-headless sudo && \
    echo developer ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/developer && \
    chmod 0440 /etc/sudoers.d/developer && \
    apt-get clean

COPY entrypoint.sh /entrypoint.sh

# workaround for git on windows may copy .sh file in crlf line endings
RUN dos2unix /entrypoint.sh && chmod 755 /entrypoint.sh

USER developer
WORKDIR /home/developer

ENV HOME /home/developer
ENV SHELL /bin/bash

# clang shows readable compile error
ENV CC /usr/bin/clang
ENV CXX /usr/bin/clang++

# jre is required to use XML editor extension
ENV JAVA_HOME /usr/lib/jvm/java-8-openjdk-amd64

# enable bash completion
RUN git clone --depth=1 https://github.com/Bash-it/bash-it.git ~/.bash_it && \
    ~/.bash_it/install.sh --silent && \
    rm ~/.bashrc.bak && \
    echo "source /usr/share/bash-completion/bash_completion" >> ~/.bashrc && \
    bash -i -c "bash-it enable completion pip"

# basic python packages
RUN pip3 install jedi autopep8 pylint

# enter ROS world
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

ENTRYPOINT [ "/entrypoint.sh" ]
CMD [ "bash" ]
