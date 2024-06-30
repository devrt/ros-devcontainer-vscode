ARG BASE_IMAGE=ros:noetic

FROM maven:3.8 AS xsdcache

# install schema-fetcher
RUN apt update && \
    apt install -y git && \
    git clone --depth=1 https://github.com/mfalaize/schema-fetcher.git && \
    cd schema-fetcher && \
    sed -i 's|>1.7<|>1.8<|g' pom.xml && \
    mvn install

# fetch XSD file for package.xml
RUN mkdir -p /opt/xsd/package.xml && \
    java -jar schema-fetcher/target/schema-fetcher-1.0.0-SNAPSHOT.jar /opt/xsd/package.xml http://download.ros.org/schema/package_format2.xsd

# fetch XSD file for roslaunch
RUN mkdir -p /opt/xsd/roslaunch && \
    java -jar schema-fetcher/target/schema-fetcher-1.0.0-SNAPSHOT.jar /opt/xsd/roslaunch https://gist.githubusercontent.com/nalt/dfa2abc9d2e3ae4feb82ca5608090387/raw/roslaunch.xsd

# fetch XSD file for ros2launch
RUN mkdir -p /opt/xsd/ros2launch && \
    java -jar schema-fetcher/target/schema-fetcher-1.0.0-SNAPSHOT.jar /opt/xsd/ros2launch https://raw.githubusercontent.com/ros2/design/gh-pages/articles/specs/launch.0.1.1.xsd

# fetch XSD files for SDF
RUN mkdir -p /opt/xsd/sdf && \
    java -jar schema-fetcher/target/schema-fetcher-1.0.0-SNAPSHOT.jar /opt/xsd/sdf http://sdformat.org/schemas/root.xsd && \
    sed -i 's|http://sdformat.org/schemas/||g' /opt/xsd/sdf/*

# fetch XSD file for URDF
RUN mkdir -p /opt/xsd/urdf && \
    java -jar schema-fetcher/target/schema-fetcher-1.0.0-SNAPSHOT.jar /opt/xsd/urdf https://raw.githubusercontent.com/devrt/urdfdom/xsd-with-xacro/xsd/urdf.xsd

FROM $BASE_IMAGE

MAINTAINER Yosuke Matsusaka <yosuke.matsusaka@gmail.com>

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive

# workaround to enable bash completion for apt-get
# see: https://github.com/tianon/docker-brew-ubuntu-core/issues/75
RUN rm /etc/apt/apt.conf.d/docker-clean

# use closest mirror for apt updates
RUN sed -i -e 's/http:\/\/archive/mirror:\/\/mirrors/' -e 's/http:\/\/security/mirror:\/\/mirrors/' -e 's/\/ubuntu\//\/mirrors.txt/' /etc/apt/sources.list

RUN apt-get update || true && \
    apt-get install -y curl apt-transport-https ca-certificates && \
    apt-get clean

# need to renew the key for some reason
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# OSRF distribution is better for gazebo
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    curl -L http://packages.osrfoundation.org/gazebo.key | apt-key add -

# nice to have nodejs for web goodies
RUN sh -c 'echo "deb https://deb.nodesource.com/node_14.x `lsb_release -cs` main" > /etc/apt/sources.list.d/nodesource.list' && \
    curl -sSL https://deb.nodesource.com/gpgkey/nodesource.gpg.key | apt-key add -

# vscode
RUN curl -sSL https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg && \
    install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg && \
    sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list' && \
    rm -f packages.microsoft.gpg

# install depending packages
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y bash-completion less wget language-pack-en code vim-tiny iputils-ping net-tools openssh-client git openjdk-8-jdk-headless nodejs sudo imagemagick byzanz python-dev libsecret-1-dev && \
    npm install -g yarn && \
    apt-get clean

# basic python packages
RUN if [ $(lsb_release -cs) = "focal" ]; then \
        apt-get update; \
        apt-get install -y python-is-python3 python3-catkin-tools python3-colcon-common-extensions; \
        apt-get clean; \
        curl -kL https://bootstrap.pypa.io/get-pip.py | python; \
    else \
        curl -kL https://bootstrap.pypa.io/pip/2.7/get-pip.py | python; \
    fi && \
    pip install --upgrade --ignore-installed --no-cache-dir pyassimp pylint==1.9.4 autopep8 python-language-server[all] notebook~=5.7 Pygments matplotlib ipywidgets jupyter_contrib_nbextensions nbimporter supervisor supervisor_twiddler argcomplete

# jupyter extensions
RUN jupyter nbextension enable --py widgetsnbextension && \
    jupyter contrib nbextension install --system

# add non-root user
RUN useradd -m developer && \
    echo developer ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/developer && \
    chmod 0440 /etc/sudoers.d/developer

# install depending packages (install moveit! algorithms on the workspace side, since moveit-commander loads it from the workspace)
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y ros-$ROS_DISTRO-desktop ros-$ROS_DISTRO-gazebo-msgs ros-$ROS_DISTRO-moveit ros-$ROS_DISTRO-moveit-commander ros-$ROS_DISTRO-moveit-ros-visualization ros-$ROS_DISTRO-trac-ik ros-$ROS_DISTRO-move-base-msgs ros-$ROS_DISTRO-ros-numpy && \
    apt-get clean

# temporal hack to fix ros_numpy problem
RUN if [ $(lsb_release -cs) = "focal" ]; then \
        curl -sSL https://raw.githubusercontent.com/eric-wieser/ros_numpy/master/src/ros_numpy/point_cloud2.py > /opt/ros/$ROS_DISTRO/lib/python3/dist-packages/ros_numpy/point_cloud2.py; \
    fi

# install bio_ik
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    mkdir -p /bio_ik_ws/src && \
    cd /bio_ik_ws/src && \
    catkin_init_workspace && \
    git clone --depth=1 https://github.com/TAMS-Group/bio_ik.git && \
    cd .. && \
    catkin_make install -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO -DCATKIN_ENABLE_TESTING=0 && \
    cd / && rm -r /bio_ik_ws

# configure services
RUN mkdir -p /etc/supervisor/conf.d
COPY .devcontainer/supervisord.conf /etc/supervisor/supervisord.conf
COPY .devcontainer/code-server.conf /etc/supervisor/conf.d/code-server.conf
COPY .devcontainer/jupyter.conf /etc/supervisor/conf.d/jupyter.conf

COPY .devcontainer/entrypoint.sh /entrypoint.sh

COPY .devcontainer/sim.py /usr/bin/sim

COPY --from=xsdcache /opt/xsd /opt/xsd

USER developer
WORKDIR /home/developer

ENV HOME /home/developer
ENV SHELL /bin/bash

# jre is required to use XML editor extension
ENV JAVA_HOME /usr/lib/jvm/java-8-openjdk-amd64

# enable bash completion
RUN git clone --depth=1 https://github.com/Bash-it/bash-it.git ~/.bash_it && \
    ~/.bash_it/install.sh --silent && \
    rm ~/.bashrc.bak && \
    echo "source /usr/share/bash-completion/bash_completion" >> ~/.bashrc && \
    bash -i -c "bash-it enable completion git"

RUN echo 'eval "$(register-python-argcomplete sim)"' >> ~/.bashrc

RUN git clone --depth 1 https://github.com/junegunn/fzf.git ~/.fzf && \
    ~/.fzf/install --all

#RUN git clone --depth 1 https://github.com/b4b4r07/enhancd.git ~/.enhancd && \
#    echo "source ~/.enhancd/init.sh" >> ~/.bashrc

# colorize less
RUN lesspipe >> ~/.bashrc && \
    echo "export LESS='-R'" >> ~/.bashrc && \
    echo "export PYGMENTIZE_STYLE='monokai'" >> ~/.bashrc && \
    curl -sSL https://raw.githubusercontent.com/CoeJoder/lessfilter-pygmentize/master/.lessfilter > ~/.lessfilter && \
    chmod 755 ~/.lessfilter

# init rosdep
RUN rosdep update

# global vscode config
ADD .vscode /home/developer/.vscode
RUN ln -s /home/developer/.vscode /home/developer/.vscode-server
ADD .devcontainer/compile_flags.txt /home/developer/compile_flags.txt
ADD .devcontainer/templates /home/developer/templates
RUN sudo chown -R developer:developer /home/developer

RUN code --install-extension ms-python.python && \
    code --install-extension ms-vscode.cpptools-extension-pack && \
    code --install-extension redhat.vscode-xml

# enable jupyter extensions
RUN jupyter nbextension enable hinterland/hinterland && \
    jupyter nbextension enable toc2/main && \
    jupyter nbextension enable code_prettify/autopep8 && \
    jupyter nbextension enable nbTranslate/main && \
    mkdir -p /home/developer/.ipython/profile_default && \
    echo "c.Completer.use_jedi = False" >> /home/developer/.ipython/profile_default/ipython_kernel_config.py

# ROS goodies
RUN echo "alias rte='rostopic list | fzf --preview \"rostopic echo -c {}\"'" >> ~/.bashrc
RUN echo "alias rti='rostopic list | fzf --preview \"rostopic info {}\"'" >> ~/.bashrc
RUN echo "alias rni='rosnode list | fzf --preview \"rosnode info {}\"'" >> ~/.bashrc
RUN echo "alias rsi='rosservice list | fzf --preview \"rosservice info {}\"'" >> ~/.bashrc
RUN echo "alias rmi='rosmsg list | fzf --preview \"rosmsg info {}\"'" >> ~/.bashrc
RUN echo "alias rcd='roscd \$(rospack list-names | fzf --preview=\"rospack find {} && pygmentize \\\$(rospack find {})/package.xml\")'" >> ~/.bashrc

# enter ROS world
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

EXPOSE 3000 8888

ENTRYPOINT [ "/entrypoint.sh" ]
CMD [ "sudo", "-E", "/usr/local/bin/supervisord", "-c", "/etc/supervisor/supervisord.conf"]
