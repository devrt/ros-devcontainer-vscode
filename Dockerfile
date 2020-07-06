ARG BASE_IMAGE=osrf/ros:melodic-desktop

FROM maven AS xsdcache

# install schema-fetcher
RUN yum -y update && \
    yum -y install git && \
    git clone --depth=1 https://github.com/mfalaize/schema-fetcher.git && \
    cd schema-fetcher && \
    mvn install

# fetch XSD file for package.xml
RUN mkdir -p /opt/xsd/package.xml && \
    java -jar schema-fetcher/target/schema-fetcher-1.0.0-SNAPSHOT.jar /opt/xsd/package.xml http://download.ros.org/schema/package_format2.xsd

# fetch XSD file for roslaunch
RUN mkdir -p /opt/xsd/roslaunch && \
    java -jar schema-fetcher/target/schema-fetcher-1.0.0-SNAPSHOT.jar /opt/xsd/roslaunch https://gist.githubusercontent.com/nalt/dfa2abc9d2e3ae4feb82ca5608090387/raw/roslaunch.xsd

# fetch XSD files for SDF
RUN mkdir -p /opt/xsd/sdf && \
    java -jar schema-fetcher/target/schema-fetcher-1.0.0-SNAPSHOT.jar /opt/xsd/sdf http://sdformat.org/schemas/root.xsd && \
    sed -i 's|http://sdformat.org/schemas/||g' /opt/xsd/sdf/*

# fetch XSD file for URDF
RUN mkdir -p /opt/xsd/urdf && \
    java -jar schema-fetcher/target/schema-fetcher-1.0.0-SNAPSHOT.jar /opt/xsd/urdf https://raw.githubusercontent.com/devrt/urdfdom/xsd-with-xacro/xsd/urdf.xsd

FROM $BASE_IMAGE

MAINTAINER Yosuke Matsusaka <yosuke.matsusaka@gmail.com>

RUN useradd -m developer

# need to renew the key for some reason
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# workaround to enable bash completion for apt-get
# see: https://github.com/tianon/docker-brew-ubuntu-core/issues/75
RUN rm /etc/apt/apt.conf.d/docker-clean

RUN apt-get update && \
    apt-get install -y apt-transport-https && \
    apt-get clean

# OSRF distribution is better for gazebo
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    curl -L http://packages.osrfoundation.org/gazebo.key | apt-key add -

# nice to have nodejs for web goodies
RUN sh -c 'echo "deb https://deb.nodesource.com/node_10.x `lsb_release -cs` main" > /etc/apt/sources.list.d/nodesource.list' && \
    curl -sSL https://deb.nodesource.com/gpgkey/nodesource.gpg.key | apt-key add -

RUN apt-get update && \
    apt-get install -y bash-completion less wget vim-tiny iputils-ping net-tools git openjdk-8-jdk-headless nodejs sudo byzanz && \
    npm install -g yarn && \
    echo developer ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/developer && \
    chmod 0440 /etc/sudoers.d/developer && \
    apt-get clean

# basic python packages
RUN if [ $(lsb_release -cs) = "focal" ]; then apt-get install -y python-is-python3; fi && \
    curl -kL https://bootstrap.pypa.io/get-pip.py | python && \
    pip install --upgrade --ignore-installed --no-cache-dir pylint==1.9.4 autopep8 python-language-server[all] notebook~=5.7 Pygments matplotlib ipywidgets jupyter_contrib_nbextensions nbimporter supervisor supervisor_twiddler argcomplete

RUN jupyter nbextension enable --py widgetsnbextension && \
    jupyter contrib nbextension install --system

# use closest mirror for apt updates
RUN sed -i -e 's/http:\/\/archive/mirror:\/\/mirrors/' -e 's/http:\/\/security/mirror:\/\/mirrors/' -e 's/\/ubuntu\//\/mirrors.txt/' /etc/apt/sources.list

RUN mkdir -p /etc/supervisor/conf.d
COPY .devcontainer/supervisord.conf /etc/supervisor/supervisord.conf
COPY .devcontainer/theia.conf /etc/supervisor/conf.d/theia.conf
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

# colorize less
RUN echo "export LESS='-R'" >> ~/.bash_profile && \
    echo "export LESSOPEN='|pygmentize -g %s'" >> ~/.bash_profile

# enable bash completion
RUN git clone --depth=1 https://github.com/Bash-it/bash-it.git ~/.bash_it && \
    ~/.bash_it/install.sh --silent && \
    rm ~/.bashrc.bak && \
    echo "source /usr/share/bash-completion/bash_completion" >> ~/.bashrc && \
    bash -i -c "bash-it enable completion git"

RUN echo 'eval "$(register-python-argcomplete sim)"' >> ~/.bashrc

RUN git clone --depth 1 https://github.com/junegunn/fzf.git ~/.fzf && \
    ~/.fzf/install --all

RUN git clone --depth 1 https://github.com/b4b4r07/enhancd.git ~/.enhancd && \
    echo "source ~/.enhancd/init.sh" >> ~/.bashrc

# global vscode config
ADD .vscode /home/developer/.vscode
ADD .vscode /home/developer/.theia
ADD .devcontainer/compile_flags.txt /home/developer/compile_flags.txt
ADD .devcontainer/templates /home/developer/templates
RUN sudo chown -R developer:developer /home/developer

# install theia web IDE
COPY .devcontainer/theia-latest.package.json /home/developer/package.json
RUN yarn --cache-folder ./ycache && rm -rf ./ycache && \
    NODE_OPTIONS="--max_old_space_size=4096" yarn theia build ;\
    yarn theia download:plugins

ENV THEIA_DEFAULT_PLUGINS local-dir:/home/developer/plugins

# enable jupyter extensions
RUN jupyter nbextension enable hinterland/hinterland && \
    jupyter nbextension enable toc2/main && \
    jupyter nbextension enable code_prettify/autopep8

# enter ROS world
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

EXPOSE 3000 8888

ENTRYPOINT [ "/entrypoint.sh" ]
CMD [ "sudo", "-E", "/usr/local/bin/supervisord", "-c", "/etc/supervisor/supervisord.conf"]
