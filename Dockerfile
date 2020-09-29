# Allow for dynamic base image during build
FROM osrf/ros:foxy-desktop

# use bash here and in the build-pipeline
SHELL ["bash", "-c"]

# Install depedencies
RUN apt-get -q2 update && apt-get install -y python3-wstool
RUN rosdep update

# How to start ROS2 code when we run a container
COPY entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]

# Internal arguments
ARG WS_PATH=/ws
ARG WS_SRC_PATH=${WS_PATH}/src

# Copy workspace
COPY . ${WS_SRC_PATH}/repo_contents

# Install package dependencies
RUN wstool init ${WS_SRC_PATH} ${WS_SRC_PATH}/repo_contents/src_dependencies.rosinstall
RUN rosdep install -q --default-yes --ignore-packages-from-source --from-path ${WS_SRC_PATH}

# Build project
RUN echo "Building packages..."
WORKDIR ${WS_PATH}
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build

LABEL version="0.0.0"
LABEL ros.distro=${ROS_DISTRO}
LABEL vcs-url="http://robotgit.localdom.net/ai-box/applications/moni2"
LABEL maintainer="Rasmus Lunding Henriksen <rlh@teknologisk.dk>"
LABEL description="This image will automatically start the moni2 application."

ENV PACKAGE_NAME="moni2"
ENV LAUNCHFILE_NAME="moni2.launch.py"

RUN adduser --quiet --disabled-password qtuser

# Needed for QT to be testable
# ENV QT_QPA_PLATFORM=offscreen
# ENV XDG_RUNTIME_DIR=/tmp