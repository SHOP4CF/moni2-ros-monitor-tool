# FROM robotgit.localdom.net:5000/ros2/eloquent-onbuild as common_base
FROM eloquent-onbuild:0.0.9

LABEL version="0.0.5"
LABEL ros.distro=${ROS_DISTRO}
LABEL vcs-url="http://robotgit.localdom.net/docker/ros-2/examples/chat_py"
LABEL maintainer="Rasmus Lunding Henriksen <rlh@teknologisk.dk>"
LABEL description="This image will automatically start a listener (implemented in python)."

ENV PACKAGE_NAME="moni2"
ENV LAUNCHFILE_NAME="moni2.launch.py"
