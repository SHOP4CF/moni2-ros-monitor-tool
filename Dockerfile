#FROM robotgit.localdom.net:5000/ros2/foxy-onbuild
FROM foxy-onbuild

LABEL version="0.0.0"
LABEL ros.distro=${ROS_DISTRO}
LABEL vcs-url="http://robotgit.localdom.net/ai-box/applications/moni2"
LABEL maintainer="Rasmus Lunding Henriksen <rlh@teknologisk.dk>"
LABEL description="This image will automatically start the moni2 application."

ENV PACKAGE_NAME="moni2"
ENV LAUNCHFILE_NAME="moni2.launch.py"

RUN adduser --quiet --disabled-password qtuser
