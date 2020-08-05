# Moni2
a ROS2/Python3/Qt5 monitoring tool

## Requirements
* Python >= 3.6
* ROS2 foxy
* PyQt5

## Build
* **Native ROS2**: create a workspace, include this package and run `colcon build`.
* **Docker**: Run the following command:
  ```
  $ cd [somepath]/moni2
  $ docker build -t moni2:latest .
  ```

## Run
* **Native ROS2**:  
  1. `ros2 run moni2 moni2`
  2. `ros2 launch moni2 moni2.launch.py`
* **Docker**: 
  1. `docker run -it --rm moni2`
  2. `docker run -it --rm robotgit.localdom.net:5000/aibox/moni2:latest`
  
## Configuration
TODO