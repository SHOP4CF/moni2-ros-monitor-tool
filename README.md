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
  1. `docker run -it --rm -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY -u qtuser moni2`
  2. `docker run -it --rm -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY -u qtuser robotgit.localdom.net:5000/aibox/moni2:latest`

## How to use
The program will look similar to this when you run it:
![gui](resource/images/gui.png)

### Configuration
1. **Create a new configuration** (i.e. select which nodes to monitor):
   * Press `File` (in the menu) and then `New`
   * or press `[ctrl]+[N]`  
   You will see a dialog similar to this:
   ![config](resource/images/config.png)
2. **Select nodes**
   * "Online nodes" list contains all current online nodes.
      * these can be dragged to the "Selected node" list.
      * the list should automatically update every 3rd second.
   * "Selected node" list contains the nodes that will be monitored.
      * you can add nodes by dragging from the "Online nodes" or manually add nodes using the input field to the right.
      * you can drag nodes from the trash-can if you wish to delete them.
3. **Save configuration**
   * You will be asked to save your configuration somewhere (it is `.json`).
4. **Have fun!**
5. **Edit**
   * You can edit your configuration:
     * Press `File` (in the menu) and then `Edit`
     * or press `[ctrl]+[E]`

### Settings
To open settings:
* Press `File` (in the menu) and then `Settings`
* or press `[ctrl][alt]+[S]`  
![settings](resource/images/settings.png)
