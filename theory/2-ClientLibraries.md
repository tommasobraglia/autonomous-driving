# Using colcon to build packages
colcon is an iteration on the ROS build tools catkin_make, catkin_make_isolated, catkin_tools and ament_tools.<br>

We must first install it:
```bash
$ sudo apt install python3-colcon-common-extensions
```
## Basics
A ROS workspace is a directory with a particular structure.<br> Commonly there is a src subdirectory.<br>
Inside that subdirectory is where the source code of ROS packages will be located. Typically the directory starts otherwise empty.<br>
colcon performs out-of-source builds.<br>
By default it will create the following directories as peers of the src directory:
- The build directory will be where intermediate files are stored.<br> 
For each package a subfolder will be created in which e.g. CMake is being invoked.
- The install directory is where each package will be installed to.<br>
By default each package will be installed into a separate subdirectory.
- The log directory contains various logging information about each colcon invocation.
### Create a workspace
```bash
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws
```