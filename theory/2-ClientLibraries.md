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

#Then, clone the examples into the src directory of the workspace
```
It is important that we have sourced the environment for an existing ROS 2 installation that will provide our workspace with the necessary build dependencies for the example packages.<br>  
This is achieved by sourcing the setup script provided by a binary installation or a source installation, i.e. another colcon workspace (see Installation).<br>
We call this environment an underlay.
Our workspace, ros2_ws, will be an overlay on top of the existing ROS 2 installation.<br> 
In general, it is recommended to use an overlay when you plan to iterate on a small number of packages, rather than putting all of your packages into the same workspace.
### Build the workspace
In the root of the workspace, run colcon build.
```bash
# Source source /opt/ros/humble/setup.bash, then:
$ colcon build --executor sequential --symlink-install
# Check what this does on the documentation if needed
# After the build is finished, we should see the build, install, and log directories
```
### Run tests
```bash
#To run tests for the packages we just built:
$ colcon test
```
### Source the environment
When colcon has completed building successfully, the output will be in the install directory.<br> 
Before you can use any of the installed executables or libraries, you will need to add them to your path and library paths.<br>  
colcon will have generated bash/bat files in the install directory to help set up the environment.<br> 
These files will add all of the required elements to your path and library paths as well as provide any bash or shell commands exported by packages.
```bash
#To run tests for the packages we just built:
$ source install/setup.bash
```