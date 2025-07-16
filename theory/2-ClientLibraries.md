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
## Create your own package
Check the documentation for a Python/C++ example.<br>
colcon uses the package.xml specification.<br>
colcon supports multiple build types.<br> 
The recommended build types are ament_cmake and ament_python. Also supported are pure cmake packages.<br>
For convenience, you can use the tool ros2 pkg create to create a new package based on a template. More on that in a later chapter.
## Setup colcon_cd
The command colcon_cd allows you to quickly change the current working directory of your shell to the directory of a package.<br>
As an example colcon_cd some_ros_package would quickly bring you to the directory ~/ros2_ws/src/some_ros_package.<br>
To set up colcon_cd you need to run the following commands to modify your shell startup script:
```bash
#To run tests for the packages we just built:
$ echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
$ echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc
'''
Depending on the way you installed colcon_cd and where your workspace is, the instructions above may vary, please refer to the documentation for more details. To undo this in Linux and macOS, locate your system’s shell startup script and remove the appended source and export commands.
'''
```
## Setup colcon tab completion
https://colcon.readthedocs.io/en/released/user/installation.html#enable-completion
## Tips
1. If you do not want to build a specific package, then place an empty file named COLCON_IGNORE in the directory and it will not be indexed.
2. If you want to avoid configuring and building tests in CMake packages you can pass: --cmake-args -DBUILD_TESTING=0.
3. If you want to run a single particular test from a package:
```bash
$ colcon test --packages-select YOUR_PKG_NAME --ctest-args -R YOUR_TEST_IN_PKG
```
## Setup colcon mixins
Various command line options are tedious to write and/or difficult to remember.<br>
To make common command line options easier to invoke this repository makes these “shortcuts” available.<br>
To install the default colcon mixins, run the following:
```bash
$ colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
$ colcon mixin update default

# Then, try out using the debug mixin:
# $colcon build --mixin debug
```

---
# Creating a workspace
Continue...