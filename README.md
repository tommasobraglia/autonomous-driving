# ros2_exercises

This repository contains ROS 2 example packages and nodes for learning and experimentation.

---

## Requirements

- Ubuntu 20.04 or 22.04 (recommended)
- ROS 2 Humble Hawksbill
  - [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- `colcon` build tool installed
- `rosdep` installed

---

## 📦 General Workflow (for external users)

### 1. Clone the repository

```bash
$ git clone https://github.com/tommasobraglia ros2_exercises.git
$ cd ros2_exercises/test1/ros2_ws
```

### 2. Install ROS 2
Install the appropriate ROS 2 distribution following the official guide:
📄 https://docs.ros.org/en/humble/Installation.html

### 3. Install package dependencies
sudo apt update <br>
rosdep update <br>
rosdep install --from-paths src --ignore-src -r -y

### 4. Build the workspace
colcon build    

### 5. Source the workspace
source install/setup.bash

### 6. Run a node
To execute a compiled node:
```bash
$ ros2 run my_package my_node
#Replace my_package and my_node with the appropriate names.
```

### 7. Rebuild after making changes
If you edit code in any package (e.g., src/my_node.cpp):
```bash
$ colcon build --packages-select my_package
$ source install/setup.bash
$ ros2 run my_package my_node
```