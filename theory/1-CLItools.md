# Configuring environment
## Background
ROS 2 relies on the notion of combining workspaces using the shell environment. 
**“Workspace”** is a ROS term for the location on your system where you’re developing with ROS 2. The core ROS 2 workspace is called the **underlay**. 
Subsequent local workspaces are called **overlays**. 
When developing with ROS 2, you will typically have several workspaces active concurrently.

Combining workspaces makes developing against different versions of ROS 2, or against different sets of packages, easier.

This is accomplished by *sourcing setup files every time you open a new shell*, or by adding the source command to your shell startup script once. Without sourcing the setup files, you won’t be able to access ROS 2 commands, or find or use ROS 2 packages. 
In other words, you won’t be able to use ROS 2.

## Tasks
### 1. Source the setup files
You will need to run this command on every new shell to have access to the ROS 2 commands:
```bash
$ source /opt/ros/humble/setup.bash
```
### 2. Add sourcing to your shell startup script
If you want to skip task 1, then you can add the command to your shell startup script:
```bash
$ echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
To undo this, locate your system's shell startup script and remove the appended source command.
### 3. Check environment variables
Sourcing ROS 2 setup files will set several environment variables necessary for operating ROS 2. If you ever have problems finding or using your ROS 2 packages, make sure that your environment is properly set up using the following command:
```bash
$ printenv | grep -i ROS
```
Check that variables like ROS_DISTRO and ROS_VERSION are set.
If the environment variables are not set correctly, return to the ROS 2 package installation section of the installation guide you followed.
#### 3.1 The ROS_DOMAIN_ID variable
The default middleware that ROS 2 uses for communication is DDS (read about it on ChatGPT).
In DDS, the primary mechanism for having different logical networks share a physical network is known as the Domain ID. 
ROS 2 nodes on the same domain can freely discover and send messages to each other.
ROS 2 nodes on different domains cannot. 
All ROS 2 nodes use domain ID 0 by default. 
To avoid interference between different groups of computers running ROS 2 on the same network, a different domain ID should be set for each group.
##### Choosing a domain ID (short version)
Simply choose a domain ID between 0 and 101, inclusive.
##### Choosing a domain ID (long version)
Simply choose a domain ID between 0 and 101, inclusive.
The domain ID is used by DDS to compute the UDP ports that will be used for discovery and communication.
Remembering our basic networking, the UDP port is an unsigned 16-bit integer. Thus, the highest port number that can be allocated is 65535.
The highest domain ID that can possibly be assigned is 232, while the lowest that can be assigned is 0.<br>
https://community.rti.com/content/forum-topic/statically-configure-firewall-let-omg-dds-traffic-through
https://en.wikipedia.org/wiki/User_Datagram_Protocol#Ports
###### Platform-specific constraints
For maximum compatibility, some additional platform-specific constraints should be followed when choosing a domain ID. 
In particular, it is best to avoid allocating domain IDs in the operating system’s ephemeral port range. This avoids possible conflicts between the ports used by the ROS 2 nodes and other networking services on the computers.<br>What's an ephemeral port: https://en.wikipedia.org/wiki/Ephemeral_port
>By default, the Linux kernel uses ports 32768-60999 for ephemeral ports. 
>This means that domain IDs 0-101 and 215-232 can be safely used without colliding with ephemeral ports. 
>The ephemeral port range is configurable in Linux by setting custom values in /proc/sys/net/ipv4/ip_local_port_range. 
>If a custom ephemeral port range is used, the above numbers may have to be adjusted accordingly.
###### Participant constraints
For each ROS 2 process running on a computer, one DDS “participant” is created. 
Since each DDS participant takes up two ports on the computer, running more than 120 ROS 2 processes on one computer may spill over into other domain IDs or the ephemeral ports.
Check example: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Domain-ID.html#id3

Once you have determined a unique integer for your group of ROS 2 nodes, you can set the environment variable with the following command: (this must be done in every new shell, like the sourcing of the setup files)
```bash
$ export ROS_DOMAIN_ID=<your_domain_id>
```
To maintain this setting between shell sessions, you can add the command to your shell startup script:
```bash
$ echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bashrc
```
#### 3.2 The ROS_LOCALHOST_ONLY variable
By default, ROS 2 communication is not limited to localhost. 
ROS_LOCALHOST_ONLY environment variable allows you to limit ROS 2 communication to localhost only. This means your ROS 2 system, and its topics, services, and actions will not be visible to other computers on the local network.Using ROS_LOCALHOST_ONLY is helpful in certain settings, such as classrooms, where multiple robots may publish to the same topic causing strange behaviors. <br>
You can set the environment variable with the following command:
```bash
$ export ROS_LOCALHOST_ONLY=1
```
```bash
$ echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc 
```

---

# Using turtlesim, ros2 and rqt
**Turtlesim** is a lightweight simulator for learning ROS 2. It illustrates what ROS 2 does at the most basic level to give you an idea of what you will do with a real robot or a robot simulation later on.<br>
The ros2 tool is how the user manages, introspects, and interacts with a ROS system. It supports multiple commands that target different aspects of the system and its operation.<br>
One might use it to start a node, set a parameter, listen to a topic, and many more.

**rqt** is a graphical user interface (GUI) tool for ROS 2.<br>
Everything done in rqt can be done on the command line, but rqt provides a more user-friendly way to manipulate ROS 2 elements.
## Tasks
### 1. Install turtlesim
As always, start by sourcing your setup files in a new terminal.<br>
Check the Tasks chapter above.<br>
Then, install the turtlesim package for your ROS 2 distro:
```bash
$ sudo apt update
$ sudo apt install ros-humble-turtlesim
```
To check if the package is installed, check if the following command returns a list of executables:
```bash
$ ros2 pkg executables turtlesim
```
### 2. Start turtlesim
To start turtlesim:
```bash
$ ros2 run turtlesim turtlesim_node
```
Under the command, you will see messages from the node. 
There you can see the default turtle’s name and the coordinates where it spawns.
### 3. Use turtlesim
Open a new terminal and source ROS 2 again.<br>
Now you will run a new node to control the turtle in the first node:
```bash
$ ros2 run turtlesim turtle_teleop_key
```
At this point you should have three windows open: a terminal running turtlesim_node, a terminal running turtle_teleop_key and the turtlesim window.<br>
A quick note: pressing an arrow key will only cause the turtle to move a short distance and then stop. This is because, realistically, you wouldn’t want a robot to continue carrying on an instruction if, for example, the operator lost the connection to the robot.<br>
You can see the nodes, and their associated topics, services, and actions, using the list subcommands of the respective commands:
```bash
$ ros2 node list
$ ros2 topic list
$ ros2 service list
$ ros2 action list
#we'll talk about these later
#You don't have to know what those are yet!
```
### 4. Install rqt
We must install rqt to be able to use it:
```bash
$ sudo apt update
$ sudo apt install '~nros-humble-rqt*'

#You can then run it:
$ rqt
```
### 5. Use rqt
When running rqt for the first time, the window will be blank.<br>
No worries; just select Plugins > Services > Service Caller from the menu bar at the top.<br>
It may take some time for rqt to locate all the plugins. If you click on Plugins but don’t see Services or any other options, you should close rqt and enter the command rqt --force-discover in your terminal.<br>
Click on the Service dropdown list to see turtlesim’s services, and select the /spawn service.
#### 5.1 Try the spawn service
use rqt to call the /spawn service.<br>
/spawn will create another turtle in the turtlesim window.<br>
Give the new turtle a unique name and set its spawn cordinates. Then call the service by clicking the call button on the upper right side of the rqt window.<br>
If you refresh the service list in rqt, you'll see that now there are services related to the new turtle.
#### 5.2 Try the set_pen service
Let's give turtle1 a unique pen using the /set_pen service.
You can set the color of the pen and the width.<br>
Don't forget to Call the service when you're done tweaking the parameters.
### 6. Remapping
You need a second teleop node in order to control turtle2.<br>
However, if you try to run the same command as before, you will notice that this one also controls turtle1. The way to change this behavior is by remapping the cmd_vel topic.
In a new terminal, source ROS 2, and run:
```bash
$ ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel
```
Now, you can move turtle2 when this terminal is active, and turtle1 when the other terminal running turtle_teleop_key is active.
### 7. Close turtlesim
Ctrl+C, and q in the turtle_teleop_key terminals.

---

# Understanding nodes
## Background
### 1. The ROS 2 graph
The ROS graph is a network of ROS 2 elements processing data together at the same time.<br>
It encompasses all executables and the connections between them if you were to map them all out and visualize them.
### 2. Nodes in ROS 2
A node is a fundamental ROS 2 element that serves a single, modular purpose in a robotics system.<br>
Each node in ROS should be responsible for a single, modular purpose, e.g. controlling the wheel motors or publishing the sensor data from a laser range-finder.<br> 
Each node can send and receive data from other nodes via topics, services, actions, or parameters.<br>
A full robotic system is comprised of many nodes working in concert.<br>
In ROS 2, a single executable (C++ program, Python program, etc.) can contain one or more nodes.<br>

<img src="https://docs.ros.org/en/humble/_images/Nodes-TopicandService.gif" width="500">

## Tasks
### 1. ros2 run
The command *ros2 run* launches an executable from a package.
```bash
$ ros2 run <package_name> <executable_name>

#For example, in: ros2 run turtlesim turtlesim_node
#Here, the package name is turtlesi, and the executable name is turtlesim_node
#We still don't know the node name. Node names can be found by using: ros2 node list
```
### 2. ros2 node list
*ros2 node list* will show you the names of all running nodes.<br>
This is especially useful when you want to interact with a node, or when you have a system running many nodes and need to keep track of them.<br>
Open a new terminal while turtlesim is still running in the other one, and enter the following command. 
```bash
$ ros2 node list
#The terminal will return the node name
#If you startL ros2 run turtlesim turtle_teleop_key, it will be shown in the nodes list
```
#### 2.1 Remapping
Remapping allows you to reassign default node properties, like node name, topic names, service names, etc., to custom values.<br>
In the last tutorial, you used remapping on turtle_teleop_key to change the cmd_vel topic and target turtle2.<br>
Let’s reassign the name of our /turtlesim node. In a new terminal, run the following command:
```bash
$ ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
```
If you return to the terminal where you ran ros2 node list, and run it again, you will see three node names.
### 3. ros2 node info
You can access more information about your nodes with: 
```bash
$ ros2 node info <node_name>
```
ros2 node info returns a list of subscribers, publishers, services, and actions. i.e. the ROS graph connections that interact with that node.<br>
Try running the same command on other nodes, and see how its connections differ from my_turtle.

---

# Understanding topics
ROS 2 breaks complex systems down into many modular nodes.<br> 
Topics are a vital element of the ROS graph that act as a bus for nodes to exchange messages.

<img src="https://docs.ros.org/en/humble/_images/Topic-SinglePublisherandSingleSubscriber.gif" width="500">

A node may publish data to any number of topics and simultaneously have subscriptions to any number of topics.

<img src="https://docs.ros.org/en/humble/_images/Topic-MultiplePublisherandMultipleSubscriber.gif" width="500">

Topics are one of the main ways in which data is moved between nodes and therefore between different parts of the system.
## Tasks
### 1. Setup
The usual steps:
```bash
$ ros2 run turtlesim turtlesim_node
#open another terminal and run:
$ ros2 run turtlesim turtle_teleop_key
```
### 2. rqt_graph
rqt_graph lets you visualize the changing nodes and topics, as well as the connections between them.<br>
To run rqt_graph, open a new terminal and enter the command:
```bash
$ rqt_graph

#You can also open rqt_graph by opening rqt and selecting Plugins > Introspection > Node Graph
```
The graph is depicting how the /turtlesim node and the /teleop_turtle node are communicating with each other over a topic. <br>
The /teleop_turtle node is publishing data (the keystrokes you enter to move the turtle around) to the /turtle1/cmd_vel topic, and the /turtlesim node is subscribed to that topic to receive the data.<br>
The highlighting feature of rqt_graph is very helpful when examining more complex systems with many nodes and topics connected in many different ways.
### 3. ros2 topic list
If you want a list of all the topics currently active in the system, run the following command:
```bash
$ ros2 topic list
# ros2 topic list -t will return the same list of topics, this time with the topic type appended in brackets
```
These attributes, particularly the type, are how nodes know they’re talking about the same information as it moves over topics.
### 4. ros2 topic echo
To see the data being published on a topic, use:
```bash
$ ros2 topic echo <topic_name>

#Since we know that /teleop_turtle publishes data to /turtlesim over the /turtle1/cmd_vel topic:

$ ros2 topic echo /turtle1/cmd_vel
```
At first, this command won’t return any data. That’s because it’s waiting for /teleop_turtle to publish something.<br>
Return to the terminal where turtle_teleop_key is running and use the arrows to move the turtle around. Watch the terminal where your echo is running at the same time, and you’ll see position data being published for every movement you make.<br>
Now return to rqt_graph and uncheck the Debug box.<br>
/_ros2cli_26646 is the node created by the echo command we just ran (the number might be different).<br>
Now you can see that the publisher is publishing data over the cmd_vel topic, and two subscribers are subscribed to it.
### 5. ros2 topic info
Topics don’t have to only be one-to-one communication; they can be one-to-many, many-to-one, or many-to-many.<br>
Another way to look at this is running:
```bash
$ ros2 topic info /turtle1/cmd_vel
#You could do ros2 topic info for other turtles/things too
```
### 6. ros2 interface show
Nodes send data over topics using messages.<br>
Publishers and subscribers must send and receive the same type of message to communicate.<br>
The topic types we saw earlier after running ros2 topic list -t let us know what message type is used on each topic. Recall that the cmd_vel topic has the type:
```bash
geometry_msgs/msg/Twist
```
This means that in the package geometry_msgs there is a msg called Twist.<br>
Now we can run ros2 interface show msg_type on this type to learn its details. Specifically, what structure of data the message expects.
```bash
$ ros2 interface show geometry_msgs/msg/Twist

# This expresses velocity in free space broken into its linear and angular parts.
    Vector3  linear
            float64 x
            float64 y
            float64 z
    Vector3  angular
            float64 x
            float64 y
            float64 z
            
#The mex that's received is that the /turtlesim node is expecting a message with two vectors, linear and angular, of three elements each.
```
### 7. ros2 topic pub
Now that you have the message structure, you can publish data to a topic directly from the command line using:
```bash
$ ros2 topic pub <topic_name> <msg_type> '<args>' 
```
The '<args>' argument is the actual data you’ll pass to the topic, in the structure you just discovered in the previous section.<br>
The turtle (and commonly the real robots which it is meant to emulate) require a steady stream of commands to operate continuously. <br>
So, to get the turtle moving, and keep it moving, you can use the following command.<br> 
It’s important to note that this argument needs to be input in YAML syntax.<br>
Input the full command like so:
```bash
$ ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

#With no command-line options, ros2 topic pub publishes the command in a steady stream at 1 Hz.
```
At times you may want to publish data to your topic only once (rather than continuously).<br>
To publish your command just once add the --once option:
```bash
$ ros2 topic pub --once -w 2 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

# --once is an optional argument meaning “publish one message then exit”.
# -w 2 is an optional argument meaning “wait for two matching subscriptions”. 
#This is needed because we have both turtlesim and the topic echo subscribed.
```
<br>
You can refresh rqt_graph to see what’s happening graphically.<br>
<img src="https://docs.ros.org/en/humble/_images/rqt_graph2.png" width="500"><br>
Finally, you can run echo on the pose topic and recheck rqt_graph:
```bash
$ ros2 topic echo /turtle1/pose
```
<img src="https://docs.ros.org/en/humble/_images/rqt_graph3.png" width="500">
<br>
You can see that the /turtlesim node is also publishing to the pose topic, which the new echo node has subscribed to.<br>
When publishing messages with timestamps, pub has two methods to automatically fill them out with the current time.<br>
For messages with a std_msgs/msg/Header, the header field can be set to auto to fill out the stamp field.

```bash
$ ros2 topic pub /pose geometry_msgs/msg/PoseStamped '{header: "auto", pose: {position: {x: 1.0, y: 2.0, z: 3.0}}}'

#If the message does not use a full header, but just has a field with type builtin_interfaces/msg/Time, that can be set to the value now.

$ ros2 topic pub /reference sensor_msgs/msg/TimeReference '{header: "auto", time_ref: "now", source: "dumy"}'
```
### 8. ros2 topic hz
You can also view the rate at which data is published using:
```bash
$ ros2 topic hz /turtle1/pose

#It will return data on the rate at which the /turtlesim node is publishing data to the pose topic.
#Recall that you set the rate of turtle1/cmd_vel to publish at a steady 1 Hz using ros2 topic pub --rate 1. 
#If you run the above command with turtle1/cmd_vel instead of turtle1/pose, you will see an average reflecting that rate.
```
### 9. ros2 topic find
To list a list of available topics of a given type use:
```bash
$ ros2 topic find <topic_type>
```
Recall that the cmd_vel topic has the type: geometry_msgs/msg/Twist
Using the find command outputs topics available when given the message type:
```bash
$ ros2 topic find geometry_msgs/msg/Twist
# /turtle1/cmd_vel
```

---

# Understanding services
Services are another method of communication for nodes in the ROS graph.<br>
Services are based on a call-and-response model versus the publisher-subscriber model of topics.<br> 
While topics allow nodes to subscribe to data streams and get continual updates, services only provide data when they are specifically called by a client.

<img src="https://docs.ros.org/en/humble/_images/Service-SingleServiceClient.gif" width="500"><br>

<img src="https://docs.ros.org/en/humble/_images/Service-MultipleServiceClient.gif" width="500"><br>

## Tasks
### 1. Setup
Start up the two turtlesim nodes, /turtlesim and /teleop_turtle. <br>
Then, in a new terminal, after the usual sourcing, run:
```bash
$ ros2 topic turtlesim turtlesim_node
```
In another terminal, run:
```bash
$ ros2 run turtlesim turle_teleop_key
```
### 2. ros2 service list
Running the ros2 service list command in a new terminal will return a list of all the services currently active in the system:
```bash
$ ros2 service list
# /clear
# /kill
# ...
```
You will see that both nodes have the same six services with parameters in their names.<br>
Nearly every node in ROS 2 has these infrastructure services that parameters are built off of.<br>
There will be more about parameters in the next tutorial. In this tutorial, the parameter services will be omitted from the discussion.
### 3. ros2 service type
Services have types that describe how the request and response data of a service is structured.<br>
Service types are defined similarly to topic types, except service types have two parts: one message for the request and another for the response.<br>
To find out the type of a service:
```bash
$ ros2 service type <service_name>

#Example
$ ros2 service type /clear
# will print: std_srvs/srv/Empty
# The Empty type means the service call sends no data when making a request and receives no data when receiving a response.

#To see the types of all the active services at the same time, you can append the --show-types option, abbreviated as -t, to the list command:
$ ros2 service list -t
```
### 4. ros2 service find
If you want to find all the services of a specific type:
```bash
$ ros2 service find <type_name>

#Example to find all the Emtpy typed services:
$ ros2 service find std_srvs/srv/Empty
```
### 5. ros2 interface show
You can call services from the command line, but first you need to know the structure of the input arguments:
```bash
$ ros2 interface show <type_name>

#Example on the /clear service's type, Empty:
$ ros2 interface show std_srvs/srv/Empty
# Output: ---
```
The --- separates the request structure (above) from the response structure (below).<br>
But, as you learned earlier, the Empty type doesn’t send or receive any data. So, naturally, its structure is blank.
Let’s introspect a service with a type that sends and receives data, like /spawn.<br> 
From the results of ros2 service list -t, we know /spawn’s type is turtlesim/srv/Spawn.<br>
To see the request and response arguments of the /spawn service, run the command:
```bash
$ ros2 interface show turtlesim/srv/Spawn
'''
float32 x
float32 y
float32 theta
string name # Optional.  A unique name will be created and returned if this is empty
---
string name
'''

'''
The information above the --- line tells us the arguments needed to call /spawn. x, y and theta determine the 2D pose of the spawned turtle, and name is clearly optional.
'''
```
### 6. ros2 service call
Now that you know what a service type is, how to find a service’s type, and how to find the structure of that type’s arguments, you can call a service using:
```bash
$ ros2 service call <service_name> <service_type> <arguments>
#The <arguments> part is optional. For example, you know that Empty typed services don’t have any arguments:
```
Example:
<'arguments'> in a service call from the cmd need to be in YAML syntax
```bash
#We want to spawn a new turtle by calling /spawn and setting arguments.
$ ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"

'''
response:
turtlesim.srv.Spawn_Response(name='turtle2')
'''
```

---

# Understanding parameters
A parameter is a configuration value of a node.<br> 
You can think of parameters as node settings.<br>
A node can store parameters as integers, floats, booleans, strings, and lists.<br> 
In ROS 2, each node maintains its own parameters. 
## Tasks
### 1. Setup
Same old stuff, and start /turtlesim and /teleop_turtle.
### 2. ros2 param list
To see the parameters belonging to your nodes:
```bash
$ ros2 param list
#A list of parameters for each node will show on the terminal
```
### 3. ros2 param get
To display the type and current value of a parameter:
```bash
$ ros2 param get <node_name> <parameter_name>

#Example:
$ ros2 param get /turtlesim background_g
# -> Integer value is: X
# You can do he same for background_r and background_b...
# and for every param
```
### 4. ros2 param set
To change a parameter’s value at runtime:
```bash
$ ros2 param set <node_name> <parameter_name> <value>

#Example:
$ ros2 param set /turtlesim background_r 150
```
Setting parameters with the set command will only change them in your current session, not permanently.<br>
However, you can save your settings and reload them the next time you start a node.
### 5. ros2 param dump
You can view all of a node’s current parameter values by using the command:
```bash
$ ros2 param dump <node_name>
#The command prints to the standard output (stdout) by default

#To save your current config of /turtlesim's parameters into the turtlesim.yaml:
$ ros2 param dump /turtlesim > turtlesim.yaml
```
### 6. ros2 param load
You can load parameters from a file to a currently running node using the command:
```bash
$ ros2 param load <nome_name> <parameter_file>

#Example:
$ ros2 param load /turtlesim turtlesim.yaml
```
### 7. ros2 param file on node startup
To start the same node using your saved parameter values, use:
```bash
$ ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>

#This is the same command you always use to start turtlesim, with the added flags --ros-args and --params-file, followed by the file you want to load.
```
When a parameter file is used at node startup, all parameters, including the read-only ones, will be updated.

---

# Understanding actions
Actions are one of the communication types in ROS 2 and are intended for long running tasks.<br>
They consist of three parts: 
1. a goal
2. feedback
3. a result <br>

Actions are built on topics and services. Their functionality is similar to services, except actions can be canceled. They also provide steady feedback, as opposed to services which return a single response.<br>
A robot system would likely use actions for navigation.<br>
An action goal could tell a robot to travel to a position. While the robot navigates to the position, it can send updates along the way (i.e. feedback), and then a final result message once it’s reached its destination.
Actions use a client-server model, similar to the publisher-subscriber model (described in the topics tutorial).<br>
An “action client” node sends a goal to an “action server” node that acknowledges the goal and returns a stream of feedback and a result.
<img src="https://docs.ros.org/en/humble/_images/Action-SingleActionClient.gif" width="500">

## Tasks
### 1. Setup
Same thing of every other setup. 
### 2. Use actions
When you launch the /teleop_turtle node (which lets you make the turte move with the arrows), there's a message on the terminal ("Use arrow keys to...").<br>
The second line of that message corresponds to an action.<br>
Pay attention to the terminal where the /turtlesim node is running.<br>
Each time you press one of these keys, you are sending a goal to an action server that is part of the /turtlesim node. <br>
The goal is to rotate the turtle to face a particular direction. <br>
A message relaying the result of the goal should display once the turtle completes its rotation.<br>
Not only can the client-side (your input in the teleop) stop a goal, but the server-side (the /turtlesim node) can as well. <br>
When the server-side chooses to stop processing a goal, it is said to “abort” the goal.<br>
Try hitting the D key, then the G key before the first rotation can complete.<br>
In the terminal where the /turtlesim node is running, you will see the message:
>[WARN] [turtlesim]: Rotation goal received before a previous goal finished. Aborting previous goal

This action server chose to abort the first goal because it got a new one. It could have chosen something else, like reject the new goal or execute the second goal after the first one finished. Don’t assume every action server will choose to abort the current goal when it gets a new one.
### 3. ros2 node info
To see the list of actions a node provides, /turtlesim in this case:
```bash
$ ros2 node info /turtlesim
```
Notice that the /turtle1/rotate_absolute action for /turtlesim is under Action Servers.<br> 
This means /turtlesim responds to and provides feedback for the /turtle1/rotate_absolute action.<br>
The /teleop_turtle node has the name /turtle1/rotate_absolute under Action Clients meaning that it sends goals for that action name.<br> 
To see that, run:
```bash
$ ros2 node info /teleop_turtle
```
### 4. ros2 action list
To identify all the actions in the ROS graph:
```bash
$ ros2 action list
```
Actions have types. To find an action's type:
```bash
$ ros2 action list -t
'''
In brackets to the right of each action name (in this case only /turtle1/rotate_absolute) is the action type, turtlesim/action/RotateAbsolute. You will need this when you want to execute an action from the command line or from code.
'''
```
### 5. ros2 action info
You can further introspect an action with the command:
```bash
$ ros2 action info <action>

#Example: 
$ ros2 action info /turtle1/rotate_absolute
```
### 6. ros2 interface show
One more piece of information you will need before sending or executing an action goal yourself is the structure of the action type.<br>
After you've identified an action's type, enter the following command with the action type in your terminal:
```bash
$ ros2 interface show turtlesim/action/RotateAbsolute
'''
A mex will show up on the terminal.
The section of this message above the first --- is the structure (data type and name) of the goal request. The next section is the structure of the result. The last section is the structure of the feedback.
'''
```
### 7. ros2 action send_goal
To send an action goal from the command line:
```bash
$ ros2 action send_goal <action_name> <action_type> <values>
#<value> need to be in YAML format

#Keeping an eye on the turtlesi window, enter the following command:
$ ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"
#You should see the turtle rotating.
```
All goals have a unique ID, shown in the return message. You can also see the result, a field with the name delta, which is the displacement to the starting position.<br>
To see the feedback of this goal, add --feedback to the ros2 action send_goal command:
```bash
$ ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: -1.57}" --feedback
```

---

# Using rqt_console to view logs
rqt_console is a GUI tool used to introspect log messages in ROS 2.<br>
Typically, log messages show up in your terminal.<br>
With rqt_console, you can collect those messages over time, view them closely and in a more organized manner, filter them, save them and even reload the saved files to introspect at a different time.<br>
Nodes use logs to output messages concerning events and status in a variety of ways. Their content is usually informational, for the sake of the user.
## Tasks
### 1. Setup
Start rqt_console:
```bash
$ ros2 run rqt_console rqt_console
```
The first section of the console is where log messages from your system will display.<br>
In the middle you have the option to filter messages by excluding severity levels. You can also add more exclusion filters using the plus-sign button to the right.<br>
The bottom section is for highlighting messages that include a string you input. 
You can add more filters to this section as well.
### 2. Logger levels
ROS 2’s logger levels are ordered by severity:
1. Fatal
2. Error
3. Warn
4. Info
5. Debug
There is no exact standard for what each level indicates, but the names speak for themselves.<br>
The default level is Info. You will only see messages of the default severity level and more-severe levels.<br>
Normally, only Debug messages are hidden because they’re the only level less severe than Info.<br>
For example, if you set the default level to Warn, you would only see messages of severity Warn, Error, and Fatal.
#### 2.1 Set default logger level
You can set the default logger level when you first run the /turtlesim node using remapping:
```bash
$ ros2 run turtlesim turtlesim_node --ros-args --log-level WARN
```
Now you won’t see the initial Info level messages that came up in the console last time you started turtlesim.

---

# Launching nodes
As you create more complex systems with more and more nodes running simultaneously, opening terminals and reentering configuration details becomes tedious.<br>
Launch files allow you to start up and configure a number of executables containing ROS 2 nodes simultaneously.<br>
Running a single launch file with the ros2 launch command will start up your entire system - all nodes and their configurations - at once.
## Tasks
### Running a Launch File
Open a terminal and run:
```bash
$ ros2 launch turtlesim multisim.launch.py
#This will run two turtlesim nodes
```
Now that these nodes are running, you can control them like any other ROS 2 nodes.<br>
For example, you can make the turtles drive in opposite directions by opening up two additional terminals and running the following commands:
```bash
#First terminal:
$ ros2 topic pub  /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
#Second terminal:
$ ros2 topic pub  /turtlesim2/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"
```

---

# Recording and playing back data
ros2 bag is a command line tool for recording data published on topics in your system.<br>
It accumulates the data passed on any number of topics and saves it in a database.<br>
You can then replay the data to reproduce the results of your tests and experiments.<br>
Recording topics is also a great way to share your work and allow others to recreate it.
## Tasks
### 1. Setup
Run turtlesim_node and turtle_teleop_key in two terminals.<br>
Let's make a new directory to store our saved recordings.
### 2. Choose a topic
To see the list of your system’s topics, open a new terminal and run the command:
```bash
$ ros2 topic list
```
In the topics tutorial, you learned that the /turtle_teleop node publishes commands on the /turtle1/cmd_vel topic to make the turtle move in turtlesim.<br>
To see the data that /turtle1/cmd_vel is publishing, run the command:
```bash
$ ros2 topic echo /turtle1/cmd_vel
```
### 3. ros2 bag record
#### 3.1 Record a single topic
```bash
#Move to the dir where you want to save your data before running the command
$ ros2 bag record <topic_name>
#Ctrl+C to stop recording. The directory will contain a metadata.yaml along with the bag file in the recorded format.
```
#### 3.2 Record multiple topics
```bash
#Move to the dir where you want to save your data before running the command
$ ros2 bag record -o subset /turtle1/cmd_vel /turtle1/pose

#The -o option allows you to choose a unique name for your bag file. The following string, in this case subset, is the file name.
#To record more than one topic at a time, simply list each topic separated by a space.
```
### 4. ros2 bag info
You can see details about your recording by running:
```bash
#Move to the dir where you want to save your data before running the command
$ ros2 bag info <bag_file_name>
```
### 5. ros2 bag play
In the terminal where teleop is running:
```bash
#Ctrl+C, then: 
$ ros2 bag play subset
'''
Your turtle will follow the same path you entered while recording (though not 100% exactly; turtlesim is sensitive to small changes in the system’s timing).
'''
```
Because the subset file recorded the /turtle1/pose topic, the ros2 bag play command won’t quit for as long as you had turtlesim running, even if you weren’t moving.<br>
To get an idea of how often position data is published, you can run the command:
```bash 
$ ros2 topic hz /turtle1/pose
```