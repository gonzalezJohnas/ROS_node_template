# Template code to create a ROS NODE

This code is based on the ros node code example  [ROSNodeTutorialC++](http://wiki.ros.org/ROSNodeTutorialC%2B%2B).


## Dynamic Reconfigure
The <em>dynamic_reconfigure</em> package provided a standard way to expose a subset of a node's parameters to external reconfiguration. Client programs, e.g., GUIs, can query the node for the set of reconfigurable parameters, including their names, types, and ranges, and present a customized interface to the user.

To add or change parameter that can be changed by the <em>dynamic_reconfigure</em> package you have to:
    
    - Change the .cfg file under the cfg directory
    - Update the reference of the parameters in the node files (.h, .cpp)


## Ros message
Nodes communicate with each other by publishing messages to topics. A message is a simple data structure, comprising typed fields. Standard primitive types (integer, floating point, boolean, etc.) are supported, as are arrays of primitive types. Messages can include arbitrarily nested structures and arrays (much like C structs).

To add or change messages you have to:

    - Change the .msg file under the msf directory
    - Update the reference of the message data in the node files (.h, .cpp)


## Ros node
A node is a process that performs computation. Nodes are combined together into a graph and communicate with one another using streaming topics, RPC services, and the Parameter Server. These nodes are meant to operate at a fine-grained scale; a robot control system will usually comprise many nodes. 

The files to modify to implement your algorithms are:

    include/template_node/node_example_core.h
    src/node_example_core.cpp

And the main file that actually run the node

    publisher.cpp

## Ros launch 
Roslaunch is a tool for easily launching multiple ROS nodes locally and remotely via SSH, as well as setting parameters on the Parameter Server. It includes options to automatically respawn processes that have already died. roslaunch takes in one or more XML configuration files (with the .launch extension) that specify the parameters to set and nodes to launch, as well as the machines that they should be run on.

An example of launch file to run the node_exmaple is provide under the launch directory.

## Build the package 
Create a catkin workspace

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make

Clone this repository under the src folder

    cd ~/catkin_ws/src
    git clone https://github.com/gonzalezJohnas/ROS_node_template.git

Build the package
    cd ~/catkin_ws
    catkin_make