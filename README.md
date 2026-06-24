# ROS 2 Basics

---

# 1. Introduction to ROS 2

## What is ROS?

* **ROS (Robot Operating System)** is an open-source robotics middleware.
* It is **not an operating system** like Linux or Windows.
* Provides libraries, tools, and communication infrastructure for robot software development.
* Helps developers build complex robotic systems using reusable modules.

## Why ROS 2?

ROS 2 is the successor to ROS 1 and addresses many limitations of ROS 1.

### Key Features

* Distributed communication
* Multi-robot support
* Real-time capabilities
* Improved security
* Cross-platform support
* DDS-based communication
* Better scalability

### Supported Platforms

* Ubuntu Linux
* Windows
* macOS
* Embedded Systems

---

# 2. ROS 2 Architecture

ROS 2 follows a distributed architecture.

```text
+------------+        Topic         +------------+
|   Node A   | -------------------> |   Node B   |
| Publisher  |                      | Subscriber |
+------------+                      +------------+

       |                                   |
       +--------- ROS 2 Middleware --------+
                    (DDS)
```

Main Components:

* Nodes
* Topics
* Messages
* Services
* Actions
* Parameters
* Launch Files
* DDS Middleware

---

# 3. Core ROS 2 Concepts

## Node

A node is an executable process that performs a specific task.

Examples:

* Camera Node
* Lidar Node
* Motor Controller Node
* Navigation Node

### Commands

```bash
ros2 node list

ros2 node info /node_name
```

---

## Topic

Topics are named communication channels.

Used for asynchronous communication.

### Publisher

Sends data to a topic.

### Subscriber

Receives data from a topic.

Example:

```text
Camera Node
      |
      v
/image_raw
      |
      v
Detection Node
```

### Commands

```bash
ros2 topic list

ros2 topic list -t

ros2 topic info /topic_name

ros2 topic echo /topic_name

ros2 topic hz /topic_name

ros2 topic bw /topic_name
```

---

## Message

Messages define the data structure exchanged between nodes.

Example:

```bash
ros2 interface show geometry_msgs/msg/Twist
```

Output:

```text
Vector3 linear
Vector3 angular
```

Common Fields:

```text
linear.x   -> Forward
linear.y   -> Sideways
linear.z   -> Vertical

angular.x  -> Roll
angular.y  -> Pitch
angular.z  -> Yaw
```

### Commands

```bash
ros2 interface list

ros2 msg list

ros2 msg show geometry_msgs/msg/Twist
```

---

## Service

Services use Request-Response communication.

Suitable when an immediate reply is required.

Example:

```text
Client ------ Request ------> Server

Client <----- Response ------ Server
```

### Commands

```bash
ros2 service list

ros2 service type /service_name

ros2 service find service_type

ros2 service call /service_name service_type "{}"
```

---

## Action

Actions are used for long-running operations.

Examples:

* Navigation
* Object Pickup
* Docking

Action provides:

* Goal
* Feedback
* Result

### Commands

```bash
ros2 action list

ros2 action info /action_name

ros2 action send_goal /action_name action_type "{}"

ros2 action send_goal /action_name action_type "{}" --feedback
```

---

## Parameters

Parameters are runtime configurable values.

Examples:

* Robot speed
* Camera resolution
* Threshold values

### Commands

```bash
ros2 param list

ros2 param get /node_name parameter_name

ros2 param set /node_name parameter_name value

ros2 param describe /node_name parameter_name
```

---

# 4. ROS 2 Command Line Interface (CLI)

The `ros2` command is the primary tool used to interact with ROS systems.

---

## Node Commands

```bash
ros2 node list

ros2 node info /node_name
```

---

## Topic Commands

```bash
ros2 topic list

ros2 topic list -t

ros2 topic info /topic_name

ros2 topic echo /topic_name

ros2 topic hz /topic_name

ros2 topic bw /topic_name
```

---

## Publishing Commands

Publish once:

```bash
ros2 topic pub --once \
/turtle1/cmd_vel \
geometry_msgs/msg/Twist \
"{linear: {x: 2.0}, angular: {z: 1.8}}"
```

Publish continuously:

```bash
ros2 topic pub --rate 1 \
/turtle1/cmd_vel \
geometry_msgs/msg/Twist \
"{linear: {x: 1.5}, angular: {z: 1.0}}"
```

---

## Service Commands

```bash
ros2 service list

ros2 service type /service_name

ros2 service call \
/service_name \
service_type \
"{}"
```

---

## Action Commands

```bash
ros2 action list

ros2 action info /action_name

ros2 action send_goal \
/action_name \
action_type \
"{}"
```

---

## Interface Commands

```bash
ros2 interface list

ros2 interface show geometry_msgs/msg/Twist

ros2 msg list

ros2 srv list

ros2 action list
```

---

## Package Commands

```bash
ros2 pkg list

ros2 pkg prefix package_name

ros2 pkg executables package_name
```

---

# 5. ROS 2 Workspaces and Packages

## Workspace

A workspace contains multiple ROS packages.

Typical Structure:

```text
ros2_ws/
├── src/
├── build/
├── install/
└── log/
```

---

## src/

Contains package source code.

```text
src/
├── cpp_pkg/
├── py_pkg/
└── robot_pkg/
```

---

## build/

Generated during compilation.

Contains:

* Object files
* Temporary files
* Build metadata

Generated using:

```bash
colcon build
```

---

## install/

Contains final executable files.

Important:

```bash
source install/setup.bash
```

---

## log/

Contains:

* Build logs
* Error logs
* Runtime logs

---

## Package

A package is a collection of:

* Nodes
* Scripts
* Launch Files
* Configurations
* Libraries

Important Files:

```text
package.xml
setup.py
setup.cfg
CMakeLists.txt
```

---

# 6. Creating ROS 2 Packages

## Python Package

```bash
cd ~/ros2_ws/src

ros2 pkg create \
--build-type ament_python \
my_robot_pkg \
--dependencies rclpy
```

---

## C++ Package

```bash
cd ~/ros2_ws/src

ros2 pkg create \
--build-type ament_cmake \
my_cpp_pkg
```

---

# 7. Building Packages

Move to workspace root:

```bash
cd ~/ros2_ws
```

Build:

```bash
colcon build
```

Build specific package:

```bash
colcon build \
--packages-select my_robot_pkg
```

Source workspace:

```bash
source install/setup.bash
```

---

# 8. Running Nodes

Run node:

```bash
ros2 run package_name node_name
```

Example:

```bash
ros2 run turtlesim turtlesim_node
```

---

# 9. Launch Files

Launch files start multiple nodes together.

Example:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package="turtlesim",
            executable="turtlesim_node"
        ),

        Node(
            package="turtlesim",
            executable="turtle_teleop_key"
        )
    ])
```

Run launch file:

```bash
ros2 launch package_name launch_file.py
```

Show arguments:

```bash
ros2 launch package_name launch_file.py --show-args
```

---

# 10. rqt_graph

Visualizes node communication.

Install:

```bash
sudo apt install ros-humble-rqt-graph
```

Run:

```bash
rqt_graph
```

Example:

```text
Node A --> /cmd_vel --> Node B
```

Useful for debugging publishers and subscribers.

---

# 11. ROS 2 Bags

Record topic data.

Record all topics:

```bash
ros2 bag record -a
```

Record one topic:

```bash
ros2 bag record /cmd_vel
```

Playback:

```bash
ros2 bag play bag_folder
```

Information:

```bash
ros2 bag info bag_folder
```

---

# 12. TF (Transforms)

TF manages coordinate frames.

Example:

```text
map
 └── odom
      └── base_link
           └── laser
```

Commands:

```bash
ros2 run tf2_tools view_frames

ros2 run tf2_ros tf2_echo frame1 frame2

ros2 run tf2_ros tf2_monitor
```

---

# 13. Useful Visualization Tools

## rqt_graph

```bash
rqt_graph
```

## rqt

```bash
rqt
```

## RViz2

```bash
rviz2
```

---

# 14. Environment Setup

Source ROS installation:

```bash
source /opt/ros/humble/setup.bash
```

Source workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

Add permanently:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

source ~/.bashrc
```

---

# 15. Typical ROS 2 Workflow

```bash
mkdir -p ~/ros2_ws/src

cd ~/ros2_ws/src

ros2 pkg create \
--build-type ament_python \
my_robot_pkg

# Write code

cd ~/ros2_ws

colcon build

source install/setup.bash

ros2 run my_robot_pkg my_node
```

---

# 16. Quick Summary

| Component   | Description              |
| ----------- | ------------------------ |
| Node        | Executable Process       |
| Topic       | Communication Channel    |
| Publisher   | Sends Data               |
| Subscriber  | Receives Data            |
| Message     | Data Structure           |
| Service     | Request/Response         |
| Action      | Goal/Feedback/Result     |
| Parameter   | Runtime Configuration    |
| Package     | Collection of Nodes      |
| Workspace   | Collection of Packages   |
| Launch File | Start Multiple Nodes     |
| Bag         | Record and Replay Data   |
| TF          | Coordinate Frames        |
| DDS         | Communication Middleware |
| RViz2       | Visualization Tool       |
| rqt_graph   | Communication Graph      |
