# ROS 2 Basics
---

## 1. Introduction to ROS 2

### What is ROS?

* **ROS** stands for *Robot Operating System*.
* It is **not** a traditional OS like Windows or Linux, but a **middleware** — a software layer on top of an existing OS.
* Provides tools, libraries, and conventions to standardize communication between different parts of a robot.
* Acts as a **toolbox** and **rulebook** for creating complex robotic applications efficiently.

### Why ROS 2?

ROS 2 is a ground-up redesign of ROS 1 with modern robotics in mind.

**Key Improvements:**

* **Multi-robot systems**: Better support for fleets of robots.
* **Real-time support**: Suitable for time-critical tasks.
* **Resilient communication**: Handles unreliable networks well.
* **Broader platform support**: Works on Linux, Windows, macOS.
* Uses **DDS (Data Distribution Service)** as its communication backbone.

---

## 2. Core ROS 2 Concepts

ROS 2 uses a **graph architecture** where independent processes (nodes) communicate.

### Nodes

* Fundamental processing unit in ROS.
* Each node performs a **single, specific task**.
* Example: One node for the camera, one for motors, etc.

### Topics (Pub/Sub Communication)

* **Topics** are named data channels for asynchronous communication.
* **Publishers** send data; **Subscribers** receive data.
* Example: Camera node publishes `/image_data`; image processor subscribes to it.

### Messages

* Structured data formats used on topics.
* Example: `geometry_msgs/msg/Twist` for velocity commands.

### Services (Request/Response)

* Synchronous, one-to-one communication.
* Example: `/reset` service for simulation.

### Actions (Goal/Feedback/Result)

* Asynchronous, long-running tasks with feedback.
* Example: "Navigate to coordinates X, Y" with progress updates.

---

## 3. ROS 2 Command Line Interface (CLI)

The `ros2` command is the main tool for interacting with ROS 2 systems.

### Node Commands

```bash
ros2 node list
ros2 node info /<node_name>
```

### Topic Commands

```bash
ros2 topic list
ros2 topic list -t
ros2 topic info /<topic_name>
ros2 topic echo /<topic_name>
ros2 topic pub --once /<topic_name> <message_type> '<args>'
```

### Service Commands

```bash
ros2 service list
ros2 service type /<service_name>
ros2 service call /<service_name> <service_type> '<args>'
```

### Action Commands

```bash
ros2 action list
ros2 action info /<action_name>
ros2 action send_goal /<action_name> <action_type> '<goal>' --feedback
```

### Type Inspection

```bash
ros2 msg show <message_type>
ros2 srv show <service_type>
ros2 action show <action_type>
```

---

## 4. Workspaces and Packages

### Workspace Structure

* **src/**: Source code.
* **build/**: Temporary build files.
* **install/**: Final executables, libraries, configs.
* **log/**: Build logs.

### Package Structure

* `package.xml`: Metadata and dependencies.
* `setup.py` or `CMakeLists.txt`: Build instructions.

### Building

```bash
colcon build
source install/setup.bash
```

---

## 5. Creating a ROS 2 Package and Node (Python Example)

### Create a Package

```bash
ros2 pkg create --build-type ament_python my_robot_pkg --dependencies rclpy
```

### Example Node (`my_first_node.py`)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("my_first_node")
        self.get_logger().info("Hello from ROS 2!")

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

### Configure `setup.py`

```python
entry_points={
    'console_scripts': [
        "my_node = my_robot_pkg.my_first_node:main"
    ],
},
```

### Build and Run

```bash
colcon build
source install/setup.bash
ros2 run my_robot_pkg my_node
```

---

## 6. Launch Files

Launch files start multiple nodes at once.

### Example Launch File (`my_launch_file.py`)

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='my_turtle'
        ),
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='my_teleop'
        )
    ])
```

### Run Launch File

```bash
ros2 launch my_robot_pkg my_launch_file.py
```

---
