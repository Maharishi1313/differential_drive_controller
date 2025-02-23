# Differential Drive Controller (ROS2)

**Differential Drive Controller**, a ROS2 package designed for controlling a differential drive robot. This package consists of two key nodes: 
1. **repub_node.cpp** – Converts velocity commands into wheel RPM values.
2. **waypoint_follower.py** – Implements waypoint navigation using PID control.

---

## Features
- ✅ Converts `/cmd_vel` (linear & angular velocity) into individual wheel RPM values.
- ✅ Publishes RPM values for left and right wheels, useful for implementing differential drive Gazebo Plugin.
- ✅ Implements PID-based waypoint following with real-time feedback from `/odom`.
- ✅ Fully configurable via ROS2 parameters.

---

## Package Overview

### Directory Structure
```
differential_drive_controller/
│── py_scripts/                 # python scripts 
│   │── waypoint_follower.py    # Waypoint following node
│── src/
│   │── repub_node.cpp          # Velocity to RPM conversion node
│── CMakeLists.txt              # Build system configuration
│── package.xml                 # ROS2 package metadata
│── README.md                 # README file

```

### 1) repub_node.cpp (Velocity to RPM Converter)
- This C++ node subscribes to the `/cmd_vel` topic to get the velocities(linear and angular) of a differential drive robot 
- Then processes the obtained velocities to calculate the RPM of each of the wheels(here, only two wheels left and right are considered) 
- These RPM values are then published on the `/left_wheel_rpm` and `/right_wheel_rpm` topics which can be used by the gazebo plugin to set the the joint velocities of the robot.
#### Parameters:
| Parameter Name | Type   | Description |
|---------------|--------|-------------|
| `wheelbase`   | double | Distance between left and right wheels (m) |
| `wheel_radius`| double | Radius of each wheel (m) |
| `max_rpm`     | double | Maximum allowed RPM for safety |

#### 🔗 Topics:
| Topic Name         | Message Type              | Role |
|--------------------|--------------------------|------------------------------|
| `/cmd_vel`        | `geometry_msgs::Twist`   | Input: Robot velocity command |
| `/left_wheel_rpm` | `std_msgs::Float64`      | Output: Left wheel RPM |
| `/right_wheel_rpm`| `std_msgs::Float64`      | Output: Right wheel RPM |

---

### 2) waypoint_follower.py (Waypoint Navigation using PID)
- This python node makes the robot follow the given waypoints (currently two, which can be modified further) by implementing **PD control** over the linear and angular velocities of the robot 
- It obtains the position of robot (through the `/odom` topic) as feedback and passes the error distance into the controller. The node has coordinates of waypoints [(x1,y1), (x2,y2)] and the constants of PD (kp, ki, kd) as parameters 
- By default, kp is set to 1.2, kd is set to 0.1 while ki set to 0 as in this system **PD controller** is sufficient and using integrator would lead to instability.

#### Parameters:
| Parameter Name   | Type   | Description |
|-----------------|--------|----------------------------|
| `waypoint_1_x`  | double | X-coordinate of the first waypoint |
| `waypoint_1_y`  | double | Y-coordinate of the first waypoint |
| `waypoint_2_x`  | double | X-coordinate of the second waypoint |
| `waypoint_2_y`  | double | Y-coordinate of the second waypoint |
| `kp`            | double | Proportional gain for PID (default: 1.2) |
| `ki`            | double | Integral gain for PID (default: 0.0, disabled) |
| `kd`            | double | Derivative gain for PID (default: 0.1) |

#### 🔗 Topics:
| Topic Name  | Message Type           | Role |
|------------|------------------------|-------------------------------------|
| `/odom`    | `nav_msgs::Odometry`   | Input: Current robot position |
| `/cmd_vel` | `geometry_msgs::Twist` | Output: Velocity command |

---

## How It Works
1. The **repub_node** reads `/cmd_vel` and computes RPM values for each wheel.
2. The **waypoint_follower** reads `/odom`, calculates position error, and adjusts speed dynamically using a PID controller.
3. The robot moves toward each waypoint while correcting deviations.

---

## Installation & Usage
### Install Dependencies
Ensure you have ROS2 installed. Then, clone this package into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/Maharishi1313/differential_drive_controller
cd ~/ros2_ws
colcon build --packages-select differential_drive_controller
source install/setup.bash
```

### Running the Nodes
1. **Start the repub_node:**
   ```bash
   ros2 run differential_drive_controller repub_node --ros-args -p wheel_base:=0.287 -p wheel_radius:=0.033 - 
   p max_rpm:=150.0
   ```
2. **Run the waypoint follower:**
   ```bash
   ros2 run differential_drive_controller waypoint_navigation.py --ros-args 
   -p waypoint_1_x:=2.0 -p waypoint_1_y:=1.0 -p waypoint_2_x:=4.0 -p waypoint_2_y:=3.0

   ```

---

## Notes
- The **PD controller** is tuned for stability. The integral term is disabled (`ki = 0`) to prevent instability.
- The package is fully customizable via ROS2 parameters.
- Ensure correct wheel parameters are set for accurate motion control.

---

