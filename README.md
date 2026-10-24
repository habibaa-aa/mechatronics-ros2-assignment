# ROS 2 Mechatronics Assignment

**Course:** Design of Mechatronics Systems — Lab 1  
**University:** Alexandria University, Faculty of Engineering  
**ROS 2 Distribution:** Jazzy

## Team Members

- Habiba Amr Mostafa — ID: 9590
- Habiba Mohammed Hassan — ID: 9435

## Overview

This project implements a turtle-control system using ROS 2 Jazzy and Turtlesim. It demonstrates custom ROS 2 interfaces, services, actions, configurable speed limits, launch files, rosbag recording and playback, and the Strategy design pattern.

## Requirements

- Ubuntu Linux
- ROS 2 Jazzy
- Turtlesim
- Python 3
- `colcon` build tools

## Workspace Structure

```text
assignment_ws/
├── README.md
└── src/
    ├── assignment_interfaces/
    │   ├── action/
    │   │   ├── DrawSquare.action
    │   │   └── GoToPose.action
    │   ├── msg/
    │   │   └── TargetPose.msg
    │   ├── srv/
    │   │   └── SetSpeed.srv
    │   ├── CMakeLists.txt
    │   └── package.xml
    └── assignment_nodes/
        ├── assignment_nodes/
        │   ├── reset_node.py
        │   ├── go_to_pose_node.py
        │   ├── draw_square_node.py
        │   └── motion_strategy.py
        ├── launch/
        │   └── assignment.launch.py
        ├── package.xml
        ├── setup.py
        └── setup.cfg
```

## Implemented Requirements

### R1 — Turtlesim

The launch file starts the Turtlesim simulation.

### R2 — Custom Interfaces

The `assignment_interfaces` package defines:

- `TargetPose.msg`: target position (`x`, `y`) and orientation (`yaw`).
- `GoToPose.action`: requests movement to a target pose and reports feedback and a result.
- `DrawSquare.action`: requests square drawing and reports the current side.
- `SetSpeed.srv`: requests updates to maximum linear and angular speeds.

### R3 — GoToPose Action Server

The `/goToPose` action server receives a target pose, controls the turtle's linear and angular velocities, publishes feedback, and reports completion.

### R4 — Reset Service

The `/reset_turtle` service clears the drawing and returns the turtle to its starting position by calling Turtlesim services.

### R5 — DrawSquare Action Server

The `/draw_square` action server resets the turtle and uses the `/goToPose` action four times to draw a square.

### R6 — Launch File

The `assignment.launch.py` file starts Turtlesim and all three assignment nodes together.

### R7 — Configurable Speed Limits

The movement node declares `max_linear_speed` and `max_angular_speed` parameters. The `/set_speed` service updates both limits while the system is running. Invalid non-positive speed values are rejected.

### R8 — Package Organization

The project separates interface definitions in `assignment_interfaces` from executable Python nodes in `assignment_nodes`.

## Build Instructions

Open a terminal and run:

```bash
cd ~/assignment_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to assignment_nodes
source ~/assignment_ws/install/setup.bash
```

## Run the Assignment

Start the complete system with one command:

```bash
source /opt/ros/jazzy/setup.bash
source ~/assignment_ws/install/setup.bash
ros2 launch assignment_nodes assignment.launch.py
```

This launches Turtlesim, the reset service node, the movement action server, and the square action server.

Keep this terminal running. Open another terminal for the example commands below, sourcing the ROS 2 and workspace setup files first:

```bash
source /opt/ros/jazzy/setup.bash
source ~/assignment_ws/install/setup.bash
```

## Example Commands

### Move to a Target Pose

```bash
ros2 action send_goal /goToPose assignment_interfaces/action/GoToPose "{target: {x: 8.0, y: 8.0, yaw: 0.0}}" --feedback
```

### Draw a Square

```bash
ros2 action send_goal /draw_square assignment_interfaces/action/DrawSquare "{side_length: 1.5}" --feedback
```

### Reset the Turtle

```bash
ros2 service call /reset_turtle std_srvs/srv/Empty "{}"
```

### Change Speed Limits

```bash
ros2 service call /set_speed assignment_interfaces/srv/SetSpeed "{max_linear_speed: 1.0, max_angular_speed: 2.0}"
```

### Inspect Speed Parameters

```bash
ros2 param get /go_to_pose_node max_linear_speed
ros2 param get /go_to_pose_node max_angular_speed
```

The speed service rejects values less than or equal to zero.

## Notes

- Source both the ROS 2 Jazzy setup file and the workspace setup file in every new terminal before running assignment commands.
- The launch file starts Turtlesim automatically; do not start a second Turtlesim instance.
- Choose a square side length that fits within the Turtlesim canvas.

## Bonus 1 — ROS 2 Bag Recording and Playback

The rosbag bonus was completed by recording and replaying Turtlesim topics.

### Topics Recorded

- `/turtle1/cmd_vel` — turtle velocity commands
- `/turtle1/pose` — turtle position and orientation
- `/turtle1/color_sensor` — turtle color sensor data

### Record a Demonstration

With the assignment launch file running, open another terminal and run:

```bash
cd ~/assignment_ws
mkdir -p rosbag_recordings
ros2 bag record -o rosbag_recordings/turtle_demo /turtle1/cmd_vel /turtle1/pose /turtle1/color_sensor
```

While recording, run the square action in another terminal:

```bash
ros2 action send_goal /draw_square assignment_interfaces/action/DrawSquare "{side_length: 1.5}" --feedback
```

Press `Ctrl+C` in the recording terminal to stop recording and save the bag.

### Inspect and Replay

```bash
ros2 bag info rosbag_recordings/turtle_demo
ros2 bag play rosbag_recordings/turtle_demo
```

### Recording Results

- Storage format: MCAP
- Duration: approximately 52 seconds
- Total messages: 6,642
- Recording and playback were tested successfully.

## Bonus 2 — Strategy Design Pattern

The Strategy Pattern separates turtle motion calculations from the `/goToPose` action server.

- **`motion_strategy.py`:** Implements `MotionStrategy`, which calculates velocity commands using the current pose, target pose, and speed limits.
- **`go_to_pose_node.py`:** Uses the strategy while managing the ROS 2 action goal, feedback, cancellation, and result.
- **Live speed updates:** The `/set_speed` service updates the node parameters and the strategy's speed limits while the node is running.

This separation makes the code easier to understand, test, maintain, and extend. Alternative movement strategies could be added without rewriting the action server.

**Testing:** Both `/goToPose` and `/draw_square` were tested successfully after the refactor.
