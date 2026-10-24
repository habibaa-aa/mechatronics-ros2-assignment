                                                                ROS 2 MECHATRONICS ASSIGNMENT
OVERVIEW

    This project implements a turtle-control system using ROS 2 Jazzy and Turtlesim for the Design of Mechatronics Systems laboratory assignment at Alexandria University, Faculty of Engineering.

    The system demonstrates custom ROS 2 interfaces, services, actions, configurable speed limits, and launch files.

REQUIREMENTS
    
    Ubuntu Linux
    ROS 2 Jazzy
    Turtlesim
    Python 3
    colcon build tools


WORKSPACE STRUCTURE

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
            │   └── draw_square_node.py
            ├── launch/
            │   └── assignment.launch.py
            ├── package.xml
            └── setup.py


IMPLEMENTED FEATURES

    R1 — Turtlesim
        Runs the Turtlesim simulation.

    R2 — Custom Interfaces
        Defines:
            TargetPose.msg: target x, y, and yaw.
            GoToPose.action: moves the turtle to a target pose and reports feedback and a result.
            DrawSquare.action: requests square drawing and reports the current side.
            SetSpeed.srv: changes the maximum linear and angular speeds.

    R3 — GoToPose Action Server
        Receives a target pose, controls the turtle's linear and angular velocities, and reports movement feedback and completion status.

    R4 — Reset Service
        Provides /reset_turtle to clear the drawing and return the turtle to its starting position.

    R5 — DrawSquare Action Server
        Provides /draw_square. It resets the turtle and uses the goToPose action to draw four sides.

    R6 — Launch File
        The assignment.launch.py file starts the reset, movement, and square-drawing nodes together.

    R7 — Configurable Speed Limits
        The movement node provides max_linear_speed and max_angular_speed parameters and a /set_speed service to update the limits while running.

    R8 — Package Organization
        Separates custom interface definitions from Python nodes into two ROS 2 packages.


BUILD INSTRUCTIONS

    Open a terminal and run:

        cd ~/assignment_ws
        source /opt/ros/jazzy/setup.bash
        colcon build --packages-up-to assignment_nodes
        source ~/assignment_ws/install/setup.bash


RUN INSTRUCTIONS
    1. Start Turtlesim
        In a regular Ubuntu terminal:
            source /opt/ros/jazzy/setup.bash
            ros2 run turtlesim turtlesim_node

    2. Start the assignment nodes
        In another terminal:
            source /opt/ros/jazzy/setup.bash
            source ~/assignment_ws/install/setup.bash
            ros2 launch assignment_nodes assignment.launch.py

EXAMPLE COMMANDS

    Draw a square

        ros2 action send_goal /draw_square assignment_interfaces/action/DrawSquare "{side_length: 1.5}" --feedback

    Reset the turtle

        ros2 service call /reset_turtle std_srvs/srv/Empty "{}"

    Change the speed limits

        ros2 service call /set_speed assignment_interfaces/srv/SetSpeed "{max_linear_speed: 1.0, max_angular_speed: 2.0}"

    Move to a target pose

        ros2 action send_goal /goToPose assignment_interfaces/action/GoToPose "{target: {x: 8.0, y: 8.0, yaw: 0.0}}" --feedback

NOTES

    Source the ROS 2 Jazzy setup file and workspace setup file in each new terminal before running assignment commands.

    Start Turtlesim before launching the assignment nodes.

    The square side length must fit within the Turtlesim canvas to avoid reaching its boundaries.


BONUS: ROS 2 Bag Recording and Playback

    The rosbag bonus was completed by recording and replaying turtlesim topics using ROS 2 Jazzy.

    Topics Recorded

     `/turtle1/cmd_vel` — turtle velocity commands
     `/turtle1/pose` — turtle position and orientation
     `/turtle1/color_sensor` — turtle color sensor data

    
    Record a Demonstration

        Start turtlesim and the assignment launch file in separate terminals. Then run:

            cd ~/assignment_ws
            mkdir -p rosbag_recordings
            ros2 bag record -o rosbag_recordings/turtle_demo /turtle1/cmd_vel /turtle1/pose /turtle1/color_sensor
  

        While recording, run the square action in another terminal:

            ros2 action send_goal /draw_square assignment_interfaces/action/DrawSquare "{side_length: 1.5}" --feedback
   
        Press `Ctrl+C` in the recording terminal to stop and save the recording.

    
    Inspect and Replay the Recording

        Inspect the recorded topics and message counts:

            ros2 bag info rosbag_recordings/turtle_demo

        Replay the recording:

            ros2 bag play rosbag_recordings/turtle_demo

    
    Recording Results

    - Storage format: MCAP
    - Duration: approximately 52 seconds
    - Total messages: 6,642
    - Recording and playback were tested successfully.
