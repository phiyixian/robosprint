# Robosprint 2026 ROS2 Robot System

This project is a ROS2-based robot system developed for the REC Robosprint 2026 competition.  
The robot supports two modes:
- Phase 1: Autonomous line-following cube collection
- Phase 2: Manual control for cube placement and Tic-Tac-Toe gameplay

---

# Workspace Structure

```

robosprint_ws/
└── src/
├── robot_bringup
├── vision_pkg
├── decision_pkg
├── navigation_pkg
├── control_pkg
├── teleop_pkg
├── game_state_pkg
└── robot_interfaces

````

---

# Packages Overview

---

## robot_interfaces

Contains all custom ROS2 messages and services used for communication between nodes.

### Messages
- DetectedCube.msg  
  Contains cube label, position, and real/fake classification
- Junction.msg  
  Contains detected zone information (pick, storing, start, conversion, rack)
- LineCommand.msg  
  Used by FSM to send navigation target zones
- GamePhase.msg  
  Defines current game phase (setup, phase1, phase2, end)

### Services
- FirePneumatic.srv  
  Controls pneumatic system for cube pickup and release

---

## vision_pkg

Handles all perception-related tasks using camera input.

### Nodes

### camera_node
- Publishes raw camera frames
- Topic: /image_raw

### cube_detector
- Detects cubes from camera input
- Publishes detected cube information
- Topic: /detected_cubes

### junction_detector
- Detects line intersections and zone markers
- Publishes zone information
- Topic: /junction_event

---

## decision_pkg

Handles decision-making logic and game state interpretation.

### Nodes

### line_fsm_controller
- Core finite state machine for Phase 1
- Determines next target zone based on game state and junction input
- Publishes navigation commands
- Topic: /line_command

### pick_place
- Handles cube pickup and placement logic
- Sends commands to manipulator based on detected cubes

---

## navigation_pkg

Handles robot movement during autonomous phase.

### Nodes

### line_follower
- Implements black line following control
- Subscribes to /line_command
- Outputs velocity commands
- Publishes: /cmd_vel

---

## control_pkg

Controls hardware-level actuation including motors and pneumatic system.

### Nodes

### motor_driver
- Converts velocity commands into motor signals
- Subscribes: /cmd_vel

### manipulator
- Controls pneumatic actuator for cube handling
- Provides service: /fire_pneumatic

---

## teleop_pkg

Handles manual control during Phase 2.

### Nodes

### ps4_input
- Reads PS4 controller inputs

### teleop_filter
- Enables or disables manual control based on game phase
- Outputs filtered velocity commands

---

## game_state_pkg

Manages game phases, scoring, and rule enforcement.

### Nodes

### game_state
- Controls overall competition state
- Manages transitions between setup, phase 1, phase 2, and end
- Publishes: /game_phase

---

## robot_bringup

Contains launch files to start the full system.

### Launch File
- robot.launch.py

### Launches
- camera_node
- cube_detector
- junction_detector
- line_fsm_controller
- line_follower
- motor_driver
- manipulator
- game_state

---

# System Flow

---

## Phase 1: Autonomous Mode

1. camera_node publishes image data
2. cube_detector identifies real and fake cubes
3. junction_detector detects zone intersections
4. line_fsm_controller decides next target zone
5. line_follower follows black line paths
6. motor_driver executes movement
7. manipulator picks and drops cubes

---

## Phase 2: Manual Mode

1. ps4_input receives controller input
2. teleop_filter enables manual control
3. robot operates under user control
4. cubes are placed on rack
5. opponent cubes can be removed and converted

---

# ROS2 Topics

- /image_raw
- /detected_cubes
- /junction_event
- /line_command
- /cmd_vel
- /game_phase

---

# Build Instructions

```bash
cd ~/robosprint_ws
colcon build
source install/setup.bash
````

---

# Run Instructions

```bash
ros2 launch robot_bringup robot.launch.py
```

---

# Notes

* Phase 1 uses line-following navigation only
* All zones are defined by black line intersections
* FSM controls all autonomous decisions
* System is designed for ROS2 Jazzy on Ubuntu 24

```
```
