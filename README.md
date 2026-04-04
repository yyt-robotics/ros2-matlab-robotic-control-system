# ROS2-Based Robotic Task Execution System with MATLAB Integration

## 🚀 Highlights

- End-to-end robotic control pipeline from task-level command to joint-level execution  
- ROS2 Action-based asynchronous task execution  
- Custom inverse kinematics solver with velocity-constrained trajectory generation  
- Real-time integration with MATLAB/Simulink via HTTP communication  
- Achieved ~10⁻³ rad tracking accuracy in closed-loop simulation  

## Overview

This project implements a robotic control pipeline combining ROS2, inverse kinematics, trajectory planning, and MATLAB/Simulink simulation for a 6-DOF manipulator.

Two modes are supported:

- Offline mode: trajectory generation (CSV) + MATLAB simulation
- Real-time mode: interactive control via web GUI

The system covers the full pipeline from task-level pose input to joint-level execution.

![System Architecture](media/architecture.png)

---

## 🎥 Demo Videos

This project supports both offline trajectory simulation and real-time robotic control.  
See the demonstrations below:

### 🔹 Offline Mode (Trajectory Generation + MATLAB Simulation)
[![Offline Demo](https://img.youtube.com/vi/54vFBiX_kls/0.jpg)](https://youtu.be/54vFBiX_kls)

### 🔹 Real-Time Mode (ROS2 + Web GUI + MATLAB Integration)
[![Realtime Demo](https://img.youtube.com/vi/jeSNi3DXFD4/0.jpg)](https://youtu.be/jeSNi3DXFD4)

---

## System Architecture

Web GUI -> ROS2 Action -> IK Solver -> Trajectory Generation -> /joint_ref -> HTTP Bridge -> MATLAB/Simulink -> Robot Simulation

---

## Key Features

- ROS2 action-based task execution (`MoveToPose`)
- Custom IK solver (least-squares, multi-initial guess)
- Velocity-constrained joint trajectory generation
- Flask-based web GUI for pose control
- MATLAB/Simulink closed-loop simulation
- CSV-based offline trajectory replay and evaluation

---

## Environment

Tested with:
- ROS2 Jazzy
- Python 3.12
- MATLAB/Simulink
- Simscape Multibody
- NumPy / SciPy / Flask

---

## Demo

### Offline Mode
1. Send target pose via GUI  
2. ROS2 generates joint trajectory (CSV)  
3. MATLAB runs simulation (`run_full_demo.m`)  
4. Tracking and error plots are generated  

### Offline Simulation Result

![Tracking](media/tracking.png)

### Error Plot

![Error](media/error.png)

### Real-Time Mode
1. Adjust pose via GUI sliders  
2. ROS2 computes IK and publishes joint references  
3. MATLAB reads reference via HTTP  
4. Robot responds in real time  

---

## Project Structure

```text
.
├── matlab/
│   ├── get_joint_ref_http.m
│   ├── pid_control.slx
│   ├── run_full_demo.m
│   └── trajectory/
│       └── .gitkeep
├── ros2_ws/
│   ├── robot_interfaces/
│   │   ├── CMakeLists.txt
│   │   ├── action/
│   │   │   └── MoveToPose.action
│   │   └── package.xml
│   └── robot_task_manager/
│       ├── package.xml
│       ├── resource/
│       │   └── robot_task_manager
│       ├── robot_task_manager/
│       │   ├── __init__.py
│       │   ├── ik_solver_opt.py
│       │   ├── joint_ref_bridge.py
│       │   ├── joint_state_publisher_node.py
│       │   ├── pose_web_gui.py
│       │   └── robot_task_manager.py
│       ├── setup.cfg
│       └── setup.py
├── urdf/
│   └── gluon_6l3.urdf
├── .gitignore
└── README.md
```

## How to Run

### ROS2

cd ros2_ws  
colcon build  
source install/setup.bash  
ros2 run robot_task_manager robot_task_manager  

### Web GUI

ros2 run robot_task_manager pose_web_gui  

Open browser:

http://localhost:8080  

### MATLAB (offline)

run('matlab/run_full_demo.m')

### Requirements

- ROS2 Jazzy installed
- Python 3.10+
- MATLAB with Simulink + Simscape Multibody

## Technologies

ROS2, Python, Flask, NumPy, SciPy, MATLAB/Simulink, URDF
