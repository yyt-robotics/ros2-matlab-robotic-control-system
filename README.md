# ros2-matlab-robotic-control-system
ROS2-based robotic control system with trajectory planning and MATLAB/Simulink integration

📌 Overview

This project implements a modular robotic control system that integrates ROS2, inverse kinematics, trajectory planning, and MATLAB/Simulink-based closed-loop control.

The system supports both:
	•	Offline trajectory execution (CSV-based)
	•	Real-time interactive control via Web GUI

It demonstrates a full pipeline from high-level task commands to low-level joint control and dynamic simulation.

🧠 System Architecture
Web GUI → ROS2 Action → IK Solver → Trajectory Generation → /joint_ref → HTTP Bridge → MATLAB/Simulink PID → Robot Simulation

⚙️ Key Features

🔹 ROS2 Task Execution Framework
	•	Action-based asynchronous control (MoveToPose)
	•	Feedback and cancellation support
	•	State management (idle / busy / completed)

🔹 Inverse Kinematics (IK)
	•	Custom numerical IK solver (least squares)
	•	Multi-initial guess strategy for robustness
	•	Joint limit handling and workspace validation

🔹 Trajectory Planning
	•	Joint-space trajectory generation
	•	Velocity-constrained motion planning
	•	Adaptive timing based on joint displacement

🔹 Real-Time Control Interface
	•	Web-based GUI (Flask)
	•	Interactive pose control via sliders
	•	Immediate robot response using ROS2 Action

🔹 MATLAB / Simulink Integration
	•	HTTP-based data bridge (webread)
	•	6-DOF PID control loop
	•	Simscape Multibody robot simulation

🔹 Offline Simulation Pipeline
	•	CSV trajectory export from ROS2
	•	MATLAB batch simulation (run_full_demo.m)
	•	Tracking and error visualization

📊 Performance
	•	Joint tracking error: ~10⁻³ rad level
	•	Stable closed-loop response
	•	Smooth motion via velocity-constrained planning

🖥️ Demo

🎬 Offline Mode
	•	Send target pose via GUI
	•	ROS2 generates trajectory (CSV)
	•	MATLAB runs simulation
	•	Outputs tracking + error plots

⚡ Real-Time Mode
	•	Interactive slider control
	•	Immediate robot response
	•	Workspace validation feedback

📁 Project Structure
.
├── matlab/
│   ├── run_full_demo.m
│   ├── get_joint_ref_http.m
│   ├── pid_control.slx
│   └── trajectory/
│       └── .gitkeep
│
├── ros2_ws/
│   ├── robot_task_manager/
│   └── robot_interfaces/
│
├── urdf/
│   └── gluon_6l3.urdf
│
└── README.md

🚀 How to Run
1. ROS2 (Task Execution)
  cd ros2_ws
  colcon build
  source install/setup.bash
  ros2 run robot_task_manager robot_task_manager
2. Web GUI
   ros2 run robot_task_manager pose_web_gui
   Open browser: http://localhost:8080
3. MATLAB Simulation
   Offline mode: run('matlab/run_full_demo.m')
   Real-time mode:
   •	Simulink reads joint references via HTTP
	 •	PID controller tracks reference

🧩 Technologies Used
	•	ROS2 (rclpy, Action interface)
	•	Python (Flask, NumPy, SciPy)
	•	MATLAB / Simulink
	•	Simscape Multibody
	•	Robotics Kinematics

💡 Engineering Highlights
	•	Modular system design (ROS2 + MATLAB decoupling)
	•	Asynchronous task execution with feedback
	•	Real-time and offline dual-mode control
	•	Velocity-constrained trajectory planning
	•	End-to-end pipeline from UI → control → simulation

📎 Future Work
	•	RViz / Gazebo integration
	•	Motion planning (MoveIt)
	•	Hardware deployment
	•	Sensor feedback integration

🧑‍💻 Author

Yantong Yang
Robotic Systems Engineering, RWTH Aachen University
:::
