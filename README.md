# Autonomous UAV Navigation in Complex Environments

This project presents a navigation framework for autonomous UAVs operating in cluttered, GPS-denied environments such as forests. The system integrates ORB-SLAM3, a path planner (Fast Planner / RAPTOR), and object detection to ensure safe, real-time trajectory planning.

# Key Features
Simulation Environment: Gazebo-based forest simulations for testing.
SLAM & Localization: ORB-SLAM3 with stereo vision for accurate obstacle detection.
Path Planning: Fast Planner (RAPTOR) for real-time, collision-free trajectory generation.
Object Detection: YOLOv8-based model distinguishing hard (trees) and soft (bushes) objects.
Geometric Control: Ensures stable and accurate trajectory following.
System Components

# Visual SLAM
[ORB-SLAM3 ROS PACKAGE](https://github.com/thien94/orb_slam3_ros)
A ROS implementation of ORB_SLAM3 by thien94. 

-ORB-SLAM3 (stereo) for real-time mapping and localization.
Integration with Intel RealSense D435 for depth sensing. Correcting axes coordinates. 

# Path Planning
[Fast Planener](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)

HKUST-Aerial-Robotics, A Robust and Efficient Trajectory Planner for Quadrotors

-RAPTOR planner generates optimized paths using ESDF-based trajectory smoothing.
Collision avoidance and dynamic re-planning in real-time.
To connect with controller and flight manager, the code is arranged

# Controller
[Geometric Controller](https://github.com/Jaeyoung-Lim/mavros_controllers)
Jaeyoung-Lim, Aggressive trajectory tracking using mavros for PX4 enabled vehicles.

-Geometric tracking controller for robust flight stability.
PID-based tuning with integral compensation.

# Object Detection
[YoloV3](https://github.com/leggedrobotics/darknet_ros)
leggedrobotics, YOLO ROS: Real-Time Object Detection for ROS.

-Trained YOLOv3 model with 265K+ labeled forest images.
Smart polygon annotations for improved accuracy.

# Global Planner (A Manager)

A local planner utilizes goal information to generate safe trajectories for the geometric controller, avoiding collisions with obstacles. This information is managed by a  dedicated system that assigns new goals after each completed trajectory, ensuring continuous operation. If the planner detects obstacles or object detection identifies hard objects, the state manager selects the next goal from its pre-defined list. It’s
important to note that this component is not a robust global planner for environment exploration. 
[State Manager](resources/statemanager.pdf) 

# Integration with PX4, Geometric Controller and Fast Planner 
[PX4 Fast Planner](https://github.com/mzahana/px4_fast_planner)
Mzahana, Integration of Fast-Planner with PX4 autopilot for multi-rotor fast navigation with obstacle avoidance.

This is a really helpful implementation for understanding TF frames needed by fast-planner and integration of the geometric controller. 

# Simulation
![Autonomous UAV in Gazebo](resources/drone_gazebo.png) 

Simulation experiments were conducted using a desktop computer with an Nvidia RTX 3060 graphics card. To achieve better results, Nvidia CUDA and CUDNN plugins were utilized. The experiments were performed on Ubuntu 20.04 LTS and ROS Noetic within PX4’s Software-In-The-Loop (SITL) environment. The Gazebo simulator version 11.0 was used with a simulated Intel RealSense D435 depth camera plugin connected to the Forest Drone shown in Figure. 

-PX4 SITL and Gazebo 11.
Test cases include various forest densities and obstacle layouts.
Performance & Results
Localization Accuracy: Mean error between 0.5 - 0.8 meters.
Trajectory Following: Average deviation ~0.1m.
Object Detection: High precision for hard/soft object classification.
Future Work
Real-world testing with an actual drone platform.
Extended multi-UAV navigation for cooperative exploration.
Integration with reinforcement learning for adaptive flight behavior.
