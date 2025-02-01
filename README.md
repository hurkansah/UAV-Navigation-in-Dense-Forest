# Autonomous UAV Navigation in Complex Environments

# Overview
This project presents a navigation framework for autonomous UAVs operating in cluttered, GPS-denied environments such as forests. The system integrates ORB-SLAM3, a path planner (Fast Planner / RAPTOR), and object detection to ensure safe, real-time trajectory planning.

# Key Features
Simulation Environment: Gazebo-based forest simulations for testing.
SLAM & Localization: ORB-SLAM3 with stereo vision for accurate obstacle detection.
Path Planning: Fast Planner (RAPTOR) for real-time, collision-free trajectory generation.
Object Detection: YOLOv8-based model distinguishing hard (trees) and soft (bushes) objects.
Geometric Control: Ensures stable and accurate trajectory following.
System Components
Visual SLAM

#ORB-SLAM3 (stereo) for real-time mapping and localization.
Integration with Intel RealSense D435 for depth sensing.
Path Planning

-RAPTOR planner generates optimized paths using ESDF-based trajectory smoothing.
Collision avoidance and dynamic re-planning in real-time.
Controller

-Geometric tracking controller for robust flight stability.
PID-based tuning with integral compensation.
Object Detection

-Trained YOLOv8 model with 265K+ labeled forest images.
Smart polygon annotations for improved accuracy.
Simulation Setup

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

 
