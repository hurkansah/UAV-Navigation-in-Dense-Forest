 rosbag record /mavros/local_position/pose /planning/ref_traj /orb_slam3/camera_pose /command/trajectory /mavros/local_position/odom /planning_vis/trajectory 
 
 rosservice call /orb_slam3/save_map slam_test1_forest0
 
  rosservice call /orb_slam3/save_traj slam_test1_traj
  
  rosrun plotjuggler plotjuggler

