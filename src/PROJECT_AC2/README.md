
>https://education.github.com/git-cheat-sheet-education.pdf
>

# PROJECT_AC2

Before start to build, copy paste the poseutils lib (it is inside of the Fast Planner) to your (usr/share/lib)

And also check the other projects' git pages to meet system requirements. 

To do a catkin build you should first build multi_map_surver

>catkin build multi_map_surver
>

Then build all packages 

>catkin build
>

Still, if you get some error use this command 

> catkin config --merge-devel
>

clean workspace 

>catkin clean
>

and do catkin build again

Also, in Fast-Planner, I used these sources to edit some codes

> https://github.com/HKUST-Aerial-Robotics/Fast-Planner/issues/117
>

>https://blog.csdn.net/weixin_43793717/article/details/131072185
>

Also, you should install nlopt using the source file from this one 

>https://github.com/Junking1/Fast-Planner-for-ubuntu20.04
>


SOURCE GIT PAGES for this project 

>https://github.com/HKPolyU-UAV/E2ES
>

>https://github.com/thien94/orb_slam3_ros
>

>https://github.com/HKUST-Aerial-Robotics/Fast-Planner
>


>https://github.com/deepak-1530/FastPlannerOctomap
>

>https://github.com/Jaeyoung-Lim/mavros_controllers
>

>https://github.com/appliedAI-Initiative/orb_slam_2_ros
>

To run whole code,

1) go roscd e2es and run the gazebo and px4 sitl using this command 
>./sim.sh
>

2.)run controller 

>roslaunch geometric_controller sitl_trajectory_track_circle.launch
>

3.) To run planner parts 

- run orb slam2 (current because of the frame problem we are using orbslam2)
>roslaunch orb_slam2_ros orb_slam2_d435_mono.launch
>

-run the mapping (rviz and octomap server) 
>roslaunch FastPlannerOctomap MappingSim_2.launch
>

-run the planner node 
> rosrun FastPlannerOctomap Planner
>

enter a number, then assign a 2d navigation point from Rviz, it will ask you the desired altitude also. 



