gnome-terminal --tab -e  "bash -c \"source ~/catkin_ws/devel/setup.bash;sleep .2;rosrun master_discovery_fkie master_discovery;bash\""  --tab -e  "bash -c \"source ~/catkin_ws/devel/setup.bash;sleep .2;roslaunch master_sync_fkie msync.launch;bash\""       --tab -e "bash -c \"source ~/HKU_Robot/hku_robot_ws/devel/setup.bash; roslaunch rtabmap_ros turtlebot_launch.launch; bash\""  --tab -e "bash -c \"source ~/HKU_Robot/hku_robot_ws/devel/setup.bash; sleep 3; . amcl_nav.sh; bash\""  --tab -e "bash -c \"sleep 3;roslaunch turtlebot_rviz_launchers view_navigation.launch --screen;bash\""



