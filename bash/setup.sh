gnome-terminal --tab -e  "bash -c \"sleep .4;rosrun master_discovery_fkie master_discovery;bash\""  --tab -e  "bash -c \"sleep .4;roslaunch master_sync_fkie msync.launch;bash\""       --tab -e "bash -c \"source ~/HKU_Robot/hku_robot_ws/devel/setup.bash; roslaunch rtabmap_ros turtlebot_launch.launch; bash\""  --tab -e "bash -c \"source ~/HKU_Robot/hku_robot_ws/devel/setup.bash; sleep 5; . amcl_nav.sh; bash\""  




