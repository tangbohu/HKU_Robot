#gnome-terminal -e  roslaunch turtlebot_bringup minimal.launch
gnome-terminal --tab -e "bash -c \"source ~/HKU_Robot/hku_robot_ws/devel/setup.bash; roslaunch rtabmap_ros turtlebot_launch.launch; bash\""  --tab -e "bash -c \"source ~/HKU_Robot/hku_robot_ws/devel/setup.bash; sleep 5; . amcl_nav.sh; bash\""  --tab -e  "bash -c \" source ~/HKU_Robot/hku_robot_ws/devel/setup.bash; sleep 2;. rtabmap_without_ui.sh; bash\""   
#--tab -e "bash -c \"source ~/turtlebot/devel/setup.bash; sleep 5;roslaunch turtlebot_rviz_launchers view_navigation.launch --screen; bash\""    
#--tab -e  "bash -c \"sleep .4;rosrun master_discovery_fkie master_discovery\""  
#  --tab -e  "bash -c \"sleep .4;roslaunch master_sync_fkie msync.launch\""    
#rostopic echo /move_base_simple/goal



