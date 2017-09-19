roslaunch rtabmap_ros rtabmap_without_ui.launch rtabmap_args:="--delete_db_on_start"  namespace:="rtabmap1" rgb_topic:="/camera/rgb/image_rect_color" depth_topic:="/camera/depth_registered/image_raw" camera_info_topic:="/camera/rgb/camera_info" rgbd_topic:="/camera/rgbd_image" frame_id:="camera_link" visual_odometry:=true

