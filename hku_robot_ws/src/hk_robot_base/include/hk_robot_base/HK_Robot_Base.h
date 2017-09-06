#ifndef HK_ROBOT_BASE_H
#define HK_ROBOT_BASE_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/callback_queue.h>
#include <tf/transform_listener.h>

#include"vec.h"
#include"conf.h"

class HK_Bobot_Base
{
    private:
    Vector3f currentPose;

public:

     HK_Robot_Base();
     ~HK_Robot_Base();

    /////////////// robot control

    void gotoPose(Vector3f pose);
    void getPose(Vector3f& pose);
    void setSpeed(Vector3f speed); //x,y,r
    void stop();



    //////////////// Ros related
    ros::NodeHandle nh_,n_;
    ros::Publisher   motor_publisher_;
    ros::Subscriber  pose_subscriber_;

    void PoseSubscriberCallBack(const  TurbleBotSlamTopicName &);


}



#endif // HK_ROBOT_BASE_H
