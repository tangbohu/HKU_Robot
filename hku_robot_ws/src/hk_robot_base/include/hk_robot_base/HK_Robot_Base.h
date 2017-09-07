#ifndef HK_ROBOT_BASE_H
#define HK_ROBOT_BASE_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/callback_queue.h>
#include <tf/transform_listener.h>

#include"vec.h"
#include"conf.h"

class HK_Robot_Base
{
    private:
    Vector3f currentPose;

public:

     HK_Robot_Base();
     ~HK_Robot_Base();

    /////////////// robot control

    void gotoPose(Vector3f pose);
    void getPose(Vector3f& pose);
    Vector3f getPose();
    bool arriveAtPose(Vector3f pose, float tolerance =0.1, float r_tolerance=3.14/6.0);
    void setSpeed(Vector3f speed); //x,y,r
    void stop();



    //////////////// Ros related
    ros::NodeHandle nh_,n_;
    ros::Publisher   motor_publisher_;
    ros::Subscriber  pose_subscriber_;

    void poseSubscriberCallBack(const  TurbleBotSlamTopicName& );

    void workFlowSpin();

    //////////////////////Please Fill the Task
    void taskFlow();

};



#endif // HK_ROBOT_BASE_H
