#ifndef HK_ROBOT_BASE_H
#define HK_ROBOT_BASE_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include<geometry_msgs/Quaternion.h>
#include <ros/callback_queue.h>
#include <tf/transform_listener.h>
#include<vector>

#include"vec.h"
#include"conf.h"

using namespace std;

class HK_Robot_Base
{
    private:
    geometry_msgs::PoseStamped currentPose;
    bool gotoPose_init_=true;
    bool aTask_init_=true;

public:

     HK_Robot_Base();
     ~HK_Robot_Base();

    /////////////// robot control

    void gotoPose(geometry_msgs::PoseStamped pose);
    void getPose(geometry_msgs::PoseStamped& pose);
    geometry_msgs::PoseStamped getPose();
    bool arriveAtPose(geometry_msgs::PoseStamped pose, float tolerance =0.1, float r_tolerance=100);
    void setSpeed(Vector3f speed); //x,y,r
    void stop();



    //////////////// Ros related
    ros::NodeHandle nh_,n_;
    ros::Publisher   motor_publisher_;
    ros::Subscriber  pose_subscriber_;
    tf::TransformListener tfListener;
    //void poseSubscriberCallBack(const  TurbleBotSlamTopicName& );

    void workFlowSpin(vector<geometry_msgs::PoseStamped> poses_ );

    //////////////////////Please Fill the Task

    void InitAGotoPoseSchedule();
    bool AGotoPoseScheduleFromSendPosUntilArrive(geometry_msgs::PoseStamped pos_);

    void InitATaskSchedule();
    bool ATaskOfGotoPose(geometry_msgs::PoseStamped pos_);

    bool taskFlow(vector<geometry_msgs::PoseStamped>& poses_ );

};



#endif // HK_ROBOT_BASE_H
