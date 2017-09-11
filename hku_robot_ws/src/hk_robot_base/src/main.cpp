
#include "hk_robot_base/HK_Robot_Base.h"
#include<vector>
#include<iostream>

std::vector<geometry_msgs::PoseStamped>  poses_;



void setAllPoses()   // the first pos should list in the back of the vector
{
    geometry_msgs::PoseStamped pos1;
    pos1.pose.position.x=0.0;
    pos1.pose.position.y=0.0;
    pos1.pose.position.z=0.0;
    pos1.pose.orientation= tf::createQuaternionMsgFromYaw(0.0);
    poses_.push_back(pos1);



}

int main(int argc,char **argv)
{
    setAllPoses();
    ROS_INFO("before  %d",poses_.size());


    ros::init(argc, argv, "hku_robot_base");
    HK_Robot_Base robotBase;
    robotBase.InitATaskSchedule();

    robotBase.workFlowSpin(poses_);
    return 0;
}
