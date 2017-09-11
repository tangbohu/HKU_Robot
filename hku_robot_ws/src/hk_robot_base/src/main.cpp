
#include "hk_robot_base/HK_Robot_Base.h"
#include<vector>
#include<iostream>

std::vector<Vector3f>  poses_;



void setAllPoses()   // the first pos should list in the back of the vector
{
    poses_.push_back(Vector3f());
    poses_.back()[0]=0.0;
    poses_.back()[1]=0.0;
    poses_.back()[2]=0.0;


}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "hku_robot_base");
    HK_Robot_Base robotBase;
    robotBase.InitATaskSchedule();

    robotBase.workFlowSpin(poses_);
    return 0;
}
