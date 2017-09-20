
#include "hk_robot_base/HK_Robot_Base.h"
#include<vector>
#include<iostream>
#include<fstream>
std::vector<geometry_msgs::PoseStamped>  poses_;



bool setAllPoses(std::string fname)   // the first pos should list in the back of the vector
{
    std::ifstream fin;
    fin.open(fname.c_str());
    if(fin.is_open()==false)
    {
        ROS_INFO("failed to open ");
        return false;
    }
    else
    {
        int num;
        fin>>num;
        for(int i=0;i<num;i++)
        {
            geometry_msgs::PoseStamped pos1;
            fin>>pos1.pose.position.x>>pos1.pose.position.y>>pos1.pose.position.z;
            fin>> pos1.pose.orientation.x>> pos1.pose.orientation.y>> pos1.pose.orientation.z>> pos1.pose.orientation.w;
            pos1.pose.orientation= tf::createQuaternionMsgFromYaw(0.0);
            pos1.header.frame_id="map";
            poses_.push_back(pos1);

        }



    }
    fin.close();


}

int main(int argc,char **argv)
{
    if(argc<1)
    {
       ROS_INFO("should input path file name containing path point");
        return 0;
    }


    setAllPoses(argv[1]);
    ROS_INFO("before  %d",poses_.size());


    ros::init(argc, argv, "hku_robot_base");
    HK_Robot_Base robotBase;

    sleep(3);  //ros need several second to start
    robotBase.InitATaskSchedule();

    robotBase.workFlowSpin(poses_);
    return 0;
}
