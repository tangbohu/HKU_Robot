#include "../include/hk_robot_base/HK_Robot_Base.h"

HK_Robot_Base::HK_Robot_Base():nh_("~")
{
    pose_subscriber_ = nh_.subscribe(TurtbleBotSlamNodeName, 10,
                &HK_Robot_Base::PoseSubscriberCallBack, this);
    motor_publisher_=nh_.advertise<TurtbleBotSlamCmdTopicName>(TurtbleBotSlamCmdNodeName,5);
}


HK_Robot_Base::~HK_Robot_Base()
{

}

void HK_Robot_Base::PoseSubscriberCallBack(const  TurbleBotSlamTopicName & msg)
{
    currentPose[0]=1;
    currentPose[1]=1;
    currentPose[2]=1;
}


void HK_Robot_Base::gotoPose(Vector3f pose)
{
   TurtbleBotSlamCmdTopicName PoseMessage;
   PoseMessage.header.stamp=ros::Time::now();
   PoseMessage.header.frame_id="/base_link";

   motor_publisher_.publish(PoseMessage);
}

void HK_Robot_Base::getPose(Vector3f& pose)
{
   pose=currentPose;
}

void HK_Robot_Base::setSpeed(Vector3f speed)
{

}

void HK_Robot_Base::stop()
{

}
