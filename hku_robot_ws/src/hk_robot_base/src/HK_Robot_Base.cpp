#include "../include/hk_robot_base/HK_Robot_Base.h"

HK_Robot_Base::HK_Robot_Base():nh_("~")
{
    pose_subscriber_ = nh_.subscribe(TurtbleBotSlamNodeName, 10,
                &HK_Robot_Base::poseSubscriberCallBack, this);
    motor_publisher_=nh_.advertise<TurtbleBotSlamCmdTopicName>(TurtbleBotSlamCmdNodeName,5);
}


HK_Robot_Base::~HK_Robot_Base()
{

}

void HK_Robot_Base::poseSubscriberCallBack(const  TurbleBotSlamTopicName & msg)
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
   pose=currentPose_;
}

Vector3f HK_Robot_Base::getPose()
{
    Vector3f finalPose;
    getPose(finalPose);

    return finalPose;
}

 bool HK_Robot_Base::arriveAtPose(Vector3f pose, float tolerance =0.1, float r_tolerance)
 {
     Vector3i currentPose=getPose();
      if( pow(currentPose[0]-pose[0],2)+ pow(currentPose[1]-pose[1],2)<= tolerance*tolerance && std::abs(currentPose[2]-pose[2])<= r_tolerance)
          return true;
      else
          return false;
 }

void HK_Robot_Base::setSpeed(Vector3f speed)
{

}

void HK_Robot_Base::stop()
{

}


void HK_Robot_Base::workFlowSpin()
{
    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        taskFlow();

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void HK_Robot_Base::taskFlow()
{



}
