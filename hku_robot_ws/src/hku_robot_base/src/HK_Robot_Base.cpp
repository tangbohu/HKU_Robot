#include "../include/hk_robot_base/HK_Robot_Base.h"
#include"tf2/impl/convert.h"
#include"tf/tf.h"
#include"move_base/move_base.h"
#include"move_base_msgs/MoveBaseGoal.h"
#include"tf/transform_listener.h"

HK_Robot_Base::HK_Robot_Base():nh_("~")
{
//    pose_subscriber_ = nh_.subscribe(TurtbleBotSlamNodeName, 10,
//                &HK_Robot_Base::poseSubscriberCallBack, this);

    motor_publisher_=nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",20);

}


HK_Robot_Base::~HK_Robot_Base()
{

}

//void HK_Robot_Base::poseSubscriberCallBack(const  TurbleBotSlamTopicName & msg)
//{
//    currentPose[0]=1;
//    currentPose[1]=1;
//    currentPose[2]=1;
//}

void HK_Robot_Base::gotoPose(geometry_msgs::PoseStamped pose)
{
   geometry_msgs::PoseStamped PoseMessage;
   PoseMessage.header.stamp=ros::Time::now();
   PoseMessage.header.frame_id="map";
   PoseMessage.pose=pose.pose;
   motor_publisher_.publish(PoseMessage);
}

void HK_Robot_Base::getPose(geometry_msgs::PoseStamped& pose)
{
   //pose=currentPose_;

   geometry_msgs::PoseStamped pBase;
   pBase.header.frame_id = "base_link";
   pBase.pose.position .x = 0.0;
   pBase.pose.position.y = 0.0;
   pBase.pose.position.z = 0.0;
   pBase.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
   ros::Time current_transform = ros::Time::now();
   tfListener.getLatestCommonTime(pBase.header.frame_id, "map", current_transform, NULL);
   pBase.header.stamp = current_transform;


   tf::StampedTransform xform;
   try
   {
     // The waypoint frame is the source frame
         tfListener.waitForTransform("map", "base_link", ros::Time::now(),ros::Duration(3));
         tfListener.lookupTransform("map", "base_link", ros::Time(0), xform);
   }
   catch (tf::TransformException ex)
   {
     ROS_ERROR("%s", ex.what());
   }

   geometry_msgs::PoseStamped pMap;

   pMap.header.stamp = ros::Time::now();
   pMap.header.frame_id = "map";

   pMap.pose.position.x = xform.getOrigin().getX();
   pMap.pose.position.y = xform.getOrigin().getY();
   pMap.pose.position.z = xform.getOrigin().getZ();

   pMap.pose.orientation.x = xform.getRotation().getX();
   pMap.pose.orientation.y = xform.getRotation().getY();
   pMap.pose.orientation.z = xform.getRotation().getZ();
   pMap.pose.orientation.w = xform.getRotation().getW();


   pose=pMap;

}

geometry_msgs::PoseStamped HK_Robot_Base::getPose()
{
    geometry_msgs::PoseStamped finalPose;
    getPose(finalPose);

    return finalPose;
}

 bool HK_Robot_Base::arriveAtPose(geometry_msgs::PoseStamped pose, float tolerance , float r_tolerance)
 {
     geometry_msgs::PoseStamped currentPose=getPose();

     geometry_msgs::Point currentpoint= currentPose.pose.position;
     geometry_msgs::Quaternion currentori=currentPose.pose.orientation;
     geometry_msgs::Point point= pose.pose.position;
     geometry_msgs::Quaternion ori=pose.pose.orientation;

      if( pow(currentpoint.x-point.x ,2)+ pow(currentpoint.y-point.y,2)+pow(currentpoint.z-point.z,2)<= tolerance*tolerance &&
               pow(currentori.w-ori.w ,2)+ pow(currentori.x-ori.x ,2) +pow(currentori.y- ori.y  ,2)+pow(currentori.z - ori.z ,2)<= r_tolerance*r_tolerance
              )
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

void HK_Robot_Base::workFlowSpin(vector<geometry_msgs::PoseStamped> poses_ )
{
    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        taskFlow(poses_);

        ros::spinOnce();
        loop_rate.sleep();
    }
}



void HK_Robot_Base::InitAGotoPoseSchedule()
{
    gotoPose_init_=true;
}

bool HK_Robot_Base::AGotoPoseScheduleFromSendPosUntilArrive(geometry_msgs::PoseStamped pos_)
{
    if(gotoPose_init_)  // should deliver only once
    {
    ROS_INFO("a gxxxxxxxxxxxxxxxxxx %f, %f, %f",pos_.pose.position.x,pos_.pose.position.y,pos_.pose.position.z);
        gotoPose(pos_);

        ROS_INFO("a yyyyyyyyyyyyyyyyyyyy");

        gotoPose_init_=false;
    }

    if(arriveAtPose(pos_))
        return true;
    else
    {
    //   ROS_INFO("pub again task");
     //      gotoPose(pos_);
        return false;
    }
}

void HK_Robot_Base::InitATaskSchedule()
{
    aTask_init_=true;
}

bool HK_Robot_Base::ATaskOfGotoPose(geometry_msgs::PoseStamped pos_)
{
    if(aTask_init_)
    {
        InitAGotoPoseSchedule();
        aTask_init_=false;
    }
    return AGotoPoseScheduleFromSendPosUntilArrive(pos_);
}

bool HK_Robot_Base::taskFlow(vector<geometry_msgs::PoseStamped>& poses_)
{
   ROS_INFO("In task flow %d",poses_.size());

    if(!poses_.empty())
    {
       if( ATaskOfGotoPose(poses_.back()))  // a goto pose task is finished.
       {
              geometry_msgs::PoseStamped ps= poses_.back();

              geometry_msgs::Point p=ps.pose.position;
            ROS_INFO("The Position (%.2f, %.2f, %.2f) is arrived!", p.x,p.y,p.z);
            ROS_INFO("The Position (%.2f, %.2f, %.2f) is arrived!", p.x,p.y,p.z);
            ROS_INFO("The Position (%.2f, %.2f, %.2f) is arrived!", p.x,p.y,p.z);
            ROS_INFO("The Position (%.2f, %.2f, %.2f) is arrived!", p.x,p.y,p.z);

            poses_.pop_back();
            ROS_INFO("Poses left %d!", poses_.size());

            InitATaskSchedule();
       }

       return false;
    }
    return true;
}
