
#include "hk_robot_base/HK_Robot_Base.h"

int main(int argc,char **argv)
{
    ros::init(argc, argv, "hku_robot_base");
    HK_Robot_Base robotBase;

    robotBase.workFlowSpin();

    return 0;
}
