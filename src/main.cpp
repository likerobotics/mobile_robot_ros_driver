#include "omnicar_ros.h"

#define LOOP_RATE 100

 
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "omnicar_driver");
    OmnicarRos robot;
    robot.initialize();
    while(ros::ok()){
        robot.loop();
    }
}