#include "omnicar_ros.h"

OmnicarRos::OmnicarRos()
{

    ros::param::param<std::string>("ns", robotNamespace, "");
    ros::param::param<std::string>("serial_port", serialPort, "/dev/ttyUSB0");
    ros::param::param<int>("control_freq_update", controlFreqUpdate, 50);
    ros::param::param<std::string>("cmd_vel_topic", cmdVelTopic, "cmd_vel");
    // ros::param::param<std::string>("odometry_topic", cmdVelTopic, "odom");
    ros::param::param<std::vector<std::string>>("robot_joint_names", robotJointNames, {"fl_caster_wheel", "fr_caster_wheel", "rl_caster_wheel", "rr_caster_wheel"});
    // if (ros::param::has("robot_joint_names")){

    // }

    jointStatesPublisher = n.advertise<sensor_msgs::JointState>("/joint_states", 2);
    // odomPublisher = n.advertise<nav_msgs::Odometry>("odom", 2);
    cmdVelSubscriber = n.subscribe(cmdVelTopic, 2, &OmnicarRos::controlLinearVelocityCallback, this);
    loopRatePtr = std::make_shared<ros::Rate>(controlFreqUpdate);

    serialCommunicationPtr = std::make_shared<SerialCommunication>(serialPort);
}
OmnicarRos::~OmnicarRos()
{
    delete jointStatesPublisher;
    // delete odomPublisher;
    delete cmdVelSubscriber;
    loopRatePtr.reset();
    serialCommunicationPtr->stop();
    serialCommunicationPtr.reset();

    ROS_INFO("Communication deactivated!");
}

void OmnicarRos::publishRobotState(){
    rosJointState.header.stamp = currentTime;
    rosJointState.name = robotJointNames;
    // Convert types
    rosJointState.position.assign(
        serialCommunicationPtr->robotState.jointsPosition.data(), 
        serialCommunicationPtr->robotState.jointsPosition.data()+serialCommunicationPtr->robotState.jointsPosition.rows()
    );
    rosJointState.velocity.assign(
        serialCommunicationPtr->robotState.jointsVelocity.data(),
        serialCommunicationPtr->robotState.jointsVelocity.data()+serialCommunicationPtr->robotState.jointsVelocity.rows()
    );
    
    jointStatesPublisher.publish(rosJointState);
    jointStatesPublisher.publish(rosJointState);
}

void OmnicarRos::controlLinearVelocityCallback(const geometry_msgs::Twist& msg){
    serialCommunicationPtr->linearVelocityControl<<msg.angular.z, msg.linear.x, msg.linear.y;
}


void OmnicarRos::initialize(){
    ROS_INFO("Trying to communicate...");
    serialCommunicationPtr->run();
    while(!isConnectionEnstablished()){
        loop();
    }
    ROS_INFO("Communication established!");
}

void OmnicarRos::loop(){
    ros::spinOnce();
    currentTime = ros::Time::now();
    publishRobotState();
    // publishOdom();
    loopRatePtr->sleep();
}

bool OmnicarRos::isConnectionEnstablished(){
    return serialCommunicationPtr->isConnectionEnstablished();
}


