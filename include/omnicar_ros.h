#include <ros/ros.h>
#include "serial_communication.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class OmnicarRos{
public:
    OmnicarRos();
    ~OmnicarRos();
    void initialize();
    bool isConnectionEnstablished();
    void loop();

private:
    int controlFreqUpdate;
    std::string serialPort;
    std::string cmdVelTopic;
    std::string robotNamespace;
    std::vector<std::string> robotJointNames;
    ros::Time currentTime;

    ros::NodeHandle n;
    tf::TransformBroadcaster odomBroadcaster;
    ros::Publisher jointStatesPublisher;
    ros::Publisher odomPublisher;
    ros::Subscriber cmdVelSubscriber;

    std::shared_ptr<ros::Rate> loopRatePtr;
    std::shared_ptr<SerialCommunication> serialCommunicationPtr;

    sensor_msgs::JointState rosJointState;

    void publishRobotState();
    void publishOdom();
    void controlLinearVelocityCallback(const geometry_msgs::Twist& msg);
    
};