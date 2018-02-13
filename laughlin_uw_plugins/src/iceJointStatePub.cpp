#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_broadcaster.h>

#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>

class IceTfPub
{
public:
    IceTfPub();
    sensor_msgs::JointState joint_state;
    ros::Publisher joint_pub;
    tf2_ros::TransformBroadcaster broadcaster;
    ros::Subscriber sub_gaz_states ;
    geometry_msgs::TransformStamped ice_transformation;
    ros::Timer timer;
private:
    void gazeboModelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& modelStates);
    void timerCallback(const ros::TimerEvent& event);
    ros::NodeHandle n;

};

IceTfPub::IceTfPub()
{
    joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    sub_gaz_states = n.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, &IceTfPub::gazeboModelStatesCallback, this);
    ice_transformation.header.frame_id = "/world";
    ice_transformation.child_frame_id = "/sheet";

    timer = n.createTimer(ros::Duration(0.1), &IceTfPub::timerCallback, this);

        //update joint_state
        joint_state.name.resize(2);
        joint_state.position.resize(2);
        joint_state.name[0] ="ice_gps1";
        joint_state.position[0] = 0;
        joint_state.name[1] ="ice_gps2";
        joint_state.position[1] = 0;
        // joint_state.name[2] ="ice_hull";
        // joint_state.position[2] = 0;

}

void IceTfPub::timerCallback(const ros::TimerEvent& event)
{
        joint_state.header.stamp = ros::Time::now();

        //send the joint state and transform
        joint_pub.publish(joint_state);
        broadcaster.sendTransform(ice_transformation);

}

void IceTfPub::gazeboModelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& modelStates)
{
    enum {groundPlane, auv, ice};

    ice_transformation.header.stamp = ros::Time::now();

    ice_transformation.transform.translation.x = modelStates->pose[ice].position.x;
    ice_transformation.transform.translation.y = modelStates->pose[ice].position.y;
    ice_transformation.transform.translation.z = modelStates->pose[ice].position.z;
    ice_transformation.transform.rotation = modelStates->pose[ice].orientation;
}

int main(int argc, char** argv) 
{

    ros::init(argc, argv, "joint_state_publisher");

    IceTfPub tfPub;

    while (ros::ok()) 
    {
        ros::spin();
    }


    return 0;
}