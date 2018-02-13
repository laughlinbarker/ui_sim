// This node computes and publishes the vehicle's ice-relative pose, by subscribing to geometry_msgs/Odometry
// messages published by the gazebo_ros_p3d.so plugin, which publishes a body's relative position, attitude,
// angular and linear velocity in the Gazebo simulation. 
// 
// This node was written because the UUV controller uses the vehicle's true linear and angular velocity
// in computing controll commands. Recent versions of buffer_core.cpp had a method lookupTwist(), which
// computed relative twist between two frames, but has been commented out in source since 2013 (see John Boren
// comment on Git). Thus we manually compute relative twist and pose between two frames given their world-referenced
// poses and twists.

#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

class RelOdom
{
public:
	RelOdom();
protected:
	ros::NodeHandle nh_;

	ros::Subscriber vehicleOdomSub_;
	ros::Subscriber iceOdomSub_;

	ros::Publisher iceRelativeOdomPub_;

	//world referenced poses for vehicle and ice
	geometry_msgs::PoseStamped vehPose_;
	geometry_msgs::PoseStamped icePose_;

	//world-referenced velocities
	geometry_msgs::TwistStamped vehTwist_;
	geometry_msgs::TwistStamped iceTwist_;

	//ice-referenced pose & vel
	nav_msgs::Odometry iceRelOdom_;

	//ros has two quaternion types, 'msg' and 'tf', we use 'tf' type for math
	tf::Quaternion iceQuat_;
	tf::Quaternion vehQuat_;

	tf::Transform iceTF_;
	tf::Transform vehTF_;
	tf::Transform iceRelativeTF_;

	tf::Vector3 iceRelLinVel_;
	tf::Vector3 iceRelAngVel_;

	//subscribing and publishing topics
	std::string vehOdomTopic_;
	std::string iceOdomTopic_;
	std::string iceRelativeOdomTopic_;

	void VehOdomCallback(const nav_msgs::Odometry::ConstPtr& odom);
	void IceOdomCallback(const nav_msgs::Odometry::ConstPtr& odom);

};

RelOdom::RelOdom()
{
	//get topic names for subscribers and publishers
	if(!nh_.getParam("/ice_rel_odom/ice_odom_topic",iceOdomTopic_))
	{
		ROS_ERROR("/ice_rel_odom/ice_odom_topic parameter not specified, assuming default: /ice/pose_gt");
		iceOdomTopic_ = "/ice/pose_gt";
	}

	if(!nh_.getParam("/ice_rel_odom/veh_odom_topic",vehOdomTopic_))
	{
		ROS_ERROR("/ice_rel_odom/veh_odom_topic parameter not specified, assuming default: /rexrov/pose_gt");
		vehOdomTopic_ = "/rexrov/pose_gt";
	}

	if(!nh_.getParam("/ice_rel_odom/ice_relative_odom_topic",iceRelativeOdomTopic_))
	{
		ROS_ERROR("/ice_rel_odom/ice_relative_odom_topic parameter not specified, assuming default: /rexrov/ice_pose_gt");
		iceRelativeOdomTopic_ = "/rexrov/ice_pose_gt";
	}

	//subscribe to ground truth topics
	vehicleOdomSub_ = nh_.subscribe<nav_msgs::Odometry>(vehOdomTopic_, 10, &RelOdom::VehOdomCallback, this);
	iceOdomSub_ = nh_.subscribe<nav_msgs::Odometry>(iceOdomTopic_, 10, &RelOdom::IceOdomCallback, this);

	//advertise ice-relative odometry topic
	iceRelativeOdomPub_ = nh_.advertise<nav_msgs::Odometry>(iceRelativeOdomTopic_, 10);
}

void RelOdom::VehOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
	//convert pose message to TF transformation
	poseMsgToTF(odom->pose.pose, vehTF_);

	//compute relative ice-vehicle transformation
	iceRelativeTF_ = iceTF_.inverse() * vehTF_;

	poseTFToMsg(iceRelativeTF_, iceRelOdom_.pose.pose);

	// compute relative orientation of vehicle in ice frame
	quaternionMsgToTF(icePose_.pose.orientation, iceQuat_);
	quaternionMsgToTF(odom->pose.pose.orientation, vehQuat_);

	// //compute relative attitude, converto to TF type quat and save to appropriate iceRelOdom variable
	// quaternionTFToMsg(vehQuat_ * iceQuat_.inverse(), iceRelOdom_.pose.pose.orientation);

	//compute relative twist of vehicle (this is in world frame)
	iceRelLinVel_.setX(odom->twist.twist.linear.x - iceTwist_.twist.linear.x);
	iceRelLinVel_.setY(odom->twist.twist.linear.y - iceTwist_.twist.linear.y);
	iceRelLinVel_.setZ(odom->twist.twist.linear.z - iceTwist_.twist.linear.z);

	iceRelAngVel_.setX(odom->twist.twist.angular.x - iceTwist_.twist.angular.x);
	iceRelAngVel_.setY(odom->twist.twist.angular.y - iceTwist_.twist.angular.y);
	iceRelAngVel_.setZ(odom->twist.twist.angular.z - iceTwist_.twist.angular.z);

	//rotate velocities into ice frame
	iceRelLinVel_ = iceRelLinVel_.rotate(iceQuat_.getAxis(),-iceQuat_.getAngle());
	iceRelAngVel_ = iceRelAngVel_.rotate(iceQuat_.getAxis(),-iceQuat_.getAngle());

	iceRelOdom_.twist.twist.linear.x = iceRelLinVel_.getX();
	iceRelOdom_.twist.twist.linear.y = iceRelLinVel_.getY();
	iceRelOdom_.twist.twist.linear.z = iceRelLinVel_.getZ();

	iceRelOdom_.twist.twist.angular.x = iceRelAngVel_.getX();
	iceRelOdom_.twist.twist.angular.y = iceRelAngVel_.getY();
	iceRelOdom_.twist.twist.angular.z = iceRelAngVel_.getZ();

	// iceRelOdom_.twist.twist.linear.x = odom->twist.twist.linear.x - iceTwist_.twist.linear.x;
	// iceRelOdom_.twist.twist.linear.y = odom->twist.twist.linear.y - iceTwist_.twist.linear.y;
	// iceRelOdom_.twist.twist.linear.z = odom->twist.twist.linear.z - iceTwist_.twist.linear.z;

	// iceRelOdom_.twist.twist.angular.x = odom->twist.twist.angular.x - iceTwist_.twist.angular.x;
	// iceRelOdom_.twist.twist.angular.y = odom->twist.twist.angular.y - iceTwist_.twist.angular.y;
	// iceRelOdom_.twist.twist.angular.z = odom->twist.twist.angular.z - iceTwist_.twist.angular.z;
	
	//publish relative pose
	iceRelativeOdomPub_.publish(iceRelOdom_);
}

void RelOdom::IceOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
	//set pose
	icePose_.pose = odom->pose.pose;
	
	//set twist
	iceTwist_.twist = odom->twist.twist;

	poseMsgToTF(icePose_.pose,iceTF_);
}

int main(int argc, char* argv[])
{

	ros::init(argc, argv, "ice_rel_odom_broadcaster");

	RelOdom relOdomBroadcaster;

    while(ros::ok())
    {
    	ros::spin();
    }

	return 0;

}