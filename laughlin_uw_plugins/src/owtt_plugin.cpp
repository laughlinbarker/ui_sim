/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/*
 * Desc: 3D position interface for ground truth.
 * Author: Sachin Chitta and John Hsu
 * Date: 1 June 2008
 */

#include "../include/laughlin_uw_plugins/owtt_plugin.hpp"
#include <math.h>
#include <random>


namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(OWTTPlugin)

////////////////////////////////////////////////////////////////////////////////
// Constructor
OWTTPlugin::OWTTPlugin()
{
  this->seed = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
OWTTPlugin::~OWTTPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->update_connection_);
  // Finalize the controller
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void OWTTPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // save pointers
	# if GAZEBO_MAJOR_VERSION >= 7
	  std::string worldName = _parent->WorldName();
	# else
	  std::string worldName = _parent->GetWorldName();
	# endif

	  this->world_ = physics::get_world(worldName);
  this->sdf = _sdf;
  this->sensorParent = _parent;

  // ros callback queue for processing subscription
  this->deferred_load_thread_ = boost::thread(
    boost::bind(&OWTTPlugin::LoadThread, this));
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void OWTTPlugin::LoadThread()
{
  // load parameters
  this->robot_namespace_ = "";
  if (this->sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = this->sdf->Get<std::string>("robotNamespace");

  if (!this->sdf->HasElement("topicName"))
  {
    ROS_WARN("OWTT plugin missing <topicName>, defaults to /ship/bow_range");
    this->topic_name_ = "/ship/bow_ragne";
  }
  else
    this->topic_name_ = "/" + this->sdf->Get<std::string>("topicName");

  if (!this->sdf->HasElement("beaconLinkName"))
  {
    ROS_FATAL("OWTT plugin missing <baconLinkName>, cannot proceed");
    return;
  }
  else
    this->beacon_link_name_ = this->sdf->Get<std::string>("beaconLinkName");

  if (!this->sdf->HasElement("dunkerLinkName"))
  {
    ROS_FATAL("OWTT plugin missing <dunkerLinkName>, cannot proceed");
    return;
  }
  else
    this->dunker_link_name_ = this->sdf->Get<std::string>("dunkerLinkName");

  if (!this->sdf->HasElement("tdmaTopic"))
  {
    ROS_WARN("OWTT plugin missing <tdmaTopic>, defaulting to /tdma ");
    this->tdma_topic_ = "/tdma";
  }
  else
    this->tdma_topic_ = this->sdf->Get<std::string>("tdmaTopic");

  //get the beacon number, a required element since we rely upon a TDMA broadcast to trigger
  //range measurements
  if (!this->sdf->HasElement("beaconNum"))
  {
    ROS_FATAL("OWTT plugin missing <beaconNum>, cannot proceed ");
    return;
  }
  else
    this->beacon_number_ = this->sdf->GetElement("beaconNum")->Get<int>();

  if(beacon_number_ > 2)
  {
    ROS_WARN("You have specified more than two range beacons! Is this correct?");
  }

  if (!this->sdf->HasElement("rangeAccuracy"))
  {
    ROS_WARN("plugin missing <rangeAccuracy> parameter, defaults to 0.2 percent slant range");
    this->rngAccuracy_ = 0.002;
  }
  else
    this->rngAccuracy_ = this->sdf->GetElement("rangeAccuracy")->Get<double>();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // publish multi queue
  this->pmq.startServiceThread();

  // assert that the body by link_name_ exists
  this->beacon_link = boost::dynamic_pointer_cast<physics::Link>(
    this->world_->GetEntity(this->beacon_link_name_));
  if (!this->beacon_link)
  {
	  //std::cout << "Beacon id: " << this->beacon_link->GetId() << "\n";
    ROS_FATAL("OWTT plugin error: beaconLinkName: %s does not exist\n",
      this->beacon_link_name_.c_str());
    return;
  }

  // assert that the body by link_name_ exists
  this->dunker_link = boost::dynamic_pointer_cast<physics::Link>(
    this->world_->GetEntity(this->dunker_link_name_));
  if (!this->dunker_link)
  {
    //std::cout << "Beacon id: " << this->beacon_link->GetId() << "\n";
    ROS_FATAL("OWTT plugin error: dunkerLinkName: %s does not exist\n",
      this->dunker_link_name_.c_str());
    return;
  }

  //subscribe to TDMA topic that publishes integers corresponding
  //to which range sensor should fire
  tdma_sub_ = this->rosnode_->subscribe<std_msgs::Int16>
    (tdma_topic_, 10, boost::bind(&OWTTPlugin::tdmaCallback, this, _1));

  // if topic name specified as empty, do not publish
  if (this->topic_name_ != "")
  {
    this->pub_Queue = this->pmq.addPub<laughlin_uw_plugins::OWTT>();
    this->pub_ = this->rosnode_->advertise<laughlin_uw_plugins::OWTT>(
      this->topic_name_, 1);

  }

  // Initialize the controller
  this->last_time_ = this->world_->GetSimTime();

  // start custom queue for OWTT
  this->callback_queue_thread_ =
    boost::thread(boost::bind(&OWTTPlugin::OWTTQueueThread, this));


  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&OWTTPlugin::UpdateChild, this));
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void OWTTPlugin::UpdateChild()
{
  if(make_measurement_)
  {
  common::Time cur_time = this->world_->GetSimTime();

	// get transformations to beacon
	this->g_Inertial_OWTTBeacon.block<3,3>(0,0) = this->gazMatrix3_EigMatrix3d(this->beacon_link->GetWorldPose().rot.GetAsMatrix3());
	this->g_Inertial_OWTTBeacon(0,3) = this->beacon_link->GetWorldPose().pos[0];
	this->g_Inertial_OWTTBeacon(1,3) = this->beacon_link->GetWorldPose().pos[1];
	this->g_Inertial_OWTTBeacon(2,3) = this->beacon_link->GetWorldPose().pos[2];
	this->g_Inertial_OWTTBeacon(3,3) = 1;

	// get transformation to dunking transducer
	physics::EntityPtr sensorParentEntity = this->world_->GetEntity(this->sensorParent->ParentName());

	this->g_Inertial_OWTTDunker.block<3,3>(0,0) = this->gazMatrix3_EigMatrix3d(this->dunker_link->GetWorldPose().rot.GetAsMatrix3());
	this->g_Inertial_OWTTDunker(0,3) = this->dunker_link->GetWorldPose().pos[0];
	this->g_Inertial_OWTTDunker(1,3) = this->dunker_link->GetWorldPose().pos[1];
	this->g_Inertial_OWTTDunker(2,3) = this->dunker_link->GetWorldPose().pos[2];
	this->g_Inertial_OWTTDunker(3,3) = 1;

	// find location of beacon in dunking transducer frame
	this->g_tmp = g_Inertial_OWTTDunker.inverse() * g_Inertial_OWTTBeacon;
	this->v_tmp = g_tmp.block<4,1>(0,3);
	this->v_tmp(3) = 0;

	//compute range measurement and distance varrying standard deviation
	this->range_ = this->v_tmp.norm();
	this->rng_std_dev_ = this->rngAccuracy_ * this->range_;

	//compute noise and add to range
	std::normal_distribution<double> distribution(0.0,this->rng_std_dev_);
	this->range_ = this->range_ + distribution(this->generator);

	//recompute range but with added noise
	this->owtt_msg_.range = this->range_;
  this->owtt_msg_.header.stamp.sec = this->world_->GetSimTime().sec;
  this->owtt_msg_.header.stamp.nsec = this->world_->GetSimTime().nsec;

  this->owtt_msg_.header.frame_id = this->dunker_link_name_;

	{
      boost::mutex::scoped_lock lock(this->lock_);
      // publish to ros
      if (this->pub_.getNumSubscribers() > 0 && this->topic_name_ != "")
          this->pub_Queue->push(this->owtt_msg_, this->pub_);
  }
  make_measurement_ = false;  
}
}

////////////////////////////////////////////////////////////////////////////////
// Put data to the interface
void OWTTPlugin::OWTTQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->owtt_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

Eigen::Matrix3d OWTTPlugin::gazMatrix3_EigMatrix3d(math::Matrix3 inMatrix)
{
	Eigen::Matrix3d m;

	m(0,0) = inMatrix[0][0];
	m(0,1) = inMatrix[0][1];
	m(0,2) = inMatrix[0][2];
	m(1,0) = inMatrix[1][0];
	m(1,1) = inMatrix[1][1];
	m(1,2) = inMatrix[1][2];
	m(2,0) = inMatrix[2][0];
	m(2,1) = inMatrix[2][1];
	m(2,2) = inMatrix[2][2];

	return m;
}

double OWTTPlugin::wrapto360(double x){
    x = fmod(x,360);
    if (x < 0)
        x += 360;
    return x;
}

void OWTTPlugin::tdmaCallback(const std_msgs::Int16::ConstPtr& tdma)
{
  if(tdma->data == beacon_number_)
  {
    make_measurement_ = true;
  }
}

}
