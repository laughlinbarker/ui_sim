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

#include "../include/laughlin_uw_plugins/gps_plugin.hpp"

#include <math.h>
#include <random>


namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GPSPlugin)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GPSPlugin::GPSPlugin()
{
  this->seed = 0;
  this->earth = GeographicLib::Geocentric::WGS84();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GPSPlugin::~GPSPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->update_connection_);
  // Finalize the controller
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GPSPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
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
    boost::bind(&GPSPlugin::LoadThread, this));
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GPSPlugin::LoadThread()
{
  // load parameters
  this->robot_namespace_ = "";
  if (this->sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = this->sdf->Get<std::string>("robotNamespace");

  if (this->sdf->HasElement("topicName"))
    this->topic_name_ = "/" + this->sdf->Get<std::string>("topicName");
  else
	  gzthrow("GPS plugin requires a <topicName>!")

  if (this->sdf->HasElement("velStdDev")) //units m/s
	   this->vel_stddev = this->sdf->Get<double>("velStdDev");
  else
	  this->vel_stddev = 0.1; //[m/s] according to uBlox LEA-6 data sheet
  
  if (this->sdf->HasElement("hdgStdDev")) //units degrees
  this->hdg_stddev = this->sdf->Get<double>("hdgStdDev");
  else
    this->hdg_stddev = 0.4237; //corresponds to 0.5deg CEP, (0.4237 deg)


  if (!this->sdf->HasElement("horizStdDev"))  //units m
  {
    ROS_INFO("GPS plugin missing <horizStdDev>, defaults to 0.0");
    this->horiz_stddev = 0.0;
  }
  else
    this->horiz_stddev = this->sdf->Get<double>("horizStdDev");

  if (!this->sdf->HasElement("vertStdDev"))   //units m
  {
    ROS_INFO("GPS plugin missing <vertStdDev>, defaults to 0.0");
    this->vert_stddev = 0.0;
  }
  else
    this->vert_stddev = this->sdf->Get<double>("vertStdDev");

  if (!this->sdf->HasElement("updateRate"))
  {
    ROS_DEBUG("GPS plugin missing <updateRate>, defaults to 1.0");
    this->update_rate_ = 1.0;
  }
  else
    this->update_rate_ = this->sdf->GetElement("updateRate")->Get<double>();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  //get world coordinates
  this->sphericalCoords_ = this->world_->GetSphericalCoordinates();

  // publish multi queue
  this->pmq.startServiceThread();

  // if topic name specified as empty, do not publish
  if (this->topic_name_ != "")
  {
    this->pub_Queue = this->pmq.addPub<gps_common::GPSFix>();
    this->pub_ = this->rosnode_->advertise<gps_common::GPSFix>(
      this->topic_name_, 1);

  }

  // Initialize the controller
  this->last_time_ = this->world_->GetSimTime();

  // start custom queue for USBL
  this->callback_queue_thread_ =
    boost::thread(boost::bind(&GPSPlugin::GPSQueueThread, this));

  //initialize GeographicLib stuff

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GPSPlugin::UpdateChild, this));
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GPSPlugin::UpdateChild()
{
  common::Time cur_time = this->world_->GetSimTime();

  // rate control
  if (this->update_rate_ > 0 &&
      (cur_time - this->last_time_).Double() < (1.0 / this->update_rate_))
    return;

  if ((this->pub_.getNumSubscribers() > 0) && this->topic_name_ != "")
  {

	// get GPS position in world XYZ
	physics::EntityPtr sensorParentEntity = this->world_->GetEntity(this->sensorParent->ParentName());
	this->positionXYZ = sensorParentEntity->GetWorldPose().pos.Ign();

	//compute noise and add to position
	std::normal_distribution<double> h_distribution(0.0,this->horiz_stddev);
	std::normal_distribution<double> v_distribution(0.0,this->vert_stddev);

	this->positionXYZ.X(this->positionXYZ.X() + h_distribution(this->generator));
	this->positionXYZ.Y(this->positionXYZ.Y() + h_distribution(this->generator));
	this->positionXYZ.Z(this->positionXYZ.Z() + v_distribution(this->generator));

	//convert to global coordinates
	double lat = this->sphericalCoords_->LatitudeReference().Degree();
	double lon = this->sphericalCoords_->LongitudeReference().Degree();

	this->localCart = GeographicLib::LocalCartesian(lat,lon,0,this->earth);


	//bug in Gazebo SphericaFromLocal using GeographicLib instead
	//ignition::math::Vector3d latLongAlt = this->sphericalCoords_->SphericalFromLocal(positionXYZ);

	//build NavSatFix message
	this->gps_msg_.header.stamp = ros::Time::now();
	this->gps_msg_.header.frame_id = "/world";
	//this->gps_msg_.latitude = latLongAlt.X();
	//this->gps_msg_.longitude = latLongAlt.Y();
	//this->gps_msg_.altitude = latLongAlt.Z();

	localCart.Reverse(this->positionXYZ.X(), this->positionXYZ.Y(), this->positionXYZ.Z(),
			gps_msg_.latitude, gps_msg_.longitude, gps_msg_.altitude);

	//velocity of sensor in world coords
	gazebo::math::Vector3 link_velocity = sensorParentEntity->GetWorldLinearVel();

  // Speed over ground noise distribution
  std::normal_distribution<double> vel_distribution(0.0,this->vel_stddev);

  //adding equally distribuited noise directional component of velocity
  link_velocity.x += vel_distribution(this->generator);
  link_velocity.y += vel_distribution(this->generator);
  link_velocity.z += vel_distribution(this->generator);
  

  //get heading measurement (measured CCW from x-axis)
  double hdg = atan2(link_velocity.y, link_velocity.x) * 180/M_PI;
  hdg = 90 + (360 - wrapto360(hdg));

  
	// std::normal_distribution<double> hdg_dist(0.0, (this->hdg_stddev * M_PI/180));

  //set heading plus noise
  // gps_msg_.track = wrapto360(hdg + hdg_dist(this->generator));
  gps_msg_.track = wrapto360(hdg);

	double XYvelocity = sqrt(pow(link_velocity.x,2) + pow(link_velocity.y,2));

	// if (XYvelocity > 0)
	// {
	// 	XYvelocity += vel_distribution(this->generator);
	// }
	// else
	// {
	// 	XYvelocity = 0;
	// }

	this->gps_msg_.speed = XYvelocity;

  //add time to header
  this->gps_msg_.header.stamp.sec = this->world_->GetSimTime().sec;
  this->gps_msg_.header.stamp.nsec = this->world_->GetSimTime().nsec;

	{
      boost::mutex::scoped_lock lock(this->lock_);
      // publish to ros
      this->pub_Queue->push(this->gps_msg_, this->pub_);
    }

    // save last time stamp
    this->last_time_ = cur_time;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Put data to the interface
void GPSPlugin::GPSQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->usbl_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

double GPSPlugin::wrapto360(double x){
    x = fmod(x,360);
    if (x < 0)
        x += 360;
    return x;
}
}
