/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013-2015, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Jose Capriles, Bence Magyar. */

#include "../include/laughlin_uw_plugins/dvl_plugin.hpp"
#include <gazebo_plugins/gazebo_ros_utils.h>

#include <algorithm>
#include <string>
#include <assert.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <sdf/sdf.hh>
#include <sdf/Param.hh>

#include <tf/tf.h>


namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosDVL)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosDVL::GazeboRosDVL()
{
  this->seed = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosDVL::~GazeboRosDVL()
{
  this->range_queue_.clear();
  this->range_queue_.disable();
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();

  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosDVL::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // load plugin
  RayPlugin::Load(_parent, this->sdf);
  // Get then name of the parent sensor
  this->parent_sensor_ = _parent;
  // Get the world name.
# if GAZEBO_MAJOR_VERSION >= 7
  std::string worldName = _parent->WorldName();
# else
  std::string worldName = _parent->GetWorldName();
# endif
  std::cout << "DVL Plugin, _parent->WorldName(): " << worldName << std::endl;
  this->world_ = physics::get_world(worldName);
  // save pointers
  this->sdf = _sdf;

  this->last_update_time_ = common::Time(0);

  GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
  this->parent_ray_sensor_ =
    dynamic_pointer_cast<sensors::RaySensor>(this->parent_sensor_);

  if (!this->parent_ray_sensor_)
    gzthrow("GazeboRosDVL controller requires a Ray Sensor as its parent");

  if (!this->sdf->HasElement("surfaceModelName"))
  {
	  gzthrow("GazeboRosDVL plugin requires a surfaceModelName to track velocity against");
	  ROS_WARN("GazeboRosDVL plugin requires a surfaceModelName to track velocity against");
  }
  else
  {
	  this->surfaceModelName = this->sdf->Get<std::string>("surfaceModelName");
	  //check that it actually exists
	  if (!this->world_->GetModel(this->surfaceModelName))
	  {}
		  //gzthrow("Specified surfaceModelName does not exist");
  }
  this->robot_namespace_ = "";
  if (this->sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = this->sdf->Get<std::string>("robotNamespace");

  if (!this->sdf->HasElement("topicName"))
  {
    ROS_INFO("DVL plugin missing <topicName>, defaults to /range");
    this->topic_name_ = "/range";
  }
  else
    this->topic_name_ = this->sdf->Get<std::string>("topicName");

  if (!this->sdf->HasElement("velStdDev"))
  {
    ROS_INFO("DVL plugin missing <velStdDev>, defaults to 0.0");
    this->vel_std_dev_ = 0;
  }
  else
    this->vel_std_dev_ = this->sdf->Get<double>("velStdDev");

  if (!this->sdf->HasElement("rangeUncertainty"))
  {
    ROS_WARN("DVL plugin missing <rangeUncertainty>, defaults to 0.0");
    this->rng_uncertainty_ = 0;
  }
  else
    this->rng_uncertainty_ = this->sdf->Get<double>("rangeUncertainty");

  if (!this->sdf->HasElement("updateRate"))
  {
    gzthrow("DVL plugin missing <updateRate>, defaults to 0");
    this->update_rate_ = 0;
  }
  else
    this->update_rate_ = this->sdf->Get<double>("updateRate");

  // prepare to throttle this plugin at the same rate
  // ideally, we should invoke a plugin update when the sensor updates,
  // have to think about how to do that properly later
  if (this->update_rate_ > 0.0)
    this->update_period_ = 1.0/this->update_rate_;
  else
    this->update_period_ = 0.0;

  this->range_connect_count_ = 0;

# if GAZEBO_MAJOR_VERSION >= 7
  this->max_range = this->parent_ray_sensor_->RangeMax();
  this->min_range = this->parent_ray_sensor_->RangeMin();
# else
  this->max_range = this->parent_ray_sensor_->GetRangeMax();
  this->min_range = this->parent_ray_sensor_->GetRangeMin();
# endif

  // Init ROS
  if (ros::isInitialized())
  {
    // ros callback queue for processing subscription
    this->deferred_load_thread_ = boost::thread(
      boost::bind(&GazeboRosDVL::LoadThread, this));
  }
  else
  {
    gzerr << "Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
  }
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosDVL::LoadThread()
{
  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  if (this->topic_name_ != "")
  {
    ros::AdvertiseOptions ao =
      ros::AdvertiseOptions::create<laughlin_uw_plugins::DVL>(
      this->topic_name_, 1,
      boost::bind(&GazeboRosDVL::RangeConnect, this),
      boost::bind(&GazeboRosDVL::RangeDisconnect, this),
      ros::VoidPtr(), &this->range_queue_);
    this->pub_ = this->rosnode_->advertise(ao);
  }


  // Initialize the controller

  // sensor generation off by default
  this->parent_ray_sensor_->SetActive(false);
  // start custom queue for range
  this->callback_queue_thread_ =
    boost::thread(boost::bind(&GazeboRosDVL::RangeQueueThread, this));
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosDVL::RangeConnect()
{
  this->range_connect_count_++;
  this->parent_ray_sensor_->SetActive(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosDVL::RangeDisconnect()
{
  this->range_connect_count_--;

  if (this->range_connect_count_ == 0)
    this->parent_ray_sensor_->SetActive(false);
}


////////////////////////////////////////////////////////////////////////////////
// Update the plugin
void GazeboRosDVL::OnNewLaserScans()
{
  if (this->topic_name_ != "")
  {
    common::Time cur_time = this->world_->GetSimTime();
    if (cur_time - this->last_update_time_ >= this->update_period_)
    {
      common::Time sensor_update_time =
# if GAZEBO_MAJOR_VERSION >= 7
        this->parent_sensor_->LastUpdateTime();
# else
        this->parent_sensor_->GetLastUpdateTime();
# endif
      this->PutRangeData(sensor_update_time);
      this->last_update_time_ = cur_time;
    }
  }
  else
  {
    ROS_INFO("gazebo_ros_dvl topic name not set");
  }
}

////////////////////////////////////////////////////////////////////////////////
// Put range data to the interface
void GazeboRosDVL::PutRangeData(common::Time &_updateTime)
{
  this->parent_ray_sensor_->SetActive(false);

  /***************************************************************/
  /*                                                             */
  /*  point scan from ray sensor                                 */
  /*                                                             */
  /***************************************************************/
  {
    boost::mutex::scoped_lock lock(this->lock_);

    // find ray with minimal range
    this->range = max_range;

# if GAZEBO_MAJOR_VERSION >= 7
    int num_ranges = parent_ray_sensor_->LaserShape()->GetSampleCount() * parent_ray_sensor_->LaserShape()->GetVerticalSampleCount();
# else
    int num_ranges = parent_ray_sensor_->GetLaserShape()->GetSampleCount() * parent_ray_sensor_->GetLaserShape()->GetVerticalSampleCount();
# endif

    for(int i = 0; i < num_ranges; ++i)
    {
# if GAZEBO_MAJOR_VERSION >= 7
        double ray = parent_ray_sensor_->LaserShape()->GetRange(i);
# else
        double ray = parent_ray_sensor_->GetLaserShape()->GetRange(i);
# endif
        if (ray < range)
        {
            range = ray;
        }
    }

    //Begin DVL modifications
    //sensor class doesn't provide a way to get global location of sensor,
    //so we get global pose of parent link, and then pose of sensor
    //in the link frame, and then compute global pose of sensor manually

    //TODO !! ENSURE Sensor pose is [0 0 0 0 -pi/2 0]

    //parent link location in global frame
    physics::EntityPtr sensorParentEntity = this->world_->GetEntity(parent_sensor_->ParentName());

    this->g_Inertial_dvlLink.block<3,3>(0,0) = this->gazMatrix3_EigMatrix3d(sensorParentEntity->GetWorldPose().rot.GetAsMatrix3());
    this->g_Inertial_dvlLink(0,3) = sensorParentEntity->GetWorldPose().pos.x;
    this->g_Inertial_dvlLink(1,3) = sensorParentEntity->GetWorldPose().pos.y;
    this->g_Inertial_dvlLink(2,3) = sensorParentEntity->GetWorldPose().pos.z;
    this->g_Inertial_dvlLink(3,3) = 1;

    //ice pose
    physics::ModelPtr icePtr = this->world_->GetModel(this->surfaceModelName);

    this->g_Inertial_ice.block<3,3>(0,0) = this->gazMatrix3_EigMatrix3d(icePtr->GetWorldPose().rot.GetAsMatrix3());
    this->g_Inertial_ice(0,3) = icePtr->GetWorldPose().pos.x;
    this->g_Inertial_ice(1,3) = icePtr->GetWorldPose().pos.y;
    this->g_Inertial_ice(2,3) = icePtr->GetWorldPose().pos.z;
    this->g_Inertial_ice(3,3) = 1;

    //beam point in DVL frame
    this->beamPt_dvl << 0, 0, range, 1;

    //beam point in inertial frame
    this->beamPt_I = this->g_Inertial_dvlLink * this->beamPt_dvl;

    //std::cout << "BeamSpot Inertial Frame: " << this->beamPt_I.transpose() << std::endl;

    //beam point in ice frame
    this->beamPt_ice = this->g_Inertial_ice.inverse() * this->beamPt_I;

    //std::cout << "BeamSpot Ice Frame: " << this->beamPt_ice.transpose() << std::endl;

    //find velocity of beamPt_ice in world frame
    this->v_ice(0) = icePtr->GetWorldLinearVel().x;
    this->v_ice(1) = icePtr->GetWorldLinearVel().y;
    this->v_ice(2) = icePtr->GetWorldLinearVel().z;

    this->w_ice(0) = icePtr->GetWorldAngularVel().x;
    this->w_ice(1) = icePtr->GetWorldAngularVel().y;
    this->w_ice(2) = icePtr->GetWorldAngularVel().z;

    //std::cout << "Ice velocity [v w]' :" << this->v_ice.transpose() << " " << this->w_ice.transpose() << std::endl;

    // calculate world velocity of beam-illuminated spot
    // inertial velocity is = VelLinear + cross(w,r), where r is vector from axis of rotation to point of interest,
    // and w is angular velocity of rotation about said axis.
    // in this case, r is the vector from surface origin, to illuminated point
    this->v_ice = this->v_ice + this->w_ice.cross(this->beamPt_I.head<3>() - this->g_Inertial_ice.block<3,1>(0,3));

    // std::cout << "vw_p (velocity of ensonified pt): " << this->v_ice.transpose() << std::endl;

    //velocity of DVL in inertial frame
    this->v_dvl[0] = sensorParentEntity->GetWorldLinearVel().x;
    this->v_dvl[1] = sensorParentEntity->GetWorldLinearVel().y;
    this->v_dvl[2] = sensorParentEntity->GetWorldLinearVel().z;

    //std::cout << "world v_dvl: " << this->v_dvl.transpose() << std::endl;
    //this->v_dvl = this->gazMatrix3_EigMatrix3d(sensorParentEntity->GetWorldPose().rot.GetAsMatrix3()).transpose() * v_dvl;
    //std::cout << "instrument v_dvl " << this->v_dvl.transpose() << std::endl;

    //relative velocity ice-DVL velocity in inertial frame
    this->dvl_observed_velocity3 = this->v_ice - this->v_dvl;

    //std::cout << "DVL Observed velocity: " << this->dvl_observed_velocity3.transpose() << std::endl;

    //observed veloctiy of surface in DVL frame
    this->dvl_observed_velocity3 = gazMatrix3_EigMatrix3d(sensorParentEntity->GetWorldPose().rot.GetAsMatrix3()).transpose() * this->dvl_observed_velocity3;
    //std::cout << "DVL Observed velocity: " << this->dvl_observed_velocity3.transpose() << std::endl;

    this->dvl_observed_velocity4.head<3>() = this->dvl_observed_velocity3;
    this->dvl_observed_velocity4(3) = 0;

    //this->dvl_observed_velocity4 = this->g_Inertial_dvlLink.inverse() * this->dvl_observed_velocity4;

    //report range (with noise) if surface is within range, else report 0
    if (range < max_range)
    {
    	//compute noise and add to position (noise is linear function of slant range)
    	std::normal_distribution<double> velErrorDist(0.0,this->vel_std_dev_);

    	this->dvl_msg_.velocity.linear.x = this->dvl_observed_velocity4(0) + velErrorDist(this->generator);
    	this->dvl_msg_.velocity.linear.y = this->dvl_observed_velocity4(1) + velErrorDist(this->generator);
    	this->dvl_msg_.velocity.linear.z = this->dvl_observed_velocity4(2) + velErrorDist(this->generator);
      
      double rngStdDev = range * rng_uncertainty_;

      std::normal_distribution<double> rangeErrorDist(0.0,rngStdDev);
      //std::cout << "Real Range: " << range << std::endl;
      this->dvl_msg_.range = range + rangeErrorDist(this->generator);
      //std::cout << "Range w/1% noise: " << this->dvl_msg_.range << std::endl;
    }
    else
    {
    	this->dvl_msg_.velocity.linear.x = 0;
    	this->dvl_msg_.velocity.linear.y = 0;
    	this->dvl_msg_.velocity.linear.z = 0;
      this->dvl_msg_.range = 0;
    }
    this->dvl_msg_.velocity.angular.x = 0;
    this->dvl_msg_.velocity.angular.y = 0;
    this->dvl_msg_.velocity.angular.z = 0;

    //add time to header
    this->dvl_msg_.header.stamp.sec = this->world_->GetSimTime().sec;
    this->dvl_msg_.header.stamp.nsec = this->world_->GetSimTime().nsec;

    this->parent_ray_sensor_->SetActive(true);

    // send data out via ros message
    if (this->range_connect_count_ > 0 && this->topic_name_ != "")
        this->pub_.publish(this->dvl_msg_);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Put range data to the interface
void GazeboRosDVL::RangeQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->range_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

Eigen::Matrix3d GazeboRosDVL::gazMatrix3_EigMatrix3d(math::Matrix3 inMatrix)
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
}
