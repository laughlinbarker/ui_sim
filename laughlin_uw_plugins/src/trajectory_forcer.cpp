/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
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
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
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
 *********************************************************************/

/**
 *  \author Jonathan Bohren
 *  \desc   A "hand-of-god" plugin which drives a floating object around based
 *  on the location of a TF frame. This plugin is useful for connecting human input
 *  devices to "god-like" objects in a Gazebo simulation.
 */

#include "../include/laughlin_uw_plugins/trajectory_forcer.hpp"
#include <ros/ros.h>

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(TrajectoryForcer);

  ////////////////////////////////////////////////////////////////////////////////
  // Constructor
  TrajectoryForcer::TrajectoryForcer() :
    ModelPlugin(),
    robot_namespace_(""),
    frame_id_("hog"),
    kl_(200),
    ka_(200),
	v_lin_des_(0.1),
	w_ang_des_(969.627e-9),	//0.2 deg/hr * 1 h/3600 s * 3.14159 rad/180 deg
	object_("ice"),		// default is ice, other option is "vehicle"
	missionStarted_(false),
	missionFinished_(false)
  {
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Destructor
  TrajectoryForcer::~TrajectoryForcer()
  {
	  this->rosnode_->shutdown();
	  this->callback_queue_thread_.join();
	  delete this->rosnode_;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Load the controller
  void TrajectoryForcer::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
  {
    // Make sure the ROS node for Gazebo has already been initalized
    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                       << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    this->rosnode_ = new ros::NodeHandle("trajGenerator");

    // publish multi queue
    this->pmq.startServiceThread();

    this->pub_Queue_ = this->pmq.addPub<geometry_msgs::Pose>();
    this->pub_ = this->rosnode_->advertise<geometry_msgs::Pose>(
        this->topic_, 1);

      // start custom queue for Publisher
    this->callback_queue_thread_ =
        boost::thread(boost::bind(&TrajectoryForcer::QueueThread, this));

    // Get sdf parameters
    if(_sdf->HasElement("robotNamespace")) {
      this->robot_namespace_ = _sdf->Get<std::string>("robotNamespace");
    }

    if(_sdf->HasElement("frameId")) {
      this->frame_id_ = _sdf->Get<std::string>("frameId");
    }

    if(_sdf->HasElement("kl")) {
      this->kl_ = _sdf->Get<double>("kl");
    }
    if(_sdf->HasElement("ka")) {
      this->ka_ = _sdf->Get<double>("ka");
    }

    if(_sdf->HasElement("linkName")) {
      this->link_name_ = _sdf->Get<std::string>("linkName");
    } else {
      ROS_FATAL_STREAM("The hand-of-god plugin requires a `linkName` parameter tag");
      return;
    }

    if(_sdf->HasElement("translationVector")) {
    	this->translation_vector_ = _sdf->Get<gazebo::math::Vector3>("translationVector");
    }
	else{
		ROS_FATAL_STREAM("Plugin requires a translation vector");
		return;
	}

    if(_sdf->HasElement("linearVelocity"))
    	this->v_lin_des_ = _sdf->Get<double>("linearVelocity");
    if(_sdf->HasElement("angularVelocity"))
    	this->w_ang_des_ = _sdf->Get<double>("angularVelocity");

    if(_sdf->HasElement("forcedObject"))
    	this->object_ = _sdf->Get<std::string>("forcedObject");

    // Store the model
    model_ = _parent;

    // Get the floating link
    floating_link_ = model_->GetLink(link_name_);
    // Disable gravity for the hog
    floating_link_->SetGravityMode(false);
    if(!floating_link_) {
      ROS_ERROR("Floating link not found!");
      const std::vector<physics::LinkPtr> &links = model_->GetLinks();
      for(unsigned i=0; i < links.size(); i++) {
        ROS_ERROR_STREAM(" -- Link "<<i<<": "<<links[i]->GetName());
      }
      return;
    }

    // c = 2 * sqrt(k*m) is critically damped
    cl_ = 2.0 * sqrt(kl_*floating_link_->GetInertial()->GetMass());
    ca_ = 2.0 * sqrt(ka_*floating_link_->GetInertial()->GetIZZ());

    // Register update event handler
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&TrajectoryForcer::GazeboUpdate, this));

    this->last_time_ = this->model_->GetWorld()->GetSimTime();
    std::cout << "TRAJECTORY STARTED" << "\n";

    //slam vehicle to starting position
    if(!this->object_.compare("vehicle"))
    {
    	gazebo::math::Pose x_i(this->wpt_[0][0], this->wpt_[0][1], this->wpt_[0][2], M_PI, 0, this->hdg_deg2yaw_rad(this->wpt_[0][5]));
    	floating_link_->SetWorldPose(x_i,true,true);

    	//std::cout << "starting pos: " << x_i.pos << std::endl;
    	//std::cout << "starting rot: " << x_i.rot.GetAsEuler() << std::endl;

    	this->old_time = this->model_->GetWorld()->GetSimTime();
    }
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Update the controller
  void TrajectoryForcer::GazeboUpdate()
  {

	  if(!this->object_.compare("ice"))
	  {
		  common::Time cur_time_ = this->model_->GetWorld()->GetSimTime();
		      //calculate desired position based on time and desired velocity
		      this->pos_desired_ = this->v_lin_des_ * this->translation_vector_.Normalize() * cur_time_.Double();
		      this->rot_desired_ = gazebo::math::Quaternion(0, 0, this->w_ang_des_ * cur_time_.Double());


		      //calculate error
		      // Relative transform from actual to desired pose
		      gazebo::math::Pose world_pose = floating_link_->GetDirtyPose();
		      gazebo::math::Vector3 err_pos = this->pos_desired_ - world_pose.pos;

		      // Get exponential coordinates for rotation
		      gazebo::math::Quaternion err_rot = (world_pose.rot.GetAsMatrix4().Inverse() * this->rot_desired_.GetAsMatrix4()).GetRotation();
		      gazebo::math::Quaternion not_a_quaternion = err_rot.GetLog();

		      floating_link_->AddForce(
		          kl_ * err_pos - cl_ * floating_link_->GetWorldLinearVel());

		      floating_link_->AddRelativeTorque(
		          ka_ * gazebo::math::Vector3(not_a_quaternion.x, not_a_quaternion.y, not_a_quaternion.z) - ca_ * floating_link_->GetRelativeAngularVel());
	  }

	  else if (!this->object_.compare("vehicle"))
	  {
		if(!this->missionFinished_)
		{
		  //housekeeping
		  if(!this->missionStarted_)
		  {
			  this->startMission();
			  this->missionStarted_ = true;
		  }

		  //update mission and segment times
		  this->missionTime_ = this->getSimTime() - this->missionStart_;
		  this->segmentTime_ = this->getSimTime() - this->segmentStart_;
		  this->dt = this->missionTime_ - this->old_time;

		  //std::cout << "dt: " << dt.Double() << std::endl;

		  //check if current mission segment is complete (also updates trajVector_ on completion
		  this->checkSegmentCompletion();

		  //calculate current XYZ waypoint
		  this->currWpt_ = this->oldWpt_ + this->velDes_[this->i_segment_][0] * this->trajVector_.Normalize() * this->dt.Double();
		  this->oldWpt_ = this->currWpt_;
		  this->pos_desired_ = this->currWpt_;
		  //std::cout << "curWpt_: " << this->currWpt_ << std::endl;

		  //calculate heading waypoint
		  this->currHdg_ = this->oldHdg_ + this->sign(this->getHeadingError(wpt_[i_segment_+1][5],wpt_[i_segment_][5])) * this->dt.Double() * this->velDes_[this->i_segment_][1] * (180/M_PI);

		  this->currHdg_ = this->wrap360(this->currHdg_);
		  this->oldHdg_ = this->currHdg_;

		  //std::cout << "Segment: " << i_segment_ << std::endl;
		  //std::cout << "Linear Trajectory: " << this->trajVector_ << std::endl;
		  //std::cout << "Linear Vel: " << velDes_[i_segment_][0] << std::endl;
		  //std::cout << "Ang Vel: " << velDes_[i_segment_][1] << std::endl;
		  //std::cout << "Current Heading: " << currHdg_ << std::endl;


		  //update desired rotation
		  this->rot_desired_ = gazebo::math::Quaternion(M_PI,0,this->hdg_deg2yaw_rad(this->currHdg_));

		  // Get exponential coordinates for rotation
		  math::Pose world_pose = floating_link_->GetDirtyPose();
		  math::Vector3 err_pos = this->pos_desired_ - world_pose.pos;
		  math::Quaternion err_rot = (world_pose.rot.GetAsMatrix4().Inverse() * this->rot_desired_.GetAsMatrix4()).GetRotation();
		  math::Quaternion not_a_quaternion = err_rot.GetLog();

		  floating_link_->AddForce(
 			      kl_ * err_pos - cl_ * floating_link_->GetWorldLinearVel());

		  floating_link_->AddRelativeTorque(
 			      ka_ * gazebo::math::Vector3(not_a_quaternion.x, not_a_quaternion.y, not_a_quaternion.z) - ca_ * floating_link_->GetRelativeAngularVel());


		  this->desiredPose_.position.x = this->pos_desired_.x;
		  this->desiredPose_.position.y = this->pos_desired_.y;
		  this->desiredPose_.position.z = this->pos_desired_.z;

		  this->desiredPose_.orientation.x = this->rot_desired_.x;
		  this->desiredPose_.orientation.y = this->rot_desired_.y;
		  this->desiredPose_.orientation.z = this->rot_desired_.z;
		  this->desiredPose_.orientation.w = this->rot_desired_.w;

		  //publishing vehicle trajectory pose for recording
		  if ((this->missionTime_ - this->last_time_).Double() > (1.0 / this->update_rate_))
		  {
			  if (this->pub_.getNumSubscribers() > 0 )
			  {
				  this->pub_Queue_->push(this->desiredPose_, this->pub_);
			  }
			  this->last_time_ = this->missionTime_;
		  }

		  this->old_time = this->missionTime_;
		}
	  }
  }

  void TrajectoryForcer::startMission()
  {
	  //std::cout << "Mission Start" << std::endl;
	  this->missionStarted_ = true;
	  this->i_segment_ = 0;
	  this->currHdg_ = this->wpt_[0][5];
	  this->oldHdg_ = currHdg_;

	  //load wpt array into Vector3.
	  for(int i = 0; i < sizeof(this->wpt_)/sizeof(this->wpt_[0]); i++)
	  {
		  this->wpts_[i].x = this->wpt_[i][0];
		  this->wpts_[i].y = this->wpt_[i][1];
		  this->wpts_[i].z = this->wpt_[i][2];
	  }

	  //calculate segment duration times
	  for(int i = 0; i < sizeof(this->velDes_)/sizeof(this->velDes_[0]); i++)
	  {
		  double lindistance = this->dist(this->wpts_[i+1], this->wpts_[i]);
		  double angdistance = this->getHeadingError(this->wpt_[i+1][5], this->wpt_[i][5]); //in degrees

		  if (lindistance != 0)
			  this->legDuration_[i] = lindistance / this->velDes_[i][0];
		  if (angdistance != 0)
			  this->legDuration_[i] = std::abs(angdistance) * (M_PI/180) / this->velDes_[i][1];

		  //std::cout << "Leg " << i << " Linear/angular dist: [" << lindistance << "," << angdistance << "]"<< std::endl;
		  //std::cout << "Leg " << i << " duration: " << this->legDuration_[i] << std::endl;
	  }

	  //std::cout << "Size of legDuration: " << sizeof(this->legDuration_)/sizeof(this->legDuration_[0]) << std::endl;
	  //set time arrays
	  this->segmentEndTime_[0] = common::Time(this->legDuration_[0]);
	  for(int i = 1; i < sizeof(this->legDuration_)/sizeof(this->legDuration_[0]); i++)
	  {
		  this->segmentEndTime_[i] = this->segmentEndTime_[i-1] + common::Time(this->legDuration_[i]);
		  std::cout << "Segment " << i << " end time: "<< this->segmentEndTime_[i].Double() << std::endl;
	  }

	  //std::cout << "Size(segmentEndTime_)" << sizeof(segmentEndTime_) << std::endl;
	  //std::cout << "SegmentEndTime.back(): "<< segmentEndTime_.back().Double() << std::endl;

	  //std::cout << "MissionEndTime: " << this->segmentEndTime_[sizeof(this->legDuration_)/sizeof(this->legDuration_[0]) - 1].Double() << std::endl;
	  //std::cout << "MissionEndTime (using .back() command): " << this->segmentEndTime_.back().Double() << std::endl;
	  //set clocks
	  this->missionStart_ = this->getSimTime();
	  this->segmentStart_ = this->missionStart_;

	  //calculate first trajectory vector
	  this->trajVector_ = this->wpts_[this->i_segment_ + 1] - this->wpts_[this->i_segment_];

	  //std::cout << "Trajectory Vector: " << this->trajVector_ << std::endl;

	  //initialize waypoint bookeeping
	  this->oldWpt_ = wpts_[0];
  }

  double TrajectoryForcer::dist(math::Vector3 pt1, math::Vector3 pt2)
  {
	  double xsq = pow((pt2.x - pt1.x),2);
	  double ysq = pow((pt2.y - pt1.y),2);
	  double zsq = pow((pt2.z - pt1.z),2);

	  return pow((xsq + ysq + zsq),0.5);
  }

  void TrajectoryForcer::checkSegmentCompletion()
  {
	  if(this->missionTime_ < this->segmentEndTime_.back())
	  {
		  if(this->missionTime_ > this->segmentEndTime_[this->i_segment_])
		  {
			  //std::cout << "FINISHED SEGMENT " << this->i_segment_ << std::endl;
			  //std::cout << "Mission Time: " << this->missionTime_.Double() << std::endl;
			  //std::cout << "Segment Time: " << this->segmentTime_.Double() << std::endl;
			  //increment segment number
			  this->i_segment_++;

			  //reset segment stopwatch
			  this->segmentStart_ = this->getSimTime();

			  //update desired trajectory vector
			  this->trajVector_ = this->wpts_[this->i_segment_ + 1] - this->wpts_[this->i_segment_];

			  //update desired heading (yaw) value
			  //this->hdgDes_ = this->wpt_[this->i_segment_];
		  }
	  }
	  else
	  {
		  this->missionFinished_ = true;
		  ROS_WARN("MISSION FINISHED");
	  }
  }

  common::Time TrajectoryForcer::getSimTime()
  {
	  return this->model_->GetWorld()->GetSimTime();;
  }

  double TrajectoryForcer::hdg_deg2yaw_rad(double hdg){
	  hdg = hdg * M_PI/180;
	  double yaw = M_PI/2 - hdg;			//hdg = pi/2 - yaw

	  //wrap to [0,2pi]
	  yaw = (yaw - (2*M_PI * std::floor(yaw / (2*M_PI))) );
	  return yaw;
  }

  double TrajectoryForcer::yaw2hdg(double yaw){
	  double hdg = M_PI/2 - yaw;

	  //wrap
	  hdg = (hdg - (2*M_PI * std::floor(hdg / (2*M_PI))));
	  return hdg;
  }

  double TrajectoryForcer::getHeadingError(double desired, double current)
  {
	  if (desired > 360 || desired < 0 || current > 360 || current < 0)
		  ROS_WARN("getHeadingError inputs not properly wrapped to [0,360]");

	  double error = desired - current;
	  double absError = std::abs(error);

	  if (absError <= 180)
		  if (absError == 180)
			  return absError;
		  else
			  return error;
	  else if (desired > current)
		  return (absError - 360);
	  else
		  return (360-absError);
  }

  int TrajectoryForcer::sign(double num)
  {
	  if (num > 0) return 1;
	  if (num < 0) return -1;
	  return 0;
  }

  double TrajectoryForcer::wrap360(double angle)
  {
	  angle = std::fmod(angle,360);
	  if (angle < 0)
		  angle += 360;

	  return angle;
  }

  void TrajectoryForcer::QueueThread()
  {
    static const double timeout = 0.01;

    while (this->rosnode_->ok())
    {
      this->traj_queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

}
