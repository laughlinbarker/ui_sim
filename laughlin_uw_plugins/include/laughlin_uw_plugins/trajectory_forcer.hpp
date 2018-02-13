/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
 * Desc: 3D position interface.
 * Author: Sachin Chitta and John Hsu
 * Date: 10 June 2008
 */

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/Empty.h>

#include <math.h>
#include <array>
#include <cmath>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/math/gzmath.hh>

#include <gazebo_plugins/PubQueue.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

namespace gazebo
{

  class TrajectoryForcer : public ModelPlugin
  {
  /// \brief Constructor
  public: TrajectoryForcer();

  /// \brief Destructor
  public: virtual ~TrajectoryForcer();

  /// \brief Load the controller
  public: void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );
  /// \brief Update the controller
  protected: virtual void GazeboUpdate();

  /// Pointer to the update event connection
  private: event::ConnectionPtr update_connection_;
           boost::shared_ptr<tf2_ros::Buffer> tf_buffer_;
           boost::shared_ptr<tf2_ros::TransformListener> tf_listener_;
           boost::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
           physics::ModelPtr model_;
           physics::LinkPtr floating_link_;
           std::string link_name_;
           std::string robot_namespace_;
           std::string frame_id_;
           double kl_, ka_;
           double cl_, ca_;
           gazebo::math::Vector3 translation_vector_, pos_desired_;
           gazebo::math::Quaternion rot_desired_;
           double v_lin_des_, w_ang_des_;
           common::Time traj_start_time_, last_time_;
           std::string object_;
           double hdg_deg2yaw_rad(double hdg);
           double yaw2hdg(double yaw);

           //adding ros publisher to publish desired trajecotry info
           ros::NodeHandle* rosnode_;
           ros::Publisher pub_;
           PubQueue<geometry_msgs::Pose>::Ptr pub_Queue_;
           geometry_msgs::Pose desiredPose_;
           double update_rate_ = 1;
           std::string topic_ = "desired_trajectory";
           ros::CallbackQueue traj_queue_;
           boost::thread callback_queue_thread_;
           PubMultiQueue pmq;
           void QueueThread();


           //trajectory generation
           double wpt_[21][6] = 			//n waypoints with auv state consisting of [x,y,z,r,p,h]
           	   	   	   	   	   {{0,0,-4,M_PI,0,90},
        		   	   	   	    {500,0,-50,M_PI,0,90},
								{500,0,-4,M_PI,0,90},
								{1000,0,-50,M_PI,0,90},
								{1000,0,-4,M_PI,0,90},
								{1000,0,-4,M_PI,0,0},	//turn
								{1000,500,-50,M_PI,0,0},
								{1000,500,-4,M_PI,0,0},
								{1000,500,-4,M_PI,0,270},//turn
								{500,500,-50,M_PI,0,270},
								{500,500,-4,M_PI,0,270},
								{0,500,-50,M_PI,0,270},
								{0,500,-4,M_PI,0,270},
								{0,500,-4,M_PI,0,0},	//turn
								{0,1000,-50,M_PI,0,0},
								{0,1000,-4,M_PI,0,0},
								{0,1000,-4,M_PI,0,90},//turn
								{500,1000,-50,M_PI,0,90},
								{500,1000,-4,M_PI,0,90},
								{1000,1000,-50,M_PI,0,90},
								{1000,1000,-4,M_PI,0,90}};
           double velDes_[19][2] = 	//n-1 velocitiy specifications [v,w] for linear and angular vel
           	   	   	   	   	   {{1,0},
           	   	   	   	   		{0.25,0}, //up
								{1,0},
								{0.25,0}, //up
           	   	   	   	   		{0,0.314159},
								{1,0},
								{0.25,0}, //up
								{0,0.314159},
								{1,0},
								{0.25,0}, //up
								{1,0},
								{0.25,0}, //up
								{0,0.314159},
								{1,0},
								{0.25,0}, //up
								{0,0.314159},
								{1,0},
           	   	   	   	   	   	{0.25,0}, //up
           	   	   	   	   	   	{1,0}};

/*
           double wpt_[8][6] = 			//n waypoints with auv state consisting of [x,y,z,r,p,h]
            	   	   	   	   	   {{0,0,-5,M_PI,0,90},
         		   	   	   	    {25,0,-5,M_PI,0,90},
 								{25,0,-5,M_PI,0,0},
 								{25,5,-5,M_PI,0,0},
 								{25,5,-5,M_PI,0,270},
 								{0,5,-5,M_PI,0,270},
 								{0,5,-5,M_PI,0,0},
 								{0,10,-5,M_PI,0,0}};
            double velDes_[7][2] = 	//n-1 velocitiy specifications [v,w] for linear and angular vel
            	   	   	   	   	   {{1,0},
            	   	   	   	   		{0,0.314159},
 								{1,0},
 								{0,0.314159},
 								{1,0},
 								{0,0.314159},
 								{1,0}};
*/
           //currently should be 2n-3 turns, where n is #wpts
           double legDuration_[19]; //duration of each leg, computed based on distance and velocity
           std::array<math::Vector3,21> wpts_;
           std::array<common::Time,19> segmentEndTime_;

           //double legDuration_[7];
           //std::array<math::Vector3,8> wpts_;
           //std::array<common::Time,7> segmentEndTime_;


           math::Vector4 x_init_;
           math::Vector3 trajVector_, oldWpt_, currWpt_;

           common::Time missionTime_, segmentTime_, missionStart_, segmentStart_, old_time, dt;

           int i_segment_;
           double currHdg_, oldHdg_;

           void startMission();
           void checkSegmentCompletion();
           double dist(math::Vector3 pt1, math::Vector3 pt2);
           bool missionStarted_, missionFinished_;
           common::Time getSimTime();
           double getHeadingError(double final, double initial);
           int sign(double num);
           double wrap360(double angle);
  };
}
