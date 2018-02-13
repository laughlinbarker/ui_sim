/*
 * Copyright 2012 Open Source Robotics Foundation
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

#define EIGEN_INITIALIZE_MATRICES_BY_ZERO

#include <string>
#include <random>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <gps_common/GPSFix.h>
#include <std_srvs/Empty.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include "gazebo/common/CommonTypes.hh"

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/common/Plugin.hh>

#include <gazebo_plugins/PubQueue.h>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>


namespace gazebo
{

  class GPSPlugin : public SensorPlugin
  {
    /// \brief Constructor
    public: GPSPlugin();

    /// \brief Destructor
    public: virtual ~GPSPlugin();

    /// \brief Load the controller
    /// \param node XML config node
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);
    public: double wrapto360(double x);

    /// \brief Update the controller
    protected: virtual void UpdateChild();

    /// \brief The parent World
    private: physics::WorldPtr world_;
    private: sensors::SensorPtr sensorParent;

    /// \brief pointer to ros node
    private: ros::NodeHandle* rosnode_;
    private: ros::Publisher pub_;
    private: PubQueue<gps_common::GPSFix>::Ptr pub_Queue;

    /// \brief ros message
    private: gps_common::GPSFix gps_msg_;

    /// \brief store frame name
    private: std::string frame_name_;

    /// \brief topic name
    private: std::string topic_name_;
    private: std::string vel_topic_name_;

    /// \brief A mutex to lock access to fields
    /// that are used in message callbacks
    private: boost::mutex lock_;

    /// \brief save last_time
    private: common::Time last_time_;

    // rate control
    private: double update_rate_;

    /// \brief Gaussian noise
    private: double horiz_stddev;
    private: double vert_stddev;
    private: double vel_stddev;
    private: double hdg_stddev;

    //random number generator
    private: std::default_random_engine generator;

    /// \brief for setting ROS name space
    private: std::string robot_namespace_;

    private: ros::CallbackQueue usbl_queue_;
    private: void GPSQueueThread();
    private: boost::thread callback_queue_thread_;

    // Pointer to the update event connection
    private: event::ConnectionPtr update_connection_;

    // deferred load in case ros is blocking
    private: sdf::ElementPtr sdf;
    private: void LoadThread();
    private: boost::thread deferred_load_thread_;
    private: unsigned int seed;

    // ros publish multi queue, prevents publish() blocking
    private: PubMultiQueue pmq;

    private: math::Pose tmpPose;

    private: common::SphericalCoordinatesPtr sphericalCoords_;
    private: ignition::math::Vector3d positionXYZ;

    private: GeographicLib::Geocentric earth;
    private: GeographicLib::LocalCartesian localCart;
  };

}
