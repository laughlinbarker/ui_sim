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
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/common/Plugin.hh>

#include <gazebo_plugins/PubQueue.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

#include <laughlin_uw_plugins/USBL.h>

namespace gazebo
{
  class USBLPlugin : public SensorPlugin
  {
    /// \brief Constructor
    public: USBLPlugin();

    /// \brief Destructor
    public: virtual ~USBLPlugin();

    //Wrap angles to [0,360)
    public: double wrapto360(double x);

    /// \brief Load the controller
    /// \param node XML config node
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    protected: virtual void UpdateChild();

    /// \brief The parent World
    private: physics::WorldPtr world_;
    private: sensors::SensorPtr sensorParent;

    /// \brief The link referred to by this plugin
    private: physics::LinkPtr beacon_link;

    /// \brief The dunker link referred to by this plugin
    private: physics::LinkPtr dunker_link;

    /// \brief pointer to ros node
    private: ros::NodeHandle* rosnode_;
    private: ros::Publisher pub_;
    private: PubQueue<laughlin_uw_plugins::USBL>::Ptr pub_Queue;

    /// \brief ros message
    private: laughlin_uw_plugins::USBL usbl_msg_;

    /// \brief store beacon  link name
    private: std::string beacon_link_name_;

    /// \brief store dunker link name
    private: std::string dunker_link_name_;

    /// \brief store frame name
    private: std::string frame_name_;

    /// \brief topic name
    private: std::string topic_name_;

    /// \brief A mutex to lock access to fields
    /// that are used in message callbacks
    private: boost::mutex lock_;

    /// \brief save last_time
    private: common::Time last_time_;

    // rate control
    private: double update_rate_;

    /// \brief Gaussian noise
    private: double posAccuracy_;
    private: double pos_std_dev_;

    //random number generator
    private: std::default_random_engine generator;

    /// \brief for setting ROS name space
    private: std::string robot_namespace_;

    private: ros::CallbackQueue usbl_queue_;
    private: void USBLQueueThread();
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

    private: Eigen::Matrix4d g_Inertial_USBLBeacon, g_Inertial_USBLDunker;
    private: Eigen::Matrix4d g_tmp;
    private: Eigen::Vector4d v_tmp;
    private: math::Pose tmpPose;

    private: Eigen::Matrix3d gazMatrix3_EigMatrix3d(math::Matrix3 inMatrix);

  };
}
