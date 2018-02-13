/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
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

/** \author Jose Capriles. */

#define EIGEN_INITIALIZE_MATRICES_BY_ZERO

#include <string>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <geometry_msgs/Twist.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/RayPlugin.hh>

#include <sdf/Param.hh>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

#include <laughlin_uw_plugins/DVL.h>

namespace gazebo
{

class GazeboRosDVL : public RayPlugin
{

    /// \brief Constructor
    public: GazeboRosDVL();

    /// \brief Destructor
    public: ~GazeboRosDVL();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    protected: virtual void OnNewLaserScans();

    /// \brief Put range data to the ROS topic
    private: void PutRangeData(common::Time &_updateTime);

    /// \brief Keep track of number of connctions
    private: int range_connect_count_;
    private: void RangeConnect();
    private: void RangeDisconnect();

    // Pointer to the model
    private: physics::WorldPtr world_;
    /// \brief The parent sensor
    private: sensors::SensorPtr parent_sensor_;
    private: sensors::RaySensorPtr parent_ray_sensor_;

    /// \brief pointer to ros node
    private: ros::NodeHandle* rosnode_;
    private: ros::Publisher pub_;

    /// \brief ros message
    private: laughlin_uw_plugins::DVL dvl_msg_;
    /// \brief topic name
    private: std::string topic_name_;

    /// \brief frame transform name, should match link name
    private: std::string frame_name_;

    /// \brief Gaussian noise
    private: double vel_std_dev_;
    private: double rng_uncertainty_;

    //random number generator
    private: std::default_random_engine generator;

    /// \brief mutex to lock access to fields that are used in message callbacks
    private: boost::mutex lock_;

    /// \brief hack to mimic hokuyo intensity cutoff of 100
    private: double hokuyo_min_intensity_;

    /// update rate of this sensor
    private: double update_rate_;
    private: double update_period_;
    private: common::Time last_update_time_;

    /// \brief for setting ROS name space
    private: std::string robot_namespace_;

    private: ros::CallbackQueue range_queue_;
    private: void RangeQueueThread();
    private: boost::thread callback_queue_thread_;

    // deferred load in case ros is blocking
    private: sdf::ElementPtr sdf;
    private: void LoadThread();
    private: boost::thread deferred_load_thread_;
    private: unsigned int seed;

    // for DVL velocity calculation
    // homogenous transformations follow this naming convention:
    // g_<ref_frame>_<object>
    // e.g g_Inertial_DVL is the homogenous transformation representing the DVL in the Inertial frame

    private: double min_range, max_range, field_of_view, range;
    private: Eigen::Matrix4d g_Inertial_ice, g_Inertial_dvlLink;//, dvlSensor, dvl;
    private: Eigen::Vector4d beamPt_dvl, beamPt_I, beamPt_ice;		//location of beam contact point in dvl & ice frames
    private: Eigen::Vector3d w_ice, w_dvl, v_ice, v_dvl;
    private: Eigen::Vector3d dvl_observed_velocity3;
    private: Eigen::Vector4d dvl_observed_velocity4;

    private: Eigen::Quaterniond qSensor;

    private: Eigen::Matrix3d gazMatrix3_EigMatrix3d(math::Matrix3 inMatrix);


    private: std::string surfaceModelName;
};
}
