//---------------------------------------------------------------------------------------------------------------------
//  LUCAS: Lightweight framework for UAV Control And Supervision
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2024 CATEC (Advanced Centre for Aerospace Technologies)
//---------------------------------------------------------------------------------------------------------------------
// This program is free software: you may redistribute it and/or modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
// version.
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with this program. If not, see
// https://www.gnu.org/licenses/.
//---------------------------------------------------------------------------------------------------------------------

#include <cascade_pid_controller/CascadePidController.h>
#include <cascade_pid_controller_msgs/TrajCommand.h>
#include <cascade_pid_controller_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <catec_control_manager_msgs/State.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cascade_pid_controller/utils.hpp>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <cmath>
#include <iostream>

namespace cascade_pid_controller {

enum class eControlMode {
    POSITION,
    VELOCITY,
    TRAJECTORY,
    NONE
};

std::ostream& operator<<(std::ostream& os, eControlMode mode) {
    return os << static_cast<std::underlying_type_t<eControlMode>>(mode);
}

void printControlModeInfo(eControlMode mode)
{
    switch (mode) {
        case eControlMode::POSITION: ROS_INFO("Position control mode"); break;
        case eControlMode::VELOCITY: ROS_INFO("Velocity control mode"); break;
        case eControlMode::TRAJECTORY: ROS_INFO("Trajectory control mode"); break;
        case eControlMode::NONE: ROS_INFO("No control mode selected"); break;
        default: ROS_INFO("Invalid control mode"); break;
    }
}

class CascadePidControllerNode
{
  public:
    CascadePidControllerNode();
    ~CascadePidControllerNode();

    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void poseReferenceCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void twistReferenceCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void trajectoryReferenceCallback(const cascade_pid_controller_msgs::TrajCommand::ConstPtr& msg);
    void controlManagerStateCallback(const catec_control_manager_msgs::State::ConstPtr& msg);
    void safetyThread(const double& rate);
    void controlThread(const ros::TimerEvent& event);

  private:
    bool         getParamFromMavros(const std::string& param_id, double& param_value);

    void         initController();
    void         resetController();
    void         resetReferences();
    bool         isValidPose(const geometry_msgs::Pose& pose);
    bool         isValidTwist(const geometry_msgs::Twist& twist);
    void         changeControlMode(eControlMode mode);
    eControlMode getControlMode();

  private:
    ros::NodeHandle                                    _nh, _nhp;
    std::chrono::time_point<std::chrono::system_clock> _prev_time;

    std::unique_ptr<CascadePidController> _cascade_pid;

    ros::Subscriber _odometry_sub;
    ros::Subscriber _pose_reference_sub;
    ros::Subscriber _twist_reference_sub;
    ros::Subscriber _trajectory_reference_sub;
    ros::Subscriber _state_sub;
    ros::Publisher  _cmd_pub;
    ros::Publisher  _control_mode_pub;

    ros::Time _last_control_manager_state_ts;
    ros::Time _last_pose_ref_ts;
    ros::Time _last_twist_ref_ts;
    ros::Time _last_odom_ts;
    ros::Time _last_trajectory_ref_ts;

    nav_msgs::Odometry                       _odometry;
    geometry_msgs::PoseStamped               _pose_reference;
    geometry_msgs::TwistStamped              _twist_reference;
    cascade_pid_controller_msgs::TrajCommand _trajectory_reference;
    tf2_ros::Buffer                          _tf_buffer;
    tf2_ros::TransformListener               _tf_listener;
    geometry_msgs::TransformStamped          _tf_2_ODOM;

    double           _loop_freq;
    std::thread      _safety_thread;
    std::atomic_bool _stop_atomic = false;

    ros::Timer _control_timer;

    bool _safety_checks_ok;
    bool _uav_authority;

    double _max_vel_xy, _max_vel_z, _max_yaw_rate, _max_roll, _max_pitch;
    double _WPNAV_SPEED_DN, _WPNAV_SPEED_UP;

    utils::Integrator<double> _yaw_integrator;
    eControlMode              _control_mode = eControlMode::NONE;
    
    ros::Time last_event_time_;


};
} // namespace cascade_pid_controller