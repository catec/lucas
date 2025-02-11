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
#include <cascade_pid_controller_msgs/msg/traj_command.hpp>
#include <cascade_pid_controller_msgs/msg/state.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <catec_control_manager_msgs/msg/state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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

rmw_qos_profile_t qos_profile{
        RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        1,
        RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
        RMW_QOS_POLICY_DURABILITY_VOLATILE,
        RMW_QOS_DEADLINE_DEFAULT,
        RMW_QOS_LIFESPAN_DEFAULT,
        RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
        RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
        false};

enum class eControlMode {
    POSITION,
    VELOCITY,
    TRAJECTORY,
    NONE
};

std::ostream& operator<<(std::ostream& os, eControlMode mode) {
    return os << static_cast<std::underlying_type_t<eControlMode>>(mode);
}

class CascadePidControllerNode : public rclcpp::Node
{
  public:
    CascadePidControllerNode(const rclcpp::NodeOptions& options);
    ~CascadePidControllerNode();

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void poseReferenceCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void twistReferenceCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void trajectoryReferenceCallback(const cascade_pid_controller_msgs::msg::TrajCommand::SharedPtr msg);
    void controlManagerStateCallback(const catec_control_manager_msgs::msg::State::SharedPtr msg);
    void safetyThread(const double& rate);
    void controlThread();


  private:
    bool         getParamFromMavros(const std::string& param_id, double& param_value);
    void         printControlModeInfo(eControlMode mode);

    void         initController();
    void         resetController();
    void         resetReferences();
    bool         isValidPose(const geometry_msgs::msg::Pose& pose);
    bool         isValidTwist(const geometry_msgs::msg::Twist& twist);
    void         changeControlMode(eControlMode mode);
    eControlMode getControlMode();

  private:
    std::chrono::time_point<std::chrono::system_clock> _prev_time;

    std::unique_ptr<CascadePidController> _cascade_pid;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odometry_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _pose_reference_sub;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr _twist_reference_sub;
    rclcpp::Subscription<cascade_pid_controller_msgs::msg::TrajCommand>::SharedPtr _trajectory_reference_sub;
    rclcpp::Subscription<catec_control_manager_msgs::msg::State>::SharedPtr _state_sub;

    rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr  _cmd_pub;
    rclcpp::Publisher<cascade_pid_controller_msgs::msg::State>::SharedPtr  _control_mode_pub;

    rclcpp::Time _last_control_manager_state_ts;
    rclcpp::Time _last_pose_ref_ts;
    rclcpp::Time _last_twist_ref_ts;
    rclcpp::Time _last_odom_ts;
    rclcpp::Time _last_trajectory_ref_ts;

    nav_msgs::msg::Odometry                       _odometry;
    geometry_msgs::msg::PoseStamped               _pose_reference;
    geometry_msgs::msg::TwistStamped              _twist_reference;
    cascade_pid_controller_msgs::msg::TrajCommand _trajectory_reference;

    double           _loop_freq;
    std::thread      _safety_thread;
    std::atomic_bool _stop_atomic = false;

    rclcpp::TimerBase::SharedPtr _control_timer;

    bool _safety_checks_ok;
    bool _uav_authority;

    bool _pid_initialize_flag = false;

    double _max_vel_xy, _max_vel_z, _max_yaw_rate, _max_roll, _max_pitch;
    double _WPNAV_SPEED_DN, _WPNAV_SPEED_UP;

    utils::Integrator<double> _yaw_integrator;
    eControlMode              _control_mode = eControlMode::NONE;
    
    rclcpp::Time last_event_time_;
};
} // namespace cascade_pid_controller