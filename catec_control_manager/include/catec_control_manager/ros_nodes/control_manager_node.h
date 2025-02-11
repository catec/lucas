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

#pragma once

#include <cascade_pid_controller_msgs/msg/state.hpp>
#include <cascade_pid_controller_msgs/msg/traj_command.hpp>
#include <catec_control_manager_msgs/msg/state.hpp>
#include <catec_control_manager_msgs/srv/go_to_waypoint.hpp>
#include <catec_control_manager_msgs/srv/take_off.hpp>
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/rc_out.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "catec_control_manager/common/types.h"

namespace catec {

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

class ControlManagerStateMachine;

class ControlManagerNode : public rclcpp::Node
{
  public:
    ControlManagerNode(const rclcpp::NodeOptions& options);
    ~ControlManagerNode();

  private:
    void configureTopics();
    void configureServices();

    void mavrosStateCb(const mavros_msgs::msg::State::SharedPtr msg);
    void mavrosExtendedStateCb(const mavros_msgs::msg::ExtendedState::SharedPtr msg);
    void controllerStateCb(const cascade_pid_controller_msgs::msg::State::SharedPtr msg);
    void odometryCb(const nav_msgs::msg::Odometry::SharedPtr msg);
    void imuCb(const sensor_msgs::msg::Imu::SharedPtr msg);
    void pwmOutCb(const mavros_msgs::msg::RCOut::SharedPtr msg);
    void mavrosRangefinderCb(const sensor_msgs::msg::Range::SharedPtr msg);
    void offboardReferenceCb(const cascade_pid_controller_msgs::msg::TrajCommand::SharedPtr msg);
    void twistReferenceCb(const geometry_msgs::msg::Twist::SharedPtr msg);

    void publishCurrentState();

    bool getAuthorityServer(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
            std::shared_ptr<std_srvs::srv::Trigger::Response>      res);
    bool takeOffServer(
            const std::shared_ptr<catec_control_manager_msgs::srv::TakeOff::Request> req,
            std::shared_ptr<catec_control_manager_msgs::srv::TakeOff::Response>      res);
    bool landServer(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
            std::shared_ptr<std_srvs::srv::Trigger::Response>      res);
    bool setModeHoverServer(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
            std::shared_ptr<std_srvs::srv::Trigger::Response>      res);
    bool setModeAssistedServer(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
            std::shared_ptr<std_srvs::srv::Trigger::Response>      res);
    bool setModeOffboardServer(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
            std::shared_ptr<std_srvs::srv::Trigger::Response>      res);

    bool goToWaypointServer(
            const std::shared_ptr<catec_control_manager_msgs::srv::GoToWaypoint::Request> req,
            std::shared_ptr<catec_control_manager_msgs::srv::GoToWaypoint::Response>      res);

    void sendRosMessage(const CommandMsg& ref);
    void callRosService(const MavrosState& mode);

  private:
    rclcpp::TimerBase::SharedPtr _state_pub_timer;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                       _odometry_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr                         _imu_sub;
    rclcpp::Subscription<mavros_msgs::msg::RCOut>::SharedPtr                       _pwm_out_sub;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr                       _mavros_rangefinder_sub;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr                       _mavros_cur_state_sub;
    rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr               _mavros_cur_extended_state_sub;
    rclcpp::Subscription<cascade_pid_controller_msgs::msg::State>::SharedPtr       _controller_state_sub;
    rclcpp::Subscription<cascade_pid_controller_msgs::msg::TrajCommand>::SharedPtr _offboard_reference_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr                     _twist_reference_sub;

    rclcpp::Publisher<catec_control_manager_msgs::msg::State>::SharedPtr        _current_state_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr               _controller_pose_reference_pub;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr              _controller_twist_reference_pub;
    rclcpp::Publisher<cascade_pid_controller_msgs::msg::TrajCommand>::SharedPtr _controller_trajectory_reference_pub;

    rclcpp::Service<catec_control_manager_msgs::srv::TakeOff>::SharedPtr      _takeoff_server;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                        _land_server;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                        _get_authority_server;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                        _set_mode_hover_server;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                        _set_mode_assisted_server;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                        _set_mode_offboard_server;
    rclcpp::Service<catec_control_manager_msgs::srv::GoToWaypoint>::SharedPtr _go_to_waypoint_server;

    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr _set_flight_mode_client;

    std::unique_ptr<ControlManagerStateMachine> _control_manager_sm;
};
} // namespace catec
