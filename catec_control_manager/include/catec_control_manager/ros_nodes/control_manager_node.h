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

#include <catec_control_manager_msgs/GoToWaypoint.h>
#include <catec_control_manager_msgs/TakeOff.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/RCOut.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <std_srvs/Trigger.h>

#include <Eigen/Eigen>

#include "cascade_pid_controller_msgs/State.h"
#include "cascade_pid_controller_msgs/TrajCommand.h"
#include "catec_control_manager/common/types.h"

namespace catec {
class ControlManagerStateMachine;

class ControlManagerNode
{
  public:
    ControlManagerNode();
    ~ControlManagerNode();

  private:
    void configureTopics();
    void configureServices();

    void mavrosStateCb(const mavros_msgs::State::ConstPtr& msg);
    void mavrosExtendedStateCb(const mavros_msgs::ExtendedState::ConstPtr& msg);
    void controllerStateCb(const cascade_pid_controller_msgs::State::ConstPtr& msg);
    void odometryCb(const nav_msgs::Odometry::ConstPtr& msg);
    void imuCb(const sensor_msgs::Imu::ConstPtr& msg);
    void pwmOutCb(const mavros_msgs::RCOut::ConstPtr& msg);
    void mavrosRangefinderCb(const sensor_msgs::Range::ConstPtr& msg);
    void offboardReferenceCb(const cascade_pid_controller_msgs::TrajCommand::ConstPtr& msg);
    void twistReferenceCb(const geometry_msgs::Twist::ConstPtr& msg);

    void publishCurrentState();

    bool getAuthorityServer(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool takeOffServer(
            catec_control_manager_msgs::TakeOff::Request&  req,
            catec_control_manager_msgs::TakeOff::Response& res);
    bool landServer(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool setModeHoverServer(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool setModeAssistedServer(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool setModeOffboardServer(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool goToWaypointServer(
            catec_control_manager_msgs::GoToWaypoint::Request&  req,
            catec_control_manager_msgs::GoToWaypoint::Response& res);

    void sendRosMessage(const CommandMsg& ref);
    void callRosService(const MavrosState& mode);

  private:
    ros::NodeHandle _nh;

    ros::Timer _state_pub_timer;

    ros::Subscriber _odometry_sub;
    ros::Subscriber _imu_sub;
    ros::Subscriber _pwm_out_sub;
    ros::Subscriber _mavros_rangefinder_sub;
    ros::Subscriber _mavros_cur_state_sub;
    ros::Subscriber _mavros_cur_extended_state_sub;
    ros::Subscriber _controller_state_sub;
    ros::Subscriber _offboard_reference_sub;
    ros::Subscriber _twist_reference_sub;

    ros::Publisher _current_state_pub;
    ros::Publisher _controller_pose_reference_pub;
    ros::Publisher _controller_twist_reference_pub;
    ros::Publisher _controller_trajectory_reference_pub;

    ros::ServiceServer _takeoff_server;
    ros::ServiceServer _land_server;
    ros::ServiceServer _get_authority_server;
    ros::ServiceServer _set_mode_hover_server;
    ros::ServiceServer _set_mode_assisted_server;
    ros::ServiceServer _set_mode_offboard_server;
    ros::ServiceServer _go_to_waypoint_server;

    ros::ServiceClient _set_flight_mode_client;

    std::unique_ptr<ControlManagerStateMachine> _control_manager_sm;
};
} // namespace catec
