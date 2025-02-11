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

#include "catec_control_manager/ros_nodes/control_manager_node.h"

#include <catec_control_manager_msgs/State.h>
#include <mavros_msgs/SetMode.h>

#include "cascade_pid_controller_msgs/TrajCommand.h"
#include "catec_control_manager/common/log_manager.h"
#include "catec_control_manager/control_manager_state_machine.h"
#include "catec_control_manager/ros_nodes/ros_parsers.h"

namespace catec {
ControlManagerNode::ControlManagerNode() :
        _nh(ros::NodeHandle("~")),
        _control_manager_sm(std::make_unique<ControlManagerStateMachine>())
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    configureTopics();
    configureServices();

    std::string parameters_file_path;
    _nh.param<std::string>("parameters_file_path", parameters_file_path, "parameters_file_path");
    _control_manager_sm->loadParametersFile(parameters_file_path);

    _control_manager_sm->sendCommandToRosCb = std::bind(
            static_cast<void (ControlManagerNode::*)(const CommandMsg&)>(&ControlManagerNode::sendRosMessage),
            this,
            std::placeholders::_1);

    _control_manager_sm->setStateToRosCb = std::bind(
            static_cast<void (ControlManagerNode::*)(const MavrosState&)>(&ControlManagerNode::callRosService),
            this,
            std::placeholders::_1);

    _control_manager_sm->start();
}

ControlManagerNode::~ControlManagerNode()
{
    LOG_TRACE(__PRETTY_FUNCTION__);
}

void ControlManagerNode::configureTopics()
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    std::string odometry_topic, imu_topic, pwm_out_topic, mavros_rangefinder_topic, mavros_rc_in_topic,
            mavros_extended_state_topic, mavros_state_topic, controller_state_topic, twist_assisted_reference_topic,
            offboard_reference_topic, current_state_topic, controller_pose_reference_topic,
            controller_twist_reference_topic, controller_trajectory_reference_topic, set_mavros_mode_topic;
    float state_pub_rate;
    _nh.param<std::string>("odometry_topic", odometry_topic, "odometry");
    _nh.param<std::string>("imu_topic", imu_topic, "imu");
    _nh.param<std::string>("pwm_out_topic", pwm_out_topic, "pwm_out_topic");
    _nh.param<std::string>("mavros_rangefinder_topic", mavros_rangefinder_topic, "pwm_out_topic");
    _nh.param<std::string>("mavros_extended_state_topic", mavros_extended_state_topic, "/mavros/extended_state");
    _nh.param<std::string>("mavros_state_topic", mavros_state_topic, "/mavros/state");
    _nh.param<std::string>("controller_state_topic", controller_state_topic, "/cascade_pid_controller/control_mode");
    _nh.param<std::string>("offboard_reference_topic", offboard_reference_topic, "offboard_reference_topic");
    _nh.param<std::string>(
            "twist_assisted_reference_topic", twist_assisted_reference_topic, "/twist_assisted_reference_topic");

    _nh.param<std::string>("current_state_topic", current_state_topic, "state");
    _nh.param<std::string>("controller_pose_reference_topic", controller_pose_reference_topic, "pose_reference_topic");
    _nh.param<std::string>(
            "controller_twist_reference_topic", controller_twist_reference_topic, "pose_reference_topic");
    _nh.param<std::string>(
            "controller_trajectory_reference_topic", controller_trajectory_reference_topic, "pose_reference_topic");
    _nh.param<float>("state_pub_rate", state_pub_rate, 30);

    // clang-format off
    _odometry_sub = _nh.subscribe<nav_msgs::Odometry>(odometry_topic, 1, &ControlManagerNode::odometryCb, 
                                                      this, ros::TransportHints().tcpNoDelay());
    _imu_sub = _nh.subscribe<sensor_msgs::Imu>(imu_topic, 1, &ControlManagerNode::imuCb, 
                                                      this, ros::TransportHints().tcpNoDelay());
    _pwm_out_sub = _nh.subscribe<mavros_msgs::RCOut>(pwm_out_topic, 1, &ControlManagerNode::pwmOutCb, 
                                                      this, ros::TransportHints().tcpNoDelay());
    _mavros_rangefinder_sub = _nh.subscribe<sensor_msgs::Range>(mavros_rangefinder_topic, 1, &ControlManagerNode::mavrosRangefinderCb, 
                                                      this, ros::TransportHints().tcpNoDelay());

    _mavros_cur_state_sub = 
        _nh.subscribe<mavros_msgs::State>(mavros_state_topic, 1, &ControlManagerNode::mavrosStateCb,
                                          this, ros::TransportHints().tcpNoDelay());
    _mavros_cur_extended_state_sub = 
        _nh.subscribe<mavros_msgs::ExtendedState>(mavros_extended_state_topic, 1, &ControlManagerNode::mavrosExtendedStateCb,
                                          this, ros::TransportHints().tcpNoDelay());

    _controller_state_sub = 
        _nh.subscribe<cascade_pid_controller_msgs::State>(controller_state_topic, 1, &ControlManagerNode::controllerStateCb,
                                          this, ros::TransportHints().tcpNoDelay());

    _twist_reference_sub = 
        _nh.subscribe<geometry_msgs::Twist>(twist_assisted_reference_topic, 1, &ControlManagerNode::twistReferenceCb,
                                         this, ros::TransportHints().tcpNoDelay());

    _offboard_reference_sub = 
        _nh.subscribe<cascade_pid_controller_msgs::TrajCommand>(offboard_reference_topic, 1, &ControlManagerNode::offboardReferenceCb,
                                         this, ros::TransportHints().tcpNoDelay());
    // clang-format on

    _state_pub_timer = _nh.createTimer(
            ros::Duration(1.0 / state_pub_rate), std::bind(&ControlManagerNode::publishCurrentState, this));
    _current_state_pub              = _nh.advertise<catec_control_manager_msgs::State>(current_state_topic, 1);
    _controller_pose_reference_pub  = _nh.advertise<geometry_msgs::PoseStamped>(controller_pose_reference_topic, 1);
    _controller_twist_reference_pub = _nh.advertise<geometry_msgs::TwistStamped>(controller_twist_reference_topic, 1);
    _controller_trajectory_reference_pub
            = _nh.advertise<cascade_pid_controller_msgs::TrajCommand>(controller_trajectory_reference_topic, 1);
}

void ControlManagerNode::configureServices()
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    std::string mavros_set_move_srv, takeoff_srv, land_srv, get_authority_srv, set_mode_assisted_srv,
            set_mode_offboard_srv, set_mode_hover_srv, go_to_waypoint_srv;
    _nh.param<std::string>("mavros_set_move_srv", mavros_set_move_srv, "set_mode");
    _nh.param<std::string>("get_authority_srv", get_authority_srv, "get_authority");
    _nh.param<std::string>("takeoff_srv", takeoff_srv, "take_off");
    _nh.param<std::string>("land_srv", land_srv, "land");
    _nh.param<std::string>("set_mode_hover_srv", set_mode_hover_srv, "set_mode_hover");
    _nh.param<std::string>("set_mode_assisted_srv", set_mode_assisted_srv, "set_mode_assisted");
    _nh.param<std::string>("set_mode_offboard_srv", set_mode_offboard_srv, "set_mode_offboard");
    _nh.param<std::string>("go_to_waypoint_srv", go_to_waypoint_srv, "set_mode_offboard");

    _set_flight_mode_client = _nh.serviceClient<mavros_msgs::SetMode>(mavros_set_move_srv);

    _get_authority_server  = _nh.advertiseService(get_authority_srv, &ControlManagerNode::getAuthorityServer, this);
    _takeoff_server        = _nh.advertiseService(takeoff_srv, &ControlManagerNode::takeOffServer, this);
    _land_server           = _nh.advertiseService(land_srv, &ControlManagerNode::landServer, this);
    _set_mode_hover_server = _nh.advertiseService(set_mode_hover_srv, &ControlManagerNode::setModeHoverServer, this);
    _set_mode_assisted_server
            = _nh.advertiseService(set_mode_assisted_srv, &ControlManagerNode::setModeAssistedServer, this);
    _set_mode_offboard_server
            = _nh.advertiseService(set_mode_offboard_srv, &ControlManagerNode::setModeOffboardServer, this);
    _go_to_waypoint_server = _nh.advertiseService(go_to_waypoint_srv, &ControlManagerNode::goToWaypointServer, this);
}

void ControlManagerNode::publishCurrentState()
{
    // LOG_TRACE(__PRETTY_FUNCTION__);

    const auto current_state = _control_manager_sm->getCurrentState();

    catec_control_manager_msgs::State current_state_msg;
    current_state_msg.state = static_cast<uint8_t>(current_state);
    current_state_msg.name  = ToString(current_state);
    _current_state_pub.publish(current_state_msg);
}

void ControlManagerNode::mavrosStateCb(const mavros_msgs::State::ConstPtr& msg)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    MavrosState state = MavrosState::NONE;

    auto it = mavros_state_table.find(msg->mode);
    if (it != mavros_state_table.end()) {
        state = it->second;
    } else {
        LOG_WARN("{} - Unable to match received mavros state: '{}'", __PRETTY_FUNCTION__, msg->mode);
        return;
    }

    MavrosStateComplete received_state;
    received_state.state = state;
    received_state.armed = msg->armed;

    _control_manager_sm->updateMavrosState(received_state);
}

void ControlManagerNode::mavrosExtendedStateCb(const mavros_msgs::ExtendedState::ConstPtr& msg)
{
    // LOG_TRACE(__PRETTY_FUNCTION__);

    MavrosExtendedState received_extended_state = static_cast<MavrosExtendedState>(msg->landed_state);

    _control_manager_sm->updateMavrosExtendedState(received_extended_state);
}

void ControlManagerNode::controllerStateCb(const cascade_pid_controller_msgs::State::ConstPtr& msg)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    CommandMask controller_state = static_cast<CommandMask>(msg->state);

    _control_manager_sm->updateControllerState(controller_state);
}

void ControlManagerNode::odometryCb(const nav_msgs::Odometry::ConstPtr& msg)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    if (!msg) {
        LOG_WARN("{} - Received nullptr odometry", __PRETTY_FUNCTION__);
        return;
    }
    const auto odom_msg = convertFromRosMsg(*msg);
    _control_manager_sm->updateOdometry(odom_msg);
}

void ControlManagerNode::imuCb(const sensor_msgs::Imu::ConstPtr& msg)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    if (!msg) {
        LOG_WARN("{} - Received nullptr imu message", __PRETTY_FUNCTION__);
        return;
    }
    const auto imu_msg = convertFromRosMsg(*msg);
    _control_manager_sm->updateImu(imu_msg);
}

void ControlManagerNode::pwmOutCb(const mavros_msgs::RCOut::ConstPtr& msg)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    if (!msg) {
        LOG_WARN("{} - Received nullptr pwm out msg", __PRETTY_FUNCTION__);
        return;
    }
    const auto pwm_msg = convertFromRosMsg(*msg);
    _control_manager_sm->updatePwm(pwm_msg);
}

void ControlManagerNode::mavrosRangefinderCb(const sensor_msgs::Range::ConstPtr& msg)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    if (!msg) {
        LOG_WARN("{} - Received nullptr mavros rangefinder msg", __PRETTY_FUNCTION__);
        return;
    }

    _control_manager_sm->resetRangefinderChecker();
}

void ControlManagerNode::twistReferenceCb(const geometry_msgs::Twist::ConstPtr& msg)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    if (!msg) {
        LOG_WARN("{} - Received nullptr twist reference", __PRETTY_FUNCTION__);
        return;
    }
    const auto twist_msg = convertFromRosMsg(*msg);

    // \todo Check sm state before update reference here? or implement a method in sm that checks before update?
    // Bad thing is that we will have a moment without reference.

    const auto current_state = _control_manager_sm->getCurrentState();

    // \note In assisted mode we check if generic reference exist in the entry, so we allow to update the reference in
    // HOVER state
    if (current_state == ControlManagerState::ASSISTED) {
        _control_manager_sm->updateGenericReference(twist_msg);
    } else {
        LOG_DEBUG_THROTTLE(
                1, "{} - Twist reference is not being updated because mode is not ASSISTED", __PRETTY_FUNCTION__);
    }
}

void ControlManagerNode::offboardReferenceCb(const cascade_pid_controller_msgs::TrajCommand::ConstPtr& msg)

{
    LOG_TRACE(__PRETTY_FUNCTION__);

    if (!msg) {
        LOG_WARN("{} - Received nullptr offboard reference", __PRETTY_FUNCTION__);

        return;
    }

    const auto offboard_msg = convertFromRosMsg(*msg);

    /// \todo Check sm state before update reference here? or implement a method in sm that checks before update?
    /// Bad thing is that we will have a moment without reference. Maybe use a different pointer to store offboard
    /// references and another for twist.

    const auto current_state = _control_manager_sm->getCurrentState();

    /// \note In offboard mode we do hover if it is a generic reference that is not initialized, so we do not need to
    /// update the reference in HOVER state, what would be dangerous because of twist reference callback
    if (current_state == ControlManagerState::OFFBOARD) {
        _control_manager_sm->updateGenericReference(offboard_msg);
    } else {
        LOG_DEBUG("{} - Offboard reference is not being updated because mode is not OFFBOARD", __PRETTY_FUNCTION__);
    }
}

bool ControlManagerNode::getAuthorityServer(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    // WHO IS IN CHARGE OF CALL MAVROS SERVICE?

    auto response = _control_manager_sm->requestAuthority();

    // HOVER

    res.success = response.first;
    res.message = response.second;

    // if dispatch to hover, call mavros set mode GUIDED NO GPS ¿CHECK REFERENCES?

    // if mavros state is guided no gps?

    return true;
}

bool ControlManagerNode::takeOffServer(
        catec_control_manager_msgs::TakeOff::Request&  req,
        catec_control_manager_msgs::TakeOff::Response& res)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    auto response = _control_manager_sm->requestTakeOff(req.height);

    res.success = response.first;
    res.message = response.second;
    return true;
}

bool ControlManagerNode::landServer(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    auto response = _control_manager_sm->requestLand();

    res.success = response.first;
    res.message = response.second;
    return true;
}

bool ControlManagerNode::setModeHoverServer(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    auto response = _control_manager_sm->requestChangeToHover();

    res.success = response.first;
    res.message = response.second;
    return true;
}

bool ControlManagerNode::setModeAssistedServer(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    auto response = _control_manager_sm->requestChangeToAssisted();

    res.success = response.first;
    res.message = response.second;
    return true;
}

bool ControlManagerNode::setModeOffboardServer(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    auto response = _control_manager_sm->requestChangeToOffboard();

    res.success = response.first;
    res.message = response.second;
    return true;
}

bool ControlManagerNode::goToWaypointServer(
        catec_control_manager_msgs::GoToWaypoint::Request&  req,
        catec_control_manager_msgs::GoToWaypoint::Response& res)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    const auto frame_id = req.waypoint.header.frame_id;

    auto response = _control_manager_sm->requestGoToWaypoint(frame_id, convertFromRosMsg(req.waypoint.pose));

    res.success = response.first;
    res.message = response.second;
    return true;
}

void ControlManagerNode::sendRosMessage(const CommandMsg& msg)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    const std::string frame_id{"odom"};
    if (msg.mask == CommandMask::POSITION) {
        auto pose_msg            = convertToRosMsg<geometry_msgs::PoseStamped>(msg);
        pose_msg.header.frame_id = frame_id;
        if (isSafeRosMsg(pose_msg)) {
            _controller_pose_reference_pub.publish(pose_msg);
        } else {
            LOG_ERROR("{} - PoseStamped msg not safe", __PRETTY_FUNCTION__);
        }
    } else if (msg.mask == CommandMask::VELOCITY) {
        auto twist_msg            = convertToRosMsg<geometry_msgs::TwistStamped>(msg);
        twist_msg.header.frame_id = frame_id;
        if (isSafeRosMsg(twist_msg)) {
            _controller_twist_reference_pub.publish(twist_msg);
        } else {
            LOG_ERROR("{} - TwistStamped msg not safe", __PRETTY_FUNCTION__);
        }
    } else if (msg.mask == CommandMask::TRAJECTORY) {
        auto trajectory_msg            = convertToRosMsg<cascade_pid_controller_msgs::TrajCommand>(msg);
        trajectory_msg.header.frame_id = frame_id;
        trajectory_msg.acceleration.x  = 0.0;
        if (isSafeRosMsg(trajectory_msg)) {
            _controller_trajectory_reference_pub.publish(trajectory_msg);
        } else {
            LOG_ERROR("{} - TrajCommand msg not safe", __PRETTY_FUNCTION__);
        }
    } else {
        LOG_WARN("{} - Trying to send ROS message with undedined command mask", __PRETTY_FUNCTION__);
    }
}

void ControlManagerNode::callRosService(const MavrosState& mode)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    LOG_INFO("{} - Calling ROS service", __PRETTY_FUNCTION__);

    mavros_msgs::SetMode srv;

    auto it = std::find_if(
            mavros_state_table.cbegin(), mavros_state_table.cend(), [&mode](auto&& p) { return p.second == mode; });
    if (it == mavros_state_table.cend()) {
        LOG_WARN("{} - Unable to match current state with states table", __PRETTY_FUNCTION__);
        return;
    }

    srv.request.custom_mode = it->first;

    if (_set_flight_mode_client.call(srv)) {
        if (srv.response.mode_sent) {
            LOG_INFO("{} - Succesfully mode send to autopilot '{}'", __PRETTY_FUNCTION__, srv.request.custom_mode);
        } else {
            LOG_ERROR("{} - Failed to send mode to autopilot '{}'", __PRETTY_FUNCTION__, srv.request.custom_mode);
        }
    } else {
        LOG_ERROR("{} - Failed to call mavros set mode service '{}'", __PRETTY_FUNCTION__, srv.request.custom_mode);
    }
}

} // namespace catec
