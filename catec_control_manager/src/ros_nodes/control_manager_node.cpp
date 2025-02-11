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

#include "catec_control_manager/control_manager_state_machine.h"
#include "catec_control_manager/ros_nodes/ros_parsers.h"

namespace catec {
ControlManagerNode::ControlManagerNode(const rclcpp::NodeOptions& options) :
        Node("control_manager_node", options),
        _control_manager_sm(std::make_unique<ControlManagerStateMachine>())
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    configureTopics();
    configureServices();

    std::string parameters_file_path = this->declare_parameter<std::string>("parameters_file_path", "");
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

    auto odometry_topic = this->declare_parameter<std::string>("odometry_topic", "odometry");
    auto imu_topic      = this->declare_parameter<std::string>("imu_topic", "imu");
    auto pwm_out_topic  = this->declare_parameter<std::string>("pwm_out_topic", "pwm_out_topic");
    auto mavros_rangefinder_topic
            = this->declare_parameter<std::string>("mavros_rangefinder_topic", "mavros_rangefinder_topic");
    auto mavros_extended_state_topic
            = this->declare_parameter<std::string>("mavros_extended_state_topic", "/mavros/extended_state");
    auto mavros_state_topic = this->declare_parameter<std::string>("mavros_state_topic", "/mavros/state");
    auto controller_state_topic
            = this->declare_parameter<std::string>("controller_state_topic", "/cascade_pid_controller/control_mode");
    auto offboard_reference_topic
            = this->declare_parameter<std::string>("offboard_reference_topic", "/offboard_reference_topic");
    auto twist_assisted_reference_topic
            = this->declare_parameter<std::string>("twist_assisted_reference_topic", "/twist_assisted_reference_topic");
    auto current_state_topic             = this->declare_parameter<std::string>("current_state_topic", "state");
    auto controller_pose_reference_topic = this->declare_parameter<std::string>(
            "controller_pose_reference_topic", "/controller_pose_reference_topic");
    auto controller_twist_reference_topic = this->declare_parameter<std::string>(
            "controller_twist_reference_topic", "/controller_twist_reference_topic");
    auto controller_trajectory_reference_topic = this->declare_parameter<std::string>(
            "controller_trajectory_reference_topic", "/controller_trajectory_reference_topic");
    auto state_pub_rate = this->declare_parameter<double>("state_pub_rate", 30.0);

    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

    _odometry_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            odometry_topic, qos, std::bind(&ControlManagerNode::odometryCb, this, std::placeholders::_1));

    _imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, rclcpp::SensorDataQoS(), std::bind(&ControlManagerNode::imuCb, this, std::placeholders::_1));

    _pwm_out_sub = this->create_subscription<mavros_msgs::msg::RCOut>(
            pwm_out_topic,
            rclcpp::SensorDataQoS(),
            std::bind(&ControlManagerNode::pwmOutCb, this, std::placeholders::_1));

    _mavros_rangefinder_sub = this->create_subscription<sensor_msgs::msg::Range>(
            mavros_rangefinder_topic,
            rclcpp::SensorDataQoS(),
            std::bind(&ControlManagerNode::mavrosRangefinderCb, this, std::placeholders::_1));

    _mavros_cur_state_sub = this->create_subscription<mavros_msgs::msg::State>(
            mavros_state_topic,
            rclcpp::SensorDataQoS(),
            std::bind(&ControlManagerNode::mavrosStateCb, this, std::placeholders::_1));

    _mavros_cur_extended_state_sub = this->create_subscription<mavros_msgs::msg::ExtendedState>(
            mavros_extended_state_topic,
            rclcpp::SensorDataQoS(),
            std::bind(&ControlManagerNode::mavrosExtendedStateCb, this, std::placeholders::_1));

    _controller_state_sub = this->create_subscription<cascade_pid_controller_msgs::msg::State>(
            controller_state_topic,
            qos,
            std::bind(&ControlManagerNode::controllerStateCb, this, std::placeholders::_1));

    _twist_reference_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            twist_assisted_reference_topic,
            qos,
            std::bind(&ControlManagerNode::twistReferenceCb, this, std::placeholders::_1));

    _offboard_reference_sub = this->create_subscription<cascade_pid_controller_msgs::msg::TrajCommand>(
            offboard_reference_topic,
            qos,
            std::bind(&ControlManagerNode::offboardReferenceCb, this, std::placeholders::_1));

    _state_pub_timer = this->create_wall_timer(
            std::chrono::duration<double>(1. / state_pub_rate),
            std::bind(&ControlManagerNode::publishCurrentState, this));

    _current_state_pub = this->create_publisher<catec_control_manager_msgs::msg::State>(current_state_topic, qos);
    _controller_pose_reference_pub
            = this->create_publisher<geometry_msgs::msg::PoseStamped>(controller_pose_reference_topic, qos);
    _controller_twist_reference_pub
            = this->create_publisher<geometry_msgs::msg::TwistStamped>(controller_twist_reference_topic, qos);
    _controller_trajectory_reference_pub = this->create_publisher<cascade_pid_controller_msgs::msg::TrajCommand>(
            controller_trajectory_reference_topic, qos);
}

void ControlManagerNode::configureServices()
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    auto mavros_set_move_srv   = this->declare_parameter<std::string>("mavros_set_move_srv", "set_mode");
    auto get_authority_srv     = this->declare_parameter<std::string>("get_authority_srv", "get_authority");
    auto takeoff_srv           = this->declare_parameter<std::string>("takeoff_srv", "take_off");
    auto land_srv              = this->declare_parameter<std::string>("land_srv", "land");
    auto go_to_waypoint_srv    = this->declare_parameter<std::string>("go_to_waypoint_srv", "gotowp");
    auto set_mode_hover_srv    = this->declare_parameter<std::string>("set_mode_hover_srv", "set_mode_hover");
    auto set_mode_assisted_srv = this->declare_parameter<std::string>("set_mode_assisted_srv", "set_mode_assisted");
    auto set_mode_offboard_srv = this->declare_parameter<std::string>("set_mode_offboard_srv", "set_mode_offboard");

    _set_flight_mode_client = this->create_client<mavros_msgs::srv::SetMode>(mavros_set_move_srv);

    _get_authority_server = this->create_service<std_srvs::srv::Trigger>(
            get_authority_srv,
            std::bind(&ControlManagerNode::getAuthorityServer, this, std::placeholders::_1, std::placeholders::_2));
    _takeoff_server = this->create_service<catec_control_manager_msgs::srv::TakeOff>(
            takeoff_srv,
            std::bind(&ControlManagerNode::takeOffServer, this, std::placeholders::_1, std::placeholders::_2));
    _land_server = this->create_service<std_srvs::srv::Trigger>(
            land_srv, std::bind(&ControlManagerNode::landServer, this, std::placeholders::_1, std::placeholders::_2));
    _go_to_waypoint_server = this->create_service<catec_control_manager_msgs::srv::GoToWaypoint>(
            go_to_waypoint_srv,
            std::bind(&ControlManagerNode::goToWaypointServer, this, std::placeholders::_1, std::placeholders::_2));
    _set_mode_hover_server = this->create_service<std_srvs::srv::Trigger>(
            set_mode_hover_srv,
            std::bind(&ControlManagerNode::setModeHoverServer, this, std::placeholders::_1, std::placeholders::_2));
    _set_mode_assisted_server = this->create_service<std_srvs::srv::Trigger>(
            set_mode_assisted_srv,
            std::bind(&ControlManagerNode::setModeAssistedServer, this, std::placeholders::_1, std::placeholders::_2));
    _set_mode_offboard_server = this->create_service<std_srvs::srv::Trigger>(
            set_mode_offboard_srv,
            std::bind(&ControlManagerNode::setModeOffboardServer, this, std::placeholders::_1, std::placeholders::_2));
}

void ControlManagerNode::publishCurrentState()
{
    // LOG_TRACE(__PRETTY_FUNCTION__);

    const auto current_state = _control_manager_sm->getCurrentState();

    catec_control_manager_msgs::msg::State current_state_msg;
    current_state_msg.state = static_cast<uint8_t>(current_state);
    current_state_msg.name  = ToString(current_state);
    _current_state_pub->publish(current_state_msg);
}

void ControlManagerNode::mavrosStateCb(const mavros_msgs::msg::State::SharedPtr msg)
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

void ControlManagerNode::mavrosExtendedStateCb(const mavros_msgs::msg::ExtendedState::SharedPtr msg)
{
    // LOG_TRACE(__PRETTY_FUNCTION__);

    MavrosExtendedState received_extended_state = static_cast<MavrosExtendedState>(msg->landed_state);

    _control_manager_sm->updateMavrosExtendedState(received_extended_state);
}

void ControlManagerNode::controllerStateCb(const cascade_pid_controller_msgs::msg::State::SharedPtr msg)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    CommandMask controller_state = static_cast<CommandMask>(msg->state);

    _control_manager_sm->updateControllerState(controller_state);
}

void ControlManagerNode::odometryCb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    if (!msg) {
        LOG_WARN("{} - Received nullptr odometry", __PRETTY_FUNCTION__);
        return;
    }
    const auto odom_msg = convertFromRosMsg(*msg);
    _control_manager_sm->updateOdometry(odom_msg);
}

void ControlManagerNode::imuCb(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    if (!msg) {
        LOG_WARN("{} - Received nullptr imu message", __PRETTY_FUNCTION__);
        return;
    }
    const auto imu_msg = convertFromRosMsg(*msg);
    _control_manager_sm->updateImu(imu_msg);
}

void ControlManagerNode::pwmOutCb(const mavros_msgs::msg::RCOut::SharedPtr msg)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    if (!msg) {
        LOG_WARN("{} - Received nullptr pwm out msg", __PRETTY_FUNCTION__);
        return;
    }
    const auto pwm_msg = convertFromRosMsg(*msg);
    _control_manager_sm->updatePwm(pwm_msg);
}

void ControlManagerNode::mavrosRangefinderCb(const sensor_msgs::msg::Range::SharedPtr msg)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    if (!msg) {
        LOG_WARN("{} - Received nullptr mavros rangefinder msg", __PRETTY_FUNCTION__);
        return;
    }

    _control_manager_sm->resetRangefinderChecker();
}

void ControlManagerNode::twistReferenceCb(const geometry_msgs::msg::Twist::SharedPtr msg)
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

void ControlManagerNode::offboardReferenceCb(const cascade_pid_controller_msgs::msg::TrajCommand::SharedPtr msg)

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

bool ControlManagerNode::getAuthorityServer(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    // WHO IS IN CHARGE OF CALL MAVROS SERVICE?

    auto response = _control_manager_sm->requestAuthority();

    // HOVER

    res->success = response.first;
    res->message = response.second;

    // if dispatch to hover, call mavros set mode GUIDED NO GPS ¿CHECK REFERENCES?

    // if mavros state is guided no gps?

    return true;
}

bool ControlManagerNode::takeOffServer(
        const std::shared_ptr<catec_control_manager_msgs::srv::TakeOff::Request> req,
        std::shared_ptr<catec_control_manager_msgs::srv::TakeOff::Response>      res)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    auto response = _control_manager_sm->requestTakeOff(req->height);

    res->success = response.first;
    res->message = response.second;
    return true;
}

bool ControlManagerNode::landServer(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    auto response = _control_manager_sm->requestLand();

    res->success = response.first;
    res->message = response.second;
    return true;
}

bool ControlManagerNode::setModeHoverServer(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    auto response = _control_manager_sm->requestChangeToHover();

    res->success = response.first;
    res->message = response.second;
    return true;
}

bool ControlManagerNode::setModeAssistedServer(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    auto response = _control_manager_sm->requestChangeToAssisted();

    res->success = response.first;
    res->message = response.second;
    return true;
}

bool ControlManagerNode::setModeOffboardServer(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    auto response = _control_manager_sm->requestChangeToOffboard();

    res->success = response.first;
    res->message = response.second;
    return true;
}

bool ControlManagerNode::goToWaypointServer(
        const std::shared_ptr<catec_control_manager_msgs::srv::GoToWaypoint::Request> req,
        std::shared_ptr<catec_control_manager_msgs::srv::GoToWaypoint::Response>      res)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    const auto frame_id = req->waypoint.header.frame_id;

    auto response = _control_manager_sm->requestGoToWaypoint(frame_id, convertFromRosMsg(req->waypoint.pose));

    res->success = response.first;
    res->message = response.second;
    return true;
}

void ControlManagerNode::sendRosMessage(const CommandMsg& msg)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    LOG_INFO("{} - Sending reference to ROS", __PRETTY_FUNCTION__);

    auto now = this->get_clock()->now();

    const std::string frame_id{"odom"};
    if (msg.mask == CommandMask::POSITION) {
        auto pose_msg            = convertToRosMsg<geometry_msgs::msg::PoseStamped>(msg, now);
        pose_msg.header.frame_id = frame_id;
        if (isSafeRosMsg(pose_msg)) {
            _controller_pose_reference_pub->publish(pose_msg);
        } else {
            LOG_ERROR("{} - PoseStamped msg not safe", __PRETTY_FUNCTION__);
        }
    } else if (msg.mask == CommandMask::VELOCITY) {
        auto twist_msg            = convertToRosMsg<geometry_msgs::msg::TwistStamped>(msg, now);
        twist_msg.header.frame_id = frame_id;
        if (isSafeRosMsg(twist_msg)) {
            _controller_twist_reference_pub->publish(twist_msg);
        } else {
            LOG_ERROR("{} - TwistStamped msg not safe", __PRETTY_FUNCTION__);
        }
    } else if (msg.mask == CommandMask::TRAJECTORY) {
        auto trajectory_msg            = convertToRosMsg<cascade_pid_controller_msgs::msg::TrajCommand>(msg, now);
        trajectory_msg.header.frame_id = frame_id;
        trajectory_msg.acceleration.x  = 0.0;
        if (isSafeRosMsg(trajectory_msg)) {
            _controller_trajectory_reference_pub->publish(trajectory_msg);
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

    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();

    auto it = std::find_if(
            mavros_state_table.cbegin(), mavros_state_table.cend(), [&mode](auto&& p) { return p.second == mode; });
    if (it == mavros_state_table.cend()) {
        LOG_WARN("{} - Unable to match current state with states table", __PRETTY_FUNCTION__);
        return;
    }

    request->custom_mode = it->first;

    auto response = _set_flight_mode_client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(shared_from_this(), response) == rclcpp::FutureReturnCode::SUCCESS) {
        LOG_INFO("{} - Succesfully called mavros set mode service '{}'", __PRETTY_FUNCTION__);
        if (response.get()->mode_sent) {
            LOG_INFO("{} - Succesfully mode send to autopilot '{}'", __PRETTY_FUNCTION__, request->custom_mode);
        } else {
            LOG_ERROR("{} - Failed to send mode to autopilot '{}'", __PRETTY_FUNCTION__, request->custom_mode);
        }
    } else {
        LOG_ERROR("{} - Failed to call mavros set mode service '{}'", __PRETTY_FUNCTION__, request->custom_mode);
    }
}

} // namespace catec
