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

#include <cascade_pid_controller/nodes/CascadePidControllerNode.h>
#include "rcl_interfaces/srv/get_parameters.hpp"  

#include <cascade_pid_controller/utils.hpp>

namespace cascade_pid_controller {

CascadePidControllerNode::CascadePidControllerNode(const rclcpp::NodeOptions& options) :
        Node("cascade_pid_controller_node", options),
        _cascade_pid(std::make_unique<CascadePidController>())
{


    RCLCPP_INFO(this->get_logger(), "Node '%s' created.", this->get_name());

    _last_control_manager_state_ts = rclcpp::Time(0, 0, RCL_ROS_TIME);
    _last_pose_ref_ts = rclcpp::Time(0, 0, RCL_ROS_TIME);
    _last_twist_ref_ts = rclcpp::Time(0, 0, RCL_ROS_TIME);
    _last_odom_ts = rclcpp::Time(0, 0, RCL_ROS_TIME);
    _last_trajectory_ref_ts = rclcpp::Time(0, 0, RCL_ROS_TIME);

    auto odometry_topic = this->declare_parameter<std::string>("odometry_topic", "odometry");
    auto pose_reference_topic = this->declare_parameter<std::string>("pose_reference_topic", "pose_reference");
    auto twist_reference_topic = this->declare_parameter<std::string>("twist_reference_topic", "twist_reference");
    auto trajectory_reference_topic = this->declare_parameter<std::string>("trajectory_reference_topic", "trajectory_reference");
    auto cmd_topic = this->declare_parameter<std::string>("cmd_topic", "cmd");
    auto state_topic = this->declare_parameter<std::string>("state_topic", "state");

    _loop_freq = this->declare_parameter<double>("loop_freq", 50.0);

    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

     _odometry_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            odometry_topic, qos, std::bind(&CascadePidControllerNode::odometryCallback, this, std::placeholders::_1));

    _pose_reference_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_reference_topic, qos, std::bind(&CascadePidControllerNode::poseReferenceCallback, this, std::placeholders::_1));

    _twist_reference_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            twist_reference_topic, qos, std::bind(&CascadePidControllerNode::twistReferenceCallback, this, std::placeholders::_1));

    _trajectory_reference_sub = this->create_subscription<cascade_pid_controller_msgs::msg::TrajCommand>(
            trajectory_reference_topic, qos, std::bind(&CascadePidControllerNode::trajectoryReferenceCallback, this, std::placeholders::_1));

    _state_sub = this->create_subscription<catec_control_manager_msgs::msg::State>(
            state_topic, qos, std::bind(&CascadePidControllerNode::controlManagerStateCallback, this, std::placeholders::_1));

    _control_mode_pub = this->create_publisher<cascade_pid_controller_msgs::msg::State>("control_mode", qos);

    _cmd_pub = this->create_publisher<mavros_msgs::msg::AttitudeTarget>("/mavros/setpoint_raw/attitude", qos);

    initController();

    _safety_checks_ok = false;

    RCLCPP_INFO(this->get_logger(), "Init Node!");
}

CascadePidControllerNode::~CascadePidControllerNode()
{
    _stop_atomic = true;
    if (_safety_thread.joinable())
        _safety_thread.join();
}

bool CascadePidControllerNode::getParamFromMavros(const std::string& param_id, double& param_value)
{
    auto get_param_srv = this->create_client<rcl_interfaces::srv::GetParameters>("/mavros/param/get_parameters");

    if (!get_param_srv->wait_for_service(std::chrono::seconds(10))) {
        RCLCPP_WARN(this->get_logger(), "Mavros Service didn't exist. 10s Timeout jumped!");
        return false;
    }

    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back(param_id);

    auto result = get_param_srv->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = result.get();
        if (!response->values.empty()) {
            const auto& value = response->values[0];

            if (value.type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                RCLCPP_INFO(this->get_logger(), "Parameter value: %f", value.double_value);
                param_value = value.double_value;
            } else if (value.type == rclcpp::ParameterType::PARAMETER_INTEGER) {
                RCLCPP_INFO(this->get_logger(), "Parameter value: %ld", value.integer_value);
            } else {
                RCLCPP_WARN(this->get_logger(), "Parameter found but is not a double or integer.");
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Parameter not found or empty response.");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Error calling get param from Mavros");
        return false;
    }

    return true;
}


void CascadePidControllerNode::initController()
{
    _max_vel_xy = this->declare_parameter<double>("max_vel_xy", 0.5);
    _max_vel_z = this->declare_parameter<double>("max_vel_z", 0.5);
    _max_yaw_rate =  this->declare_parameter<double>("max_yaw_rate", 30.0);
    _max_roll = this->declare_parameter<double>("max_roll", 20.0);
    _max_pitch =  this->declare_parameter<double>("max_pitch",20.0);

    double wpnav_speed_up_value{250};
    while (!getParamFromMavros("WPNAV_SPEED_UP", wpnav_speed_up_value)) {
        RCLCPP_WARN(this->get_logger(), "Error getting param: 'WPNAV_SPEED_UP'");
        rclcpp::sleep_for(std::chrono::seconds(1));
    }
    RCLCPP_INFO(this->get_logger(), "Readed param: '_WPNAV_SPEED_UP': '%f'", wpnav_speed_up_value);

    _WPNAV_SPEED_UP = wpnav_speed_up_value * 0.01; // cm/s to m/s

    double wpnav_speed_dn_value{150};
    while (!getParamFromMavros("WPNAV_SPEED_DN", wpnav_speed_dn_value)) {
        RCLCPP_WARN(this->get_logger(), "Error getting param: 'WPNAV_SPEED_DN'");
        rclcpp::sleep_for(std::chrono::seconds(1));
    }
    RCLCPP_INFO(this->get_logger(), "Readed param: '_WPNAV_SPEED_DN': '%f'", wpnav_speed_dn_value);

    _WPNAV_SPEED_DN = wpnav_speed_dn_value * 0.01; // cm/s to m/s

    _safety_thread = std::thread(&CascadePidControllerNode::safetyThread, this, _loop_freq);

    _control_timer = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / _loop_freq), 
        std::bind(&CascadePidControllerNode::controlThread, this)
    );
}

void CascadePidControllerNode::controlThread()
{
    if (!_pid_initialize_flag) {
        rclcpp::sleep_for(std::chrono::seconds(1));
        _cascade_pid->init(this->shared_from_this()); // This is a workaround for not pass shared ptr in constructor
        _pid_initialize_flag = true;
    }

    // Return inmediately if not initialized
    if (_safety_checks_ok == false || _control_mode == eControlMode::NONE) {
        return;
    }

    double dt = 1.0 / _loop_freq;

    //  ------------------------------ Run control loop ------------------------------------------ //

    Eigen::Vector3d pos_error;
    Eigen::Vector3d vel_ref;
    Eigen::Vector3d vel_error;
    Eigen::Vector3d vel_error_dot;
    double          yaw_rate;

    // Get Yaw from odometry msg
    double          roll, pitch, yaw;
    tf2::Quaternion q_orientation;
    tf2::convert(_odometry.pose.pose.orientation, q_orientation);
    tf2::Matrix3x3 m(q_orientation);
    m.getRPY(roll, pitch, yaw);

    // Get Yaw from _pose_reference
    double          roll_ref, pitch_ref, yaw_ref;
    tf2::Quaternion q_ref;
    tf2::convert(_pose_reference.pose.orientation, q_ref);
    tf2::Matrix3x3 m_ref(q_ref);
    m_ref.getRPY(roll_ref, pitch_ref, yaw_ref);
    if (std::abs(roll_ref) > 0.01 || std::abs(pitch_ref) > 0.01) {
        RCLCPP_WARN(this->get_logger(), "_pose_reference.pose.orientation must be a pure Yaw rotation");
    }

    if (_control_mode == eControlMode::POSITION) {
        // Error calculation
        pos_error(0) = _pose_reference.pose.position.x - _odometry.pose.pose.position.x;
        pos_error(1) = _pose_reference.pose.position.y - _odometry.pose.pose.position.y;
        pos_error(2) = _pose_reference.pose.position.z - _odometry.pose.pose.position.z;

        vel_ref = _cascade_pid->updatePosPid(pos_error, rclcpp::Duration::from_seconds(dt));

    } else if (_control_mode == eControlMode::VELOCITY) {
        vel_ref(0) = _twist_reference.twist.linear.x;
        vel_ref(1) = _twist_reference.twist.linear.y;
        vel_ref(2) = _twist_reference.twist.linear.z;

        yaw_rate = _twist_reference.twist.angular.z;

        // Limit yaw_rate
        yaw_rate = std::min(std::max(yaw_rate, -1 * _max_yaw_rate * M_PI / 180.0), _max_yaw_rate * M_PI / 180.0);

        _yaw_integrator.integrate(yaw_rate, dt);
        yaw_ref = utils::normalizeAngle(_yaw_integrator.getValue());

    } else if (_control_mode == eControlMode::TRAJECTORY) {
        // Error calculation
        pos_error(0) = _trajectory_reference.position.x - _odometry.pose.pose.position.x;
        pos_error(1) = _trajectory_reference.position.y - _odometry.pose.pose.position.y;
        pos_error(2) = _trajectory_reference.position.z - _odometry.pose.pose.position.z;
        yaw_ref      = _trajectory_reference.yaw;

        vel_ref = _cascade_pid->updatePosPid(pos_error, rclcpp::Duration::from_seconds(dt));

        vel_ref(0) += _trajectory_reference.velocity.x;
        vel_ref(1) += _trajectory_reference.velocity.y;
        vel_ref(2) += _trajectory_reference.velocity.z;
    }

    // Yaw control
    double yaw_err = utils::angdiff(utils::wrapAngle(yaw), utils::wrapAngle(yaw_ref));
    yaw_rate       = _cascade_pid->updateYawPid(yaw_err, rclcpp::Duration::from_seconds(dt));

    // Limit yaw_rate
    yaw_rate = std::min(std::max(yaw_rate, -1 * _max_yaw_rate * M_PI / 180.0), _max_yaw_rate * M_PI / 180.0);

    // Saturate Velocity
    double v_xy_norm, v_z;
    v_xy_norm = sqrt(pow(vel_ref(0), 2) + pow(vel_ref(1), 2));
    if (v_xy_norm > _max_vel_xy) {
        double alpha = atan2(vel_ref(1), vel_ref(0));
        vel_ref(0)   = _max_vel_xy * cos(alpha);
        vel_ref(1)   = _max_vel_xy * sin(alpha);
    }
    v_z = std::min(std::max(vel_ref(2), -1 * _max_vel_z), _max_vel_z);

    // Velocity error calculation
    vel_error(0) = vel_ref(0) - _odometry.twist.twist.linear.x;
    vel_error(1) = vel_ref(1) - _odometry.twist.twist.linear.y;
    vel_error(2) = vel_ref(2) - _odometry.twist.twist.linear.z;

    // Compute derivative term using numerical derivative and LP filter
    // take the derivative of the state (not the error) to avoid the "derivative kick"
    vel_error_dot(0) = _cascade_pid->_vel_x_deriv_lpf.update(-1 * _odometry.twist.twist.linear.x, dt);
    vel_error_dot(1) = _cascade_pid->_vel_y_deriv_lpf.update(-1 * _odometry.twist.twist.linear.y, dt);
    vel_error_dot(2) = _cascade_pid->_vel_z_deriv_lpf.update(-1 * _odometry.twist.twist.linear.z, dt);

    Eigen::Vector3d acc_cmd;
    acc_cmd = _cascade_pid->updateVelPid(vel_error, vel_error_dot, rclcpp::Duration::from_seconds(dt)); // TODO: PUBLISHER

    if (_control_mode == eControlMode::TRAJECTORY) {
        acc_cmd(0) += _trajectory_reference.acceleration.x;
        acc_cmd(1) += _trajectory_reference.acceleration.y;
        acc_cmd(2) += _trajectory_reference.acceleration.z;
    }

    // Convert XY acceleration to roll and pitch references
    // Get attitude from odometry msg
    Eigen::Quaterniond q_uav;
    tf2::fromMsg(_odometry.pose.pose.orientation, q_uav);
    _cascade_pid->accelerationToAttitude(acc_cmd, q_uav, roll_ref, pitch_ref);

    // Limit roll and pitch
    roll_ref  = std::min(std::max(roll_ref, -1 * _max_roll * M_PI / 180.0), _max_roll * M_PI / 180.0);
    pitch_ref = std::min(std::max(pitch_ref, -1 * _max_pitch * M_PI / 180.0), _max_pitch * M_PI / 180.0);

    // Map vertical velocity to [0,1]
    double normalized_climb_rate;

    if (v_z >= 0.0) {
        normalized_climb_rate = utils::mapValue(v_z, 0.0, _WPNAV_SPEED_UP, 0.5, 1.0);
    } else {
        normalized_climb_rate = utils::mapValue(v_z, -1.0 * _WPNAV_SPEED_DN, 0.0, 0.0, 0.5);
    }

    // Limit climb rate
    normalized_climb_rate = std::min(std::max(normalized_climb_rate, 0.0), 1.0);

    // Create AttitudeTarget msg
    mavros_msgs::msg::AttitudeTarget attitude_target;

    // Set orientation
    tf2::Quaternion quaternion;                  // AttitudeTarget expects orientation as a quaternion
    quaternion.setRPY(roll_ref, pitch_ref, 0.0); // Yaw will be ignored
    attitude_target.header.stamp = this->get_clock()->now();
    tf2::convert(quaternion, attitude_target.orientation);

    // Set thrust (normalized climb rate)
    attitude_target.thrust = normalized_climb_rate;

    // Set yaw_rate
    attitude_target.body_rate.z = yaw_rate;

    // Set mask
    // TODO: check in AP code if yaw_rate can be accepted  (instead of yaw)
    // "Should always be 0b00000111 / 0x07 / 7 (decimal)"  from:
    // https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
    attitude_target.type_mask
            = mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE || mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE;

    // Check for NaN
    if (std::isnan(attitude_target.orientation.x) || std::isnan(attitude_target.orientation.y)
        || std::isnan(attitude_target.orientation.z) || std::isnan(attitude_target.orientation.w)) {
        RCLCPP_ERROR(this->get_logger(), "NaN detected in attitude_target.orientation");
        return;
    }
    if (std::isnan(attitude_target.body_rate.z)) {
        RCLCPP_ERROR(this->get_logger(), "NaN detected in attitude_target.body_rate.z");
        return;
    }
    if (std::isnan(attitude_target.thrust)) {
        RCLCPP_ERROR(this->get_logger(), "NaN detected in attitude_target.thrust");
        return;
    }

    // Publish command
    if (_control_mode != eControlMode::NONE) {
        _cmd_pub->publish(attitude_target);
    }

    // Measure dt
    // ros::Duration duration = event.current_expected - last_event_time_;
    // last_event_time_       = event.current_expected; // Update the last event time
    // Log the duration
    // ROS_INFO("Time between callbacks: %f seconds", duration.seconds());
}

void CascadePidControllerNode::safetyThread(const double& rate)
{
    const auto period = std::chrono::milliseconds(static_cast<int>((1.0f / rate) * 1000.0f));
    auto const start  = std::chrono::system_clock::now();

    eControlMode mode = eControlMode::NONE;

    _safety_checks_ok = false;

    while (!_stop_atomic.load(std::memory_order_relaxed)) {
        rclcpp::Time t_now = this->get_clock()->now();
        cascade_pid_controller_msgs::msg::State _control_mode_msg;
        _control_mode_msg.state = static_cast<uint8_t>(_control_mode);
        _control_mode_pub->publish(_control_mode_msg);

        rclcpp::Time odom_header_time(_odometry.header.stamp);

        if ((t_now - odom_header_time).seconds() > 0.5) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Odometry too old! Control not executed");
            if (getControlMode() != eControlMode::NONE) {
                mode = eControlMode::NONE;
                changeControlMode(mode);
                // resetReferences();
                _safety_checks_ok = false;
            }
            // Wait
            auto now        = std::chrono::system_clock::now();
            auto iterations = (now - start) / period;
            auto next_start = start + (iterations + 1) * period;
            std::this_thread::sleep_until(next_start);
            continue;
        }

        if (!isValidPose(_odometry.pose.pose)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for odometry...");
            if (getControlMode() != eControlMode::NONE) {
                mode = eControlMode::NONE;
                changeControlMode(mode);
                // resetReferences();
                _safety_checks_ok = false;
            }
            // Wait
            auto now        = std::chrono::system_clock::now();
            auto iterations = (now - start) / period;
            auto next_start = start + (iterations + 1) * period;
            std::this_thread::sleep_until(next_start);
            continue;
        }

        if ((t_now - _last_control_manager_state_ts).seconds() > 0.5) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "ControlManager is not running");
            if (getControlMode() != eControlMode::NONE) {
                mode = eControlMode::NONE;
                changeControlMode(mode);
                // resetReferences();
                _safety_checks_ok = false;
                _uav_authority    = false;
            }
            // Wait
            auto now        = std::chrono::system_clock::now();
            auto iterations = (now - start) / period;
            auto next_start = start + (iterations + 1) * period;
            std::this_thread::sleep_until(next_start);
            continue;
        }

        if (!_uav_authority) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "UAV must be in authority");
            if (getControlMode() != eControlMode::NONE) {
                mode = eControlMode::NONE;
                changeControlMode(mode);
                // resetReferences();
                _safety_checks_ok = false;
            }
            // Wait
            auto now        = std::chrono::system_clock::now();
            auto iterations = (now - start) / period;
            auto next_start = start + (iterations + 1) * period;
            std::this_thread::sleep_until(next_start);
            continue;
        }

        if (((t_now - _last_pose_ref_ts).seconds() > 0.5 && getControlMode() == eControlMode::POSITION)
            || ((t_now - _last_twist_ref_ts).seconds() > 0.5 && getControlMode() == eControlMode::VELOCITY)
            || ((t_now - _last_trajectory_ref_ts).seconds() > 0.5 && getControlMode() == eControlMode::TRAJECTORY)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Reference is not being received, change control to NONE");
            if (getControlMode() != eControlMode::NONE) {
                mode = eControlMode::NONE;
                changeControlMode(mode);
                // resetReferences();
                _safety_checks_ok = false;
            }
            // Wait
            auto now        = std::chrono::system_clock::now();
            auto iterations = (now - start) / period;
            auto next_start = start + (iterations + 1) * period;
            std::this_thread::sleep_until(next_start);
            continue;
        }

        // If we've reached this point, the controller can be initialized

        if (!_safety_checks_ok && _pid_initialize_flag) {
            RCLCPP_INFO(this->get_logger(), "All safety checks are met. Controller can be initialized");
            resetController();
            _safety_checks_ok = true;
        }

        // Wait
        auto now        = std::chrono::system_clock::now();
        auto iterations = (now - start) / period;
        auto next_start = start + (iterations + 1) * period;
        std::this_thread::sleep_until(next_start);
    }
}

void CascadePidControllerNode::resetReferences()
{
    _pose_reference       = geometry_msgs::msg::PoseStamped();
    _twist_reference      = geometry_msgs::msg::TwistStamped();
    _trajectory_reference = cascade_pid_controller_msgs::msg::TrajCommand();
}

void CascadePidControllerNode::resetController()
{
    _cascade_pid->reset();

    double roll_ref, pitch_ref, yaw_ref;

    // Get Yaw from odometry
    tf2::Quaternion q_ref;
    tf2::convert(_odometry.pose.pose.orientation, q_ref);
    tf2::Matrix3x3 m(q_ref);
    m.getRPY(roll_ref, pitch_ref, yaw_ref);

    _yaw_integrator.reset(yaw_ref);

    _pose_reference.pose = _odometry.pose.pose;
}

bool CascadePidControllerNode::isValidPose(const geometry_msgs::msg::Pose& pose)
{
    return !(std::isnan(pose.position.x) || std::isnan(pose.position.y) || std::isnan(pose.position.z));
}

bool CascadePidControllerNode::isValidTwist(const geometry_msgs::msg::Twist& twist)
{
    return !(std::isnan(twist.linear.x) || std::isnan(twist.linear.y) || std::isnan(twist.linear.z));
}

void CascadePidControllerNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Get attitude from odometry msg
    Eigen::Quaterniond q_uav;
    tf2::fromMsg(msg->pose.pose.orientation, q_uav);

    // Transform twist from base_link to odom
    Eigen::Vector3d vel_linear_b, vel_linear_o;
    Eigen::Vector3d vel_angular_b, vel_angular_o;

    tf2::fromMsg(msg->twist.twist.linear, vel_linear_b);
    tf2::fromMsg(msg->twist.twist.angular, vel_angular_b);

    vel_linear_o  = q_uav * vel_linear_b;
    vel_angular_o = q_uav * vel_angular_b;

    _odometry.header         = msg->header;
    _odometry.child_frame_id = "odom";
    _odometry.pose           = msg->pose;

    // Assign the transformed vel to the twist field of the odometry msg
    tf2::toMsg(vel_linear_o, _odometry.twist.twist.linear);
    tf2::toMsg(vel_angular_o, _odometry.twist.twist.angular);
}

void CascadePidControllerNode::changeControlMode(eControlMode mode)
{
    resetController();
    _control_mode = mode;
}

eControlMode CascadePidControllerNode::getControlMode()
{
    return _control_mode;
}

void CascadePidControllerNode::poseReferenceCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg) // TODO: TRANSFORM ANY FRAME TO ODOM?
{
    _pose_reference   = *msg;
    _last_pose_ref_ts = this->get_clock()->now();

    if (getControlMode() == eControlMode::NONE && _safety_checks_ok) {
        const auto mode = eControlMode::POSITION;
        changeControlMode(mode);
        printControlModeInfo(_control_mode);
    } else if (getControlMode() != eControlMode::NONE && getControlMode() != eControlMode::POSITION) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Cannot change control mode to POSITION, maybe you are publishing another reference");
    }
}

void CascadePidControllerNode::twistReferenceCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    _twist_reference   = *msg;
    _last_twist_ref_ts = this->get_clock()->now();
    if (getControlMode() == eControlMode::NONE && _safety_checks_ok) {
        const auto mode = eControlMode::VELOCITY;
        changeControlMode(mode);
        printControlModeInfo(_control_mode);
    } else if (getControlMode() != eControlMode::NONE && getControlMode() != eControlMode::VELOCITY) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Cannot change control mode to VELOCITY, maybe you are publishing another reference");
    }
}

void CascadePidControllerNode::trajectoryReferenceCallback(
        const cascade_pid_controller_msgs::msg::TrajCommand::SharedPtr msg)
{
    _trajectory_reference   = *msg;
    _last_trajectory_ref_ts = this->get_clock()->now();
    if (getControlMode() == eControlMode::NONE && _safety_checks_ok) {
        const auto mode = eControlMode::TRAJECTORY;
        changeControlMode(mode);
        printControlModeInfo(_control_mode);
    } else if (getControlMode() != eControlMode::NONE && getControlMode() != eControlMode::TRAJECTORY) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Cannot change control mode to TRAJECTORY, maybe you are publishing another reference");
    }
}

void CascadePidControllerNode::controlManagerStateCallback(const catec_control_manager_msgs::msg::State::SharedPtr msg)
{
    _uav_authority
            = (msg->name == "TAKING_OFF" || msg->name == "HOVER" || msg->name == "OFFBOARD" || msg->name == "ASSISTED"
               || msg->name == "GOING_TO_WP" || msg->name == "LANDING" || msg->name == "PRELANDING");

    _last_control_manager_state_ts = this->get_clock()->now();
}

void CascadePidControllerNode::printControlModeInfo(eControlMode mode)
{
    switch (mode) {
        case eControlMode::POSITION: RCLCPP_INFO(this->get_logger(), "Position control mode"); break;
        case eControlMode::VELOCITY: RCLCPP_INFO(this->get_logger(), "Velocity control mode"); break;
        case eControlMode::TRAJECTORY: RCLCPP_INFO(this->get_logger(), "Trajectory control mode"); break;
        case eControlMode::NONE: RCLCPP_INFO(this->get_logger(), "No control mode selected"); break;
        default: RCLCPP_INFO(this->get_logger(), "Invalid control mode"); break;
    }
}

} // namespace cascade_pid_controller
