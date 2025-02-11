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
#include <mavros_msgs/ParamGet.h>

#include <cascade_pid_controller/utils.hpp>

namespace cascade_pid_controller {

CascadePidControllerNode::CascadePidControllerNode() :
        _nh(ros::NodeHandle("~")),
        _cascade_pid(std::make_unique<CascadePidController>()),
        _tf_listener(_tf_buffer)
{
    std::string odometry_topic, pose_reference_topic, twist_reference_topic, trajectory_reference_topic, cmd_topic,
            state_topic;

    _nh.param<std::string>("odometry_topic", odometry_topic, "");
    _nh.param<std::string>("pose_reference_topic", pose_reference_topic, "");
    _nh.param<std::string>("twist_reference_topic", twist_reference_topic, "");
    _nh.param<std::string>("trajectory_reference_topic", trajectory_reference_topic, "");
    _nh.param<std::string>("cmd_topic", cmd_topic, "");
    _nh.param<std::string>("state_topic", state_topic, "");
    _nh.param<double>("loop_freq", _loop_freq, 50.0);

    _odometry_sub = _nh.subscribe(odometry_topic, 1, &CascadePidControllerNode::odometryCallback, this);
    _pose_reference_sub
            = _nh.subscribe(pose_reference_topic, 1, &CascadePidControllerNode::poseReferenceCallback, this);
    _twist_reference_sub
            = _nh.subscribe(twist_reference_topic, 1, &CascadePidControllerNode::twistReferenceCallback, this);

    _trajectory_reference_sub = _nh.subscribe(
            trajectory_reference_topic, 1, &CascadePidControllerNode::trajectoryReferenceCallback, this);

    _state_sub = _nh.subscribe(state_topic, 1, &CascadePidControllerNode::controlManagerStateCallback, this);

    _cmd_pub = _nh.advertise<mavros_msgs::AttitudeTarget>(cmd_topic, 1);

    _control_mode_pub = _nh.advertise<cascade_pid_controller_msgs::State>("control_mode", 1);

    initController();

    _safety_checks_ok = false;

    last_event_time_ = ros::Time::now();

    ROS_INFO("Init Node!");
}

CascadePidControllerNode::~CascadePidControllerNode()
{
    _stop_atomic = true;
    if (_safety_thread.joinable())
        _safety_thread.join();
}

bool CascadePidControllerNode::getParamFromMavros(const std::string& param_id, double& param_value)
{
    ros::ServiceClient get_param_srv = _nh.serviceClient<mavros_msgs::ParamGet>("/mavros/param/get", false);
    if (!get_param_srv.waitForExistence(ros::Duration(30.0))) {
        ROS_WARN("Mavros Service didnt exists. 30s Timeout jumped!");
        return false;
    }

    mavros_msgs::ParamGet param_srv;
    param_srv.request.param_id = param_id;
    if (get_param_srv.call(param_srv)) {
        if (param_srv.response.success) {
            ROS_INFO("Succesfully received param from mavros");

            if (param_srv.response.value.real != 0) {
                param_value = param_srv.response.value.real;
            } else {
                param_value = param_srv.response.value.integer;
            }
        } else {
            ROS_WARN("Fail receiving param from mavros");
            return false;
        }
    } else {
        ROS_ERROR("Error calling get param from Mavros");
        return false;
    }
    return true;
}

void CascadePidControllerNode::initController()
{
    // pos_x PID params
    if (!_nh.hasParam("pos/x/p"))
        _nh.setParam("pos/x/p", 0.5);
    if (!_nh.hasParam("pos/x/i"))
        _nh.setParam("pos/x/i", 0.0);
    if (!_nh.hasParam("pos/x/d"))
        _nh.setParam("pos/x/d", 0.25);
    if (!_nh.hasParam("pos/x/i_clamp_min"))
        _nh.setParam("pos/x/i_clamp_min", -10.0);
    if (!_nh.hasParam("pos/x/i_clamp_max"))
        _nh.setParam("pos/x/i_clamp_max", 10.0);
    if (!_nh.hasParam("pos/x/antiwindup"))
        _nh.setParam("pos/x/antiwindup", false);
    _nh.setParam("pos/x/publish_state", true);

    // pos_y PID params
    if (!_nh.hasParam("pos/y/p"))
        _nh.setParam("pos/y/p", 0.5);
    if (!_nh.hasParam("pos/y/i"))
        _nh.setParam("pos/y/i", 0.0);
    if (!_nh.hasParam("pos/y/d"))
        _nh.setParam("pos/y/d", 0.25);
    if (!_nh.hasParam("pos/y/i_clamp_min"))
        _nh.setParam("pos/y/i_clamp_min", -10.0);
    if (!_nh.hasParam("pos/y/i_clamp_max"))
        _nh.setParam("pos/y/i_clamp_max", 10.0);
    if (!_nh.hasParam("pos/y/antiwindup"))
        _nh.setParam("pos/y/antiwindup", false);
    _nh.setParam("pos/y/publish_state", true);

    // pos_z PID params
    if (!_nh.hasParam("pos/z/p"))
        _nh.setParam("pos/z/p", 0.5);
    if (!_nh.hasParam("pos/z/i"))
        _nh.setParam("pos/z/i", 0.0);
    if (!_nh.hasParam("pos/z/d"))
        _nh.setParam("pos/z/d", 0.25);
    if (!_nh.hasParam("pos/z/i_clamp_min"))
        _nh.setParam("pos/z/i_clamp_min", -10.0);
    if (!_nh.hasParam("pos/z/i_clamp_max"))
        _nh.setParam("pos/z/i_clamp_max", 10.0);
    if (!_nh.hasParam("pos/z/antiwindup"))
        _nh.setParam("pos/z/antiwindup", false);
    _nh.setParam("pos/z/publish_state", true);

    // vel x PID params
    if (!_nh.hasParam("vel/x/p"))
        _nh.setParam("vel/x/p", 0.5);
    if (!_nh.hasParam("vel/x/i"))
        _nh.setParam("vel/x/i", 0.0);
    if (!_nh.hasParam("vel/x/d"))
        _nh.setParam("vel/x/d", 0.25);
    if (!_nh.hasParam("vel/x/i_clamp_min"))
        _nh.setParam("vel/x/i_clamp_min", -10.0);
    if (!_nh.hasParam("vel/x/i_clamp_max"))
        _nh.setParam("vel/x/i_clamp_max", 10.0);
    if (!_nh.hasParam("vel/x/antiwindup"))
        _nh.setParam("vel/x/antiwindup", false);
    _nh.setParam("vel/x/publish_state", true);

    // vel y PID params
    if (!_nh.hasParam("vel/y/p"))
        _nh.setParam("vel/y/p", 0.5);
    if (!_nh.hasParam("vel/y/i"))
        _nh.setParam("vel/y/i", 0.0);
    if (!_nh.hasParam("vel/y/d"))
        _nh.setParam("vel/y/d", 0.25);
    if (!_nh.hasParam("vel/y/i_clamp_min"))
        _nh.setParam("vel/y/i_clamp_min", -10.0);
    if (!_nh.hasParam("vel/y/i_clamp_max"))
        _nh.setParam("vel/y/i_clamp_max", 10.0);
    if (!_nh.hasParam("vel/y/antiwindup"))
        _nh.setParam("vel/y/antiwindup", false);
    _nh.setParam("vel/y/publish_state", true);

    // vel z PID params
    if (!_nh.hasParam("vel/z/p"))
        _nh.setParam("vel/z/p", 0.5);
    if (!_nh.hasParam("vel/z/i"))
        _nh.setParam("vel/z/i", 0.0);
    if (!_nh.hasParam("vel/z/d"))
        _nh.setParam("vel/z/d", 0.25);
    if (!_nh.hasParam("vel/z/i_clamp_min"))
        _nh.setParam("vel/z/i_clamp_min", -10.0);
    if (!_nh.hasParam("vel/z/i_clamp_max"))
        _nh.setParam("vel/z/i_clamp_max", 10.0);
    if (!_nh.hasParam("vel/z/antiwindup"))
        _nh.setParam("vel/z/antiwindup", false);
    _nh.setParam("vel/z/publish_state", true);

    // yaw PID params
    if (!_nh.hasParam("yaw/p"))
        _nh.setParam("yaw/p", 0.5);
    if (!_nh.hasParam("yaw/i"))
        _nh.setParam("yaw/i", 0.0);
    if (!_nh.hasParam("yaw/d"))
        _nh.setParam("yaw/d", 0.0);
    if (!_nh.hasParam("yaw/i_clamp_min"))
        _nh.setParam("yaw/i_clamp_min", -10.0);
    if (!_nh.hasParam("yaw/i_clamp_max"))
        _nh.setParam("yaw/i_clamp_max", 10.0);
    if (!_nh.hasParam("yaw/antiwindup"))
        _nh.setParam("yaw/antiwindup", false);
    _nh.setParam("yaw/publish_state", true);

    _nh.param<double>("max_vel_xy", _max_vel_xy, 5.0);
    _nh.param<double>("max_vel_z", _max_vel_z, 2.0);
    _nh.param<double>("max_yaw_rate", _max_yaw_rate, 30.0);
    _nh.param<double>("max_roll", _max_roll, 35.0);
    _nh.param<double>("max_pitch", _max_pitch, 35.0);

    double wpnav_speed_up_value{2.5};
    while (!getParamFromMavros("WPNAV_SPEED_UP", wpnav_speed_up_value)) {
        ROS_WARN("Error getting param: 'WPNAV_SPEED_UP'");
        ros::Duration(1.0).sleep();
    }
    ROS_INFO_STREAM("Readed param: 'WPNAV_SPEED_UP': '" << wpnav_speed_up_value << "'");

    _WPNAV_SPEED_UP = wpnav_speed_up_value * 0.01; // cm/s to m/s

    double wpnav_speed_dn_value{1.5};
    while (!getParamFromMavros("WPNAV_SPEED_DN", wpnav_speed_dn_value)) {
        ROS_WARN("Error getting param: 'WPNAV_SPEED_DN'");
        ros::Duration(1.0).sleep();
    }
    ROS_INFO_STREAM("Readed param: '_WPNAV_SPEED_DN': '" << wpnav_speed_dn_value << "'");

    _WPNAV_SPEED_DN = wpnav_speed_dn_value * 0.01; // cm/s to m/s

    _cascade_pid->init();

    _safety_thread = std::thread(&CascadePidControllerNode::safetyThread, this, _loop_freq);

    _control_timer = _nh.createTimer(ros::Duration(1.0 / _loop_freq), &CascadePidControllerNode::controlThread, this);
}

void CascadePidControllerNode::controlThread(const ros::TimerEvent& event)
{
    // Return inmediately if not initialized
    if (_safety_checks_ok == false || _control_mode == eControlMode::NONE) {
        return;
    }

    double dt = 1.0f / _loop_freq;

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
        ROS_WARN("_pose_reference.pose.orientation must be a pure Yaw rotation");
    }

    if (_control_mode == eControlMode::POSITION) {
        // Error calculation
        pos_error(0) = _pose_reference.pose.position.x - _odometry.pose.pose.position.x;
        pos_error(1) = _pose_reference.pose.position.y - _odometry.pose.pose.position.y;
        pos_error(2) = _pose_reference.pose.position.z - _odometry.pose.pose.position.z;

        vel_ref = _cascade_pid->updatePosPid(pos_error, ros::Duration(dt));

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

        vel_ref = _cascade_pid->updatePosPid(pos_error, ros::Duration(dt));

        vel_ref(0) += _trajectory_reference.velocity.x;
        vel_ref(1) += _trajectory_reference.velocity.y;
        vel_ref(2) += _trajectory_reference.velocity.z;
    }

    // Yaw control
    double yaw_err = utils::angdiff(utils::wrapAngle(yaw), utils::wrapAngle(yaw_ref));
    yaw_rate       = _cascade_pid->updateYawPid(yaw_err, ros::Duration(dt));

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
    acc_cmd = _cascade_pid->updateVelPid(vel_error, vel_error_dot, ros::Duration(dt)); // TODO: PUBLISHER

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
    mavros_msgs::AttitudeTarget attitude_target;

    // Set orientation
    tf2::Quaternion quaternion;                  // AttitudeTarget expects orientation as a quaternion
    quaternion.setRPY(roll_ref, pitch_ref, 0.0); // Yaw will be ignored
    attitude_target.header.stamp = ros::Time::now();
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
            = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE | mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE;

    // Check for NaN
    if (std::isnan(attitude_target.orientation.x) || std::isnan(attitude_target.orientation.y)
        || std::isnan(attitude_target.orientation.z) || std::isnan(attitude_target.orientation.w)) {
        ROS_ERROR("NaN detected in attitude_target.orientation");
        return;
    }
    if (std::isnan(attitude_target.body_rate.z)) {
        ROS_ERROR("NaN detected in attitude_target.body_rate.z");
        return;
    }
    if (std::isnan(attitude_target.thrust)) {
        ROS_ERROR("NaN detected in attitude_target.thrust");
        return;
    }

    // Publish command
    if (_control_mode != eControlMode::NONE) {
        _cmd_pub.publish(attitude_target);
    }

    // Measure dt
    // ros::Duration duration = event.current_expected - last_event_time_;
    // last_event_time_       = event.current_expected; // Update the last event time
    // Log the duration
    // ROS_INFO("Time between callbacks: %f seconds", duration.toSec());
}

void CascadePidControllerNode::safetyThread(const double& rate)
{
    const auto period = std::chrono::milliseconds(static_cast<int>((1.0f / rate) * 1000.0f));
    auto const start  = std::chrono::system_clock::now();

    eControlMode mode = eControlMode::NONE;

    _safety_checks_ok = false;

    while (!_stop_atomic.load(std::memory_order_relaxed)) {
        ros::Time                          t_now = ros::Time::now();
        cascade_pid_controller_msgs::State _control_mode_msg;
        _control_mode_msg.state = static_cast<uint8_t>(_control_mode);
        _control_mode_pub.publish(_control_mode_msg);

        if ((t_now - _odometry.header.stamp).toSec() > 0.5) {
            ROS_WARN_DELAYED_THROTTLE(1.0, "Odometry too old! Control not executed");
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
            ROS_WARN_DELAYED_THROTTLE(1.0, "Waiting for odometry...");
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

        if ((t_now - _last_control_manager_state_ts).toSec() > 0.5) {
            ROS_WARN_DELAYED_THROTTLE(1.0, "ControlManager is not running");
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
            ROS_WARN_DELAYED_THROTTLE(1.0, "UAV must be in authority");
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

        if (((t_now - _last_pose_ref_ts).toSec() > 0.5 && getControlMode() == eControlMode::POSITION)
            || ((t_now - _last_twist_ref_ts).toSec() > 0.5 && getControlMode() == eControlMode::VELOCITY)
            || ((t_now - _last_trajectory_ref_ts).toSec() > 0.5 && getControlMode() == eControlMode::TRAJECTORY)) {
            ROS_WARN_DELAYED_THROTTLE(1.0, "Reference is not being received, change control to NONE");
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

        if (!_safety_checks_ok) {
            ROS_INFO("All safety checks are met. Controller can be initialized");
            resetController();
            _safety_checks_ok = true;
        }

        // Wait
        auto now        = std::chrono::system_clock::now();
        auto iterations = (now - start) / period;
        auto next_start = start + (iterations + 1) * period;
        std::this_thread::sleep_until(next_start);
    }

    std::cout << "end while loop" << std::endl;
}

void CascadePidControllerNode::resetReferences()
{
    _pose_reference       = geometry_msgs::PoseStamped();
    _twist_reference      = geometry_msgs::TwistStamped();
    _trajectory_reference = cascade_pid_controller_msgs::TrajCommand();
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

bool CascadePidControllerNode::isValidPose(const geometry_msgs::Pose& pose)
{
    return !(std::isnan(pose.position.x) || std::isnan(pose.position.y) || std::isnan(pose.position.z));
}

bool CascadePidControllerNode::isValidTwist(const geometry_msgs::Twist& twist)
{
    return !(std::isnan(twist.linear.x) || std::isnan(twist.linear.y) || std::isnan(twist.linear.z));
}

void CascadePidControllerNode::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
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
        const geometry_msgs::PoseStamped::ConstPtr& msg) // TODO: TRANSFORM ANY FRAME TO ODOM?
{
    _pose_reference   = *msg;
    _last_pose_ref_ts = ros::Time::now();

    if (getControlMode() == eControlMode::NONE && _safety_checks_ok) {
        const auto mode = eControlMode::POSITION;
        changeControlMode(mode);
        printControlModeInfo(_control_mode);
    } else if (getControlMode() != eControlMode::NONE && getControlMode() != eControlMode::POSITION) {
        ROS_WARN_DELAYED_THROTTLE(
                1.0, "Cannot change control mode to POSITION, maybe you are publishing another reference");
    }
}

void CascadePidControllerNode::twistReferenceCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    _twist_reference   = *msg;
    _last_twist_ref_ts = ros::Time::now();
    if (getControlMode() == eControlMode::NONE && _safety_checks_ok) {
        const auto mode = eControlMode::VELOCITY;
        changeControlMode(mode);
        printControlModeInfo(_control_mode);
    } else if (getControlMode() != eControlMode::NONE && getControlMode() != eControlMode::VELOCITY) {
        ROS_WARN_DELAYED_THROTTLE(
                1.0, "Cannot change control mode to VELOCITY, maybe you are publishing another reference");
    }
}

void CascadePidControllerNode::trajectoryReferenceCallback(
        const cascade_pid_controller_msgs::TrajCommand::ConstPtr& msg)
{
    _trajectory_reference   = *msg;
    _last_trajectory_ref_ts = ros::Time::now();
    if (getControlMode() == eControlMode::NONE && _safety_checks_ok) {
        const auto mode = eControlMode::TRAJECTORY;
        changeControlMode(mode);
        printControlModeInfo(_control_mode);
    } else if (getControlMode() != eControlMode::NONE && getControlMode() != eControlMode::TRAJECTORY) {
        ROS_WARN_DELAYED_THROTTLE(
                1.0, "Cannot change control mode to TRAJECTORY, maybe you are publishing another reference");
    }
}

void CascadePidControllerNode::controlManagerStateCallback(const catec_control_manager_msgs::State::ConstPtr& msg)
{
    _uav_authority
            = (msg->name == "TAKING_OFF" || msg->name == "HOVER" || msg->name == "OFFBOARD" || msg->name == "ASSISTED"
               || msg->name == "GOING_TO_WP" || msg->name == "LANDING" || msg->name == "PRELANDING");

    _last_control_manager_state_ts = ros::Time::now();
}

} // namespace cascade_pid_controller
