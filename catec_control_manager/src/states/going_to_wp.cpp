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

#include "catec_control_manager/states/going_to_wp.h"

#include "catec_control_manager/common/config.h"
#include "catec_control_manager/common/parameters_yaml_parser.h"
#include "catec_control_manager/common/planner.h"
#include "catec_control_manager/common/types.h"
#include "catec_control_manager/common/utils.h"
#include "catec_control_manager/states/emergency.h"
#include "catec_control_manager/states/flying_manual.h"
#include "catec_control_manager/states/hover.h"
#include "catec_control_manager/timer_async.h"

namespace catec {
void GoingToWp::entry()
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    LOG_INFO("{} - Entering in 'GOING_TO_WP' state", __PRETTY_FUNCTION__);
    updateCurrentState(ControlManagerState::GOING_TO_WP);

    _going_to_wp_params = getParameters()->going_to_wp_config;

    const auto current_reference = getCurrentGenericReference();
    if (!current_reference) {
        LOG_WARN(
                "{} - Nullptr generic reference. Publish a proper position reference! Transit to hover...",
                __PRETTY_FUNCTION__);
        transit<Hover>();
    }

    if (current_reference->mask != CommandMask::POSITION) {
        LOG_WARN("{} - Generic reference is not a position reference! Transit to hover...", __PRETTY_FUNCTION__);
        transit<Hover>();
    }

    _waypoint.position    = current_reference->position;
    _waypoint.orientation = current_reference->orientation;

    /// \note. Assign value of current odometry to cmd members in order to start soft reference algorithm in the exec
    /// thread
    const auto current_odometry = getCurrentOdometry();

    const auto q     = current_odometry->orientation;
    _current_yaw_cmd = utils::ToEulerAngles(q.w(), q.x(), q.y(), q.z())[2];

    /// \note. Initialize time parametrized cmd planner for soft references
    _planner = std::make_unique<CmdPlanner>(
            current_odometry->position,
            current_reference->position,
            _going_to_wp_params.v_max,
            _going_to_wp_params.a_max,
            _going_to_wp_params.a_max);

    bool timeout_check = false;
    _trajectory_timer  = std::make_unique<TimerAsync>(timeout_check);

    _thread_dt = 1.0 / getParameters()->state_thread_job_rate;

    const auto q_ref       = current_odometry->orientation;
    const auto yaw_ref_rad = utils::ToEulerAngles(q_ref.w(), q_ref.x(), q_ref.y(), q_ref.z())[2];
    LOG_INFO(
            "{} - Current position [x: {:03.2f}, y: {:03.2f}, z: {:03.2f}, yaw: {:03.2f}]. Desired waypoint: "
            "[x: {:03.2f}, y: {:03.2f}, z: {:03.2f}, yaw: {:03.2f}]",
            __PRETTY_FUNCTION__,
            current_odometry->position.x(),
            current_odometry->position.y(),
            current_odometry->position.z(),
            _current_yaw_cmd * (180.0 / M_PI),
            _waypoint.position.x(),
            _waypoint.position.y(),
            _waypoint.position.z(),
            yaw_ref_rad * (180.0 / M_PI));

    _trajectory_timer->start();

    startWorkThread(getParameters()->state_thread_job_rate);
}

void GoingToWp::exit()
{
    LOG_TRACE(__PRETTY_FUNCTION__);
    _trajectory_timer->stop();
    stopWorkThread();
    resetGenericReference();
}

void GoingToWp::react(EmergencyDetectedEvent const&)
{
    LOG_TRACE(__PRETTY_FUNCTION__);
    transit<Emergency>();
}

void GoingToWp::react(RequestSetModeHoverEvent const&)
{
    LOG_TRACE(__PRETTY_FUNCTION__);
    transit<Hover>();
}

void GoingToWp::executeThreadJob()
{
    if (getCurrentMavrosState()->state != MavrosState::GUIDED_NOGPS) {
        LOG_INFO("{} - Detected revoke authority", __PRETTY_FUNCTION__);
        transit<FlyingManual>();
    }

    const double position_error_th = _going_to_wp_params.position_error_th;
    const double yaw_error_th      = _going_to_wp_params.yaw_error_th;
    double       yaw_dot_max       = _going_to_wp_params.yaw_dot_max;

    const auto current_odometry = getCurrentOdometry();

    /// \note. Transform quaternion to euler
    const auto   q_odom  = current_odometry->orientation;
    const auto   q_ref   = _waypoint.orientation;
    const double yaw_ref = utils::ToEulerAngles(q_ref.w(), q_ref.x(), q_ref.y(), q_ref.z())[2];
    const double yaw     = utils::ToEulerAngles(q_odom.w(), q_odom.x(), q_odom.y(), q_odom.z())[2];

    /// \note. Compute error cmd yaw
    double error_cmd_2_yaw = utils::angdiff(yaw_ref, _current_yaw_cmd);
    yaw_dot_max *= (error_cmd_2_yaw > 0) ? -1 : 1;
    double dist_cmd_2_yaw = abs(error_cmd_2_yaw);

    const auto elapsed     = _trajectory_timer->elapsed();
    double     elapsed_sec = static_cast<double>(elapsed) / 1000.0;

    const auto current_position_cmd = _planner->computeCmdPos(elapsed_sec);
    const auto current_velocity_cmd = _planner->computeCmdVel(elapsed_sec);

    if (elapsed_sec > _planner->getTotalTime()) {
        /// \note. Compute error position and yaw
        const double position_error = (current_position_cmd - current_odometry->position).norm();
        const double yaw_error      = utils::angdiff(yaw_ref, yaw);

        LOG_INFO(
                "{} - Waiting error converge to transit hover. Position error: {:03.2f} | Yaw error: {:03.2f}",
                __PRETTY_FUNCTION__,
                position_error,
                yaw_error);

        if (position_error < position_error_th && abs(yaw_error) < yaw_error_th) {
            transit<Hover>();
        }
    }

    /// \note. Yaw controller
    if (dist_cmd_2_yaw > MIN_YAW_DIST_TO_CONST_REF) {
        _current_yaw_cmd = utils::wrapAngle(utils::computeNextYaw(_thread_dt, _current_yaw_cmd, yaw_dot_max));
    }

    CommandMsg cmd;
    cmd.mask            = CommandMask::TRAJECTORY;
    cmd.position        = current_position_cmd;
    cmd.linear_velocity = current_velocity_cmd;
    cmd.orientation     = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
                    * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(_current_yaw_cmd, Eigen::Vector3d::UnitZ());
    // cmd.angular_velocity.z() = yaw_dot_max;

    publishControlReference(cmd);
}

} // namespace catec
