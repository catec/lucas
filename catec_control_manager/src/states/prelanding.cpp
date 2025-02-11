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

#include "catec_control_manager/states/prelanding.h"

#include "catec_control_manager/common/parameters_yaml_parser.h"
#include "catec_control_manager/states/emergency.h"
#include "catec_control_manager/states/flying_manual.h"
#include "catec_control_manager/states/hover.h"
#include "catec_control_manager/states/landing.h"

namespace catec {

void Prelanding::entry()
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    LOG_INFO("{} - Entering in 'PRELANDING' state", __PRETTY_FUNCTION__);
    updateCurrentState(ControlManagerState::PRELANDING);

    _landing_params     = getParameters()->landing_config;
    _going_to_wp_params = getParameters()->going_to_wp_config;

    const auto current_odometry = getCurrentOdometry();

    _prelanding_reference.position.x() = current_odometry->position.x();
    _prelanding_reference.position.y() = current_odometry->position.y();
    _prelanding_reference.position.z() = _landing_params.height_prelanding;

    _prelanding_reference.orientation = current_odometry->orientation;

    /// \note. Initialize time parametrized cmd planner for soft references
    _planner = std::make_unique<CmdPlanner>(
            current_odometry->position,
            _prelanding_reference.position,
            _going_to_wp_params.v_max,
            _going_to_wp_params.a_max,
            _going_to_wp_params.a_max);

    bool timeout_check = false;
    _trajectory_timer  = std::make_unique<TimerAsync>(timeout_check);

    _thread_dt = 1.0 / getParameters()->state_thread_job_rate;

    LOG_INFO("{} - Prelanding to {} m", __PRETTY_FUNCTION__, _prelanding_reference.position.z());

    _trajectory_timer->start();

    startWorkThread(getParameters()->state_thread_job_rate);
}

void Prelanding::exit()
{
    LOG_TRACE(__PRETTY_FUNCTION__);
    _trajectory_timer->stop();
    stopWorkThread();
}

void Prelanding::react(EmergencyDetectedEvent const&)
{
    LOG_TRACE(__PRETTY_FUNCTION__);
    transit<Emergency>();
}

void Prelanding::react(RequestSetModeHoverEvent const&)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    LOG_INFO("{} - Cancelled prelanding. Setting mode 'HOVER'");
    transit<Hover>();
}

void Prelanding::executeThreadJob()
{
    if (getCurrentMavrosState()->state != MavrosState::GUIDED_NOGPS) {
        LOG_INFO("{} - Detected revoke authority", __PRETTY_FUNCTION__);
        transit<FlyingManual>();
    }

    const double height_error_th = _landing_params.height_error_th_prelanding;

    const auto current_odometry = getCurrentOdometry();

    const auto elapsed     = _trajectory_timer->elapsed();
    double     elapsed_sec = static_cast<double>(elapsed) / 1000.0;

    LOG_DEBUG("{} - Time elapsed: {}", __PRETTY_FUNCTION__, elapsed_sec);

    const auto current_position_cmd = _planner->computeCmdPos(elapsed_sec);
    const auto current_velocity_cmd = _planner->computeCmdVel(elapsed_sec);

    if (elapsed_sec > _planner->getTotalTime()) {
        /// \note. Compute error position and yaw
        const double height_error = abs(current_position_cmd.z() - current_odometry->position.z());

        LOG_INFO(
                "{} - Waiting error converge to transit landing. Height error: {:03.2f}",
                __PRETTY_FUNCTION__,
                height_error);

        if (height_error < height_error_th) {
            transit<Landing>();
        }
    }

    CommandMsg cmd;
    cmd.mask            = CommandMask::TRAJECTORY;
    cmd.position        = current_position_cmd;
    cmd.linear_velocity = current_velocity_cmd;
    cmd.orientation     = _prelanding_reference.orientation;
    // cmd.angular_velocity.z() = yaw_dot_max;

    publishControlReference(cmd);
}

} // namespace catec