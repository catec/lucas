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

#include "catec_control_manager/states/assisted.h"

#include "catec_control_manager/common/config.h"
#include "catec_control_manager/common/parameters_yaml_parser.h"
#include "catec_control_manager/common/utils.h"
#include "catec_control_manager/states/emergency.h"
#include "catec_control_manager/states/flying_manual.h"
#include "catec_control_manager/states/hover.h"

namespace catec {
void Assisted::entry()
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    LOG_INFO("{} - Entering in 'ASSISTED' state", __PRETTY_FUNCTION__);
    updateCurrentState(ControlManagerState::ASSISTED);

    _assisted_params = getParameters()->assisted_config;

    bool timeout_check = false;
    _zero_vel_timer    = std::make_unique<TimerAsync>(timeout_check);

    _zero_vel_timer->start();

    startWorkThread(getParameters()->state_thread_job_rate);
}

void Assisted::react(EmergencyDetectedEvent const&)
{
    LOG_TRACE(__PRETTY_FUNCTION__);
    transit<Emergency>();
}

void Assisted::exit()
{
    LOG_TRACE(__PRETTY_FUNCTION__);
    _zero_vel_timer->stop();
    stopWorkThread();
    resetGenericReference();
}

void Assisted::react(RequestSetModeHoverEvent const&)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    LOG_WARN("{} - Requested Set mode Hover", __PRETTY_FUNCTION__);

    transit<Hover>();
}

void Assisted::executeThreadJob()
{
    if (getCurrentMavrosState()->state != MavrosState::GUIDED_NOGPS) {
        LOG_INFO("{} - Detected revoke authority", __PRETTY_FUNCTION__);
        transit<FlyingManual>();
    }

    const auto current_twist_reference        = getCurrentGenericReference();
    const auto current_reference_elapsed_time = getReferenceElapsedTime();

    bool timeout
            = (current_reference_elapsed_time >= (getParameters()->generic_reference_checker_timer_interval) * 0.95);

    if ((static_cast<double>(_zero_vel_timer->elapsed()) / 1000.0) > ASSISTED_ZERO_VEL_MAX_TIME_SEC) {
        LOG_WARN("{} - Not receiving assisted reference! Transit to hover...", __PRETTY_FUNCTION__);
        transit<Hover>();
    }

    CommandMsg cmd;
    cmd.mask = CommandMask::VELOCITY;

    if (!current_twist_reference || timeout
        || (current_twist_reference && current_twist_reference->mask != CommandMask::VELOCITY)) {
        LOG_WARN(
                "{} - Publish a proper twist reference! Command zero velocity inside ASSISTED state.",
                __PRETTY_FUNCTION__);

        cmd.linear_velocity      = Eigen::Vector3d(0.0, 0.0, 0.0);
        cmd.angular_velocity.z() = 0.0;
    } else {
        const auto current_odometry = getCurrentOdometry();

        /// \note. Rotate twist reference to body frame
        cmd.linear_velocity = current_odometry->orientation.normalized() * current_twist_reference->linear_velocity;
        cmd.linear_velocity = utils::saturateVelocity(cmd.linear_velocity, _assisted_params.v_max);

        cmd.mask = CommandMask::VELOCITY;
        cmd.angular_velocity.z()
                = utils::saturateVelocity(current_twist_reference->angular_velocity.z(), _assisted_params.yaw_dot_max);

        _zero_vel_timer->restart();
    }

    publishControlReference(cmd);
}
} // namespace catec