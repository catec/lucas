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

#include "catec_control_manager/states/taking_off.h"

#include "catec_control_manager/common/parameters_yaml_parser.h"
#include "catec_control_manager/states/emergency.h"
#include "catec_control_manager/states/flying_manual.h"
#include "catec_control_manager/states/hover.h"
#include "catec_control_manager/states/landed_auto.h"

namespace catec {

void TakingOff::entry()
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    LOG_INFO("{} - Entering in 'TAKING_OFF' state", __PRETTY_FUNCTION__);
    updateCurrentState(ControlManagerState::TAKING_OFF);

    _takingoff_params = getParameters()->takingoff_config;

    const auto current_reference = getCurrentGenericReference();

    if (!current_reference) {
        LOG_WARN(
                "{} - Nullptr generic reference. Publish a proper position reference! Transit to hover...",
                __PRETTY_FUNCTION__);
        transit<LandedAuto>();
    }

    if (current_reference->mask != CommandMask::POSITION) {
        LOG_WARN("{} - Generic reference is not a position reference! Transit to hover...", __PRETTY_FUNCTION__);
        transit<LandedAuto>();
    }

    const auto current_odometry = getCurrentOdometry();

    auto takingoff_reference = current_odometry->position;
    takingoff_reference.z()  = current_reference->position.z();

    const auto q = current_odometry->orientation;
    _yaw_cmd     = utils::ToEulerAngles(q.w(), q.x(), q.y(), q.z())[2];

    /// \note. Initialize time parametrized cmd planner for soft references
    _planner = std::make_unique<CmdPlanner>(
            current_odometry->position,
            takingoff_reference,
            _takingoff_params.v_max,
            _takingoff_params.acc,
            _takingoff_params.deacc);

    bool timeout_check = false;
    _trajectory_timer  = std::make_unique<TimerAsync>(timeout_check);

    _thread_dt = 1.0 / getParameters()->state_thread_job_rate;

    LOG_INFO("{} - Taking off to {} meters...", __PRETTY_FUNCTION__, takingoff_reference.z());

    _trajectory_timer->start();

    startWorkThread(getParameters()->state_thread_job_rate);
}

void TakingOff::exit()
{
    LOG_TRACE(__PRETTY_FUNCTION__);
    _trajectory_timer->stop();
    stopWorkThread();
    resetGenericReference();
}

void TakingOff::react(EmergencyDetectedEvent const&)
{
    LOG_TRACE(__PRETTY_FUNCTION__);
    transit<Emergency>();
}

void TakingOff::executeThreadJob()
{
    if (getCurrentMavrosState()->state != MavrosState::GUIDED_NOGPS) {
        LOG_INFO("{} - Detected revoke authority", __PRETTY_FUNCTION__);
        transit<FlyingManual>();
    }

    const double height_error_th = _takingoff_params.height_error_th;

    const auto elapsed     = _trajectory_timer->elapsed();
    double     elapsed_sec = static_cast<double>(elapsed) / 1000.0;

    const auto current_position_cmd = _planner->computeCmdPos(elapsed_sec);
    const auto current_velocity_cmd = _planner->computeCmdVel(elapsed_sec);

    const auto current_odometry = getCurrentOdometry();
    if (elapsed_sec > _planner->getTotalTime()) {
        /// \note. Compute error position and yaw
        const double height_error = abs(current_position_cmd.z() - current_odometry->position.z());

        LOG_INFO(
                "{} - Waiting error converge to transit hover. Height error: {:03.2f}",
                __PRETTY_FUNCTION__,
                height_error);

        if (height_error < height_error_th) {
            transit<Hover>();
        }
    }

    CommandMsg cmd;
    cmd.mask            = CommandMask::TRAJECTORY;
    cmd.position        = current_position_cmd;
    cmd.linear_velocity = current_velocity_cmd;
    cmd.orientation     = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
                    * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(_yaw_cmd, Eigen::Vector3d::UnitZ());
    // cmd.angular_velocity.z() = yaw_dot_max;

    publishControlReference(cmd);
}

} // namespace catec