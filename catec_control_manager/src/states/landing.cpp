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

#include "catec_control_manager/states/landing.h"

#include "catec_control_manager/common/config.h"
#include "catec_control_manager/common/parameters_yaml_parser.h"
#include "catec_control_manager/states/emergency.h"
#include "catec_control_manager/states/flying_manual.h"
#include "catec_control_manager/states/landed_armed.h"

namespace catec {

void Landing::entry()
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    LOG_INFO("{} - Entering in 'LANDING' state", __PRETTY_FUNCTION__);
    updateCurrentState(ControlManagerState::LANDING);

    _landing_params = getParameters()->landing_config;

    const auto current_odometry = getCurrentOdometry();

    _landing_position.position    = current_odometry->position;
    _landing_position.orientation = current_odometry->orientation;

    _thread_dt = 1.0 / getParameters()->state_thread_job_rate;

    _acc_lpf.set_dt(_thread_dt);
    _acc_lpf.set_fcut(ACC_LPF_FCUT);

    bool timeout_check = false;
    _landing_timer     = std::make_unique<TimerAsync>(timeout_check);

    _landing_timer->start();

    _landing_completed.store(false);

    startWorkThread(getParameters()->state_thread_job_rate);
}

void Landing::exit()
{
    LOG_TRACE(__PRETTY_FUNCTION__);
    _landing_timer->stop();
    stopWorkThread();
}

void Landing::react(EmergencyDetectedEvent const&)
{
    LOG_TRACE(__PRETTY_FUNCTION__);
    transit<Emergency>();
}

void Landing::executeThreadJob()
{
    /// \note. Land is checked because we set mode land from inside this state. When pilot transit to flying manual,
    /// Mavros State will be STABILIZE, ATLHOLD or LOITER
    const auto curr_mavros_state = getCurrentMavrosState()->state;
    if (curr_mavros_state != MavrosState::GUIDED_NOGPS && curr_mavros_state != MavrosState::LAND) {
        LOG_INFO("{} - Detected revoke authority", __PRETTY_FUNCTION__);
        transit<FlyingManual>();
    }

    if (*getCurrentMavrosExtendedState() == MavrosExtendedState::LANDED_STATE_ON_GROUND) {
        LOG_INFO("{} - Detected transition from ground to air in manual mode", __PRETTY_FUNCTION__);
        transit<LandedArmed>();
    }

    const double v_landing = _landing_params.v_landing;
    _landing_position.position.z() -= v_landing * _thread_dt;

    auto current_acceleration = getCurrentImu()->linear_acceleration;
    current_acceleration.z() -= earth_g;
    const auto current_acc_lpf = _acc_lpf.update(current_acceleration);

    const auto current_pwm = getCurrentPwm()->channels;
    const auto mean_pwm    = utils::meanValueFromVector(current_pwm);

    /// \note. Check if not accelerating, descent rate is low and pwm is low. If one of these conditions are met, reset
    /// timer
    const auto current_odometry = getCurrentOdometry();
    const bool accel_stationary = current_acc_lpf.norm() <= LAND_DETECTOR_MAX_ACC;
    const bool descent_rate_low = abs(current_odometry->linear_velocity.z()) <= LAND_DETECTOR_MAX_VEL;
    const bool pwm_low          = mean_pwm < (PWM_ARMING_VALUE * 1.05);

    LOG_DEBUG("{} - Accel lpf: {}", __PRETTY_FUNCTION__, _acc_lpf.getState().norm());
    LOG_DEBUG("{} - Mean pwm: {}", __PRETTY_FUNCTION__, mean_pwm);

    if (!accel_stationary || !descent_rate_low || !pwm_low) {
        _landing_timer->restart();
    }

    const auto elapsed     = _landing_timer->elapsed();
    double     elapsed_sec = static_cast<double>(elapsed) / 1000.0;

    CommandMsg cmd;

    LOG_DEBUG("{} - Elapsed: {}", __PRETTY_FUNCTION__, elapsed_sec);

    if (!_landing_completed.load()) {
        if (elapsed_sec > LAND_DETECTOR_TRIGGER_SEC) {
            LOG_INFO("{} - Land detected! Transit to LANDED ARMED", __PRETTY_FUNCTION__);
            setMavrosState(MavrosState::LAND);
            _landing_completed.store(true);
        } else if (elapsed_sec > LAND_DETECTOR_MAYBE_TRIGGER_SEC) {
            LOG_INFO("{} - Maybe land detected!", __PRETTY_FUNCTION__);
            /// \note. Soften control for maybe landed state
            cmd.mask            = CommandMask::TRAJECTORY;
            cmd.position        = current_odometry->position;
            cmd.linear_velocity = Eigen::Vector3d(0.0, 0.0, -v_landing);
            cmd.orientation     = current_odometry->orientation;
            publishControlReference(cmd);
        } else {
            LOG_DEBUG("{} - Landing...", __PRETTY_FUNCTION__);

            CommandMsg cmd;
            cmd.mask            = CommandMask::TRAJECTORY;
            cmd.position        = _landing_position.position;
            cmd.linear_velocity = Eigen::Vector3d(0.0, 0.0, -v_landing);
            cmd.orientation     = _landing_position.orientation;
            publishControlReference(cmd);
        }
    }
}
} // namespace catec