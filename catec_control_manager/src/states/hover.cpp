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

#include "catec_control_manager/states/hover.h"

#include "catec_control_manager/common/config.h"
#include "catec_control_manager/common/parameters_yaml_parser.h"
#include "catec_control_manager/states/assisted.h"
#include "catec_control_manager/states/emergency.h"
#include "catec_control_manager/states/flying_manual.h"
#include "catec_control_manager/states/going_to_wp.h"
#include "catec_control_manager/states/landing.h"
#include "catec_control_manager/states/offboard.h"
#include "catec_control_manager/states/prelanding.h"

namespace catec {
void Hover::entry()
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    LOG_INFO("{} - Entering in 'HOVER' state", __PRETTY_FUNCTION__);
    updateCurrentState(ControlManagerState::HOVER);

    _flag_hover = false;

    _prev_mavros_state = getCurrentMavrosState()->state;

    startWorkThread(getParameters()->state_thread_job_rate);
}

void Hover::exit()
{
    LOG_TRACE(__PRETTY_FUNCTION__);
    stopWorkThread();
}

void Hover::react(EmergencyDetectedEvent const&)
{
    LOG_TRACE(__PRETTY_FUNCTION__);
    transit<Emergency>();
}

void Hover::react(RequestLandEvent const&)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    LOG_WARN("{} - Requested LAND", __PRETTY_FUNCTION__)

    transit<Prelanding>();
}

void Hover::react(RequestSetModeAssistedEvent const&)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    LOG_INFO("{} - Received request Set mode Assisted", __PRETTY_FUNCTION__);

    transit<Assisted>();
}

void Hover::react(RequestGoToWpEvent const&)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    LOG_INFO("{} - Received request Go To Waypoint", __PRETTY_FUNCTION__);

    transit<GoingToWp>();
}

void Hover::react(RequestSetModeOffboardEvent const&)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    LOG_INFO("{} - Received request Set Mode Offboard", __PRETTY_FUNCTION__);

    transit<Offboard>();
}

void Hover::executeThreadJob()
{
    bool is_authority_revoked = getCurrentMavrosState()->state != MavrosState::GUIDED_NOGPS
                             && _prev_mavros_state == MavrosState::GUIDED_NOGPS;
    if (is_authority_revoked) {
        LOG_INFO("{} - Detected revoke authority", __PRETTY_FUNCTION__);
        transit<FlyingManual>();
    }

    /* NEW HOVER APPROACH */

    auto current_odometry = getCurrentOdometry();

    /// \note. Extract linear velocity components
    const auto vx = current_odometry->linear_velocity.x();
    const auto vy = current_odometry->linear_velocity.y();
    const auto vz = current_odometry->linear_velocity.z();

    /// \note. Compute velocity module
    const auto vmod = std::sqrt(vx * vx + vy * vy + vz * vz);

    /// \note. While velocity module > 'MIN_VEL_TO_HOVER' m/s, command zero velocity
    if (vmod > MIN_VEL_TO_HOVER && !_flag_hover) {
        CommandMsg twist_cmd;
        twist_cmd.mask                 = CommandMask::VELOCITY;
        twist_cmd.linear_velocity.x()  = 0.0;
        twist_cmd.linear_velocity.y()  = 0.0;
        twist_cmd.linear_velocity.z()  = 0.0;
        twist_cmd.angular_velocity.z() = 0.0;

        LOG_DEBUG("{} -Velocity module {}: publish zero twist", __PRETTY_FUNCTION__, vmod);

        publishControlReference(twist_cmd);
    } else if (!_flag_hover && vmod <= MIN_VEL_TO_HOVER) {
        /// \note. Parse reference to cmd message
        _cmd.mask            = CommandMask::POSITION;
        _cmd.position        = current_odometry->position;
        _cmd.orientation     = current_odometry->orientation;
        _cmd.orientation.x() = 0.0;
        _cmd.orientation.y() = 0.0;

        // Now we can start classic hover
        _flag_hover = true;
    } else if (_flag_hover) {
        LOG_DEBUG_ONCE("{} - Publish classic hover", __PRETTY_FUNCTION__);

        const auto yaw_hover = utils::ToEulerAngles(
                _cmd.orientation.w(), _cmd.orientation.x(), _cmd.orientation.y(), _cmd.orientation.z())[2];
        LOG_INFO_ONCE(
                "{} - Request hover in position: [x: {:03.2f}, y: {:03.2f}, z: {:03.2f}, yaw: {:03.2f}]. ",
                __PRETTY_FUNCTION__,
                _cmd.position.x(),
                _cmd.position.y(),
                _cmd.position.z(),
                yaw_hover * (180.0 / M_PI));

        publishControlReference(_cmd);
    }

    _prev_mavros_state = getCurrentMavrosState()->state;
}

} // namespace catec