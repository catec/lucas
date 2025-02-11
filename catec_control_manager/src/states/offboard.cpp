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

#include "catec_control_manager/states/offboard.h"

#include "catec_control_manager/common/parameters_yaml_parser.h"
#include "catec_control_manager/states/emergency.h"
#include "catec_control_manager/states/flying_manual.h"
#include "catec_control_manager/states/hover.h"

namespace catec {
void Offboard::entry()
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    LOG_INFO("{} - Entering in 'Offboard' state", __PRETTY_FUNCTION__);
    updateCurrentState(ControlManagerState::OFFBOARD);

    _hover_flag = false;

    startWorkThread(getParameters()->state_thread_job_rate);
}

void Offboard::exit()
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    stopWorkThread();
    resetGenericReference();
}

void Offboard::react(RequestSetModeHoverEvent const&)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    LOG_WARN("{} - Requested Set mode Hover", __PRETTY_FUNCTION__);

    transit<Hover>();
}

void Offboard::react(EmergencyDetectedEvent const&)
{
    LOG_TRACE(__PRETTY_FUNCTION__);
    transit<Emergency>();
}

void Offboard::executeThreadJob()
{
    if (getCurrentMavrosState()->state != MavrosState::GUIDED_NOGPS) {
        LOG_INFO("{} - Detected revoke authority", __PRETTY_FUNCTION__);
        transit<FlyingManual>();
    }

    auto current_odometry               = getCurrentOdometry();
    auto current_offboard_reference     = getCurrentGenericReference();
    auto current_reference_elapsed_time = getReferenceElapsedTime();

    bool timeout = current_reference_elapsed_time >= (getParameters()->generic_reference_checker_timer_interval) * 0.95;

    if (!current_offboard_reference || timeout
        || (current_offboard_reference && current_offboard_reference->mask == CommandMask::NONE)) {
        LOG_WARN_ONCE(
                "{} - Publish a proper offboard reference! Doing hover inside OFFBOARD state.", __PRETTY_FUNCTION__);
        if (!_hover_flag) // If not doing hover yet
        {
            _hover_cmd.mask            = CommandMask::POSITION;
            _hover_cmd.position        = current_odometry->position;
            _hover_cmd.orientation     = current_odometry->orientation;
            _hover_cmd.orientation.x() = 0.0;
            _hover_cmd.orientation.y() = 0.0;
            _hover_flag                = true;
        }
    } else {
        _hover_flag = false;
    }

    if (!_hover_flag) {
        LOG_WARN_ONCE("{} - Publishing offboard reference!", __PRETTY_FUNCTION__);

        publishControlReference(*current_offboard_reference);
    } else {
        publishControlReference(_hover_cmd);
    }
}

} // namespace catec