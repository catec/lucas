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

#include "catec_control_manager/states/emergency.h"

#include "catec_control_manager/common/parameters_yaml_parser.h"
#include "catec_control_manager/states/flying_manual.h"
#include "catec_control_manager/states/landed_armed.h"

namespace catec {
void Emergency::entry()
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    LOG_INFO("{} - Entering in 'EMERGENCY' state", __PRETTY_FUNCTION__);
    updateCurrentState(ControlManagerState::EMERGENCY);

    startWorkThread(getParameters()->state_thread_job_rate);
}

void Emergency::exit()
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    stopWorkThread();
}

void Emergency::executeThreadJob()
{
    if (getCurrentMavrosState()->state == MavrosState::GUIDED_NOGPS) {
        return;
    }

    LOG_INFO("{} - Detected revoke authority", __PRETTY_FUNCTION__);
    const auto curr_mavros_extended_state = *getCurrentMavrosExtendedState();
    if (curr_mavros_extended_state == MavrosExtendedState::LANDED_STATE_IN_AIR) {
        transit<FlyingManual>();
    } else if (curr_mavros_extended_state == MavrosExtendedState::LANDED_STATE_ON_GROUND) {
        transit<LandedArmed>();
    } else {
        LOG_WARN(
                "{} - In order to leave the EMERGENCY state, mavros must be in a state other than 'GUIDED_NOGPS' and "
                "the extended state must be 'LANDED_STATE_IN_AIR' or 'LANDED_STATE_ON_GROUND'");
    }
}

} // namespace catec