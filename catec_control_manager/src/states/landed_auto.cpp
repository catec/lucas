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

#include "catec_control_manager/states/landed_auto.h"

#include "catec_control_manager/common/parameters_yaml_parser.h"
#include "catec_control_manager/states/emergency.h"
#include "catec_control_manager/states/landed_armed.h"
#include "catec_control_manager/states/taking_off.h"

namespace catec {

void LandedAuto::entry()
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    LOG_INFO("{} - Entering in 'LANDED_AUTO' state", __PRETTY_FUNCTION__);
    updateCurrentState(ControlManagerState::LANDED_AUTO);

    startWorkThread(getParameters()->state_thread_job_rate);
}

void LandedAuto::exit()
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    stopWorkThread();
}

void LandedAuto::react(RequestTakeOffEvent const&)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    transit<TakingOff>();
}

void LandedAuto::react(EmergencyDetectedEvent const&)
{
    LOG_TRACE(__PRETTY_FUNCTION__);
    transit<Emergency>();
}

void LandedAuto::executeThreadJob()
{
    bool is_authority_revoked = getCurrentMavrosState()->state != MavrosState::GUIDED_NOGPS
                             && _prev_mavros_state == MavrosState::GUIDED_NOGPS;
    if (is_authority_revoked) {
        LOG_INFO("{} - Detected revoke authority", __PRETTY_FUNCTION__);
        transit<LandedArmed>();
    }

    _prev_mavros_state = getCurrentMavrosState()->state;
}

} // namespace catec