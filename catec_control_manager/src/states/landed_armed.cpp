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

#include "catec_control_manager/states/landed_armed.h"

#include "catec_control_manager/common/parameters_yaml_parser.h"
#include "catec_control_manager/states/flying_manual.h"
#include "catec_control_manager/states/landed_auto.h"
#include "catec_control_manager/states/landed_disarmed.h"

namespace catec {

void LandedArmed::entry()
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    LOG_INFO("{} - Entering in 'LANDED_ARMED' state", __PRETTY_FUNCTION__);
    updateCurrentState(ControlManagerState::LANDED_ARMED);

    startWorkThread(getParameters()->state_thread_job_rate);
}

void LandedArmed::exit()
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    stopWorkThread();
}

void LandedArmed::react(RequestAuthorityEvent const&)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    LOG_INFO("{} - Received request authority", __PRETTY_FUNCTION__);

    transit<LandedAuto>();
}

void LandedArmed::executeThreadJob()
{
    if (!getCurrentMavrosState()->armed) {
        LOG_INFO("{} - Detected transition from armed to disarmed in manual mode", __PRETTY_FUNCTION__);
        transit<LandedDisarmed>();
    }

    if (*getCurrentMavrosExtendedState() == MavrosExtendedState::LANDED_STATE_IN_AIR) {
        LOG_INFO("{} - Detected transition from ground to air in manual mode", __PRETTY_FUNCTION__);
        transit<FlyingManual>();
    }
}

} // namespace catec