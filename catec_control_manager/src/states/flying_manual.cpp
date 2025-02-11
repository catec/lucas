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

#include "catec_control_manager/states/flying_manual.h"

#include "catec_control_manager/common/parameters_yaml_parser.h"
#include "catec_control_manager/states/hover.h"
#include "catec_control_manager/states/landed_armed.h"
#include "catec_control_manager/states/landed_disarmed.h"

namespace catec {
void FlyingManual::entry()
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    LOG_INFO("{} - Entering in 'FLYING_MANUAL' state", __PRETTY_FUNCTION__);
    updateCurrentState(ControlManagerState::FLYING_MANUAL);

    startWorkThread(getParameters()->state_thread_job_rate);
}

void FlyingManual::exit()
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    stopWorkThread();
}

void FlyingManual::react(RequestAuthorityEvent const&)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    LOG_INFO("{} - Received request authority", __PRETTY_FUNCTION__);
    transit<Hover>();
}

void FlyingManual::executeThreadJob()
{
    if (!getCurrentMavrosState()->armed) {
        LOG_INFO("{} - Detected transition from flying to disarmed in manual mode", __PRETTY_FUNCTION__);
        transit<LandedDisarmed>();
    }

    if (*getCurrentMavrosExtendedState() == MavrosExtendedState::LANDED_STATE_ON_GROUND) {
        LOG_INFO("{} - Detected landing throw mavros extended state", __PRETTY_FUNCTION__);
        transit<LandedArmed>();
    }
}

} // namespace catec