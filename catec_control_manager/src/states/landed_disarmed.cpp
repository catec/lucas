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

#include "catec_control_manager/states/landed_disarmed.h"

#include "catec_control_manager/common/parameters_yaml_parser.h"
#include "catec_control_manager/states/landed_armed.h"

namespace catec {
void LandedDisarmed::entry()
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    LOG_INFO("{} - Entering in 'LANDED_DISARMED' state", __PRETTY_FUNCTION__);
    updateCurrentState(ControlManagerState::LANDED_DISARMED);

    startWorkThread(getParameters()->state_thread_job_rate);
}

void LandedDisarmed::exit()
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    stopWorkThread();
}

void LandedDisarmed::executeThreadJob()
{
    if (getCurrentMavrosState()->armed) {
        LOG_INFO("{} - Detected transition from disarmed to armed in manual mode", __PRETTY_FUNCTION__);
        transit<LandedArmed>();
    }
}

} // namespace catec