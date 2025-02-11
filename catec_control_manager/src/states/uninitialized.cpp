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

#include "catec_control_manager/states/uninitialized.h"

#include "catec_control_manager/common/parameters_yaml_parser.h"
#include "catec_control_manager/states/landed_disarmed.h"

namespace catec {
void Uninitialized::entry()
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    LOG_INFO("{} - Entering in 'UNINITIALIZED' state", __PRETTY_FUNCTION__);
    updateCurrentState(ControlManagerState::UNINITIALIZED);

    startWorkThread(getParameters()->state_thread_job_rate);
}

void Uninitialized::exit()
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    stopWorkThread();
}

void Uninitialized::executeThreadJob()
{
    /// \note. Do not initialize the state machine until the first mavros state message has been received
    if (getCurrentMavrosState()->state != MavrosState::NONE
        && *getCurrentMavrosExtendedState() != MavrosExtendedState::LANDED_STATE_UNDEFINED) {
        transit<LandedDisarmed>();
    }
}

} // namespace catec