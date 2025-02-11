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

#pragma once

#include "base_state.h"
#include "catec_control_manager/common/planner.h"
#include "catec_control_manager/common/types.h"
#include "catec_control_manager/common/utils.h"
#include "catec_control_manager/timer_async.h"

namespace catec {
class GoingToWp : public BaseState
{
  public:
    void entry() override;
    void exit() override;

    void react(EmergencyDetectedEvent const&) override;
    void react(RequestSetModeHoverEvent const&) override;

    /// \note. It is not necessary to execute the work loop
    void executeThreadJob() override;

  private:
    GoingToWpConfig _going_to_wp_params;
    double          _thread_dt;
    PoseMsg         _waypoint;

    double _current_yaw_cmd;

    std::unique_ptr<CmdPlanner> _planner;
    std::unique_ptr<TimerAsync> _trajectory_timer;
};

} // namespace catec