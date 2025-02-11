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
#include "catec_control_manager/common/low_pass_filter.h"
#include "catec_control_manager/common/types.h"
#include "catec_control_manager/common/utils.h"
#include "catec_control_manager/timer_async.h"

namespace catec {
class Landing : public BaseState
{
  public:
    void entry() override;
    void exit() override;

    void react(EmergencyDetectedEvent const&);

    /// \note. It is not necessary to execute the work loop
    void executeThreadJob() override;

  private:
    LandingConfig _landing_params;
    PoseMsg       _landing_position;

    double                      _thread_dt;
    std::unique_ptr<TimerAsync> _landing_timer;

    std::atomic_bool _landing_completed;

    BlockLowPass<Eigen::Vector3d> _acc_lpf;
};

} // namespace catec