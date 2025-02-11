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

namespace catec {
class Hover : public BaseState
{
  public:
    void entry() override;
    void exit() override;

    void react(RequestLandEvent const&) override;
    void react(EmergencyDetectedEvent const&) override;
    void react(RequestSetModeAssistedEvent const&) override;
    void react(RequestGoToWpEvent const&) override;
    void react(RequestSetModeOffboardEvent const&) override;

    void executeThreadJob() override;

  private:
    CommandMsg _cmd;
    bool       _flag_hover;

    MavrosState _prev_mavros_state{MavrosState::NONE};
};
} // namespace catec