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

#include "common/types.h"
#include "tinyfsm.h"

struct RequestAuthorityEvent : tinyfsm::Event
{
};

struct RequestTakeOffEvent : tinyfsm::Event
{
};

struct RequestLandEvent : tinyfsm::Event
{
};

struct RequestSetModeHoverEvent : tinyfsm::Event
{
};

struct RequestSetModeOffboardEvent : tinyfsm::Event
{
};

struct RequestSetModeAssistedEvent : tinyfsm::Event
{
};

struct EmergencyDetectedEvent : tinyfsm::Event
{
};
struct RequestGoToWpEvent : tinyfsm::Event
{
};