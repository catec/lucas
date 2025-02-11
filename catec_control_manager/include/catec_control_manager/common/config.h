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

/// @brief Minimum yaw distance, in rad, to command a constant reference in 'GOING_TO_WAYPOINT' state
// (This distance is measure between commanded and the waypoint itself)
constexpr double MIN_YAW_DIST_TO_CONST_REF{0.1};

/// @brief Minimum velocity, im m/s, to command hover in position and zero velocity, used in 'GOING_TO_WAYPOINT' and
/// 'HOVER' states
constexpr double MIN_VEL_TO_HOVER{0.2};

/// @brief Maximum vertical velocity, im m/s, to detect land
constexpr double LAND_DETECTOR_MAX_VEL{0.1};

/// @brief Maximum acceleration, im m/s, to detect land
constexpr double LAND_DETECTOR_MAX_ACC{1.0};

/// @brief If land conditions are true during this time, deactivate horizontal control
constexpr double LAND_DETECTOR_MAYBE_TRIGGER_SEC{0.25};

/// @brief If land conditions are true during this time, change MODE LAND
constexpr double LAND_DETECTOR_TRIGGER_SEC{0.50};

/// @brief Accelerometer low pass filter frequency cutoff
constexpr double ACC_LPF_FCUT{5.0};

/// @brief PWM arming value (for landing detector)
constexpr double PWM_ARMING_VALUE{1250};

/// @brief Earth gravity
constexpr double earth_g{9.81};

/// @brief Max time commanding zero velocity vector in assisted when correct reference is not being received
constexpr double ASSISTED_ZERO_VEL_MAX_TIME_SEC{1.0};