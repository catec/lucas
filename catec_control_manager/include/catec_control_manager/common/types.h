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

#include <Eigen/Eigen>
#include <cstdint> // for uint8_t

#include "catec_control_manager/common/log_manager.h"
#include "catec_control_manager/common/utils.h"

namespace catec {

enum class ControlManagerState : uint8_t {
    UNINITIALIZED = 0,
    LANDED_DISARMED,
    LANDED_ARMED,
    LANDED_AUTO,
    TAKING_OFF,
    HOVER,
    OFFBOARD,
    ASSISTED,
    GOING_TO_WP,
    LANDING,
    PRELANDING,
    FLYING_MANUAL,
    EMERGENCY
};

inline const char* ToString(const ControlManagerState v)
{
    switch (v) {
        case ControlManagerState::UNINITIALIZED: {
            return "UNINITIALIZED";
        }
        case ControlManagerState::LANDED_DISARMED: {
            return "LANDED_DISARMED";
        }
        case ControlManagerState::LANDED_ARMED: {
            return "LANDED_ARMED";
        }
        case ControlManagerState::LANDED_AUTO: {
            return "LANDED_AUTO";
        }
        case ControlManagerState::TAKING_OFF: {
            return "TAKING_OFF";
        }
        case ControlManagerState::HOVER: {
            return "HOVER";
        }
        case ControlManagerState::OFFBOARD: {
            return "OFFBOARD";
        }
        case ControlManagerState::ASSISTED: {
            return "ASSISTED";
        }
        case ControlManagerState::GOING_TO_WP: {
            return "GOING_TO_WP";
        }
        case ControlManagerState::LANDING: {
            return "LANDING";
        }
        case ControlManagerState::PRELANDING: {
            return "PRELANDING";
        }
        case ControlManagerState::FLYING_MANUAL: {
            return "FLYING_MANUAL";
        }
        case ControlManagerState::EMERGENCY: {
            return "EMERGENCY";
        }
        default: {
            return "UNKNOWN";
        }
    }
}

enum class MavrosState : uint8_t {
    NONE = 0,
    STABILIZE,
    ACRO,
    ALT_HOLD,
    AUTO,
    GUIDED,
    LOITER,
    RTL,
    CIRCLE,
    POSITION,
    LAND,
    OF_LOITER,
    DRIFT,
    SPORT,
    FLIP,
    AUTOTUNE,
    POSHOLD,
    BRAKE,
    THROW,
    AVOID_ADSB,
    GUIDED_NOGPS
};

static std::unordered_map<std::string, MavrosState> const mavros_state_table
        = {{"NONE", MavrosState::NONE},
           {"STABILIZE", MavrosState::STABILIZE},
           {"ACRO", MavrosState::ACRO},
           {"ALT_HOLD", MavrosState::ALT_HOLD},
           {"AUTO", MavrosState::AUTO},
           {"GUIDED", MavrosState::GUIDED},
           {"LOITER", MavrosState::LOITER},
           {"RTL", MavrosState::RTL},
           {"CIRCLE", MavrosState::CIRCLE},
           {"POSITION", MavrosState::POSITION},
           {"LAND", MavrosState::LAND},
           {"OF_LOITER", MavrosState::OF_LOITER},
           {"DRIFT", MavrosState::DRIFT},
           {"SPORT", MavrosState::SPORT},
           {"FLIP", MavrosState::FLIP},
           {"AUTOTUNE", MavrosState::AUTOTUNE},
           {"POSHOLD", MavrosState::POSHOLD},
           {"BRAKE", MavrosState::BRAKE},
           {"THROW", MavrosState::THROW},
           {"AVOID_ADSB", MavrosState::AVOID_ADSB},
           {"GUIDED_NOGPS", MavrosState::GUIDED_NOGPS}};

struct MavrosStateComplete
{
    MavrosState state;
    bool        armed;
};

enum class MavrosExtendedState : uint8_t {
    LANDED_STATE_UNDEFINED = 0,
    LANDED_STATE_ON_GROUND,
    LANDED_STATE_IN_AIR,
    LANDED_STATE_TAKEOFF,
    LANDED_STATE_LANDING
};

static std::unordered_map<std::string, MavrosExtendedState> const mavros_extended_state_table
        = {{"LANDED_STATE_UNDEFINED", MavrosExtendedState::LANDED_STATE_UNDEFINED},
           {"LANDED_STATE_ON_GROUND", MavrosExtendedState::LANDED_STATE_ON_GROUND},
           {"LANDED_STATE_IN_AIR", MavrosExtendedState::LANDED_STATE_IN_AIR},
           {"LANDED_STATE_TAKEOFF", MavrosExtendedState::LANDED_STATE_TAKEOFF},
           {"LANDED_STATE_LANDING", MavrosExtendedState::LANDED_STATE_LANDING}};

enum class CommandMask : uint8_t {
    POSITION = 0,
    VELOCITY,
    TRAJECTORY,
    NONE,
};

static std::unordered_map<std::string, CommandMask> const command_mask_table
        = {{"NONE", CommandMask::NONE},
           {"POSITION", CommandMask::POSITION},
           {"VELOCITY", CommandMask::VELOCITY},
           {"TRAJECTORY", CommandMask::TRAJECTORY}};

enum class CriticalMsg : uint8_t {
    NONE = 0,
    ODOM,
    IMU,
    PWM,
    RANGEFINDER,
    CONTROLLER
};

static std::unordered_map<std::string, CriticalMsg> const crital_msg_table
        = {{"NONE", CriticalMsg::NONE},
           {"ODOM", CriticalMsg::ODOM},
           {"IMU", CriticalMsg::IMU},
           {"PWM", CriticalMsg::PWM},
           {"RANGEFINDER", CriticalMsg::RANGEFINDER},
           {"CONTROLLER", CriticalMsg::CONTROLLER}};

struct PoseMsg
{
    Eigen::Vector3d    position;
    Eigen::Quaterniond orientation;

    PoseMsg(const Eigen::Vector3d&    pos = Eigen::Vector3d::Zero(),
            const Eigen::Quaterniond& ori = Eigen::Quaterniond::Identity()) :
            position(pos),
            orientation(ori)
    {}
};

struct OdometryMsg
{
    /// \todo. is it worth adding frame_id?

    Eigen::Vector3d    position;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d    angular_velocity;
    Eigen::Vector3d    linear_velocity;

    OdometryMsg(
            const Eigen::Vector3d&    pos     = Eigen::Vector3d::Zero(),
            const Eigen::Quaterniond& ori     = Eigen::Quaterniond::Identity(),
            const Eigen::Vector3d&    ang_vel = Eigen::Vector3d::Zero(),
            const Eigen::Vector3d&    lin_vel = Eigen::Vector3d::Zero()) :
            position(pos),
            orientation(ori),
            angular_velocity(ang_vel),
            linear_velocity(lin_vel)
    {}
};

struct ImuMsg
{
    /// \todo. is it worth adding frame_id?

    Eigen::Quaterniond orientation;
    Eigen::Vector3d    angular_velocity;
    Eigen::Vector3d    linear_acceleration;

    ImuMsg(const Eigen::Quaterniond& ori     = Eigen::Quaterniond::Identity(),
           const Eigen::Vector3d&    ang_vel = Eigen::Vector3d::Zero(),
           const Eigen::Vector3d&    lin_acc = Eigen::Vector3d::Zero()) :
            orientation(ori),
            angular_velocity(ang_vel),
            linear_acceleration(lin_acc)
    {}
};

struct PwmMsg
{
    std::vector<u_int16_t> channels;
};

struct CommandMsg
{
    CommandMask        mask;
    Eigen::Vector3d    position;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d    angular_velocity;
    Eigen::Vector3d    linear_velocity;
    Eigen::Vector3d    linear_acceleration;

    CommandMsg(
            CommandMask               m       = CommandMask::NONE,
            const Eigen::Vector3d&    pos     = Eigen::Vector3d::Zero(),
            const Eigen::Quaterniond& ori     = Eigen::Quaterniond::Identity(),
            const Eigen::Vector3d&    ang_vel = Eigen::Vector3d::Zero(),
            const Eigen::Vector3d&    lin_vel = Eigen::Vector3d::Zero(),
            const Eigen::Vector3d&    lin_acc = Eigen::Vector3d::Zero()) :
            mask(m),
            position(pos),
            orientation(ori),
            angular_velocity(ang_vel),
            linear_velocity(lin_vel),
            linear_acceleration(lin_acc)
    {}

    bool isSafe() const
    {
        if (!utils::isSafe(linear_velocity.x()) || !utils::isSafe(linear_velocity.y())
            || !utils::isSafe(linear_velocity.z())) {
            LOG_ERROR("Unsafe linear velocity in CommandMsg struct.");
            return false;
        }

        if (!utils::isSafe(linear_acceleration.x()) || !utils::isSafe(linear_acceleration.y())
            || !utils::isSafe(linear_acceleration.z())) {
            LOG_ERROR("Unsafe linear acceleration in CommandMsg struct.");
            return false;
        }

        if (!utils::isSafe(position.x()) || !utils::isSafe(position.y()) || !utils::isSafe(position.z())) {
            LOG_ERROR("Unsafe position in CommandMsg struct.");
            return false;
        }

        if (!utils::isSafe(orientation.x()) || !utils::isSafe(orientation.y()) || !utils::isSafe(orientation.z())
            || !utils::isSafe(orientation.w())) {
            LOG_ERROR("Unsafe orientation (quaternion) in CommandMsg struct.");
            return false;
        }

        if (!utils::isSafe(angular_velocity.x()) || !utils::isSafe(angular_velocity.y()) || !utils::isSafe(angular_velocity.z()))
        {
            LOG_ERROR("Unsafe angular velocity in CommandMsg struct.");
            return false;
        }

        return true;
    }
};

struct GoingToWpConfig
{
    double v_max;
    double a_max;
    double yaw_dot_max;
    double position_error_th;
    double yaw_error_th;
};

struct AssistedConfig
{
    double v_max;
    double yaw_dot_max;
};

struct TakingOffConfig
{
    double height_min;
    double v_max;
    double acc;
    double deacc;
    double height_error_th;
};

struct LandingConfig
{
    double v_landing;
    double height_prelanding;
    double height_error_th_prelanding;
};

} // namespace catec