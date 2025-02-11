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

#include "catec_control_manager/control_manager_state_machine.h"

#include "catec_control_manager/common/log_manager.h"
#include "catec_control_manager/common/parameters_yaml_parser.h"
#include "catec_control_manager/states/states_definition.h"
#include "catec_control_manager/timer_async.h"

namespace catec {

std::shared_ptr<OdometryMsg>         ControlManagerStateMachine::_last_odom_msg = std::make_shared<OdometryMsg>();
std::shared_ptr<ImuMsg>              ControlManagerStateMachine::_last_imu_msg  = std::make_shared<ImuMsg>();
std::shared_ptr<PwmMsg>              ControlManagerStateMachine::_last_pwm_msg  = std::make_shared<PwmMsg>();
std::shared_ptr<MavrosStateComplete> ControlManagerStateMachine::_last_mavros_state
        = std::make_shared<MavrosStateComplete>();
std::shared_ptr<MavrosExtendedState> ControlManagerStateMachine::_last_mavros_extended_state
        = std::make_shared<MavrosExtendedState>();
std::shared_ptr<CommandMask> ControlManagerStateMachine::_last_controller_state = std::make_shared<CommandMask>();
std::unique_ptr<TimerAsync>  ControlManagerStateMachine::_generic_reference_checker_timer
        = std::make_unique<TimerAsync>();
std::shared_ptr<CommandMsg> ControlManagerStateMachine::_last_generic_reference = std::make_shared<CommandMsg>();
ControlManagerState         ControlManagerStateMachine::_current_state          = ControlManagerState::UNINITIALIZED;
std::shared_ptr<ParametersYamlParser> ControlManagerStateMachine::_parameters
        = std::make_shared<ParametersYamlParser>();

std::function<void(const CommandMsg&)>  ControlManagerStateMachine::sendCommandToRosCb = nullptr;
std::function<void(const MavrosState&)> ControlManagerStateMachine::setStateToRosCb    = nullptr;

ControlManagerStateMachine::ControlManagerStateMachine()
{
    LOG_TRACE(__PRETTY_FUNCTION__);
}

ControlManagerStateMachine::~ControlManagerStateMachine()
{
    LOG_TRACE(__PRETTY_FUNCTION__);
}

void ControlManagerStateMachine::loadParametersFile(const std::string& params_file_path)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    _parameters = std::make_shared<ParametersYamlParser>(params_file_path);

    _generic_reference_checker_timer = std::make_unique<TimerAsync>(
            std::chrono::milliseconds(_parameters->generic_reference_checker_timer_interval), [] {});
    _generic_reference_checker_timer->start();

    startCriticalMsgsTimers();
}

void ControlManagerStateMachine::startCriticalMsgsTimers()
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    auto odom_checker_timer
            = std::make_unique<TimerAsync>(std::chrono::milliseconds(_parameters->odom_checker_timer_interval), [&] {
                  LOG_TRACE(__PRETTY_FUNCTION__);

                  LOG_WARN(
                          "Not received ODOMETRY during {} ms. Jumping to 'EMERGENCY' state",
                          _parameters->odom_checker_timer_interval);

                  EmergencyDetectedEvent ev;
                  dispatch(ev);
              });

    auto imu_checker_timer
            = std::make_unique<TimerAsync>(std::chrono::milliseconds(_parameters->imu_checker_timer_interval), [&] {
                  LOG_TRACE(__PRETTY_FUNCTION__);

                  LOG_WARN(
                          "Not received IMU during {} ms. Jumping to 'EMERGENCY' state",
                          _parameters->imu_checker_timer_interval);

                  EmergencyDetectedEvent ev;
                  dispatch(ev);
              });

    auto pwm_checker_timer
            = std::make_unique<TimerAsync>(std::chrono::milliseconds(_parameters->pwm_checker_timer_interval), [&] {
                  LOG_TRACE(__PRETTY_FUNCTION__);

                  LOG_WARN(
                          "Not received PWM during {} ms. Jumping to 'EMERGENCY' state",
                          _parameters->pwm_checker_timer_interval);

                  EmergencyDetectedEvent ev;
                  dispatch(ev);
              });

    auto controller_checker_timer = std::make_unique<TimerAsync>(
            std::chrono::milliseconds(_parameters->controller_checker_timer_interval), [&] {
                LOG_TRACE(__PRETTY_FUNCTION__);

                LOG_WARN(
                        "Not received CONTROLLER during {} ms. Jumping to 'EMERGENCY' state",
                        _parameters->controller_checker_timer_interval);

                EmergencyDetectedEvent ev;
                dispatch(ev);
            });

    if (getParameters()->use_rangefinder) {
        auto rangefinder_checker_timer = std::make_unique<TimerAsync>(
                std::chrono::milliseconds(_parameters->rangefinder_checker_timer_interval), [&] {
                    LOG_TRACE(__PRETTY_FUNCTION__);

                    LOG_WARN(
                            "Not received RANGEFINDER during {} ms. Jumping to 'EMERGENCY' state",
                            _parameters->rangefinder_checker_timer_interval);

                    EmergencyDetectedEvent ev;
                    dispatch(ev);
                });

        _critical_msgs_timers_checker[CriticalMsg::RANGEFINDER] = std::move(rangefinder_checker_timer);
    }

    _critical_msgs_timers_checker[CriticalMsg::ODOM]       = std::move(odom_checker_timer);
    _critical_msgs_timers_checker[CriticalMsg::IMU]        = std::move(imu_checker_timer);
    _critical_msgs_timers_checker[CriticalMsg::PWM]        = std::move(pwm_checker_timer);
    _critical_msgs_timers_checker[CriticalMsg::CONTROLLER] = std::move(controller_checker_timer);

    /// \note. Start critical msgs timers checkers
    for (const auto& [key, timer] : _critical_msgs_timers_checker) {
        timer->start();
    }
}

void ControlManagerStateMachine::updateMavrosState(const MavrosStateComplete& msg)
{
    std::lock_guard<std::mutex> guard(_mtx_mavros_state);
    *_last_mavros_state = msg;
}

void ControlManagerStateMachine::updateMavrosExtendedState(const MavrosExtendedState& msg)
{
    std::lock_guard<std::mutex> guard(_mtx_mavros_extended_state);
    *_last_mavros_extended_state = msg;
}

void ControlManagerStateMachine::updateControllerState(const CommandMask& msg)
{
    std::lock_guard<std::mutex> guard(_mtx_controller_state);
    *_last_controller_state = msg;

    _critical_msgs_timers_checker[CriticalMsg::CONTROLLER]->restart();
}

void ControlManagerStateMachine::updateOdometry(const OdometryMsg& msg)
{
    std::lock_guard<std::mutex> guard(_mtx_odometry);
    *_last_odom_msg = msg;

    _critical_msgs_timers_checker[CriticalMsg::ODOM]->restart();
}

void ControlManagerStateMachine::updateImu(const ImuMsg& msg)
{
    std::lock_guard<std::mutex> guard(_mtx_imu);
    *_last_imu_msg = msg;

    _critical_msgs_timers_checker[CriticalMsg::IMU]->restart();
}

void ControlManagerStateMachine::updatePwm(const PwmMsg& msg)
{
    std::lock_guard<std::mutex> guard(_mtx_pwm);
    *_last_pwm_msg = msg;

    _critical_msgs_timers_checker[CriticalMsg::PWM]->restart();
}

void ControlManagerStateMachine::resetRangefinderChecker()
{
    if (getParameters()->use_rangefinder) {
        _critical_msgs_timers_checker[CriticalMsg::RANGEFINDER]->restart();
    }
}

void ControlManagerStateMachine::updateCurrentState(const ControlManagerState& state)
{
    std::lock_guard<std::mutex> guard(_mtx_sm_state);
    _current_state = state;
}

void ControlManagerStateMachine::updateGenericReference(const CommandMsg& msg)
{
    std::lock_guard<std::mutex> guard(_mtx_reference);
    *_last_generic_reference = msg;

    _generic_reference_checker_timer->restart();
}

std::shared_ptr<OdometryMsg> ControlManagerStateMachine::getCurrentOdometry()
{
    std::lock_guard<std::mutex> guard(_mtx_odometry);
    return _last_odom_msg;
}

std::shared_ptr<ImuMsg> ControlManagerStateMachine::getCurrentImu()
{
    std::lock_guard<std::mutex> guard(_mtx_imu);
    return _last_imu_msg;
}

std::shared_ptr<PwmMsg> ControlManagerStateMachine::getCurrentPwm()
{
    std::lock_guard<std::mutex> guard(_mtx_pwm);
    return _last_pwm_msg;
}

std::shared_ptr<MavrosStateComplete> ControlManagerStateMachine::getCurrentMavrosState()
{
    std::lock_guard<std::mutex> guard(_mtx_mavros_state);
    return _last_mavros_state;
}

std::shared_ptr<MavrosExtendedState> ControlManagerStateMachine::getCurrentMavrosExtendedState()
{
    std::lock_guard<std::mutex> guard(_mtx_mavros_extended_state);
    return _last_mavros_extended_state;
}

std::shared_ptr<CommandMsg> ControlManagerStateMachine::getCurrentGenericReference()
{
    std::lock_guard<std::mutex> guard(_mtx_reference);
    return _last_generic_reference;
}

std::shared_ptr<ParametersYamlParser> ControlManagerStateMachine::getParameters()
{
    std::lock_guard<std::mutex> guard(_mtx_parameters);
    return _parameters;
}

ControlManagerState ControlManagerStateMachine::getCurrentState()
{
    std::lock_guard<std::mutex> guard(_mtx_sm_state);
    return _current_state;
}

bool ControlManagerStateMachine::checkIfMavrosModeChanged(
        const MavrosState& desired_mavros_state,
        const int&         check_mode_changed_max_attemps,
        const int&         wait_sleep_ms)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    /// \note. Get mavros state string
    auto it = std::find_if(mavros_state_table.cbegin(), mavros_state_table.cend(), [&desired_mavros_state](auto&& p) {
        return p.second == desired_mavros_state;
    });
    if (it == mavros_state_table.cend()) {
        LOG_WARN("{} - Unable to match current state with states table");
        return false;
    }

    const std::string mavros_state_str{it->first};

    int  curr_attemps{0};
    bool successfully_mode_changed{false};
    while (curr_attemps < check_mode_changed_max_attemps) {
        if (getCurrentMavrosState()->state != desired_mavros_state) {
            LOG_DEBUG(
                    "{} Mavros state not changed to {}. Attemp: {}/{}",
                    __PRETTY_FUNCTION__,
                    mavros_state_str,
                    curr_attemps,
                    check_mode_changed_max_attemps);
            std::this_thread::sleep_for(std::chrono::milliseconds(wait_sleep_ms));
        } else {
            successfully_mode_changed = true;
            break;
        }
        curr_attemps++;
    }
    return successfully_mode_changed;
}

std::pair<bool, std::string> ControlManagerStateMachine::requestAuthority()
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    bool in_state_landed_armed  = is_in_state<catec::LandedArmed>();
    bool in_state_flying_manual = is_in_state<catec::FlyingManual>();
    if (!in_state_landed_armed && !in_state_flying_manual) {
        LOG_WARN(
                "{} - Authority can only be requested in 'LANDED_ARMED' state or 'FLYING MANUAL' state",
                __PRETTY_FUNCTION__);
        return std::make_pair(
                false, "Authority can only be requested in 'LANDED_ARMED' state or 'FLYING MANUAL' state");
    }

    /// \note. Its assumed that if this timer is running, it is receiving metering at the desired frequency.
    if (!_critical_msgs_timers_checker[CriticalMsg::ODOM]->isRunning()) {
        LOG_WARN("{} - Authority can only be requested whether odometry is being received", __PRETTY_FUNCTION__);
        return std::make_pair(false, "Authority can only be requested whether odometry is being received");
    }

    RequestAuthorityEvent ev;
    dispatch(ev);

    /// \note. Check if controller is publishing?
    setMavrosState(MavrosState::GUIDED_NOGPS);

    bool successfully_mode_changed = checkIfMavrosModeChanged(
            MavrosState::GUIDED_NOGPS,
            _parameters->check_mode_changed_max_attemps,
            _parameters->check_mode_changed_wait_ms);

    if (!successfully_mode_changed) {
        LOG_WARN(
                "{} - Unable to change mavros state to 'GUIDED_NOGPS' after {} attemps",
                __PRETTY_FUNCTION__,
                _parameters->check_mode_changed_max_attemps);
        return std::make_pair(false, "Unable to change mavros state to 'GUIDED_NOGPS' after several attemps");
    }

    return std::make_pair(true, "Successfully received 'request authority' command");
}

std::pair<bool, std::string> ControlManagerStateMachine::requestTakeOff(const float& height)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    const bool in_state_landed_auto = is_in_state<catec::LandedAuto>();
    if (!in_state_landed_auto) {
        LOG_WARN("{} - Take Off can only be requested in 'LANDED_AUTO' state", __PRETTY_FUNCTION__);
        return std::make_pair(false, "Take Off can only be requested in 'LANDED_AUTO' state");
    }

    const double takeoff_min_height = getParameters()->takingoff_config.height_min;

    if (height < takeoff_min_height) {
        LOG_WARN("{} - Take Off requested height is less than {} m", __PRETTY_FUNCTION__, takeoff_min_height);
        return std::make_pair(false, "Take Off requested height is less than minimum");
    }

    CommandMsg takeoff_command;
    takeoff_command.mask         = CommandMask::POSITION;
    takeoff_command.position.z() = height;

    updateGenericReference(takeoff_command);

    RequestTakeOffEvent ev;
    dispatch(ev);

    return std::make_pair(true, "Successfully received 'take-off' command");
}

std::pair<bool, std::string> ControlManagerStateMachine::requestLand()
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    const bool in_state_hover = is_in_state<catec::Hover>();
    if (!in_state_hover) {
        LOG_WARN("{} - Land can only be requested in 'HOVER' state", __PRETTY_FUNCTION__);
        return std::make_pair(false, "Land can only be requested in 'HOVER' state");
    }

    RequestLandEvent ev;
    dispatch(ev);

    return std::make_pair(true, "Successfully received 'land' command");
}

std::pair<bool, std::string> ControlManagerStateMachine::requestChangeToHover()
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    const bool in_state_assisted    = is_in_state<catec::Assisted>();
    const bool in_state_offboard    = is_in_state<catec::Offboard>();
    const bool in_state_going_to_wp = is_in_state<catec::GoingToWp>();
    if (!(in_state_assisted || in_state_offboard || in_state_going_to_wp)) {
        LOG_WARN(
                "{} - Change flying mode to 'HOVER' can only be requested in 'ASSISTED', 'OFFBOARD' or "
                "'GOING_TO_WP' "
                "states",
                __PRETTY_FUNCTION__);
        return std::make_pair(
                false,
                "Change flying mode to 'HOVER' can only be requested in 'ASSISTED', 'OFFBOARD' or 'GOING_TO_WP' "
                "states");
    }

    RequestSetModeHoverEvent ev;
    dispatch(ev);

    return std::make_pair(true, "Successfully received 'set flying mode HOVER' command");
}

std::pair<bool, std::string> ControlManagerStateMachine::requestChangeToAssisted()
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    const bool in_state_hover = is_in_state<catec::Hover>();
    if (!in_state_hover) {
        LOG_WARN("{} - Change flying mode to 'ASSISTED' can only be requested in 'HOVER' state", __PRETTY_FUNCTION__);
        return std::make_pair(false, "Change flying mode to 'ASSISTED' can only be requested in 'HOVER' state");
    }

    RequestSetModeAssistedEvent ev;
    dispatch(ev);

    return std::make_pair(true, "Successfully received 'set flying mode ASSISTED' command");
}

std::pair<bool, std::string> ControlManagerStateMachine::requestChangeToOffboard()
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    const bool in_state_hover = is_in_state<catec::Hover>();
    if (!in_state_hover) {
        LOG_WARN("{} - Change flying mode to 'OFFBOARD' can only be requested in 'HOVER' state", __PRETTY_FUNCTION__);
        return std::make_pair(false, "Change flying mode to 'OFFBOARD' can only be requested in 'HOVER' state");
    }

    RequestSetModeOffboardEvent ev;
    dispatch(ev);

    return std::make_pair(true, "Successfully received 'set flying mode OFFBOARD' command");
}

std::pair<bool, std::string> ControlManagerStateMachine::requestGoToWaypoint(
        const std::string& frame_id,
        const CommandMsg&  pose)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    const bool in_state_hover = is_in_state<catec::Hover>();
    if (!in_state_hover) {
        LOG_WARN("{} - Go To Waypoint can only be requested in 'HOVER' state", __PRETTY_FUNCTION__);
        return std::make_pair(false, "Go To Waypoint can only be requested in 'HOVER' state");
    }

    if (frame_id != std::string("odom")) {
        LOG_WARN("{} - The desired waypoint need be in 'odom' frame_id", __PRETTY_FUNCTION__);
        return std::make_pair(false, "The desired waypoint need be in 'odom' frame_id");
    }

    updateGenericReference(pose);

    RequestGoToWpEvent req_ev;
    dispatch(req_ev);

    return std::make_pair(true, "Successfully received 'go to waypoint' command");
}

void ControlManagerStateMachine::setMavrosState(const MavrosState& state)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    if (!setStateToRosCb) {
        LOG_WARN("{} - Callback not implemented", __PRETTY_FUNCTION__);
        return;
    }

    setStateToRosCb(state);
}

void ControlManagerStateMachine::publishControlReference(const CommandMsg& ref)
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    if (!sendCommandToRosCb) {
        LOG_WARN("{} - Callback not implemented", __PRETTY_FUNCTION__);

        return;
    }

    if (ref.isSafe()) {
        sendCommandToRosCb(ref);
    } else {
        LOG_WARN("{} - Not normal number detected in control reference. Not publishing.", __PRETTY_FUNCTION__);
    }
}

void ControlManagerStateMachine::resetGenericReference()
{
    std::lock_guard<std::mutex> guard(_mtx_reference);
    LOG_DEBUG("{} - Reseting generic reference pointer", __PRETTY_FUNCTION__);
    // _last_generic_reference.reset(); // THIS CRASH
    _last_generic_reference = std::make_shared<CommandMsg>();
}

long ControlManagerStateMachine::getReferenceElapsedTime()
{
    LOG_TRACE(__PRETTY_FUNCTION__);

    LOG_DEBUG("{} ms", _generic_reference_checker_timer->elapsed());

    return _generic_reference_checker_timer->elapsed();
}

/// -------------------------------------------------------------------------------------------------------------------
/// \note. FSM functions
void ControlManagerStateMachine::react(tinyfsm::Event const&)
{
    LOG_TRACE(__PRETTY_FUNCTION__);
    LOG_WARN("{} - Received unhandled event", __PRETTY_FUNCTION__);
}

void ControlManagerStateMachine::react(RequestAuthorityEvent const&)
{
    LOG_TRACE(__PRETTY_FUNCTION__);
    LOG_WARN(
            "{} - Generated 'RequestAuthorityEvent' in unhandled state: '{}'",
            __PRETTY_FUNCTION__,
            ToString(getCurrentState()));
}

void ControlManagerStateMachine::react(RequestTakeOffEvent const&)
{
    LOG_TRACE(__PRETTY_FUNCTION__);
    LOG_WARN(
            "{} - Generated 'RequestTakeOffEvent' in unhandled state: '{}'",
            __PRETTY_FUNCTION__,
            ToString(getCurrentState()));
}

void ControlManagerStateMachine::react(RequestLandEvent const&)
{
    LOG_TRACE(__PRETTY_FUNCTION__);
    LOG_WARN(
            "{} - Generated 'RequestLandEvent' in unhandled state: '{}'",
            __PRETTY_FUNCTION__,
            ToString(getCurrentState()));
}

void ControlManagerStateMachine::react(RequestGoToWpEvent const&)
{
    LOG_TRACE(__PRETTY_FUNCTION__);
    LOG_WARN(
            "{} - Generated 'RequestGoToWpEvent' in unhandled state: '{}'",
            __PRETTY_FUNCTION__,
            ToString(getCurrentState()));
}

void ControlManagerStateMachine::react(RequestSetModeHoverEvent const&)
{
    LOG_TRACE(__PRETTY_FUNCTION__);
    LOG_WARN(
            "{} - Generated 'RequestSetModeHoverEvent' in unhandled state: '{}'",
            __PRETTY_FUNCTION__,
            ToString(getCurrentState()));
}

void ControlManagerStateMachine::react(RequestSetModeAssistedEvent const&)
{
    LOG_TRACE(__PRETTY_FUNCTION__);
    LOG_WARN(
            "{} - Generated 'RequestSetModeAssistedEvent' in unhandled state: '{}'",
            __PRETTY_FUNCTION__,
            ToString(getCurrentState()));
}

void ControlManagerStateMachine::react(RequestSetModeOffboardEvent const&)
{
    LOG_TRACE(__PRETTY_FUNCTION__);
    LOG_WARN(
            "{} - Generated 'RequestSetModeOffboardEvent' in unhandled state: '{}'",
            __PRETTY_FUNCTION__,
            ToString(getCurrentState()));
}

void ControlManagerStateMachine::react(EmergencyDetectedEvent const&)
{
    LOG_TRACE(__PRETTY_FUNCTION__);
    LOG_WARN(
            "{} - Generated 'EmergencyDetectedEvent' in unhandled state: '{}'",
            __PRETTY_FUNCTION__,
            ToString(getCurrentState()));
}

/* entry actions in some states */
void ControlManagerStateMachine::entry()
{
    LOG_TRACE(__PRETTY_FUNCTION__);
}

/* no exit actions at all */
void ControlManagerStateMachine::exit()
{
    LOG_TRACE(__PRETTY_FUNCTION__);
}

} // namespace catec

FSM_INITIAL_STATE(catec::ControlManagerStateMachine, catec::Uninitialized)