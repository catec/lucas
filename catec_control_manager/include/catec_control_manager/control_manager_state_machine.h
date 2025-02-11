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

#include <memory>
#include <mutex>

#include "common/log_manager.h"
#include "common/types.h"
#include "events_definition.h"
#include "tinyfsm.h"
namespace catec {

class ParametersYamlParser;
class TimerAsync;

class ControlManagerStateMachine : public tinyfsm::Fsm<ControlManagerStateMachine>
{
  public:
    ControlManagerStateMachine();
    ~ControlManagerStateMachine();

    void loadParametersFile(const std::string& params_file_path);

    void updateOdometry(const OdometryMsg& msg);
    void updateImu(const ImuMsg& msg);
    void updatePwm(const PwmMsg& msg);
    void resetRangefinderChecker();
    void updateMavrosState(const MavrosStateComplete& msg);
    void updateMavrosExtendedState(const MavrosExtendedState& msg);
    void updateControllerState(const CommandMask& msg);
    void updateGenericReference(const CommandMsg& msg);

    ControlManagerState getCurrentState();

    std::pair<bool, std::string> requestAuthority();
    std::pair<bool, std::string> requestTakeOff(const float& height);
    std::pair<bool, std::string> requestLand();
    std::pair<bool, std::string> requestChangeToHover();
    std::pair<bool, std::string> requestChangeToAssisted();
    std::pair<bool, std::string> requestChangeToOffboard();
    std::pair<bool, std::string> requestGoToWaypoint(const std::string& frame_id, const CommandMsg& pose);

    static std::function<void(const CommandMsg&)>  sendCommandToRosCb;
    static std::function<void(const MavrosState&)> setStateToRosCb;

    /// \note. FSM functions
    void react(tinyfsm::Event const& event);

    virtual void react(RequestAuthorityEvent const&);
    virtual void react(RequestTakeOffEvent const&);
    virtual void react(RequestLandEvent const&);
    virtual void react(RequestGoToWpEvent const&);
    virtual void react(RequestSetModeHoverEvent const&);
    virtual void react(RequestSetModeAssistedEvent const&);
    virtual void react(RequestSetModeOffboardEvent const&);
    virtual void react(EmergencyDetectedEvent const&);

    virtual void entry();
    virtual void exit();

  private:
    void startCriticalMsgsTimers();
    bool checkIfMavrosModeChanged(
            const MavrosState& desired_mavros_state,
            const int&         check_mode_changed_max_attemps,
            const int&         wait_sleep_ms);

  protected:
    std::shared_ptr<ParametersYamlParser> getParameters();
    std::shared_ptr<OdometryMsg>          getCurrentOdometry();
    std::shared_ptr<ImuMsg>               getCurrentImu();
    std::shared_ptr<PwmMsg>               getCurrentPwm();
    std::shared_ptr<MavrosStateComplete>  getCurrentMavrosState();
    std::shared_ptr<MavrosExtendedState>  getCurrentMavrosExtendedState();
    std::shared_ptr<CommandMask>          getCurrentControllerState();
    std::shared_ptr<CommandMsg>           getCurrentGenericReference();

    void updateCurrentState(const ControlManagerState& state);

    void publishControlReference(const CommandMsg& ref);

    void resetGenericReference();

    long getReferenceElapsedTime();

    void setMavrosState(const MavrosState& state);

  private:
    std::mutex _mtx_odometry;
    std::mutex _mtx_imu;
    std::mutex _mtx_pwm;
    std::mutex _mtx_mavros_state;
    std::mutex _mtx_mavros_extended_state;
    std::mutex _mtx_controller_state;
    std::mutex _mtx_sm_state;
    std::mutex _mtx_parameters;
    std::mutex _mtx_reference;

    std::unordered_map<CriticalMsg, std::unique_ptr<TimerAsync>> _critical_msgs_timers_checker;
    static std::unique_ptr<TimerAsync>                           _generic_reference_checker_timer;

    static std::shared_ptr<ParametersYamlParser> _parameters;

    static ControlManagerState                  _current_state;
    static std::shared_ptr<OdometryMsg>         _last_odom_msg;
    static std::shared_ptr<ImuMsg>              _last_imu_msg;
    static std::shared_ptr<PwmMsg>              _last_pwm_msg;
    static std::shared_ptr<MavrosStateComplete> _last_mavros_state;
    static std::shared_ptr<MavrosExtendedState> _last_mavros_extended_state;
    static std::shared_ptr<CommandMask>         _last_controller_state;
    static std::shared_ptr<CommandMsg>          _last_generic_reference;
};
} // namespace catec
