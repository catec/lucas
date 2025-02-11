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

#include <thread>

#include "catec_control_manager/control_manager_state_machine.h"

namespace catec {
class BaseState : public ControlManagerStateMachine
{
  public:
    ~BaseState()
    {
        LOG_TRACE(__PRETTY_FUNCTION__);

        stopWorkThread();
    }

  protected:
    virtual void executeThreadJob() = 0;

    void workThread(const float& rate)
    {
        LOG_TRACE(__PRETTY_FUNCTION__);

        const auto period = std::chrono::milliseconds(static_cast<int>((1.0f / rate) * 1000.0f));

        auto const start = std::chrono::system_clock::now();
        while (!_stop_atomic.load(std::memory_order_relaxed)) {
            executeThreadJob();

            auto now        = std::chrono::system_clock::now();
            auto iterations = (now - start) / period;
            auto next_start = start + (iterations + 1) * period;
            std::this_thread::sleep_until(next_start);
        }
    }

    void startWorkThread(const float& rate)
    {
        LOG_TRACE(__PRETTY_FUNCTION__);

        /// \note. Ensure that previous state thread join propperly
        _stop_atomic.store(false);
        if (_work_thr.joinable()) {
            _work_thr.join();
        }
        _work_thr = std::thread(&BaseState::workThread, this, rate);
    }

    void stopWorkThread()
    {
        LOG_TRACE(__PRETTY_FUNCTION__);

        _stop_atomic.store(true);
    }

  protected:
    std::thread      _work_thr;
    std::atomic_bool _stop_atomic{false};
};

} // namespace catec