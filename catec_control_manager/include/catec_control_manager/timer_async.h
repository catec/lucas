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

#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <thread>

namespace catec {

class TimerAsync
{
  public:
    TimerAsync(std::chrono::milliseconds timeout, std::function<void()> timeout_callback) :
            _running(false),
            _duration(0),
            _timeout(timeout),
            _timeout_callback(timeout_callback),
            _timeout_check(true)
    {
        LOG_TRACE(__PRETTY_FUNCTION__);
    }

    TimerAsync() : _running(false), _duration(0), _timeout_check(false) { LOG_TRACE(__PRETTY_FUNCTION__); }

    TimerAsync(bool timeout_check) : _running(false), _duration(0), _timeout_check(timeout_check)
    {
        LOG_TRACE(__PRETTY_FUNCTION__);
    }

    void start()
    {
        LOG_TRACE(__PRETTY_FUNCTION__);

        if (!_running.load()) {
            _running.store(true);
            _future = std::async(std::launch::async, [this]() {
                _start = std::chrono::high_resolution_clock::now();

                while (_running.load()) {
                    auto now     = std::chrono::high_resolution_clock::now();
                    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - _start);

                    // Check if the timeout is reached
                    if (_timeout_check) {
                        if (elapsed > _timeout) {
                            _running.store(false); // Stop the timer
                            if (_timeout_callback) {
                                _timeout_callback(); // Call the timeout callback
                            } else {
                                LOG_WARN("{} - Timeout callback not added!", __PRETTY_FUNCTION__);
                            }
                            break;
                        }
                    }

                    _duration.store(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count());
                    std::this_thread::sleep_for(std::chrono::milliseconds(1)); // Prevent busy-waiting
                }
            });
        } else {
            LOG_WARN("{} - Timer is already running!", __PRETTY_FUNCTION__);
        }
    }

    void stop()
    {
        LOG_TRACE(__PRETTY_FUNCTION__);

        if (_running.load()) {
            _running.store(false); // Signal the background thread to stop
            if (_future.valid()) {
                _future.wait(); // Wait for the async task to complete
            }
        } else {
            LOG_DEBUG("{} - Timer is not running!", __PRETTY_FUNCTION__);
        }
    }

    void restart()
    {
        LOG_TRACE(__PRETTY_FUNCTION__);

        stop();
        _duration.store(0);
        start();
    }

    long long elapsed() const
    {
        LOG_TRACE(__PRETTY_FUNCTION__);
        return _duration.load();
    }

    bool isRunning() const
    {
        LOG_TRACE(__PRETTY_FUNCTION__);
        return _running.load();
    }

  private:
    std::chrono::time_point<std::chrono::high_resolution_clock> _start;
    std::future<void>                                           _future;
    std::atomic<bool>                                           _running;
    std::atomic<long long>                                      _duration;

    std::chrono::milliseconds _timeout;
    std::function<void()>     _timeout_callback;

    bool _timeout_check;
};
} // namespace catec
