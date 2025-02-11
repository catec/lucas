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

#include <math.h>

#include "log_manager.h"
#include "utils.h"

namespace catec {

class CmdPlanner
{
  public:
    CmdPlanner(Eigen::Vector3d initial_pos, Eigen::Vector3d target_pos, double v_max, double acc, double deacc) :
            _initial_pos(initial_pos),
            _target_pos(target_pos),
            _v_max(v_max),
            _acc(acc),
            _deacc(deacc)
    {
        LOG_TRACE(__PRETTY_FUNCTION__);
        Eigen::Vector3d position_error_vector = target_pos - initial_pos;
        _error_unit_vector                    = position_error_vector.normalized();
        this->computeMaxVel();
        this->computeTrajParametrization();
    }

    Eigen::Vector3d computeCmdPos(double time)
    {
        LOG_TRACE(__PRETTY_FUNCTION__);

        if (time <= _time_to_acc) {
            // Acceleration phase
            return _initial_pos + 0.5 * _acc * time * time * _error_unit_vector;
        } else if (time <= (_time_to_acc + _time_cruise)) {
            // Constant velocity phase
            return _pos_accel + _v_max * (time - _time_to_acc) * _error_unit_vector;
        } else if (time <= _total_time) {
            // Deceleration phase
            double t_decel_phase = time - (_time_to_acc + _time_cruise);
            return _pos_cruise
                 + (_v_max * t_decel_phase - 0.5 * _deacc * t_decel_phase * t_decel_phase) * _error_unit_vector;
        } else {
            // After trajectory ends
            return _target_pos;
        }
    }

    Eigen::Vector3d computeCmdVel(double time)
    {
        LOG_TRACE(__PRETTY_FUNCTION__);

        if (time <= _time_to_acc) {
            // Acceleration phase
            return _acc * time * _error_unit_vector;
        } else if (time <= (_time_to_acc + _time_cruise)) {
            // Constant velocity phase
            return _v_max * _error_unit_vector;
        } else if (time <= _total_time) {
            // Deceleration phase
            double t_decel_phase = time - (_time_to_acc + _time_cruise);
            return (_v_max - _deacc * t_decel_phase) * _error_unit_vector;
        } else {
            // After trajectory ends
            return Eigen::Vector3d(0.0, 0.0, 0.0);
        }
    }

    double getTotalTime()
    {
        LOG_TRACE(__PRETTY_FUNCTION__);
        return _total_time;
    }

  private:
    void computeMaxVel()
    {
        LOG_TRACE(__PRETTY_FUNCTION__);
        _v_max = utils::computeMaxVel(this->getTotalDistance(), _v_max, _acc, _deacc);
    }

    double getTotalDistance()
    {
        LOG_TRACE(__PRETTY_FUNCTION__);
        return (_target_pos - _initial_pos).norm();
    }

    void computeTrajParametrization()
    {
        LOG_TRACE(__PRETTY_FUNCTION__);

        // Calculate times for each phase
        _time_to_acc   = utils::computeTimeToAcc(_v_max, _acc);
        _time_to_deacc = utils::computeTimeToAcc(_v_max, _deacc);

        double total_distance  = this->getTotalDistance();
        double cruise_distance = total_distance - utils::computeDistanceToAcc(_v_max, _acc)
                               - utils::computeDistanceToAcc(_v_max, _deacc);

        _time_cruise = cruise_distance / _v_max;

        _total_time = _time_to_acc + _time_to_deacc + _time_cruise;

        LOG_DEBUG("{} - Time to acc: {}", __PRETTY_FUNCTION__, _time_to_acc);
        LOG_DEBUG("{} - Time cruise: {}", __PRETTY_FUNCTION__, _time_cruise);
        LOG_DEBUG("{} - Time total {}", __PRETTY_FUNCTION__, _total_time);
        LOG_DEBUG("{} - Max vel {}", __PRETTY_FUNCTION__, _v_max);

        // Calculate the positions at the end of each phase
        _pos_accel = _initial_pos
                   + 0.5 * _acc * _time_to_acc * _time_to_acc * _error_unit_vector; // Position after acceleration phase
        _pos_cruise = _pos_accel + _v_max * _time_cruise * _error_unit_vector; // Position before deceleration phase
    }

  private:
    Eigen::Vector3d _initial_pos;
    Eigen::Vector3d _target_pos;
    double          _v_max;
    double          _acc;
    double          _deacc;

    double _time_to_acc;
    double _time_to_deacc;
    double _time_cruise;
    double _total_time;

    Eigen::Vector3d _pos_accel;
    Eigen::Vector3d _pos_cruise;

    Eigen::Vector3d _error_unit_vector;
};
} // namespace catec