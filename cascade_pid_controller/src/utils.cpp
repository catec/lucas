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

#include <cascade_pid_controller/utils.hpp>
#include <iostream>

namespace utils{ 

double BlockLowPass::update(double input)
{
    if (!_initialized)
    {
        setState(input);
        if (_dt > 0.0 && _fcut > 0.0)
            _initialized = true;
    }

    double b = 2 * M_PI * _fcut * _dt;
    double a = b / (1 + b);
    setState(a * input + (1 - a) * getState());
    return getState();
}

double BlockLowPass::update(double input, double dt)
{
    set_dt(dt);
    return update(input);
}

double BlockDerivative::update(double input)
{
    double output;

    if (_initialized)
    {
        output = _lowPass.update((input - _u) / _dt);
    }
    else if (_dt > 0.0 && _fcut > 0.0)
    {
        _lowPass.update(0.0);
        output = 0.0;
        _initialized = true;
    }

    _u = input;
    return output;
}


double mapValue(double value, double fromLow, double fromHigh, double toLow, double toHigh) {
    // Ensure the value is within the source range
    value = std::min(std::max(value, fromLow), fromHigh);

    // Perform linear interpolation
    double mappedValue = toLow + (toHigh - toLow) * (value - fromLow) / (fromHigh - fromLow);

    return mappedValue;
}

double BlockDerivative::update(double input, double dt)
{
    set_dt(dt);
    return update(input);
}

double angdiff(double b, double a) {
    double absdif = std::abs(a - b);

    if (absdif > M_PI) {
        absdif = 2 * M_PI - absdif;
    }

    int signo;
    if (((a - b) >= 0 && (a - b) <= M_PI) || ((a - b) <= -M_PI && (a - b) >= -2 * M_PI)) {
        signo = 1;
    } else {
        signo = -1;
    }

    return signo * absdif;
}


double wrapAngle(double angle) {
    // Wrap to interval <-pi, pi)
    return std::fmod(angle + M_PI, 2 * M_PI) - M_PI;
}

}