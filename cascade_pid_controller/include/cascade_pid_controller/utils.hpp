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

#include <math.h>

#ifndef UTILS_HPP
#define UTILS_HPP

namespace utils {

class BlockLowPass
{
  private:
    double _state;
    double _dt;
    double _fcut;
    bool   _initialized;

  public:
    double update(double input);
    double update(double input, double dt);
    void   setState(double state) { _state = state; }
    double getState() { return _state; }
    void   set_dt(double dt) { _dt = dt; };
    void   set_fcut(double fcut) { _fcut = fcut; };
    void   reset() { _initialized = false; };

    BlockLowPass() : _dt(0.0), _fcut(0.0) {}
    BlockLowPass(double dt, double fcut) : _dt(dt), _fcut(fcut) {}
};

class BlockDerivative
{
  private:
    double       _u;
    double       _dt;
    double       _fcut;
    bool         _initialized;
    BlockLowPass _lowPass;

  public:
    double update(double input);
    double update(double input, double dt);
    void   set_dt(double dt) { _dt = dt; };
    void   set_fcut(double fcut) { _fcut = fcut; };
    void   reset() { _initialized = false; };

    BlockDerivative() : _u(0.0), _dt(0.0), _fcut(0.0), _initialized(false), _lowPass() {}
    BlockDerivative(double dt, double fcut) : _u(0.0), _dt(dt), _fcut(fcut), _initialized(false), _lowPass(dt, fcut) {}
};

template <typename valueT>
class Integrator
{
  private:
    valueT _value;

  public:
    Integrator() : _value(0){};
    void         integrate(valueT rate, valueT dt) { _value += rate * dt; };
    void         reset() { _value = 0; };
    void         reset(valueT value) { _value = value; };
    const valueT getValue() const { return _value; };
};

template <typename valueT>
valueT normalizeAngle(valueT angle)
{
    valueT pi = static_cast<valueT>(M_PI);
    angle     = std::fmod(angle + pi, static_cast<valueT>(2) * pi);
    if (angle < static_cast<valueT>(0))
        angle += static_cast<valueT>(2) * pi;
    return angle - pi;
}

double mapValue(double value, double fromLow, double fromHigh, double toLow, double toHigh);

double angdiff(double b, double a);
double wrapAngle(double angle);

} // namespace utils

#endif