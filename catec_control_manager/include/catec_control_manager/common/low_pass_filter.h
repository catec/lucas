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

namespace catec {

template <class T>
class BlockLowPass
{
  public:
    T update(T input)
    {
        if (!_initialized) {
            setState(input);
            if (_dt > 0.0 && _fcut > 0.0)
                _initialized = true;
        }

        double b = 2 * M_PI * _fcut * _dt;
        double a = b / (1 + b);
        setState(a * input + (1 - a) * getState());
        return getState();
    };
    T update(T input, double dt)
    {
        set_dt(dt);
        return update(input);
    };
    void setState(T state) { _state = state; }
    T    getState() { return _state; }
    void set_dt(double dt) { _dt = dt; };
    void set_fcut(double fcut) { _fcut = fcut; };
    void reset() { _initialized = false; };

    BlockLowPass() : _dt(0.0), _fcut(0.0), _initialized(false) {}
    BlockLowPass(T dt, double fcut) : _dt(dt), _fcut(fcut) {}

  private:
    T      _state;
    double _dt;
    double _fcut;
    bool   _initialized;
};

} // namespace catec