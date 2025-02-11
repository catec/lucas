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

#include <control_toolbox/pid.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cascade_pid_controller/utils.hpp>

namespace cascade_pid_controller {

class CascadePidController
{
  public:
    CascadePidController();
    ~CascadePidController();
    void            init();
    void            printInfo();
    void            reset();
    Eigen::Vector3d updatePosPid(const Eigen::Vector3d error, ros::Duration dt);
    Eigen::Vector3d updateVelPid(const Eigen::Vector3d error, const Eigen::Vector3d error_dot, ros::Duration dt);
    double updateYawPid(double error, ros::Duration dt);

    void            accelerationToAttitude(
                       const Eigen::Vector3d    acc_des,
                       const Eigen::Quaterniond q_uav,
                       double&                  roll_cmd,
                       double&                  pitch_cmd);

    utils::BlockDerivative _vel_x_deriv_lpf, _vel_y_deriv_lpf, _vel_z_deriv_lpf;
    utils::BlockLowPass _pos_err_x_lp, _pos_err_y_lp, _pos_err_z_lp, _pos_err_yaw_lp;

  private:
    control_toolbox::Pid _pos_x_pid, _pos_y_pid, _pos_z_pid, _vel_x_pid, _vel_y_pid, _vel_z_pid, _yaw_pid;
    ros::NodeHandle      _nh;

    double _reference_v_x;
    double _reference_v_y;
    double _reference_v_z;

    
};
} // namespace cascade_pid_controller