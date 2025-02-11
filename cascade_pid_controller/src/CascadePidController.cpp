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

#include <cascade_pid_controller/CascadePidController.h>

namespace cascade_pid_controller {

CascadePidController::CascadePidController() {
    // rclcpp::NodeOptions options;
    // _nh = std::make_shared<rclcpp::Node>("pid_parameters_handler_node", options);
    // RCLCPP_INFO(_nh->get_logger(), "Node '%s' created.", _nh->get_name());
}

CascadePidController::~CascadePidController() {}

void CascadePidController::init(std::shared_ptr<rclcpp::Node> node)
{
    _nh = node;

    double loop_freq, v_filt_d_freq;
    v_filt_d_freq = _nh->declare_parameter<double>("vel/filt_d_freq", 5.0);
    _nh->get_parameter("loop_freq", loop_freq);

    _pos_x_pid = std::make_shared<control_toolbox::PidROS>(_nh, "pos/x");
    _pos_y_pid = std::make_shared<control_toolbox::PidROS>(_nh, "pos/y");
    _pos_z_pid = std::make_shared<control_toolbox::PidROS>(_nh, "pos/z");
    _vel_x_pid = std::make_shared<control_toolbox::PidROS>(_nh, "vel/x");
    _vel_y_pid = std::make_shared<control_toolbox::PidROS>(_nh, "vel/y");
    _vel_z_pid = std::make_shared<control_toolbox::PidROS>(_nh, "vel/z");
    _yaw_pid = std::make_shared<control_toolbox::PidROS>(_nh, "yaw");

    _pos_x_pid->initPid();
    _pos_y_pid->initPid();
    _pos_z_pid->initPid();
    _vel_x_pid->initPid();
    _vel_y_pid->initPid();
    _vel_z_pid->initPid();
    _yaw_pid->initPid();

    _vel_x_deriv_lpf.set_dt(1.0/loop_freq);
    _vel_y_deriv_lpf.set_dt(1.0/loop_freq);
    _vel_z_deriv_lpf.set_dt(1.0/loop_freq);

    _vel_x_deriv_lpf.set_fcut(v_filt_d_freq);
    _vel_y_deriv_lpf.set_fcut(v_filt_d_freq);
    _vel_z_deriv_lpf.set_fcut(v_filt_d_freq);

    _pos_err_x_lp.set_dt(1.0/loop_freq);
    _pos_err_x_lp.set_fcut(v_filt_d_freq);

    _pos_err_y_lp.set_dt(1.0/loop_freq);
    _pos_err_y_lp.set_fcut(v_filt_d_freq);

    _pos_err_z_lp.set_dt(1.0/loop_freq);
    _pos_err_z_lp.set_fcut(v_filt_d_freq);

    _pos_err_yaw_lp.set_dt(1.0/loop_freq);
    _pos_err_yaw_lp.set_fcut(v_filt_d_freq);

    printInfo();
}

void CascadePidController::printInfo()
{
    RCLCPP_INFO(_nh->get_logger(), "Printing POS X PID INFO: \n");
    _pos_x_pid->printValues();

    RCLCPP_INFO(_nh->get_logger(), "Printing POS Y PID INFO: \n");
    _pos_y_pid->printValues();

    RCLCPP_INFO(_nh->get_logger(), "Printing POS Z PID INFO: \n");
    _pos_z_pid->printValues();

    RCLCPP_INFO(_nh->get_logger(), "Printing VEL X PID INFO: \n");
    _vel_x_pid->printValues();

    RCLCPP_INFO(_nh->get_logger(), "Printing VEL Y PID INFO: \n");
    _vel_y_pid->printValues();

    RCLCPP_INFO(_nh->get_logger(), "Printing VEL Z PID INFO: \n");
    _vel_z_pid->printValues();

    RCLCPP_INFO(_nh->get_logger(), "Printing YAW PID INFO: \n");
    _yaw_pid->printValues();
}

void CascadePidController::reset()
{
    _pos_x_pid->reset();
    _pos_y_pid->reset();
    _pos_z_pid->reset();
    _vel_x_pid->reset();
    _vel_y_pid->reset();
    _vel_z_pid->reset();
    _yaw_pid->reset();

    _vel_x_deriv_lpf.reset();
    _vel_y_deriv_lpf.reset();
    _vel_z_deriv_lpf.reset();

    _pos_err_x_lp.reset();
    _pos_err_y_lp.reset();
    _pos_err_z_lp.reset();
    _pos_err_yaw_lp.reset();
}

Eigen::Vector3d CascadePidController::updatePosPid(const Eigen::Vector3d error, rclcpp::Duration dt)
{
    Eigen::Vector3d vel_ref;

    const auto ex = _pos_err_x_lp.update(error(0));
    const auto ey = _pos_err_y_lp.update(error(1));
    const auto ez = _pos_err_z_lp.update(error(2));

    // double enorm = sqrt(pow(ex, 2) + pow(ey, 2));
    
    // TODO: check against NaN in atan2
    // double alpha = atan2(ey, ex);

    // double vxy = _pos_xy_pid.computeCommand(enorm, dt);

    // vel_ref(0) = vxy * cos(alpha);
    // vel_ref(1) = vxy * sin(alpha);
    
    vel_ref(0) = _pos_x_pid->computeCommand(ex, dt);
    vel_ref(1) = _pos_y_pid->computeCommand(ey, dt);
    vel_ref(2) = _pos_z_pid->computeCommand(ez, dt);

    return vel_ref;
}

Eigen::Vector3d CascadePidController::updateVelPid(const Eigen::Vector3d error,const Eigen::Vector3d error_dot, rclcpp::Duration dt)
{
    Eigen::Vector3d acc_cmd;

    double ax = _vel_x_pid->computeCommand(error(0), error_dot(0), dt);
    double ay = _vel_y_pid->computeCommand(error(1), error_dot(1), dt);
    double az = _vel_z_pid->computeCommand(error(2), error_dot(2), dt);
    
    acc_cmd(0) = ax;
    acc_cmd(1) = ay;
    acc_cmd(2) = az;

    return acc_cmd;
}

double CascadePidController::updateYawPid(double error, rclcpp::Duration dt)
{
    double yaw_rate = _yaw_pid->computeCommand(_pos_err_yaw_lp.update(error),dt);

    // std::cout << "error: " << error << std::endl;
    // std::cout << "error_filt: " << _pos_err_yaw_lp.getState() << std::endl;
    // std::cout << "yaw_rate: " << yaw_rate << std::endl;

    
    return yaw_rate;

}

void CascadePidController::accelerationToAttitude(
        const Eigen::Vector3d acc_des,
        const Eigen::Quaterniond q_uav,
        double&               roll_cmd,
        double&               pitch_cmd)
{
    Eigen::Vector3d acc_body;

    // Convert from odom to base_link
    acc_body = q_uav.inverse() * acc_des;

    // Convert Eigen::Quaterniond to roll, pitch, and yaw
    // Eigen::Matrix3d R_uav = q_uav.toRotationMatrix();
    // Eigen::Vector3d rpy_uav = R_uav.eulerAngles(2,1,0);  // ZYX order
    // double roll_uav = rpy_uav.z();

    // Get roll and pitch
    pitch_cmd = atan2(acc_body.x(), 9.81);
    roll_cmd = atan2(-1*acc_body.y()*cos(pitch_cmd), 9.81);

}

} // namespace cascade_pid_controller