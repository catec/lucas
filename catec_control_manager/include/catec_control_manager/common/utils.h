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

#include <Eigen/Eigen>

namespace catec::utils {

inline std::vector<double> ToEulerAngles(double qw, double qx, double qy, double qz)
{
    std::vector<double> quat{qx, qy, qz, qw};

    std::vector<double> eulerAngles;
    // roll (x-axis rotation)
    const double        sinr_cosp = 2 * (quat[3] * quat[0] + quat[1] * quat[2]);
    const double        cosr_cosp = 1 - 2 * (quat[0] * quat[0] + quat[1] * quat[1]);
    eulerAngles.push_back(std::atan2(sinr_cosp, cosr_cosp));

    // pitch (y-axis rotation)
    const double sinp = 2 * (quat[3] * quat[1] - quat[2] * quat[0]);
    if (std::abs(sinp) >= 1) {
        eulerAngles.push_back(std::copysign(M_PI / 2, sinp)); // use 90 degrees if out of range
    } else {
        eulerAngles.push_back(std::asin(sinp));
    }

    // yaw (z-axis rotation)
    const double siny_cosp = 2 * (quat[3] * quat[2] + quat[0] * quat[1]);
    const double cosy_cosp = 1 - 2 * (quat[1] * quat[1] + quat[2] * quat[2]);
    eulerAngles.push_back(std::atan2(siny_cosp, cosy_cosp));

    return eulerAngles;
}

/// \note Asumming that: input={roll, pitch, yaw} and quat={w, x, y, z}
inline std::vector<double> ToQuaternion(double roll, double pitch, double yaw)
{
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);

    std::vector<double> quat{1.0f, 0.0f, 0.0f, 0.0f};
    quat[0] = cr * cp * cy + sr * sp * sy;
    quat[1] = sr * cp * cy - cr * sp * sy;
    quat[2] = cr * sp * cy + sr * cp * sy;
    quat[3] = cr * cp * sy - sr * sp * cy;

    return quat;
}

inline Eigen::Vector3d saturateVelocity(const Eigen::Vector3d& linear_velocity, const double v_max)
{
    // Saturate linear velocity
    double linear_norm = linear_velocity.norm();
    if (linear_norm > v_max) {
        return linear_velocity * (v_max / linear_norm);
    } else {
        return linear_velocity;
    }
}

inline double saturateVelocity(const double velocity, const double v_max)
{
    if (abs(velocity) > v_max) {
        // Scale the velocity to be within the max limits
        return (velocity > 0) ? v_max : -v_max;
    } else {
        return velocity;
    }
}

template <typename valueT>
valueT normalizeAngle(valueT angle)
{
    valueT pi = static_cast<valueT>(M_PI);
    angle     = std::fmod(angle + pi, static_cast<valueT>(2) * pi);
    if (angle < static_cast<valueT>(0))
        angle += static_cast<valueT>(2) * pi;
    return angle - pi;
}

inline double calc_leash_length(double max_vel, double max_acc, double kP)
{
    // Check max_acc > 0.0 and kp > 0.0 to avoid divide by zero
    double leash_lenght;

    if (max_acc <= 0.0) {
        max_acc = 0.1; // DEFINE OR PARAM (MIN ACCELERATION)
    }

    if (kP <= 0.0) {
        return 1.0; // DEFINE OR PARAM (MIN LEASH LENGTH)
    }

    // Calculate leash length
    if (max_vel <= max_acc / kP) {
        leash_lenght = max_vel / kP;
    } else {
        leash_lenght = (max_acc / (2.0 * kP * kP)) + (max_vel * max_vel / (2.0 * max_acc));
    }

    if (leash_lenght < 1.0) {
        leash_lenght = 1.0; // MIN LEASH LENGHT
    }

    return leash_lenght;
}

inline double computeNextYaw(const double dt, const double current_yaw, const double& yaw_dot)
{
    return current_yaw + (yaw_dot * dt);
}

inline double angdiff(double b, double a)
{
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

inline double wrapAngle(double angle)
{
    // Wrap to interval <-pi, pi)
    return std::fmod(angle + M_PI, 2 * M_PI) - M_PI;
}

inline double computeDistanceToAcc(double v_max, double a_max)
{
    return v_max * v_max / (2.0 * a_max);
}

inline double computeTimeToAcc(double v_max, double a_max)
{
    return v_max / a_max;
}

inline double computeMaxVel(double distance, double v_max, double a_max, double deacc_max)
{
    if ((computeDistanceToAcc(v_max, a_max) + computeDistanceToAcc(v_max, deacc_max)) > distance) {
        return sqrt((2 * distance) / ((1 / a_max) + (1 / deacc_max)));
    } else {
        return v_max;
    }
}

// Function to compare two floating-point numbers for closeness
inline bool isclose(double a, double b, double rel_tol = 1e-9, double abs_tol = 0.0)
{
    return std::fabs(a - b) <= std::max(rel_tol * std::max(std::fabs(a), std::fabs(b)), abs_tol);
}

// Function to compute mean value from std vector
inline double meanValueFromVector(const std::vector<uint16_t>& vec)
{
    uint64_t   sum = 0;
    double     mean;
    const auto size = vec.size();

    // Traverse the vector
    for (const auto& val : vec) {
        sum += val; // Add to the sum if non-zero
    }

    // Calculate and return the mean
    if (size > 0) {
        mean = sum / size;
    } else {
        mean = 0.0;
    }

    return mean;
}

inline bool isSafe(const double value)
{
    return std::fpclassify(value) == FP_NORMAL || std::fpclassify(value) == FP_ZERO;
}

} // namespace catec::utils
