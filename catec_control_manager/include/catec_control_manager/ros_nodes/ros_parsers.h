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
#include <cascade_pid_controller_msgs/TrajCommand.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/RCOut.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "catec_control_manager/common/types.h"
#include "catec_control_manager/common/utils.h"
namespace catec {

OdometryMsg convertFromRosMsg(const nav_msgs::Odometry& msg)
{
    OdometryMsg odometry;
    odometry.position.x() = msg.pose.pose.position.x;
    odometry.position.y() = msg.pose.pose.position.y;
    odometry.position.z() = msg.pose.pose.position.z;

    odometry.orientation.w() = msg.pose.pose.orientation.w;
    odometry.orientation.x() = msg.pose.pose.orientation.x;
    odometry.orientation.y() = msg.pose.pose.orientation.y;
    odometry.orientation.z() = msg.pose.pose.orientation.z;

    odometry.linear_velocity.x() = msg.twist.twist.linear.x;
    odometry.linear_velocity.y() = msg.twist.twist.linear.y;
    odometry.linear_velocity.z() = msg.twist.twist.linear.z;

    odometry.angular_velocity.x() = msg.twist.twist.angular.x;
    odometry.angular_velocity.y() = msg.twist.twist.angular.y;
    odometry.angular_velocity.z() = msg.twist.twist.angular.z;

    return odometry;
}

ImuMsg convertFromRosMsg(const sensor_msgs::Imu& msg)
{
    ImuMsg imu;
    imu.orientation.w() = msg.orientation.w;
    imu.orientation.x() = msg.orientation.x;
    imu.orientation.y() = msg.orientation.y;
    imu.orientation.z() = msg.orientation.z;

    imu.angular_velocity.x() = msg.angular_velocity.x;
    imu.angular_velocity.y() = msg.angular_velocity.y;
    imu.angular_velocity.z() = msg.angular_velocity.z;

    imu.linear_acceleration.x() = msg.linear_acceleration.x;
    imu.linear_acceleration.y() = msg.linear_acceleration.y;
    imu.linear_acceleration.z() = msg.linear_acceleration.z;

    return imu;
}

PwmMsg convertFromRosMsg(const mavros_msgs::RCOut& msg)
{
    PwmMsg pwm;

    // Traverse the vector
    for (const auto& val : msg.channels) {
        if (val != 0) { // 0 value are not assigned channels
            pwm.channels.push_back(static_cast<uint16_t>(val));
        }
    }

    return pwm;
}

nav_msgs::Odometry convertToRosMsg(const OdometryMsg& odom_msg)
{
    nav_msgs::Odometry msg;

    msg.header.stamp = ros::Time::now();
    // msg.header.frame_id = ;
    // msg.child_frame_id = ;

    msg.pose.pose.position.x = odom_msg.position.x();
    msg.pose.pose.position.y = odom_msg.position.y();
    msg.pose.pose.position.z = odom_msg.position.z();

    msg.pose.pose.orientation.w = odom_msg.orientation.w();
    msg.pose.pose.orientation.x = odom_msg.orientation.x();
    msg.pose.pose.orientation.y = odom_msg.orientation.y();
    msg.pose.pose.orientation.z = odom_msg.orientation.z();

    msg.twist.twist.linear.x = odom_msg.linear_velocity.x();
    msg.twist.twist.linear.y = odom_msg.linear_velocity.y();
    msg.twist.twist.linear.z = odom_msg.linear_velocity.z();

    msg.twist.twist.angular.x = odom_msg.angular_velocity.x();
    msg.twist.twist.angular.y = odom_msg.angular_velocity.y();
    msg.twist.twist.angular.z = odom_msg.angular_velocity.z();

    return msg;
}

template <typename T>
T convertToRosMsg(const CommandMsg& command_msg);

template <>
geometry_msgs::PoseStamped convertToRosMsg(const CommandMsg& command_msg)
{
    geometry_msgs::PoseStamped msg;

    msg.header.stamp = ros::Time::now();
    // msg.header.frame_id = ;

    msg.pose.position.x = command_msg.position.x();
    msg.pose.position.y = command_msg.position.y();
    msg.pose.position.z = command_msg.position.z();

    msg.pose.orientation.w = command_msg.orientation.w();
    msg.pose.orientation.x = command_msg.orientation.x();
    msg.pose.orientation.y = command_msg.orientation.y();
    msg.pose.orientation.z = command_msg.orientation.z();

    return msg;
}

template <>
geometry_msgs::TwistStamped convertToRosMsg(const CommandMsg& command_msg)
{
    geometry_msgs::TwistStamped msg;

    msg.header.stamp = ros::Time::now();
    // msg.header.frame_id = ;

    msg.twist.linear.x = command_msg.linear_velocity.x();
    msg.twist.linear.y = command_msg.linear_velocity.y();
    msg.twist.linear.z = command_msg.linear_velocity.z();

    msg.twist.angular.x = command_msg.angular_velocity.x();
    msg.twist.angular.y = command_msg.angular_velocity.y();
    msg.twist.angular.z = command_msg.angular_velocity.z();

    return msg;
}

template <>
cascade_pid_controller_msgs::TrajCommand convertToRosMsg(const CommandMsg& command_msg)
{
    cascade_pid_controller_msgs::TrajCommand msg;

    msg.header.stamp = ros::Time::now();
    // msg.header.frame_id = ;

    msg.position.x = command_msg.position.x();
    msg.position.y = command_msg.position.y();
    msg.position.z = command_msg.position.z();

    msg.velocity.x = command_msg.linear_velocity.x();
    msg.velocity.y = command_msg.linear_velocity.y();
    msg.velocity.z = command_msg.linear_velocity.z();

    msg.acceleration.x = command_msg.linear_acceleration.x();
    msg.acceleration.y = command_msg.linear_acceleration.y();
    msg.acceleration.z = command_msg.linear_acceleration.z();

    // Get Yaw from orientation
    tf2::Quaternion q(
            command_msg.orientation.x(),
            command_msg.orientation.y(),
            command_msg.orientation.z(),
            command_msg.orientation.w());
    tf2::Matrix3x3 m(q);
    double         roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    msg.yaw     = yaw;
    msg.yaw_dot = command_msg.angular_velocity.z();

    return msg;
}

geometry_msgs::Pose convertToRosMsg(const PoseMsg& pose_msg)
{
    geometry_msgs::Pose msg;

    // msg.header.stamp = ros::Time::now();
    // msg.header.frame_id = ;

    msg.position.x = pose_msg.position.x();
    msg.position.y = pose_msg.position.y();
    msg.position.z = pose_msg.position.z();

    msg.orientation.w = pose_msg.orientation.w();
    msg.orientation.x = pose_msg.orientation.x();
    msg.orientation.y = pose_msg.orientation.y();
    msg.orientation.z = pose_msg.orientation.z();

    return msg;
}

CommandMsg convertFromRosMsg(const geometry_msgs::Twist& msg)
{
    CommandMsg command;
    command.mask                = CommandMask::VELOCITY;
    command.linear_velocity.x() = msg.linear.x;
    command.linear_velocity.y() = msg.linear.y;
    command.linear_velocity.z() = msg.linear.z;

    command.angular_velocity.x() = msg.angular.x;
    command.angular_velocity.y() = msg.angular.y;
    command.angular_velocity.z() = msg.angular.z;

    command.linear_acceleration.x() = 0.0;
    command.linear_acceleration.y() = 0.0;
    command.linear_acceleration.z() = 0.0;

    return command;
}

CommandMsg convertFromRosMsg(const geometry_msgs::Pose& msg)
{
    CommandMsg command;
    command.mask         = CommandMask::POSITION;
    command.position.x() = msg.position.x;
    command.position.y() = msg.position.y;
    command.position.z() = msg.position.z;

    command.orientation.w() = msg.orientation.w;
    command.orientation.x() = msg.orientation.x;
    command.orientation.y() = msg.orientation.y;
    command.orientation.z() = msg.orientation.z;

    command.linear_acceleration.x() = 0.0;
    command.linear_acceleration.y() = 0.0;
    command.linear_acceleration.z() = 0.0;
    return command;
}

CommandMsg convertFromRosMsg(const cascade_pid_controller_msgs::TrajCommand& msg)
{
    CommandMsg command;

    command.mask         = CommandMask::TRAJECTORY;
    command.position.x() = msg.position.x;
    command.position.y() = msg.position.y;
    command.position.z() = msg.position.z;

    command.linear_velocity.x() = msg.velocity.x;
    command.linear_velocity.y() = msg.velocity.y;
    command.linear_velocity.z() = msg.velocity.z;

    command.linear_acceleration.x() = msg.acceleration.x;
    command.linear_acceleration.y() = msg.acceleration.y;
    command.linear_acceleration.z() = msg.acceleration.z;

    // Get Quaternion from yaw
    tf2::Quaternion q;
    q.setRPY(0, 0, msg.yaw);
    q.normalize();

    command.orientation.w() = q.getW();
    command.orientation.x() = q.getX();
    command.orientation.y() = q.getY();
    command.orientation.z() = q.getZ();

    command.angular_velocity.z() = msg.yaw_dot;

    return command;
}

bool isSafeRosMsg(const geometry_msgs::PoseStamped& msg)
{
    if (!utils::isSafe(msg.pose.position.x) || !utils::isSafe(msg.pose.position.y)
        || !utils::isSafe(msg.pose.position.z)) {
        LOG_ERROR("Unsafe position in PoseStamped message.");
        return false;
    }

    if (!utils::isSafe(msg.pose.orientation.x) || !utils::isSafe(msg.pose.orientation.y)
        || !utils::isSafe(msg.pose.orientation.z) || !utils::isSafe(msg.pose.orientation.w)) {
        LOG_ERROR("Unsafe orientation (quaternion) in PoseStamped message.");
        return false;
    }

    return true;
}

bool isSafeRosMsg(const geometry_msgs::TwistStamped& msg)
{
    if (!utils::isSafe(msg.twist.linear.x) || !utils::isSafe(msg.twist.linear.y)
        || !utils::isSafe(msg.twist.linear.z)) {
        LOG_ERROR("Unsafe linear twist in TwistStamped message.");
        return false;
    }

    if (!utils::isSafe(msg.twist.angular.x) || !utils::isSafe(msg.twist.angular.y)
        || !utils::isSafe(msg.twist.angular.z)) {
        LOG_ERROR("Unsafe angular twist in TwistStamped message.");
        return false;
    }

    return true;
}

bool isSafeRosMsg(const cascade_pid_controller_msgs::TrajCommand& msg)
{
    if (!utils::isSafe(msg.velocity.x) || !utils::isSafe(msg.velocity.y) || !utils::isSafe(msg.velocity.z)) {
        LOG_ERROR("Unsafe linear velocity in TrajCommand message.");
        return false;
    }

    if (!utils::isSafe(msg.acceleration.x) || !utils::isSafe(msg.acceleration.y)
        || !utils::isSafe(msg.acceleration.z)) {
        LOG_ERROR("Unsafe linear acceleration in TrajCommand message.");
        return false;
    }

    if (!utils::isSafe(msg.position.x) || !utils::isSafe(msg.position.y) || !utils::isSafe(msg.position.z)) {
        LOG_ERROR("Unsafe position in TrajCommand message.");
        return false;
    }

    if (!utils::isSafe(msg.yaw)) {
        LOG_ERROR("Unsafe yaw in TrajCommand message.");
        return false;
    }

    if (!utils::isSafe(msg.yaw_dot)) {
        LOG_ERROR("Unsafe yaw dot in TrajCommand message.");
        return false;
    }

    return true;
}

} // namespace catec
