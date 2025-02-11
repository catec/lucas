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

#include <chrono>
#include <memory>

#include "catec_control_manager/common/log_manager.h"
#include "catec_control_manager/ros_nodes/control_manager_node.h"

// Get current date/time, format is YYYY_MM_DD-HH_mm_ss
const std::string currentDateTime()
{
    time_t    now = time(0);
    struct tm tstruct;
    char      buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y_%m_%d-%H_%M_%S", &tstruct);
    return buf;
}

int main(int argc, char** argv)
{
    const std::string log_file_name{
            "/home/" + std::string(std::getenv("USER")) + "/.catec/logger/control_manager/" + currentDateTime()
            + ".log"};

    const auto        console_level{spdlog::level::info};
    const auto        file_level{spdlog::level::debug};
    const std::size_t n_log_files{50};
    LogManager::instance().initialize(log_file_name, console_level, file_level, n_log_files);

    /// \note: ROS2 stuff
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto                control_manager_node = std::make_shared<catec::ControlManagerNode>(options);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(control_manager_node);
    executor.spin();

    rclcpp::shutdown();
    LogManager::instance().shutdown();
    return 0;
}
