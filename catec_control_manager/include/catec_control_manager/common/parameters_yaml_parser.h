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

#include <yaml-cpp/yaml.h>

#include <filesystem>

#include "types.h"

namespace catec {
class ParametersYamlParser
{
  public:
    ParametersYamlParser() {}
    ParametersYamlParser(const std::string& yaml_path) : yaml_path(yaml_path)
    {
        if (!std::filesystem::exists(yaml_path)) {
            throw std::runtime_error("Yaml file " + yaml_path + " does not exist!");
        }

        try {
            yaml_node = YAML::LoadFile(yaml_path);
        } catch (const YAML::ParserException& e) {
            throw std::runtime_error("Yaml file " + yaml_path + " is malformed!\n" + e.what());
        }

        parse();
    }
    ~ParametersYamlParser() = default;

  public:
    std::string yaml_path;
    YAML::Node  yaml_node;

    bool            use_rangefinder;
    unsigned int    odom_checker_timer_interval;
    unsigned int    imu_checker_timer_interval;
    unsigned int    pwm_checker_timer_interval;
    unsigned int    controller_checker_timer_interval;
    unsigned int    rangefinder_checker_timer_interval;
    unsigned int    generic_reference_checker_timer_interval;
    unsigned int    check_mode_changed_max_attemps;
    unsigned int    check_mode_changed_wait_ms;
    float           state_thread_job_rate;
    float           vel_module_min_to_command_hover;
    GoingToWpConfig going_to_wp_config;
    AssistedConfig  assisted_config;
    TakingOffConfig takingoff_config;
    LandingConfig   landing_config;

  private:
    void throwYamlParsingError(const std::string& field_name, const std::string& error_msg)
    {
        throw std::runtime_error("Error parsing " + field_name + " from YAML file " + yaml_path + ":\n" + error_msg);
    }

    template <typename T>
    void parseYamlNode(YAML::Node node, const std::string& key, T& value)
    {
        try {
            value = node[key].as<T>();
        } catch (const YAML::Exception& e) {
            throwYamlParsingError(key, e.what());
        }
    }

    void parse()
    {
        parseYamlNode(yaml_node, "use_rangefinder", use_rangefinder);

        parseYamlNode(yaml_node, "odom_checker_timer_interval", odom_checker_timer_interval);
        parseYamlNode(yaml_node, "imu_checker_timer_interval", imu_checker_timer_interval);
        parseYamlNode(yaml_node, "pwm_checker_timer_interval", pwm_checker_timer_interval);
        parseYamlNode(yaml_node, "controller_checker_timer_interval", controller_checker_timer_interval);
        parseYamlNode(yaml_node, "rangefinder_checker_timer_interval", rangefinder_checker_timer_interval);
        parseYamlNode(yaml_node, "generic_reference_checker_timer_interval", generic_reference_checker_timer_interval);
        parseYamlNode(yaml_node, "check_mode_changed_max_attemps", check_mode_changed_max_attemps);
        parseYamlNode(yaml_node, "check_mode_changed_wait_ms", check_mode_changed_wait_ms);
        parseYamlNode(yaml_node, "state_thread_job_rate", state_thread_job_rate);

        parseYamlNode(yaml_node["going_to_wp_config"], "v_max", going_to_wp_config.v_max);
        parseYamlNode(yaml_node["going_to_wp_config"], "a_max", going_to_wp_config.a_max);
        parseYamlNode(yaml_node["going_to_wp_config"], "yaw_dot_max", going_to_wp_config.yaw_dot_max);
        parseYamlNode(yaml_node["going_to_wp_config"], "position_error_th", going_to_wp_config.position_error_th);
        parseYamlNode(yaml_node["going_to_wp_config"], "yaw_error_th", going_to_wp_config.yaw_error_th);

        parseYamlNode(yaml_node["assisted_config"], "v_max", assisted_config.v_max);
        parseYamlNode(yaml_node["assisted_config"], "yaw_dot_max", assisted_config.yaw_dot_max);

        parseYamlNode(yaml_node["takingoff_config"], "v_max", takingoff_config.v_max);
        parseYamlNode(yaml_node["takingoff_config"], "height_min", takingoff_config.height_min);
        parseYamlNode(yaml_node["takingoff_config"], "acc", takingoff_config.acc);
        parseYamlNode(yaml_node["takingoff_config"], "deacc", takingoff_config.deacc);
        parseYamlNode(yaml_node["takingoff_config"], "height_error_th", takingoff_config.height_error_th);

        parseYamlNode(yaml_node["landing_config"], "v_landing", landing_config.v_landing);
        parseYamlNode(yaml_node["landing_config"], "height_prelanding", landing_config.height_prelanding);
        parseYamlNode(
                yaml_node["landing_config"], "height_error_th_prelanding", landing_config.height_error_th_prelanding);
    }
};
} // namespace catec