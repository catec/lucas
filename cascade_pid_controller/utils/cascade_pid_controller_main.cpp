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

#include <cascade_pid_controller/nodes/CascadePidControllerNode.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cascade_pid_controller_node");

    cascade_pid_controller::CascadePidControllerNode cascade_pid_controller;

    ros::spin();

    return 0;
}