#!/usr/bin/env python3

##---------------------------------------------------------------------------------------------------------------------
##  LUCAS: Lightweight framework for UAV Control And Supervision
##---------------------------------------------------------------------------------------------------------------------
##  Copyright 2024 CATEC (Advanced Centre for Aerospace Technologies)
##---------------------------------------------------------------------------------------------------------------------
## This program is free software: you may redistribute it and/or modify it under the terms of the GNU General Public
## License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
## version.
## This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
## warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
## You should have received a copy of the GNU General Public License along with this program. If not, see
## https://www.gnu.org/licenses/.
##---------------------------------------------------------------------------------------------------------------------
import rospy
from mavros_msgs.msg import RCIn
import numpy as np
from geometry_msgs.msg import Twist

MAX_VEL_X = 0.5
MAX_VEL_Y = 0.5
MAX_VEL_Z = 0.5
MAX_YAWRATE = 15.0*(np.pi/180.0)

class RCSubscriberNode:
    def __init__(self):
        
        rospy.Subscriber("/mavros/rc/in", RCIn, self.__rc_callback)

        # Create a publisher for the specified topic
        self.twist_ref_pub = rospy.Publisher("/catec_control_manager/assisted_ref", Twist, queue_size=1)

        rospy.loginfo("RC Subscriber Node Initialized")
        
        self.__pitch_min = 1094
        self.__pitch_max = 1930
        
        self.__roll_min = 1097
        self.__roll_max = 1934
        
        self.__throttle_min = 1094
        self.__throttle_max = 1926
        
        self.__yaw_rate_min = 1094
        self.__yaw_rate_max = 1934
    
    def __map_channel(self, raw_value, raw_min, raw_max, ref_min, ref_max):
        """
        Map a value from one range to another using linear interpolation.

        Parameters:
        - raw_value: The input value to be mapped.
        - raw_min: The minimum value of the input range.
        - raw_max: The maximum value of the input range.
        - ref_min: The minimum value of the output range.
        - ref_max: The maximum value of the output range.

        Returns:
        - The mapped value in the output range.
        """
        # Ensure raw_value is within the specified range
        raw_value = max(min(raw_value, raw_max), raw_min)

        # Linear mapping formula
        mapped_value = ref_min + (ref_max - ref_min) * (raw_value - raw_min) / (raw_max - raw_min)

        return mapped_value
        
    def __rc_callback(self, msg):
        if len(msg.channels)<1:
            rospy.logwarn("Not receiving RC")
            return
        # Callback function to handle received messages
        # rospy.loginfo("Channels: {}".format(msg.channels))
        roll_ch_raw     = float(msg.channels[0])
        pitch_ch_raw    = float(msg.channels[1])
        throttle_ch_raw = float(msg.channels[2])
        yawrate_ch_raw  = float(msg.channels[3])
        
        vx_ref = self.__map_channel(pitch_ch_raw, self.__pitch_min, self.__pitch_max, 1, -1)
        vy_ref = self.__map_channel(roll_ch_raw, self.__roll_min, self.__roll_max, 1, -1)
        vz_ref = self.__map_channel(throttle_ch_raw, self.__throttle_min, self.__throttle_max, 0, 1)
        yawrate_ref = self.__map_channel(yawrate_ch_raw, self.__yaw_rate_min, self.__yaw_rate_max, 1, -1)

        ref = Twist()

        if vx_ref > -0.05 and vx_ref < 0.05:
            ref.linear.x = 0.0
        else:
            ref.linear.x = vx_ref*MAX_VEL_X

        if vy_ref > -0.05 and vy_ref < 0.05:
            ref.linear.y = 0.0
        else:
            ref.linear.y = vy_ref*MAX_VEL_Y

        if yawrate_ref > -0.05 and yawrate_ref < 0.05:
            ref.angular.z = 0.0
        else:
            ref.angular.z = yawrate_ref*MAX_YAWRATE
        
        if vz_ref > 0.4 and vz_ref < 0.6:
            ref.linear.z = 0.0
        elif vz_ref >= 0.6:
            ref.linear.z = MAX_VEL_Z*(vz_ref-0.6)/0.4
        elif vz_ref <= 0.4:
            ref.linear.z = MAX_VEL_Z*(vz_ref-0.4)/0.4

        self.twist_ref_pub.publish(ref)


if __name__ == '__main__':

    rospy.init_node('rc_subscriber_node', anonymous=True)
    
    # Instantiate the RCSubscriberNode class
    rc_subscriber = RCSubscriberNode()
    
    while not rospy.is_shutdown():
        # Spin to keep the script alive and handle callbacks
        rospy.spin()
