#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus
import pynmea2


if __name__ == '__main__':
        
    # Initialize ROS node
    rospy.init_node('gps_node')

    gps_pub = rospy.Publisher('fix', NavSatFix, queue_size=10)

    
    log_file = open('gps_data.log', 'r')
    
    rate = rospy.Rate(1)
    
    for line in log_file:
        
        
        if line.startswith('$GNRMC'):
            try:
                
                # Parse the RMC sentence into a NMEA object
                rmc = pynmea2.parse(line)
                # Check if the GPS fix is valid
                if rmc.status == 'A':
                    # Create NavSatFix message and fill in data
                    gps_msg = NavSatFix()
                    gps_msg.latitude = rmc.latitude
                    gps_msg.longitude = rmc.longitude
                    gps_msg.altitude = 0.0
                    gps_msg.status.status = NavSatStatus.STATUS_FIX
                    gps_msg.status.service = NavSatStatus.SERVICE_GPS
                    gps_msg.header.stamp = rospy.Time.now()
                    # Publish the NavSatFix message
                    gps_pub.publish(gps_msg)

                    rospy.loginfo(f"{gps_msg}")
                    rate.sleep()
            except pynmea2.ChecksumError:
                continue

    log_file.close()
