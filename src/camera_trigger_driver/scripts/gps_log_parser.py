#!/usr/bin/env python3

import os
os.chdir(os.path.dirname(os.path.abspath(__file__)))
import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus
import pynmea2


from camera_trigger_driver.msg import PM3DGPSData

if __name__ == '__main__':
        
    # Initialize ROS node
    rospy.init_node('gps_node')

    gps_pub = rospy.Publisher('fix', NavSatFix, queue_size=10)

    test_pub = rospy.Publisher('gps_data_test',PM3DGPSData,queue_size=1)
    
    log_file = open('gps_data.log', 'r')
    
    rate = rospy.Rate(3)
    
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

                    rospy.loginfo(f"Printing Lat : {gps_msg.latitude} and Lon : {gps_msg.longitude}")
                    # Publish the NavSatFix message
                    gps_pub.publish(gps_msg)
                    # test_gps = PM3DGPSData()
                    # test_gps.lat_and_lon = 1
                    # test_gps.gps_heading = 1

                    # test_pub.publish(test_gps)
                    rate.sleep()
            except pynmea2.ChecksumError:
                continue

    log_file.close()
