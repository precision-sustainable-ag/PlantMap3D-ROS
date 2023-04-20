#!/usr/bin/env python3

import os
os.chdir(os.path.dirname(os.path.abspath(__file__)))
import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus
import pynmea2
import sys

log_file = open('gps_data.log', 'r')

from camera_trigger_driver.msg import PM3DGPSData

def shutdown():
    
    rospy.signal_shutdown("Shutting down GPS publisher")
    log_file.close()
    sys.exit()
    
    
if __name__ == '__main__':
        
    # Initialize ROS node
    rospy.init_node('gps_node')
    rospy.on_shutdown(shutdown)
    gps_pub = rospy.Publisher('fix', NavSatFix, queue_size=10)

    test_pub = rospy.Publisher('gps_data_test',PM3DGPSData,queue_size=1)
    
    
    # log_file = open('gps_data.log', 'r')
    rospy.loginfo("Waiting 13 secs to start all nodes")
    rospy.sleep(13)
    rate = rospy.Rate(3)
    while not rospy.is_shutdown():
        
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
                        
                        rate.sleep()
                except pynmea2.ChecksumError:
                    continue

        log_file.close()
        rospy.spin()
