#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Range.h>
#include <camera_trigger_driver/PM3DGPSHeading.h>
#include <camera_trigger_driver/PM3DGPSData.h>
#include <LatLong-UTMconversion.h>
#include <iostream>
#include <math.h>

double lastNorthing, lastEasting, lastRange;
bool record;
float lGPSHeadingData;
float totalDistance = 0;
ros::WallTime previous_exposure_time;

ros::Publisher triggerPub;
ros::Publisher distancePub;
ros::Publisher gpsTriggerPub;

void gpsCallBack(const sensor_msgs::NavSatFixConstPtr& msg)
{
    double Northing, Easting;
    double d = 0.0;
    char z[4];
    std_msgs::Float64 distance;
    ros::NodeHandle srv_nh;
    ros::ServiceClient gpsHeadingClient = srv_nh.serviceClient<camera_trigger_driver::PM3DGPSHeading>("gps_heading_service");
    camera_trigger_driver::PM3DGPSData cameraTriggerMsg;

    // ROS_INFO("Received GPS position: %f %f", msg->latitude, msg->longitude);
    LLtoUTM(23, msg->latitude, msg->longitude, Northing, Easting, z);

    // ROS_INFO("In UTM %f %f %s", Northing, Easting, z);
    d = sqrt(pow(lastNorthing-Northing,2) + pow(lastEasting-Easting,2));
    totalDistance = d;
    distance.data = d;
    distancePub.publish(distance);
    float tempt_const = 0.07; // temporary variable changed from 10.0 (meters)
    float gps_heading_data;
     ROS_INFO("Printing distance %f",d);
    /*
    If d > 10m, then trigger and update last, though only if record switch is active
     Temporary threshold value set
     Final distance threshold would be read from a common launch file or config file
    */
    camera_trigger_driver::PM3DGPSHeadingRequest req;
    camera_trigger_driver::PM3DGPSHeadingResponse res;
    camera_trigger_driver::PM3DGPSData gps_msg;
    if(record && (d>tempt_const) && (((ros::WallTime::now() - previous_exposure_time).toNSec() * 1e-6)>2000))
    {
      std_msgs::Empty t;
      triggerPub.publish(t);
      ROS_INFO("------------Triggering Camera to take image------------");
      lastNorthing = Northing; lastEasting = Easting;
      previous_exposure_time = ros::WallTime::now();
      cameraTriggerMsg.latitude = msg->latitude;
      cameraTriggerMsg.longitude = msg->longitude;
      cameraTriggerMsg.gps_heading = lGPSHeadingData;
      cameraTriggerMsg.camera_trigger = true;
      cameraTriggerMsg.distance = totalDistance;
      gpsTriggerPub.publish(cameraTriggerMsg);
      
    }
    
      ROS_INFO("Received GPS Heading : %f",lGPSHeadingData);
      req.northing = Northing;
      req.lnorthing = lastNorthing;
      req.easting = Easting;
      req.leasting = lastEasting;

      
      if (gpsHeadingClient.call(req,res)){
      
      gps_heading_data = res.gps_heading;
      lGPSHeadingData = gps_heading_data;
      // ROS_INFO("Received GPS Heading : %f",gps_heading);
      
      }
      else {

      ROS_WARN("GPS HEADING SERVER NOT ONLINE...");
      }

    

     

}

void recordCallBack(const std_msgs::BoolConstPtr& msg)
{
    record = msg->data;
}

int main(int argc, char **argv)
{
  lastNorthing = 0;
  lastEasting = 0;
  lastRange = 0;
  previous_exposure_time = ros::WallTime::now();
  record = true;
  ros::init(argc, argv, "camera_trigger");
  ros::NodeHandle nh;
  ros::Subscriber gpsSub_ = nh.subscribe("/fix", 10, &gpsCallBack);
  triggerPub = nh.advertise<std_msgs::Empty>("Trigger", 32);
  gpsTriggerPub = nh.advertise<camera_trigger_driver::PM3DGPSData>("camera_trigger",3);
  distancePub = nh.advertise<std_msgs::Float64>("Distance", 32);
  
  ros::spin();
}

