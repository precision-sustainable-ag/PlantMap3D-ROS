#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Range.h>
#include <LatLong-UTMconversion.h>
#include <iostream>
#include <math.h>

double lastNorthing, lastEasting, lastRange;
bool record;
ros::WallTime previous_exposure_time;

ros::Publisher triggerPub;
ros::Publisher distancePub;


void gpsCallBack(const sensor_msgs::NavSatFixConstPtr& msg)
{
    double Northing, Easting, d;
    char z[4];
    std_msgs::Float64 distance;
     // ROS_INFO("Received GPS position: %f %f", msg->latitude, msg->longitude);
    LLtoUTM(23, msg->latitude, msg->longitude, Northing, Easting, z);
    // ROS_INFO("In UTM %f %f %s", Northing, Easting, z);
    d = sqrt(pow(lastNorthing-Northing,2) + pow(lastEasting-Easting,2));
    distance.data = d;
    distancePub.publish(distance);
    float tempt_const = 0.07; // temporary variable changed from 10.0 (meters)

    ROS_INFO("Printing distance %f",d);
    //If d > 10m, then trigger and update last, though only if record switch is active
    if(record && (d>tempt_const) && (((ros::WallTime::now() - previous_exposure_time).toNSec() * 1e-6)>2000))
    {
      std_msgs::Empty t;
      triggerPub.publish(t);
      ROS_INFO("------------Triggering-------------------------");
      lastNorthing = Northing; lastEasting = Easting;
      previous_exposure_time = ros::WallTime::now();
    }
//    lastNorthing = Northing; lastEasting = Easting;
    // ROS_INFO("Distance since last in meters %f", d);
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
  distancePub = nh.advertise<std_msgs::Float64>("Distance", 32);
  ros::spin();
}

