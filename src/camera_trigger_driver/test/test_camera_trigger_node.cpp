#ifndef TEST_CAMERA_TRIGGER_NODE_CPP
#define TEST_CAMERA_TRIGGER_NODE_CPP

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <computeGPSDistance.h>

/*NOTE : TO RUN THIS FILE */
/* rosrun ros_wrappers test_camera_trigger_node */
TEST(GPSDistanceTest,ComputeDistance){

    double expectedDistance = 810.57;
    double lastNorthing = 549312.45;
    double lastEasting = 231567.92;
    double Northing = 548710.23;
    double Easting = 231024.78;

    double compute_distance = computeDistance(Northing,lastNorthing,Easting,lastEasting);
    ASSERT_NEAR(expectedDistance, compute_distance,0.5);
}

TEST(GPSDistanceTest2,ComputeDistance){

    double expectedDistance = 782.99;
    double lastNorthing = 552450.89;
    double lastEasting = 230500.45;
    double Northing = 552100.67;
    double Easting = 229800.11;

    double compute_distance = computeDistance(Northing,lastNorthing,Easting,lastEasting);
    ASSERT_NEAR(expectedDistance, compute_distance,0.5);
}


int main(int argc, char** argv){

    ros::init(argc,argv,"test_camera_trigger");
    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}

#endif // TEST_CAMERA_TRIGGER_NODE_CPP