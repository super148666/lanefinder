#include <lane_detection.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"lane_detection");
    ros::NodeHandle n;
    LaneDetector lane_detector(n);
    lane_detector.Config();
    lane_detector.Start();
    return 0;
}