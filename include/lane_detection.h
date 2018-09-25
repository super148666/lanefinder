#ifndef LANE_DETECTION_H
#define LANE_DETECTION_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class LaneDetector{
public:
    LaneDetector(ros::NodeHandle n);
    ~LaneDetector();
    void Config();
    void Start();
    void Stop();

private:
    void image_received_callback(const sensor_msgs::ImageConstPtr msg);
    bool get_lane_points(cv::Mat mask, int& search_center, std::vector<cv::Point2f>& lane_point);
    void publish();
    cv::Mat edge();
    ros::NodeHandle n;
    ros::Publisher lane_pub;
    ros::Subscriber image_sub;
    std::string image_topic_name;
    std::string point_topic_name;
    std::string transform_parameter_path;
    std::vector<cv::Point2f> camera_points;
    std::vector<cv::Point2f> map_points;
    double map_x_up;
    double map_x_down;
    double map_y_up;
    double map_y_down;

    double processing_range_x_up;
    double processing_range_x_down;
    double processing_range_y_up;
    double processing_range_y_down;

    double sampling_distance;

    cv::Mat raw_image;
    cv::Mat gray_image;
    cv::Mat transformed_display_image;

    double mag_threshold_up;
    double mag_threshold_down;
    double dir_threshold_up;
    double dir_threshold_down;
    double asx_threshold_up;
    double asx_threshold_down;
    double asy_threshold_up;
    double asy_threshold_down;
    double sat_threshold_up;
    double sat_threshold_down;

    std::vector<cv::Point2f> left_lane;
    std::vector<cv::Point2f> right_lane;
    int previous_left_start;
    int previous_right_start;
    int number_of_lane_points;
    int y_increment;
    int x_search_range;
    bool has_left_lane;
    bool has_right_lane;
    ros::Rate rate;
    cv::Mat lambda;
    cv::Mat r_lambda;

    int image_rows;
    int image_cols;

    bool has_new_image;

    ros::Time timestamp;

    int min_road_width;

};


#endif
