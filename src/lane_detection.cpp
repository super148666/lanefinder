#include <lane_detection.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>

LaneDetector::LaneDetector(ros::NodeHandle n):
n(n),rate(10)
{
    has_left_lane = false;
    has_right_lane = false;
}

LaneDetector::~LaneDetector() {

}

void LaneDetector::Config() {
    std::string package_name = "lanefinder";
    std::string parameter_name;
    std::string *str_ptr = nullptr;
    int *int_ptr = nullptr;
    double *double_ptr = nullptr;
    bool *bool_ptr = nullptr;

    bool status = true;
    parameter_name = "image_topic_name";
    str_ptr = &image_topic_name;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *str_ptr);

    parameter_name = "point_topic_name";
    str_ptr = &point_topic_name;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *str_ptr);

    parameter_name = "transform_parameter_path";
    str_ptr = &transform_parameter_path;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *str_ptr);
    ROS_INFO_STREAM("trans path:"<<transform_parameter_path);

    parameter_name = "processing_range_x_up";
    double_ptr = &processing_range_x_up;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *double_ptr);
    ROS_INFO_STREAM("processing range x up:"<<processing_range_x_up<<"|"<<*double_ptr);

    parameter_name = "processing_range_x_down";
    double_ptr = &processing_range_x_down;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *double_ptr);
    ROS_INFO_STREAM("processing range x down:"<<processing_range_x_down<<"|"<<*double_ptr);

    parameter_name = "processing_range_y_up";
    double_ptr = &processing_range_y_up;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *double_ptr);
    ROS_INFO_STREAM("processing range y up:"<<processing_range_y_up<<"|"<<*double_ptr);

    parameter_name = "processing_range_y_down";
    double_ptr = &processing_range_y_down;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *double_ptr);
    ROS_INFO_STREAM("processing range y down:"<<processing_range_y_down<<"|"<<*double_ptr);

    parameter_name = "mag_threshold_up";
    double_ptr = &mag_threshold_up;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *double_ptr);
    ROS_INFO_STREAM("mag_threshold_up:"<<mag_threshold_up<<"|"<<*double_ptr);

    parameter_name = "mag_threshold_down";
    double_ptr = &mag_threshold_down;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *double_ptr);
    ROS_INFO_STREAM("mag_threshold_down:"<<mag_threshold_down<<"|"<<*double_ptr);

    parameter_name = "dir_threshold_up";
    double_ptr = &dir_threshold_up;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *double_ptr);
    ROS_INFO_STREAM("dir_threshold_up:"<<dir_threshold_up<<"|"<<*double_ptr);

    parameter_name = "dir_threshold_down";
    double_ptr = &dir_threshold_down;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *double_ptr);
    ROS_INFO_STREAM("dir_threshold_down:"<<dir_threshold_down<<"|"<<*double_ptr);

    parameter_name = "asx_threshold_up";
    double_ptr = &asx_threshold_up;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *double_ptr);
    ROS_INFO_STREAM("asx_threshold_up:"<<asx_threshold_up<<"|"<<*double_ptr);

    parameter_name = "asx_threshold_down";
    double_ptr = &asx_threshold_down;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *double_ptr);
    ROS_INFO_STREAM("asx_threshold_down:"<<asx_threshold_down<<"|"<<*double_ptr);

    parameter_name = "asy_threshold_up";
    double_ptr = &asy_threshold_up;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *double_ptr);
    ROS_INFO_STREAM("asy_threshold_up:"<<asy_threshold_up<<"|"<<*double_ptr);

    parameter_name = "asy_threshold_down";
    double_ptr = &asy_threshold_down;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *double_ptr);
    ROS_INFO_STREAM("asy_threshold_down:"<<asy_threshold_down<<"|"<<*double_ptr);

    parameter_name = "sat_threshold_up";
    double_ptr = &sat_threshold_up;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *double_ptr);
    sat_threshold_up *= 255.0;
    ROS_INFO_STREAM("sat_threshold_up:"<<sat_threshold_up<<"|"<<*double_ptr);

    parameter_name = "sat_threshold_down";
    double_ptr = &sat_threshold_down;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *double_ptr);
    sat_threshold_down *= 255.0;
    ROS_INFO_STREAM("sat th down:"<<sat_threshold_down);

    ROS_INFO_STREAM("number of lane points:" << number_of_lane_points);
    parameter_name = "number_of_lane_points";
    int_ptr = &number_of_lane_points;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *int_ptr);
    ROS_INFO_STREAM("number of lane points:" << number_of_lane_points << "|" << *int_ptr);

    parameter_name = "x_search_range";
    int_ptr = &x_search_range;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *int_ptr);

    parameter_name = "min_road_width";
    int_ptr = &min_road_width;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, *int_ptr);

    //lambda = cv::getPerspectiveTransform(camera_points, map_points);
    has_new_image = false;
    image_sub = n.subscribe(image_topic_name,1,&LaneDetector::image_received_callback, this);
    while(!has_new_image)
    {
        ros::spinOnce();
    }
    image_cols = raw_image.cols;
    image_rows = raw_image.rows;
    double raw_image_cols = (1.0-processing_range_x_down)*(image_cols-0)/(processing_range_x_up-processing_range_x_down);
    y_increment = (image_rows-1) / (number_of_lane_points);
    ROS_INFO_STREAM("rows:"<<image_rows<<'|'<<"cols:"<<image_cols<<'|'<<y_increment);
    has_new_image = false;
    lane_pub = n.advertise<visualization_msgs::MarkerArray>(point_topic_name, 1);

    parameter_name = "camera_point_param_1";
    double cam_param_1 = 0;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, cam_param_1);
    parameter_name = "camera_point_param_2";
    double cam_param_2 = 0;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, cam_param_2);
    parameter_name = "camera_point_param_3";
    double cam_param_3 = 0;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, cam_param_3);
    parameter_name = "camera_point_param_4";
    double cam_param_4 = 0;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, cam_param_4);

    parameter_name = "map_point_param_1";
    double map_param_1 = 0;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, map_param_1);
    parameter_name = "map_point_param_2";
    double map_param_2 = 0;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, map_param_2);
    parameter_name = "map_point_param_3";
    double map_param_3 = 0;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, map_param_3);
    parameter_name = "map_point_param_4";
    double map_param_4 = 0;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, map_param_4);

    parameter_name = "transform_x_up";
    double transform_x_up = 0;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, transform_x_up);

    parameter_name = "transform_x_down";
    double transform_x_down = 0;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, transform_x_down);

    parameter_name = "transform_y_up";
    double transform_y_up = 0;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, transform_y_up);

    parameter_name = "transform_y_down";
    double transform_y_down = 0;
    status = status && n.getParam("/" + package_name + "/" + parameter_name, transform_y_down);

    double scale_1 = (cam_param_1-cam_param_2)/(map_param_1-map_param_2);
    double scale_2 = (cam_param_3-cam_param_4)/(map_param_3-map_param_4);
    double mid_1 = (cam_param_1+cam_param_2)/2.0;
    double mid_2 = (cam_param_3+cam_param_4)/2.0;
    double scale_up = (scale_1-scale_2)/(transform_y_up-transform_y_down)*(processing_range_y_up-transform_y_up)+scale_1;
    double scale_down = (scale_1-scale_2)/(transform_y_up-transform_y_down)*(processing_range_y_down-transform_y_up)+scale_1;
    double mid_up = (mid_1-mid_2)/(transform_y_up-transform_y_down)*(processing_range_y_up-transform_y_up)+mid_1;
    double mid_down = (mid_1-mid_2)/(transform_y_up-transform_y_down)*(processing_range_y_down-transform_y_up)+mid_1;

    ROS_INFO_STREAM("c1:"<<(0.5*scale_up+mid_up)<<"|c2:"<<(-0.5*scale_up+mid_up)<<"|c3:"<<(0.5*scale_down+mid_down)<<"|c4:"<<(-0.5*scale_down+mid_down));
    camera_points.emplace_back(raw_image_cols*(0.5*scale_up+mid_up), image_rows);
    camera_points.emplace_back(raw_image_cols*(-0.5*scale_up+mid_up), image_rows);
    camera_points.emplace_back(raw_image_cols*(0.5*scale_down+mid_down), 0);
    camera_points.emplace_back(raw_image_cols*(-0.5*scale_down+mid_down), 0);

    map_points.emplace_back(raw_image_cols, image_rows);
    map_points.emplace_back(0, image_rows);
    map_points.emplace_back(raw_image_cols, 0);
    map_points.emplace_back(0, 0);
    lambda = cv::getPerspectiveTransform(camera_points, map_points);
    r_lambda = cv::getPerspectiveTransform(map_points, camera_points);
    has_left_lane = false;
    has_right_lane = false;
}

void LaneDetector::Start() {
    double time_used[6];
    double time_sum[6];
    int count = 0;
    while(ros::ok())
    {
        ros::spinOnce();
        if (!has_new_image)
            continue;
            
		auto time_1 = ros::Time::now();
		
        cv::Mat mask(edge());
        
        auto time_2 = ros::Time::now();
        
        //cv::imshow("mask", transformed_display_image);
        //cv::waitKey(0);
        cv::warpPerspective(mask, mask, lambda, mask.size());
        cv::warpPerspective(transformed_display_image, transformed_display_image, lambda, transformed_display_image.size());
        
        auto time_3 = ros::Time::now();
        
        int nRows = mask.rows;
        int nCols = mask.cols;

        left_lane.clear();
        right_lane.clear();
        
        left_lane.reserve(number_of_lane_points);
        right_lane.reserve(number_of_lane_points);
        int left_search_center, right_search_center;

        std::vector<int> histogram;

        if (has_left_lane)
        {
            left_search_center = previous_left_start;
            //ROS_INFO("has left lane");
        } else {

            histogram.resize(nCols/2,0);
            double *p_mask;
            for(int i = nRows; i > nRows*(2/3); i--)
            {
                p_mask = mask.ptr<double>(i);

                for (int j = 0; j < nCols/2; ++j)
                {
                    histogram[j] += (int) p_mask[j];
                }
            }

            int max=0;
            int index=-999;
            for (int i = 0; i < histogram.size(); i++)
            {
                if (histogram[i] > max)
                {
                    max = histogram[i];
                    index = i;
                }
            }
            left_search_center = index;
        }

        if (has_right_lane)
        {
            //ROS_INFO("has right lane");
            right_search_center = previous_right_start;
        } else {
            histogram.resize(nCols, 0);
            double *p_mask;
            for (int i = nRows; i > nRows*(2/3); i--) {
                p_mask = mask.ptr<double>(i);

                for (int j = nCols/2; j < nCols; ++j) {
                    histogram[j] += (int) p_mask[j];
                }
            }

            int max=0;
            int index=-999;
            for (int i = histogram.size()/2; i < histogram.size(); i++)
            {
                if (histogram[i] > max)
                {
                    max = histogram[i];
                    index = i;
                }
            }
            right_search_center = index;
        }
		
		auto time_4 = ros::Time::now();
        
        //ROS_INFO_STREAM("lc:"<<left_search_center<<"|rc:"<<right_search_center);
        has_left_lane = get_lane_points(mask, left_search_center, left_lane);
        has_right_lane = get_lane_points(mask, right_search_center, right_lane);
        auto time_5 = ros::Time::now();
        
        cv::imshow("transformed", transformed_display_image);
        cv::waitKey(1);


        if (!left_lane.empty()) {
            cv::perspectiveTransform(left_lane, left_lane, r_lambda);
            cv::Vec3b left_color(255, 0, 0);
            for (auto iter = left_lane.begin(); iter != left_lane.end(); iter++) {
                cv::circle(raw_image, *iter, 3, left_color, -1);
            }
            previous_left_start = left_search_center;
        }

        if (!right_lane.empty()) {
            cv::perspectiveTransform(right_lane, right_lane, r_lambda);
            cv::Vec3b right_color(0, 0, 255);
            for (auto iter = right_lane.begin(); iter != right_lane.end(); iter++) {
                cv::circle(raw_image, *iter, 3, right_color, -1);
            }
            previous_right_start = right_search_center;
        }

        publish();
        has_new_image = false;
		
		auto time_6 = ros::Time::now();
        
        cv::imshow("lane",raw_image);
        cv::waitKey(1);
        
        count += 1;
        time_used[0] = (time_2 - time_1).toSec();
        time_used[1] = (time_3 - time_2).toSec();
        time_used[2] = (time_4 - time_3).toSec();
        time_used[3] = (time_5 - time_4).toSec();
        time_used[4] = (time_6 - time_5).toSec();
        time_used[5] = (time_6 - time_1).toSec();
        for (int i=0; i<6; i++)
        {
			time_sum[i] += time_used[i];
		}
		
		/*
        ROS_INFO_STREAM("Time used:\n"<<
						"edge:	"<<time_used[0]<<'\n'<<
						"tran:	"<<time_used[1]<<'\n'<<
						"hist:	"<<time_used[2]<<'\n'<<
						"getl:	"<<time_used[3]<<'\n'<<
						"pub:	"<<time_used[4]<<'\n'<<
						"total:	"<<time_used[5]<<'\n');
		*/
						
		std::cout<<"Average Time used:\n"<<
						"edge:	"<<time_sum[0]/count<<'\n'<<
						"tran:	"<<time_sum[1]/count<<'\n'<<
						"hist:	"<<time_sum[2]/count<<'\n'<<
						"getl:	"<<time_sum[3]/count<<'\n'<<
						"pub:	"<<time_sum[4]/count<<'\n'<<
						"total:	"<<time_sum[5]/count<<'\n'<<
						"rate:	"<<1.0/(time_sum[5]/count)<<'\n';
    }
    
    
}

void LaneDetector::publish() {
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker marker_t;
    marker_t.header.frame_id="laser";
    marker_t.header.stamp=timestamp;
    marker_t.action=visualization_msgs::Marker::ADD;
    marker_t.type=visualization_msgs::Marker::CYLINDER;
    marker_t.color.r = 1.0;
    marker_t.scale.x = 0.2;
    marker_t.scale.y = 0.2;
    marker_t.scale.z = 0.5;
    marker_t.lifetime = ros::Duration(rate);
    int i=0;
    for(;i<left_lane.size();i++)
    {
        visualization_msgs::Marker marker = marker_t;
        tf::poseTFToMsg(tf::Transform(tf::createQuaternionFromRPY(0.0, 0.0, 0.0),
                                      tf::Vector3(left_lane[i].x, left_lane[i].y, 0.0)),
                        marker.pose);
        marker.id = i;
        markers.markers.push_back(marker);
    }
    lane_pub.publish(markers);
}

void LaneDetector::Stop() {

}

void LaneDetector::image_received_callback(const sensor_msgs::ImageConstPtr msg) {
    try
    {
        auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        raw_image = cv_ptr->image.clone();
        cv::imshow("raw_image",raw_image);
        cv::waitKey(1);
        raw_image = raw_image(cv::Rect(
                (int)(raw_image.cols*processing_range_x_down),
                (int)(raw_image.rows*processing_range_y_down),
                (int)(raw_image.cols*processing_range_x_up)-(int)(raw_image.cols*processing_range_x_down),
                (int)(raw_image.rows*processing_range_y_up)-(int)(raw_image.rows*processing_range_y_down))).clone();
        cv::cvtColor(raw_image, gray_image, CV_BGR2GRAY);
        timestamp = msg->header.stamp;
        has_new_image = true;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

}

cv::Mat LaneDetector::edge() {
    cv::Mat sx, sy, hls, planes[3];
    //cv::GaussianBlur( raw_image, raw_image, cv::Size( 15, 15 ),0);
    cv::cvtColor(raw_image, hls, CV_RGB2HLS);
    cv::split(hls, planes);
    sx = cv::abs(sx);
    sy = cv::abs(sy);
    cv::Sobel(gray_image, sx, CV_64F, 1, 0);
    cv::Sobel(gray_image, sy, CV_64F, 0, 1);
    cv::normalize(sx, sx, 0, 1, cv::NORM_MINMAX);
    cv::normalize(sy, sy, 0, 1, cv::NORM_MINMAX);
    cv::Mat mag(sx.rows, sx.cols, sx.type());
    cv::Mat asx = mag.clone(), asy = mag.clone(), dir = mag.clone(), sat = mag.clone(), out = mag.clone();
    transformed_display_image = cv::Mat(sx.rows, sx.cols, CV_8UC3, cv::Scalar(0,0,0));
    int channels = sx.channels();

    int nRows = sx.rows;
    int nCols = sx.cols * channels;
    if (sx.isContinuous())
    {
        nCols *= nRows;
        nRows = 1;
    }
    int i,j;
    double *p_sx, *p_sy;
    double *p_mag, *p_asx, *p_asy, *p_dir, *p_sat_out, *p_out;
    uint8_t *p_sat;
    cv::Vec3b *p_tran;
    double magnitude, direction;
    for( i = 0; i < nRows; ++i)
    {
        p_sx = sx.ptr<double>(i);
        p_sy = sy.ptr<double>(i);
        p_sat = planes[2].ptr<uint8_t >(i);
        p_asx = asx.ptr<double>(i);
        p_asy = asy.ptr<double>(i);
        p_mag = mag.ptr<double>(i);
        p_dir = dir.ptr<double>(i);
        p_sat_out = sat.ptr<double>(i);
        p_out = out.ptr<double>(i);
        p_tran = transformed_display_image.ptr<cv::Vec3b>(i);


        for ( j = 0; j < nCols; ++j)
        {
            //ROS_INFO_STREAM("psx:"<<p_sy[j]);
            //asx threshold
            if (p_sx[j] > asx_threshold_down && p_sx[j] < asx_threshold_up)
            {
                p_asx[j] = 1.0;
            } else {
                p_asx[j] = 0.0;
            }

            //asy threshold
            if ((p_sy[j] > asy_threshold_down) && (p_sy[j] < asy_threshold_up))
            {
                p_asy[j] = 1.0;
            } else {
                p_asy[j] = 0.0;
            }

                //mag threshold
            magnitude = sqrt(p_sx[j]*p_sx[j] + p_sy[j]*p_sy[j]) / sqrt(2.0);
            if (magnitude > mag_threshold_down && magnitude < mag_threshold_up)
            {
                p_mag[j] = 1.0;
            } else {
                p_mag[j] = 0.0;
            }

            //dir threshold
            direction = atan2(p_sy[j], p_sx[j]);
            if (direction > dir_threshold_down && direction < dir_threshold_up)
            {
                p_dir[j] = 1.0;
            } else {
                p_dir[j] = 0.0;
            }

            //sat threshold
            if (p_sat[j] > sat_threshold_down && p_sat[j] < sat_threshold_up)
            {
                p_sat_out[j] = 1.0;
            } else {
                p_sat_out[j] = 0.0;
            }

//          left camera
//            if ((p_mag[j]==1.0) && (p_sat_out[j]==1.0))

//          right camera
            if ((p_mag[j]==1.0) || (p_asx[j]==1.0 && p_asy[j]==1.0))
            {
                p_out[j] = 1.0;
                p_tran[j] = cv::Vec3b(255,255,255);
            } else {
                p_out[j] = 0.0;
            }

        }
    }

//    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
//                                                cv::Size(2 * 1 + 1, 2 * 1 + 1),
//                                                cv::Point(1, 1));
//    cv::erode(out, out, element);
//    cv::dilate(out, out, element);
//    cv::imshow("sx",sx);
//    cv::imshow("sy",sy);
//    cv::imshow("mag",mag);
//    cv::imshow("asx",asx);
//    cv::imshow("asy",asy);
//    cv::imshow("dir",dir);
//    cv::imshow("sat",sat);
//    cv::imshow("out",out);
//    cv::waitKey(1);

    return out;
}

bool LaneDetector::get_lane_points(cv::Mat mask, int &search_center, std::vector<cv::Point2f> &lane_point) {
    if (search_center < 0) {
        lane_point.clear();
        return false;
    }
    lane_point.resize(number_of_lane_points,cv::Point2f(-100,-100));

    int nRows = mask.rows;
    int nCols = mask.cols;
    int mid_x = nCols / 2;
    int half_width = min_road_width/2;
    if (search_center < mid_x+(half_width) && search_center > mid_x-(half_width)) {
        lane_point.clear();
        return false;
    }
    char l_or_r = 'r';
    if (search_center < mid_x) {
        l_or_r = 'l';
    }

    bool start = true;
    int valid_count = 0;
    int center_x = search_center, center_y = nRows-1 - y_increment/2;
    int count = 0;
    int y = nRows - 1;
    for (; count < number_of_lane_points; count++)
    {
        int x;

        x = center_x - x_search_range/2;
        if (x<0) x=-x;
        std::vector<int> x_hist;
        x_hist.resize(x_search_range);
        std::vector<int> y_hist;
        y_hist.resize(y_increment);
        int sum_half = 0;
        int y_count = 0;
        double *p_mask;
        cv::Vec3b *p_tran;
        for(int i=y; i > y-y_increment; i--)
        {
            p_mask = mask.ptr<double>(i);
            p_tran = transformed_display_image.ptr<cv::Vec3b>(i);
            int x_count = 0;
            for (int j=x; j<x+x_search_range; j++)
            {
                if (p_mask[j] == 1.0) {
                    x_hist[x_count] += 1;
                    y_hist[y_count] += 1;
                    sum_half += 1;
                    p_tran[j] = cv::Vec3b(0,0,255);
                }
                x_count += 1;
            }
            y_count += 1;
        }

        sum_half = sum_half/2;
        if (sum_half > 0) {
            valid_count ++;
            int accum = 0;
            bool valid = false;

            for (int i = 0; i < y_hist.size(); i++) {
                accum += y_hist[i];
                if (accum >= sum_half) {
                    center_y = y - i;
                    valid = true;
                    break;
                }
            }
            if (!valid)
                center_y = y - y_increment/2;

            accum = 0;
            for (int i = 0; i < x_hist.size(); i++) {
                accum += x_hist[i];
                if (accum >= sum_half) {
                    center_x = x + i;
                    break;
                }
            }
        } else {
            center_y = y - y_increment/2;
        }


        if (start) {
            if (l_or_r == 'l') {
                if (center_x > mid_x-half_width) {
                    center_x = mid_x-half_width;
                }
            }
            else {
                if (center_x < mid_x+half_width) {
                    center_x = mid_x+half_width;
                }
            }
            search_center = center_x;
            start = false;
        }

        lane_point[count].x = center_x;
        lane_point[count].y = center_y;

        cv::rectangle(transformed_display_image, cv::Point(x,y), cv::Point(x+x_search_range, y-y_increment), cv::Scalar(255,0,0), 1);

        y -= y_increment;
    }

    //ROS_INFO_STREAM("valid "<<(int)(((double)valid_count) / ((double)count)*100.0)<<'%');
    return ( ((double)valid_count) / ((double)count) > 0.6);

}


