//
// Created by cbbhuxx on 23-8-23.
//

#ifndef MERGE_MAP_PROCESSOR_H
#define MERGE_MAP_PROCESSOR_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "opencv2/opencv.hpp"
#include <opencv2/core/eigen.hpp>


class Processor
{
private:
    int x_,y_;
    std::string topic_loc;
    std::string topic_map;
    ros::NodeHandle nh;
    bool IShead;

    static cv::Point2f start_center;
    static cv::Mat* XL_map_ptr;

public:
    Processor(ros::NodeHandle nh_, std::string topic_loc_, std::string topic_map_, bool IShead=false);
    void topicSubscribing();     // 同时订阅位置话题，和栅格地图话题
    void mergecallback(const nav_msgs::Odometry::ConstPtr& loc,const nav_msgs::OccupancyGrid::ConstPtr& map);       //
    void CalcCorners(const cv::Mat& H, const cv::Mat& src);
    bool merge(const cv::Mat& image1, const cv::Mat& image2);
    cv::Point2f build_newMap(const nav_msgs::Odometry::ConstPtr& loc_, const cv::Mat img_);
};

#endif //MERGE_MAP_PROCESSOR_H

