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
#include "opencv2/opencv.hpp"

class processor
{
private:
    static cv::Mat XL_map;
    std::string topic_loc;
    std::string topic_map;
    ros::NodeHandle nh;
public:
    processor(std::string topic_loc_, std::string topic_map_);
    void topicSubscribing();     // 同时订阅位置话题，和栅格地图话题
    void mergecallback(const nav_msgs::Odometry::ConstPtr& loc,const nav_msgs::OccupancyGrid::ConstPtr& map);       //
    void CalcCorners(const Mat& H, const Mat& src);
    cv::Mat merge(const cv::Mat& image1, const cv::Mat& image2);
};

#endif //MERGE_MAP_PROCESSOR_H
