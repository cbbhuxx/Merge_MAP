//
// Created by cbbhuxx on 23-8-21.
//

#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include "../include/processor.h"
#include <thread>

// 创建多个对象，并接受设置参数
std::vector<Processor> objects;

// 分割输入近来的字符串
std::vector<std::string> splist(std::string input)
{
    std::vector<std::string> tokens;
    std::istringstream iss(input);
    std::string token;

    while (std::getline(iss, token, ';'))
        tokens.push_back(token);

    return tokens;
}

void threadFunc(int threadId)
{
    std::cout << "线程 " << threadId << " 执行中..." << std::endl;
    objects[threadId].topicSubscribing();

}

int main(int argc, char **argv)
{
    ROS_INFO("");
    ros::init(argc,argv,"merge_map");
    ros::NodeHandle nh;
    int num_robot;
    int head_numeber;
    std::string topics_loc;
    std::string topics_map;
    std::string IShead_;

    nh.param("num_robot", num_robot, 3);            // 机器人总数
    nh.param<std::string>("IShead_", IShead_, "1;0;0");      // 1代表为主车，0代表不是主车
    nh.param<std::string>("topics_loc", topics_loc, "loc0;loc1;loc2");  // 所有机器人位置话题，用；隔开
    nh.param<std::string>("topics_map", topics_map, "map0;map1;map2");  // 所有机器人栅格地图话题，用；隔开

    ROS_INFO("机器人总数 = %d, 主机器人编号 = %d, 所有位置话题 = %s, 所有栅格地图话题 = %s", num_robot, head_numeber, topics_loc.c_str(), topics_map.c_str());

    std::vector<std::string> loc_tokens;
    std::vector<std::string> map_tokens;
    std::vector<std::string> IShead_tokens;
    loc_tokens = splist(topics_loc);
    map_tokens = splist(topics_map);
    IShead_tokens = splist(IShead_);

    for (int i = 0; i < num_robot; i++)
        objects.emplace_back(nh,loc_tokens[i],map_tokens[i],IShead_tokens[i]);

    // 创建 num_robot 个线程
    std::vector<std::thread> threads;
    for (int i = 0; i < num_robot; i++)
        threads.emplace_back(threadFunc, i);

    ros::spin();

    // 等待所有线程执行完毕
    for (auto& thread : threads)
        thread.join();

    return 0;
}
