//
// Created by cbbhuxx on 23-8-23.
//

#include "../include/processor.h"

typedef struct
{
    cv::Point2f left_top;
    cv::Point2f left_bottom;
    cv::Point2f right_top;
    cv::Point2f right_bottom;
}four_corners_t;

four_corners_t corners;

Processor::Processor(ros::NodeHandle nh_, std::string topic_loc_, std::string topic_map_, bool IShead)
{
    topic_loc = topic_loc_;
    topic_map = topic_map_;
    nh = nh_;
    IShead = IShead;
}

//定位图像变换之后的四个角点
void Processor::CalcCorners(const cv::Mat& H, const cv::Mat& src)
{
    //H为 变换矩阵  src为需要变换的图像
    //计算配准图的角点（齐次坐标系描述）
    double v2[] = { 0,0,1 }; //左上角
    double v1[3];  //变换后的坐标值
    //构成 列向量` 这种构成方式将 向量 与 Mat 关联， Mat修改 向量也相应修改
    cv::Mat V2 = cv::Mat(3, 1, CV_64FC1, v2);
    cv::Mat V1 = cv::Mat(3, 1, CV_64FC1, v1);
    V1 = H * V2; //元素*

    //左上角（转换为一般的二维坐标系）
    corners.left_top.x = v1[0] / v1[2];
    corners.left_top.y = v1[1] / v1[2];

    //左下角（0，src.rows，1）
    v2[0] = 0;
    v2[1] = src.rows;
    v2[2] = 1;
    V2 = cv::Mat(3, 1, CV_64FC1, v2);
    V1 = cv::Mat(3, 1, CV_64FC1, v1);  //列向量
    V1 = H * V2;

    corners.left_bottom.x = v1[0] / v1[2];
    corners.left_bottom.y = v1[1] / v1[2];

    //右上角（src.cols,0,1)
    v2[0] = src.cols;
    v2[1] = 0;
    v2[2] = 1;
    V2 = cv::Mat(3, 1, CV_64FC1, v2);
    V1 = cv::Mat(3, 1, CV_64FC1, v1);
    V1 = H * V2;

    corners.right_top.x = v1[0] / v1[2];
    corners.right_top.y = v1[1] / v1[2];

    //右下角(src.cols,src.rows,1)
    v2[0] = src.cols;
    v2[1] = src.rows;
    v2[2] = 1;
    V2 = cv::Mat(3, 1, CV_64FC1, v2);  //列向量
    V1 = cv::Mat(3, 1, CV_64FC1, v1);  //列向量
    V1 = H * V2;

    corners.right_bottom.x = v1[0] / v1[2];
    corners.right_bottom.y = v1[1] / v1[2];
}

cv::Point2f Processor::build_newMap(const nav_msgs::Odometry::ConstPtr& loc_, const cv::Mat img_)
{
    // 根据当前帧的机器人方位，以及栅格地图的大小,旋转地图
    // 计算旋转矩阵
    Eigen::Quaterniond t_Q;
    t_Q.x() = loc_->pose.pose.orientation.x;
    t_Q.y() = loc_->pose.pose.orientation.y;
    t_Q.z() = loc_->pose.pose.orientation.z;
    t_Q.w() = loc_->pose.pose.orientation.w;
    cv::Mat tempMat;
    cv::eigen2cv(t_Q.toRotationMatrix(), tempMat);      // 后续可能要加逆运算

    // 计算四个角
    CalcCorners(tempMat, img_);

    cv::warpPerspective(img_, *XL_map_ptr, tempMat,
                        cv::Size(MAX(corners.right_top.x, corners.right_bottom.x), MAX(corners.left_bottom.y, corners.right_bottom.y)));

    cv::Point2f center;
    center.x = (corners.right_top.x + corners.left_top.x) / 2.0;
    center.y = (corners.right_top.y + corners.right_bottom.y) / 2.0;

    return center;
}

void Processor::mergecallback(const nav_msgs::Odometry::ConstPtr& loc, const nav_msgs::OccupancyGrid::ConstPtr& map)
{
    // 栅格地图转cv图像，并垂直翻转
    cv::Mat img = cv::Mat( map->info.height, map->info.width, CV_8SC1,
                           const_cast<signed char*>(map->data.data()) );
    // 垂直翻转
//    cv::Mat img_f;
//    cv::flip(img, img_f, 0);

    // 判断是否为头车，若为头车建立大图，最后将以头车的其实位置为坐标原点
    if(IShead)
    {
        start_center = build_newMap(loc,img);     // 初始点在图像中的位置。
        IShead = false;
        ROS_DEBUG_STREAM("起始点x=" << this->start_center.x << "起始点y=" << this->start_center.y);

        // 下面需要跳出该次回调
        // TODO

    }

    // 判断是否存在大图
    if(XL_map_ptr)
    {
        x_ = (int)(loc->pose.pose.position.x+start_center.x-map->info.width/2.0);
        y_ = (int)(loc->pose.pose.position.y+start_center.y-map->info.height/2.0);
        cv::Mat imageRIO = (*XL_map_ptr)(cv::Rect(x_, y_, map->info.width, map->info.height));
        merge(imageRIO,img);
    }
}


bool Processor::merge(const cv::Mat& image1, const cv::Mat& image2)
{
    //初始化ORB类和特征点向量
//    cv::Ptr<cv::Feature2D> siftFeature = cv::SIFT::create(2000);

    cv::Ptr<cv::ORB> fastFeature = cv::ORB::create();
    std::vector<cv::KeyPoint> keyPoint1,keyPoint2;
    fastFeature->detect(image1,keyPoint1);
    fastFeature->detect(image2,keyPoint2);

    //计算特征向量描述符
    cv::Mat descor1,descor2;
    fastFeature->compute ( image1, keyPoint1, descor1);
    fastFeature->compute ( image2, keyPoint2, descor2);

    // 互最近邻匹配
    cv::FlannBasedMatcher matcher;
    std::vector<cv::DMatch> matches;
    matcher.match(descor1, descor2, matches);

    // 绘制匹配点
    cv::Mat oriMatchRes;
    cv::drawMatches(image1, keyPoint1, image2, keyPoint2, matches, oriMatchRes, cv::Scalar(0, 255, 0), cv::Scalar::all(-1));
    cv::imshow("orign match img", oriMatchRes);

    // 筛选较好的匹配点
    double sum = 0;
    double maxDist = 0;
    double minDist = 0;
    for (auto &match : matches)
    {
        double dist = match.distance;
        maxDist = std::max(maxDist, dist);
        minDist = std::min(minDist, dist);
    }

    std::vector<cv::DMatch> goodMatches;
    std::vector<cv::Point2f> imagePoints1, imagePoints2;
    double threshold = 0.5;
    for (auto &match : matches)
    {
        if (match.distance < threshold * maxDist)
            imagePoints1.push_back(keyPoint1[match.queryIdx].pt);
        imagePoints2.push_back(keyPoint2[match.trainIdx].pt);
        goodMatches.push_back(match);
    }

    //图像2到1的映射，3*3转换阵
    cv::Mat transMat = cv::findHomography(imagePoints2,imagePoints1,cv::RANSAC);
    CalcCorners(transMat, image2);
    cv::Mat imageTransform2;
    warpPerspective(image2, imageTransform2, transMat, cv::Size(MAX(corners.right_top.x, corners.right_bottom.x), MAX(corners.left_bottom.y, corners.right_bottom.y)));

    // 将大图扩充



    imageTransform2.copyTo((*XL_map_ptr)(cv::Rect(x_, y_, imageTransform2.cols, imageTransform2.rows)));
//    image02.copyTo(dst(Rect(0, 0, image02.cols, image02.rows)));


    return true;
}


void Processor::topicSubscribing()
{
    message_filters::Subscriber<nav_msgs::Odometry> pos_sub(nh, topic_loc, 1);
    message_filters::Subscriber<nav_msgs::OccupancyGrid> map_sub(nh, topic_map, 1);
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,
            nav_msgs::OccupancyGrid> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), pos_sub, map_sub);
    sync.registerCallback(boost::bind(&Processor::mergecallback,this, _1, _2));
}

