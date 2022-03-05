#include <sstream>

// ROS标准库
#include "ros/ros.h"
// CvBridge库
#include "cv_bridge/cv_bridge.h"
// 订阅和发布图像话题消息
#include "image_transport/image_transport.h"

// OpenCV2标准库
#include "opencv2/highgui/highgui.hpp"

int main(int argc, char *argv[])
{
    if (argv[1] == nullptr)
    {
        ROS_INFO("Video source required.");
        return 1;
    }

    ros::init(argc, argv, "imgPublisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);

    // 将命令行输入的信号源转换为数字
    std::istringstream videoSourceCmd(argv[1]);
    int videoSource;
    if (!(videoSourceCmd >> videoSource))
    {
        ROS_INFO("Invalid video source input.");
        return 1;
    }

    cv::VideoCapture cap(videoSource);
    // 检查设备能否打开
    if (!cap.isOpened())
    {
        ROS_INFO("Cannot open video device.");
        return 1;
    }

    cv::Mat frame;
    sensor_msgs::ImagePtr msg;

    ros::Rate loopRate(60);
    while (nh.ok())
    {
        cap >> frame;
        if (!frame.empty())
        {
            msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", frame).toImageMsg();
            pub.publish(msg);
        }
    }

    ros::spinOnce();
    loopRate.sleep();
}