#include <sstream>

// ROS标准库
#include "ros/ros.h"
// 订阅和发布图像话题消息
#include "image_transport/image_transport.h"
// 将ROS图像转换为openCV图像
#include "cv_bridge/cv_bridge.h"

// OpenCV2 GUI
#include "opencv2/highgui.hpp"

int main(int argc, char *argv[])
{
    if (argv[1] == nullptr)
    {
        ROS_INFO("Device ID required.");
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
        ROS_INFO("Cannot convert argument(videoSource) to integer.");
        return 1;
    }

    cv::VideoCapture cap(videoSource);
    // 检查设备能否打开
    if (!cap.isOpened())
    {
        ROS_INFO("Unable to open video source.");
        return 1;
    }

    cv::Mat frame;
    sensor_msgs::ImagePtr msg;

    ros::Rate loopRate(30);
    while (ros::ok())
    {
        cap >> frame;
        if (!frame.empty())
        {
            msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", frame).toImageMsg();
            pub.publish(msg);
            ROS_INFO("Sent a image");
        }
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}