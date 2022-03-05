// ROS标准库
#include "ros/ros.h"
// CvBridge库
#include "cv_bridge/cv_bridge.h"
// ROS图像编码
#include "sensor_msgs/image_encodings.h"
// 订阅和发布图像话题消息
#include "image_transport/image_transport.h"

// OpenCV2标准库
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// 定义输入窗口名称
static const std::string INPUT = "Input";
// 定义输出窗口名称
static const std::string OUTPUT = "Output";

// 定义一个转换的类
class RGB_GRAY
{
private:
    ros::NodeHandle nh;                    // ROS句柄
    image_transport::ImageTransport it;    // image_transport实例
    image_transport::Subscriber image_sub; // ROS图像接收器
    // image_transport::Publisher image_pub; // ROS图像发布器
public:
    void image_process(cv::Mat& img)
    { // 图像处理主函数
        cv::Mat img_out;
        cv::cvtColor(img, img_out, CV_RGB2GRAY);
        cv::imshow(INPUT,img);
        cv::imshow(OUTPUT, img_out);
        cv::waitKey(5);
    }
    void convert_callback(const sensor_msgs::ImageConstPtr &msg)
    {                                 /* sensor_msgs/Image图像格式转换->cv::Mat */
        cv_bridge::CvImagePtr cv_ptr; // 声明一个CvImage指针
        try
        { // 将ROS消息中的图像信息提取，生成新CV类型的图像，复制给CvImage指针
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        }
        catch (const cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        image_process(cv_ptr->image);// 传给处理函数
    }
    RGB_GRAY() : it(nh)
    {
        image_sub = it.subscribe("camera/rgb/image_raw", 1, &RGB_GRAY::convert_callback, this);
        // image_pub = it.publishe("",1);
        cv::namedWindow(INPUT);
        cv::namedWindow(OUTPUT);
    }
    ~RGB_GRAY()
    {
        cv::destroyWindow(INPUT);
        cv::destroyWindow(OUTPUT);
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "RGB");
    RGB_GRAY obj;
    ros::spin();
}