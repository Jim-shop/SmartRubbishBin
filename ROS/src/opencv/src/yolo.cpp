#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include "opencv2/core.hpp"
#include "opencv2/dnn.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

/** YOLOv3 : 416 x 416
 *  YOLOv3-tiny : 320 x 320
 */

class YOLO
{
public:
    YOLO()
    {
        std::ifstream ifs("src/opencv/res/coco.names");
        std::string line;
        while (std::getline(ifs, line))
            className.push_back(line);
        // net = cv::dnn::readNetFromDarknet("yolov3-tiny.cfg", "yolov3-tiny.weights");
        net = cv::dnn::readNetFromDarknet("src/opencv/res/yolov3.cfg", "src/opencv/res/yolov3.weights");
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
    void detect(cv::Mat &frame)
    {
        // 预处理
        cv::Mat blob = cv::dnn::blobFromImage(
            frame,                         // 被处理图像
            1. / 255,                      // 将图像值域压缩到0~1
            cv::Size(inpWidth, inpHeight), // 尺寸缩放到网络大小
            cv::Scalar(0, 0, 0),           // 减平均值
            true,                          // 交换RB通道
            false                          // 不裁减图像
        );
        // 将预处理的图像输入网络
        net.setInput(blob);
        // 获取输出
        std::vector<cv::Mat> outs;
        net.forward(outs, net.getUnconnectedOutLayersNames()); // YOLOV3输出3个尺度的结果
        // 后处理
        postProcess(frame, outs);

        // 显示处理时间
        double freq = cv::getTickFrequency() / 1000;
        std::vector<double> layersTimes;
        double t = net.getPerfProfile(layersTimes) / freq;
        std::string label = cv::format("%.2f ms / frame", t);
        cv::putText(frame, label, cv::Point(0, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255));
    }

private:
    float confThreshold = 0.5; // 类别置信度阈值
    float nmsThreshold = 0.4;  // 非极大值抑制重叠率阈值
    int inpWidth = 416;        // 输入图像宽度
    // int inpWidth = 320;                 // 输入图像宽度
    int inpHeight = 416; // 输入图像高度
    // int inpHeight = 320;                // 输入图像高度
    std::vector<std::string> className; // 类别名称
    cv::dnn::Net net;                   // 网络
    // 后处理
    void postProcess(cv::Mat &frame, std::vector<cv::Mat> const &outs)
    {
        std::vector<int> classIds;
        std::vector<float> confidences;
        std::vector<cv::Rect> boxes;
        // 置信度过滤：将置信度小于confThreshold的框过滤掉
        for (const auto &out : outs)
        { // 对每个尺度的输出
            for (int j = 0; j < out.rows; ++j)
            {
                /** 输出格式为N行85列，N表示识别出的框数，85列分别为：
                 *  [x,y,w,h,conf,score1,socre2,...score80]
                 *  80个score是因为用的是coco数据集，支持区分出80类
                 */
                // 取出置信度部分
                cv::Mat scores = out.row(j).colRange(5, out.cols);
                // 获取置信度中最大值和最大值索引
                double confidence;
                cv::Point classIdPoint;
                cv::minMaxLoc(scores, nullptr, &confidence, nullptr, &classIdPoint);
                // 如果最大的置信度大于阈值，则记录该框
                if (confidence > confThreshold)
                { // x,y,w,h都是相对于模型输入图像的比例，这里换算成原始图像
                    int centerX = out.at<float>(j, 0) * frame.cols;
                    int centerY = out.at<float>(j, 1) * frame.rows;
                    int width = out.at<float>(j, 2) * frame.cols;
                    int height = out.at<float>(j, 3) * frame.rows;
                    int left = centerX - width / 2;
                    int top = centerY - height / 2;

                    classIds.push_back(classIdPoint.x);
                    confidences.push_back((float)confidence);
                    boxes.push_back(cv::Rect(left, top, width, height));
                }
            }
        }
        // 非极大值抑制：将重叠率大于nmsThreshold的框过滤掉
        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices); // 返回满足条件的下标
        // 绘制结果
        for (const int &idx : indices)
        {
            cv::Rect box = boxes[idx];
            // 在识别区域画一个框
            cv::rectangle(frame, box, cv::Scalar(255, 0, 0), 3);
            // 在框上方写出类别名称和置信度
            std::string label = className[classIds[idx]] + cv::format(": %.2f", confidences[idx]);
            cv::putText(frame, label, cv::Point(box.x, box.y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255));
        }
    }
} yolo;

void CallBack(const sensor_msgs::ImageConstPtr &msg)
{
    auto imp = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    yolo.detect(imp->image);
    cv::imshow("", imp->image);
    cv::waitKey(1);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "yolo");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/image", 1, CallBack);
    ros::spin();
    return 0;
}