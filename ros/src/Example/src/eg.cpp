#include "ros/ros.h"
// 头文件位于/opt/ros/noetic/include/**

int main(int argc, char *argv[])
{
    /* 第一步：执行ROS节点初始化
     * ros::init([传递命令行参数],节点名称)
     */
    ros::init(argc, argv, "example");

    /* 第二步（可选）：创建ROS节点句柄
     */
    ros::NodeHandle n;

    ROS_INFO("hello world!");

    /* 离别时：报平安
     */
    return 0;
}
