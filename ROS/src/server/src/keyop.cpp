#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <signal.h>
#include <termios.h>

namespace KeyOp
{
    /////////////////////////////////////////// 参数
    double linear_max = 1, angular_max = 15;

    /////////////////////////////////////////// 全局变量
    termios origin; // 命令行原先的状态

    /////////////////////////////////////////// 定义
    enum class KEYCODE
    {
        W = 0x77,
        A = 0x61,
        S = 0x73,
        D = 0x64,
        Q = 0x71,
        SPACE = 0x20,
    };

    /////////////////////////////////////////// 函数
    void quit(int)
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &origin);
        ros::shutdown();
        exit(0);
    }

    void start(ros::NodeHandle &nh)
    {
        tcgetattr(STDIN_FILENO, &origin); // 储存命令行原先状态
        signal(SIGINT, quit);             // 注册处理Ctrl+C的函数

        // 原生模式
        termios raw;
        memcpy(&raw, &origin, sizeof(termios));
        raw.c_lflag &= ~(ICANON | ECHO); // 立即读取、不回显
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);

        auto pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        geometry_msgs::Twist twist;
        char c;
        bool hasChanged;

        puts("Press W, A, S, D to move,\nSPACE to stop.");
        puts("---------------------------");

        while (true)
        {
            // get the next event from the keyboard
            if (read(STDIN_FILENO, &c, 1) < 0)
            {
                perror("read():");
                exit(-1);
            }
            ROS_DEBUG("value: %#02X", c);

            hasChanged = true;
            switch (static_cast<KEYCODE>(c))
            {
            case KEYCODE::A:
                twist.linear.x = 0;
                twist.angular.z = angular_max;
                break;
            case KEYCODE::D:
                twist.linear.x = 0;
                twist.angular.z = -angular_max;
                break;
            case KEYCODE::W:
                twist.linear.x = linear_max;
                twist.angular.z = 0;
                break;
            case KEYCODE::S:
                twist.linear.x = -linear_max;
                twist.angular.z = 0;
                break;
            case KEYCODE::SPACE:
                twist.linear.x = 0;
                twist.angular.z = 0;
                break;
            default:
                hasChanged = false;
                break;
            }

            if (hasChanged == true)
                pub.publish(twist);
        }
    }

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "keyop", ros::InitOption::NoSigintHandler);
    ros::NodeHandle nh;

    KeyOp::start(nh);

    return 0;
}