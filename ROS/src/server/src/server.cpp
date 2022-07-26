#include "ros/ros.h"
#include "ros/time.h"
#include "serial/serial.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include "std_msgs/Bool.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include <thread>
#include <vector>
#include <initializer_list>

float d_wheel = 0.21;
std_msgs::Bool pull_trigger;

std_msgs::Bool state; // the message from arduino which indicates whether the car is turning over the trash bin

void triggerCallback(const std_msgs::Bool::ConstPtr recv_trigger)
{
    pull_trigger.data = recv_trigger->data;
}

class ArduinoSerial
{
private:                                                // 参数
    const std::string port{"/dev/ttyUSB1"};             // 端口号（以后可以改成端口别名）
    const unsigned int baud{57600};                     // 波特率
    const unsigned int timeout{5000};                   // 延时ms
    const serial::parity_t parity{serial::parity_even}; // 偶校验
    const serial::bytesize_t bytesize{serial::eightbits};
    const serial::stopbits_t stopbits{serial::stopbits_one};
    const serial::flowcontrol_t flowcontrol{serial::flowcontrol_none};

private: // 内部工作辅助
    union Convertor
    {
        float f;
        uint8_t i[sizeof(f)];
    }; // 按字节读取器
    serial::Serial s;
    // serial::Serial s{port, baud, serial::Timeout::simpleTimeout(timeout), bytesize, parity, stopbits, flowcontrol};

public:
    ArduinoSerial()
    {
        s.setPort(port);
        s.setBaudrate(baud);
        auto tout = serial::Timeout::simpleTimeout(timeout);
        s.setTimeout(tout);
        s.setParity(parity);
        s.setBytesize(bytesize);
        s.setStopbits(stopbits);
        s.setFlowcontrol(flowcontrol);
        try
        {
            s.open(); // 打开串口通信
        }
        catch (serial::IOException &e)
        {
            ROS_ERROR("Fail to open serial port.");
            exit(-1);
        }

        while (!s.isOpen())
        {
            ROS_ERROR("Fail to initialize serial port. Retrying...");
            sleep(1);
            refresh();
        }
    }

    ~ArduinoSerial()
    {
        s.close();
    }

    enum DataType : uint8_t
    {
        Twist = 0xa0,
        Bool = 0xa1,
    };

    void send(DataType dataType, std::initializer_list<uint8_t> const &il)
    {
        uint8_t sentBuf[3 + il.size() + 2];
        uint8_t &startCode1 = sentBuf[0];
        uint8_t &startCode2 = sentBuf[1];
        uint8_t &typeCode = sentBuf[2];
        uint8_t &data[il.size()] = &sentBuf[3];
        uint8_t &checkCode1 = sentBuf[3 + il.size() + 0];
        uint8_t &checkCode2 = sentBuf[3 + il.size() + 1];

        startCode1 = 0x55, startCode2 = 0xaa, typeCode = dataType;
        for (data : il)
            sentBuf[++pos] = data;
        checkCode1 = checkCode2 = 0x55;
        for (pos = 0; pos < 3 + il.size(); pos++)
            sentBuf[3 + il.size()]
    }
    void send(std::initializer_list<float> const &il)
    {
        uint8_t check_code = 0xff; // 校验码
        s.write(&check_code, 1);   // 开始标志
        Convertor convertor;
        for (const float &data : il)
        {
            convertor.f = data;
            s.write(convertor.i, sizeof(convertor.i));
            for (const uint8_t &byte : convertor.i)
                check_code ^= byte; // 按顺序抑或，得到校验码
        }
        s.write(&check_code, 1); // 发送校验码
        s.flushOutput();         // 发送输出缓冲区中的所有数据
    }

    std::vector<float> read(const size_t &num)
    {
        if (s.available() < num * sizeof(float) + 2) // num个float+1个起始+1个校验
            return {};
        uint8_t check_code;
        s.read(&check_code, 1);
        if (check_code != 0xff) // 开始标志不符合
            return {};
        Convertor convertors[num];
        for (Convertor &convertor : convertors)
        {
            s.read(convertor.i, sizeof(convertor.i));
            for (const uint8_t &byte : convertor.i)
                check_code ^= byte;
        }
        uint8_t rev_check_code; // 接收的校验码
        s.read(&rev_check_code, 1);
        if (check_code != rev_check_code) // 校验码不对
            return {};
        std::vector<float> ret;
        for (const Convertor &convertor : convertors)
            ret.push_back(convertor.f);
        s.flushInput();
        return ret;
    }

    void refresh()
    {
        s.close();
        s.open();
    }
};

class OdomHandler
{
private: // 参数
    const std::string publishTopicName = "odom";
    const size_t queueSize = 10;
    const size_t interval = 10000; // 轮询间隔us

private: // 内部辅助
    float velocity1, velocity2, x_final, y_final, th_final;
    ArduinoSerial &as;
    ros::Publisher pub;
    nav_msgs::Odometry odom;
    tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_trans;
    geometry_msgs::Quaternion odom_quat;

    bool receive()
    {
        auto ret = as.read(5);
        if (ret.size() != 5)
            return false;
        x_final = ret[0];
        y_final = ret[1];
        th_final = ret[2];
        velocity1 = ret[3];
        velocity2 = ret[4];
        std::cout << "x_final: " << x_final << std::endl
                  << "y_final: " << y_final << std::endl
                  << "th_final: " << th_final << std::endl
                  << "velocity1: " << velocity1 << std ::endl
                  << "velocity2: " << velocity2 << std ::endl
                  << std::endl;
        usleep(interval);
        return true;
    }
    void publish()
    {
        // ros::Time now = ros::Time::now();

        // // odom_quat.x = 0.0;
        // // odom_quat.y = 0.0;
        // // odom_quat.z = sin(th_final / 2);
        // // odom_quat.w = cos(th_final / 2);
        // geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_final);

        // // tf
        // odom_trans.header.stamp = now;
        // odom_trans.header.frame_id = "odom";
        // odom_trans.child_frame_id = "base_link";
        // odom_trans.transform.translation.x = x_final;
        // odom_trans.transform.translation.y = y_final;
        // odom_trans.transform.translation.z = 0;
        // odom_trans.transform.rotation = odom_quat;
        // odom_broadcaster.sendTransform(odom_trans);

        // // nav
        // odom.header.stamp = now;
        // odom.header.frame_id = "odom";
        // odom.child_frame_id = "base_link";
        // odom.pose.pose.position.x = x_final;
        // odom.pose.pose.position.y = y_final;
        // odom.pose.pose.position.z = 0;
        // odom.pose.pose.orientation = odom_quat;
        // double averspeed = (velocity1 + velocity2) / 2;
        // odom.twist.twist.linear.x = averspeed * cos(th_final);
        // odom.twist.twist.linear.y = averspeed * sin(th_final);
        // odom.twist.twist.angular.z = (velocity2 - velocity1)/d_wheel ;
        // pub.publish(odom);
        ros::NodeHandle action_node;
        ros::Publisher action_publisher = action_node.advertise<std_msgs::Bool>("action_trigger", 10);
    }
    void looper()
    {
        while (true)
            if (receive())
                publish();
    }

public:
    OdomHandler(ros::NodeHandle &n, ArduinoSerial &as) : as{as}
    {
        pub = n.advertise<nav_msgs::Odometry>(publishTopicName, queueSize);
        std::thread(&OdomHandler::looper, this).detach();
    }
};

class TwistHandler
{
private: // 参数
    const std::string subscribeTopicName = "cmd_vel";
    const size_t queueSize = 10;

private: // 内部辅助
    ArduinoSerial &as;
    float linear_x, linear_y, angular_z;

    void callback(const geometry_msgs::Twist::ConstPtr &vel_cmd)
    {
        as.send({
            (float)vel_cmd->linear.x,
            (float)vel_cmd->linear.y,
            (float)vel_cmd->angular.z,
        });
    }

public:
    TwistHandler(ros::NodeHandle &n, ArduinoSerial &as) : as{as}
    {
        auto sub = new ros::Subscriber(n.subscribe<geometry_msgs::Twist>(subscribeTopicName, queueSize, &TwistHandler::callback, this));
    }
};

int main(int argc, char *argv[])
{
    pull_trigger.data = false;
    ros::init(argc, argv, "contact_arduino");
    ros::NodeHandle n;

    ros::Subscriber trigger_subscriber = n.subscribe<std_msgs::Bool>("pull_trigger", 10, triggerCallback);

    ArduinoSerial as;

    TwistHandler th(n, as);
    // OdomHandler oh(n, as);    --------------------I changed because it doesn't need the help from here  7.25

    ros::spin();
    return 0;
}