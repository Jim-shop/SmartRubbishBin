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

std_msgs::Bool state; // the message from arduino which indicates whether the car is turning over the trash bin

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

    void send(DataType dataType, std::vector<uint8_t> const &bytes)
    {
        uint8_t sentBuf[3 + bytes.size() + 2];
        uint8_t &startCode1 = sentBuf[0];
        uint8_t &startCode2 = sentBuf[1];
        uint8_t &typeCode = sentBuf[2];
        uint8_t *dataCode = &sentBuf[3];
        uint8_t &checkCode1 = sentBuf[3 + bytes.size() + 0];
        uint8_t &checkCode2 = sentBuf[3 + bytes.size() + 1];

        startCode1 = 0x55, startCode2 = 0xaa, typeCode = dataType;
        checkCode1 = 0x55 ^ 0xaa ^ dataType;
        checkCode2 = 0x55 + 0xaa + dataType;
        for (uint8_t const &data : bytes)
        {
            *dataCode++ = data;
            checkCode1 ^= data;
            checkCode2 += data;
        }

        s.write(sentBuf, sizeof(sentBuf));
        s.flushOutput();
    }

    template <typename T>
    std::vector<uint8_t> toBytes(std::vector<T> const &Ts)
    {
        union // 按字节读取器
        {
            T t;
            uint8_t b[sizeof(t)];
        } convertor;
        std::vector<uint8_t> ret;
        for (T const &data : Ts)
        {
            convertor.t = data;
            for (const uint8_t &byte : convertor.b)
                ret.push_back(byte);
        }
        return ret;
    }

    template <typename T>
    std::vector<T> toTs(std::vector<uint8_t> const &bytes)
    {
        union
        {
            T t;
            uint8_t b[sizeof(t)];
        } convertor;
        std::vector<T> ret;
        for (size_t pos = 0; pos < bytes.size(); pos += sizeof(convertor))
        {
            for (int i = 0; i < sizeof(convertor); i++)
                convertor.b[i] = bytes[pos + i];
            ret.push_back(convertor.t);
        }
        return ret;
    }

    std::vector<uint8_t> read(const size_t &num)
    {
        if (s.available() < 2 + num + 2) // num+2个起始+2个校验
            return {};
        uint8_t startCode1;
        s.read(&startCode1, 1);
        if (startCode1 != 0x55) // 开始标志不符合
            return {};
        uint8_t startCode2;
        s.read(&startCode2, 1);
        if (startCode2 != 0xaa)
            return {};
        uint8_t readBuf[num];
        s.read(readBuf, num);
        uint8_t checkCode1 = 0x55 ^ 0xaa, checkCode2 = 0x55 + 0xaa;
        for (const uint8_t &byte : readBuf)
            checkCode1 ^= byte, checkCode2 += byte;
        uint8_t recvCheckCode1;
        s.read(&recvCheckCode1, 1);
        if (recvCheckCode1 != checkCode1) // 校验码不对
        {
            s.flushInput();
            return {};
        }
        uint8_t recvCheckCode2;
        s.read(&recvCheckCode2, 1);
        if (recvCheckCode2 != checkCode2)
        {
            s.flushInput();
            return {};
        }
        return std::vector<uint8_t>(readBuf, readBuf + num);
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
    const float d_wheel = 0.21;

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
        auto ret = as.toTs<float>(as.read(5 * sizeof(float)));
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
        ros::Time now = ros::Time::now();

        // odom_quat.x = 0.0;
        // odom_quat.y = 0.0;
        // odom_quat.z = sin(th_final / 2);
        // odom_quat.w = cos(th_final / 2);
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_final);

        // tf
        odom_trans.header.stamp = now;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = x_final;
        odom_trans.transform.translation.y = y_final;
        odom_trans.transform.translation.z = 0;
        odom_trans.transform.rotation = odom_quat;
        odom_broadcaster.sendTransform(odom_trans);

        // nav
        odom.header.stamp = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = x_final;
        odom.pose.pose.position.y = y_final;
        odom.pose.pose.position.z = 0;
        odom.pose.pose.orientation = odom_quat;
        double averspeed = (velocity1 + velocity2) / 2;
        odom.twist.twist.linear.x = averspeed * cos(th_final);
        odom.twist.twist.linear.y = averspeed * sin(th_final);
        odom.twist.twist.angular.z = (velocity2 - velocity1) / d_wheel;
        pub.publish(odom);
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
    ros::Subscriber twistSub;
    float linear_x, linear_y, angular_z;

    void callback(const geometry_msgs::Twist::ConstPtr &vel_cmd)
    {
        as.send(as.Twist, as.toBytes(std::vector<float>{{
                              (float)vel_cmd->linear.x,
                              (float)vel_cmd->linear.y,
                              (float)vel_cmd->angular.z,
                          }}));
    }

public:
    TwistHandler(ros::NodeHandle &n, ArduinoSerial &as) : as{as}
    {
        twistSub = n.subscribe<geometry_msgs::Twist>(subscribeTopicName, queueSize, &TwistHandler::callback, this);
    }
    ~TwistHandler() {}
};

class RodHandler
{
private: // 参数
    const std::string subscribeTopicName = "pull_trigger";
    const std::string publishTopicName = "action_trigger";
    const size_t queueSize = 10;

private: // 内部辅助
    ArduinoSerial &as;
    ros::Subscriber pullSub;
    ros::Publisher actionPub;
    float linear_x, linear_y, angular_z;

    void callback(const std_msgs::Bool::ConstPtr &pull_trigger)
    {
        as.send(as.Bool, std::vector<uint8_t>{{pull_trigger->data}});
    }

public:
    RodHandler(ros::NodeHandle &n, ArduinoSerial &as) : as{as}
    {
        pullSub = n.subscribe<std_msgs::Bool>(subscribeTopicName, queueSize, &RodHandler::callback, this);
        actionPub = n.advertise<std_msgs::Bool>(publishTopicName, queueSize);
    }
    ~RodHandler() {}
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "contact_arduino");
    ros::NodeHandle n;

    ArduinoSerial as;

    TwistHandler th(n, as);
    RodHandler rh(n, as);
    // OdomHandler oh(n, as);    --------------------I changed because it doesn't need the help from here  7.25

    ros::spin();
    return 0;
}