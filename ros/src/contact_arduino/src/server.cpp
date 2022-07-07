#include "ros/ros.h"
#include "serial/serial.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "contact_arduino");
    ros::NodeHandle n;
    serial::Serial s;
    try
    {
        s.setPort("/dev/ttyUSB0");                                 // 端口号（以后可以改成端口别名）
        s.setBaudrate(230400);                                     // 波特率
        serial::Timeout tout = serial::Timeout::simpleTimeout(10); // 延时
        s.setTimeout(tout);
        s.open(); // 串口
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR("Fail to open serial port.");
        return -1;
    }
    if (!s.isOpen())
    {
        ROS_ERROR("Fail to initialize serial port.");
        return -1;
    }
    while (ros::ok())
    {
        // uint8_t buf[24];
        uint8_t check_code = 0xff;
        // s.write(&check_code, 1);
        // *(double *)&buf[0] = 3.14;
        // *(double *)&buf[8] = 2.718;
        // *(double *)&buf[16] = 114514;
        // for (int i = 0; i < 24; i++)
        //     check_code ^= buf[i];
        // s.write(buf, sizeof(buf));
        // s.write(&check_code, 1);

        // debug
        sleep(1);
        s.read(&check_code, 1);
        if (check_code != 0xff)
            return -1;

        uint8_t rev_data[3 * 8]; // 分别是 double linear_x, linear_y, angular_z
        for (int_fast8_t i = 24; i-- > 0;)
        {
            s.read(&rev_data[i], 1);
            check_code ^= rev_data[i];
        }
        // 校验码
        uint8_t rev;
        s.read(&rev, 1);
        if (check_code != rev)
            return -1;

        double linear_x = *(double *)&rev_data[0];
        double linear_y = *(double *)&rev_data[8];
        double angular_z = *(double *)&rev_data[16];
        printf("bytes: ");
        for (int i = 0; i < 24; i++)
            printf("%d ", (uint)rev_data[i]);
        printf("\nlinear_x:%lf\nlinear_y:%lf\nangular_z:%lf\n", linear_x, linear_y, angular_z);
    }
    s.close();
    return 0;
}