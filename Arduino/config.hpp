#ifndef _CONFIG_HPP
#define _CONFIG_HPP

//-----------------------------------串口设置
const auto BAUDRATE = 57600;
const auto SERIAL_MOD = SERIAL_8E1;

//-----------------------------------引脚设置
enum Pin
{
    ENCODER_A1 = 14, //电机 1
    ENCODER_B1 = 3,
    ENCODER_A2 = 15, //电机 2
    ENCODER_B2 = 2,
    PWM1 = 9,
    PWM2 = 11,
    IN1 = 19,
    IN2 = 18,
    IN3 = 17,
    IN4 = 16,
};

//-----------------------------------设备参数
const auto pulse_round = 160; //一圈发出的脉冲数   //2it changed from 415 to 160
const auto r_wheel = 0.025f;  // 轮子半径（米）
const auto d_wheel = 0.21f;   // 轮子distance（米）

//-----------------------------------算法参数
const auto CONTROL_PERIOD = 5; // 电机控制函数的调用周期（毫秒）
const auto READ_PERIOD = 2;    // 串口读取周期
const auto Kp = 10;
const auto Ti = 5;
const auto Td = 0;

#endif
