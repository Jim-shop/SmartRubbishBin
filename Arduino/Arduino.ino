#include <MsTimer2.h>
#include <math.h>

#include "config.hpp"
#include "constants.hpp"

#include "console.hpp"

//-----------------------------------全局变量
float target1;                //左 保守
float target2;                //右 速度
volatile int encoderVal1 = 0; //编码器 1 值
volatile int encoderVal2 = 0; //编码器 2 值
float velocity1;              //转速 1
float velocity2;              //转速 2
volatile float x_final = 0;
volatile float y_final = 0;
volatile float th_final = PI / 2;
float u1;
float u11, u12, u13;
float u2;
float u21, u22, u23;

union twist_buf
{
    float f[3]; // arduino 的 double 就是 float
    uint8_t i[sizeof(f)];
};
float linear_x = 1, linear_y = -1, angular_z = 0;

union odom_buf
{
    float f[5]; // arduino 的 double 就是 float
    uint8_t i[sizeof(f)];
};
/////////////////////////////////////
////         程序代码部分          ////
/////////////////////////////////////

//-----------------------------------Twist数据接收
void readTwist()
{
    static union
    {
        float f[3]; // arduino 的 double 就是 float
        uint8_t i[sizeof(f)];
    } twistBuf;
    static int_fast8_t currByteNo = -1;
    static uint8_t checkCode = 0xff;
    for (int currByte = Serial.read(); currByte >= 0; currByte = Serial.read())
    {
        switch (currByteNo)
        {
        case -1:
            if ((uint8_t)currByte == 0xff)
                currByteNo++;
            break;

        default:
            twistBuf.i[currByteNo] = (uint8_t)currByte;
            currByteNo++;
            checkCode ^= (uint8_t)currByte;
            break;

        case sizeof(twistBuf):
            if (checkCode == (uint8_t)currByte)
            {
                linear_x = twistBuf.f[0];
                linear_y = twistBuf.f[1];
                angular_z = twistBuf.f[2];
                target1 = linear_x - angular_z * d_wheel / 2;
                target2 = linear_x + angular_z * d_wheel / 2;
            }
            currByteNo = -1;
            checkCode = 0xff;
            while (Serial.available() > 0)
                Serial.read();
            return;
        }
    }
}

//-----------------------------------Odom和tf数据发送
void Publish(void)
{
    float dis_left = 2 * PI * ((float)encoderVal1 / pulse_round) * r_wheel;
    float dis_right = 2 * PI * ((float)encoderVal2 / pulse_round) * r_wheel;
    float d_x = ((dis_left + dis_right) / 2) * sin(th_final);
    float d_y = ((dis_left + dis_right) / 2) * cos(th_final);
    float d_th = (dis_right - dis_left) / d_wheel;
    dis_left = 0;
    dis_right = 0;
    x_final += d_x;
    y_final += d_y;
    th_final += d_th;

    uint8_t check_code = 0xff;
    Serial.write(&check_code, 1);
    odom_buf buf;
    buf.f[0] = linear_x; // x_final;
    buf.f[1] = y_final;
    buf.f[2] = th_final;
    buf.f[3] = velocity1;
    buf.f[4] = velocity2;
    Serial.write(buf.i, sizeof(buf));
    for (int_fast8_t i = 0; i < sizeof(buf); i++)
        check_code ^= buf.i[i];
    Serial.write(&check_code, 1);
    Serial.flush();
}

//-------------------------------control
int pidController1(float targetVelocity, float currentVelocity)
{
    // static float u1;
    // static float u11, u12, u13; these shouldn't be here
    u11 = targetVelocity - currentVelocity;
    u1 += q0 * u11 + q1 * u12 + q2 * u13;
    if (u1 > 255)
        u1 = 255;
    if (u1 < -255)
        u1 = -255;
    u13 = u12;
    u12 = u11;
    return (int)u1;
}

int pidController2(float targetVelocity, float currentVelocity)
{
    // static float u2;
    // static float u21, u22, u23;these shouldn't be here
    u21 = targetVelocity - currentVelocity;
    u2 += q0 * u21 + q1 * u22 + q2 * u23;
    if (u2 > 255)
        u2 = 255;
    if (u2 < -255)
        u2 = -255;
    u23 = u22;
    u22 = u21;
    return (int)u2;
}

//-----------------------------------编码器中断函数
inline void getEncoder1(void) // the previous model is not very accurate
{
    if (digitalRead(ENCODER_B1) ^ digitalRead(ENCODER_A1))
        encoderVal1--;
    else
        encoderVal1++;
}

inline void getEncoder2(void)
{
    if (digitalRead(ENCODER_B2) ^ digitalRead(ENCODER_A2))
        encoderVal2++;
    else
        encoderVal2--;
}

//-----------------------------------定时器初始化
inline void setTimer1(void)
{
    /* 参见：https://blog.csdn.net/Wxx_Combo/article/details/113987176 */
    noInterrupts(); // 暂时关闭中断，防止打断寄存器设置
    /* CS12 CS11 CS10 = 0 0 1 代表1分频 */
    TCCR1B &= 0b11111000; // 将CS12、CS11、CS10置零
    TCCR1B |= 0b00000001; // 将CS11置1
    interrupts(); // 恢复中断
}

//-----------------------------------主函数
void setup()
{
    // 改变9,10管脚PWM频率
    setTimer1();

    // 定时接收Twist信息
    MsTimer2::set(READ_PERIOD, readTwist);
    MsTimer2::start();

    // 电机1
    pinMode(ENCODER_A1, INPUT_PULLUP);
    pinMode(ENCODER_B1, INPUT_PULLUP);
    attachInterrupt(0, getEncoder2, CHANGE);

    // 电机2
    pinMode(ENCODER_A2, INPUT_PULLUP);
    pinMode(ENCODER_B2, INPUT_PULLUP);
    attachInterrupt(1, getEncoder1, CHANGE);

    pinMode(PWM1, OUTPUT);
    pinMode(PWM2, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // 设置串口频率
    Serial.begin(BAUDRATE, SERIAL_MOD);
}
// Console con;
void loop()
{
    // con.parse();
    // 计算轮子1,2的速度
    velocity1 = 2 * PI * ((float)encoderVal1 / pulse_round) * r_wheel * (1000 / CONTROL_PERIOD);
    velocity2 = 2 * PI * ((float)encoderVal2 / pulse_round) * r_wheel * (1000 / CONTROL_PERIOD);
    Publish();

    // 将encoderVal1, encoderVal2归零，方便下一次计算速度
    encoderVal1 = 0;
    encoderVal2 = 0;

    int output1 = 0; // pidController1(target1, velocity1);
    int output2 = 0; // pidController2(target2, velocity2);
    if (output1 > 0)
    {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(PWM1, output1);
    }
    else
    {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(PWM1, -output1);
    }
    if (output2 > 0)
    {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(PWM2, output2);
    }
    else
    {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(PWM2, -output2);
    }
    delay(CONTROL_PERIOD);
}
