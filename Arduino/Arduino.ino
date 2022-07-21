#include <math.h>

/////////////////////////////////////
////         参数设置部分          ////
/////////////////////////////////////

//-----------------------------------引脚设置
const int_fast8_t ENCODER_A1 = 14; //电机 1
const int_fast8_t ENCODER_B1 = 3;
const int_fast8_t ENCODER_A2 = 15; //电机 2
const int_fast8_t ENCODER_B2 = 2;
const int_fast8_t PWM1 = 9;
const int_fast8_t PWM2 = 11;
const int_fast8_t IN1 = 19;
const int_fast8_t IN2 = 18;
const int_fast8_t IN3 = 17;
const int_fast8_t IN4 = 16;

//-----------------------------------算法参数
const int_fast8_t PERIOD = 5; // 电机控制函数的调用周期（毫秒）
const int_fast8_t Kp = 10;
const int_fast8_t Ti = 5;
const int_fast8_t Td = 0;
const float T = PERIOD;
const float q0 = Kp * (1 + T / Ti + Td / T);
const float q1 = -Kp * (1 + 2 * Td / T);
const float q2 = Kp * Td / T;

//-----------------------------------设备参
const auto pulse_round = 160; //一圈发出的脉冲数   //2it changed from 415 to 160
const auto r_wheel = 0.025f;  // 轮子半径（米）
const auto d_wheel = 0.21f;   // 轮子distance（米）

//-----------------------------------全局变量
float target1;            //左
float target2;            //右
volatile int encoderVal1; //编码器 1 值
volatile int encoderVal2; //编码器 2 值
float velocity1;          //转速 1
float velocity2;          //转速 2
volatile float x_final;
volatile float y_final;
volatile float th_final;
float u1;
float u11, u12, u13;
float u2;
float u21, u22, u23;

float linear_x, linear_y, angular_z;

union
{
    float f[5]; // arduino 的 double 就是 float
    uint8_t i[sizeof(f)];
} odom_buf;

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
    } twist_buf;
    static int_fast8_t currByteNo = -1;
    static uint8_t checkCode = 0xff;
    int currByte = Serial.read();
    if (currByte < 0)
        return; // 未读到
    switch (currByteNo)
    {
    case -1:
        if (Serial.read() == 0xff)
            currByteNo++;
        break;

    default:
        twist_buf.i[currByteNo++] = currByte;
        checkCode ^= currByte;
        break;

    case sizeof(twist_buf):
        if (checkCode == currByte)
        {
            linear_x = twist_buf.f[0];
            linear_y = twist_buf.f[1];
            angular_z = twist_buf.f[2];
            target1 = linear_x - angular_z * d_wheel / 2;
            target2 = linear_x + angular_z * d_wheel / 2;
        }
        currByteNo = -1;
        checkCode = 0xff;
        break;
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
    odom_buf.f[0] = x_final;
    odom_buf.f[1] = y_final;
    odom_buf.f[2] = th_final;
    odom_buf.f[3] = velocity1;
    odom_buf.f[4] = velocity2;
    Serial.write(odom_buf.i, sizeof(odom_buf));
    for (int_fast8_t i = 0; i < sizeof(odom_buf); i++)
        check_code ^= odom_buf.i[i];
    Serial.write(&check_code, 1);
    Serial.flush();
}

//-------------------------------control
int pidController1(float targetVelocity, float currentVelocity)
{
    // static float u1;
    // static float u11, u12, u13;
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
    // static float u21, u22, u23;
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
inline void getEncoder1(void)
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
    /*
        Arduino定时器包括0,1,2三个。0,2是8位定时器，1是16位定时器。
        设置定时器的方法是设置它们的寄存器Timer/Counter Control Register，即TCCR。
        - Timer0是系统时钟；
        - Timer1控制9、10管脚的PWM。
        参见：https://blog.csdn.net/Wxx_Combo/article/details/113987176
     */
    noInterrupts(); // 暂时关闭中断，防止打断寄存器设置

    /*  CS12 CS11 CS10 = 0 0 1 代表1分频，也就是不分频。
        默认是0 0 0，代表没有时钟源，定时器不工作。*/
    TCCR1B &= 0b11111000; // 将CS12、CS11、CS10置零
    TCCR1B |= 0b00000001; // 将CS11置1

    interrupts(); // 恢复中断
}

//-----------------------------------主函数
void setup()
{
    // 改变9,10管脚PWM频率
    setTimer1();

    // 电机1
    pinMode(ENCODER_A1, INPUT);
    pinMode(ENCODER_B1, INPUT);
    attachInterrupt(1, getEncoder1, CHANGE);

    // 电机2
    pinMode(ENCODER_A2, INPUT);
    pinMode(ENCODER_B2, INPUT);
    attachInterrupt(0, getEncoder2, CHANGE);

    pinMode(PWM1, OUTPUT);
    pinMode(PWM2, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // 设置串口频率
    Serial.begin(57600, SERIAL_8E1);
}

void loop()
{
    // 计算轮子1,2的速度
    velocity1 = 2 * PI * ((float)encoderVal1 / pulse_round) * r_wheel * (1000 / PERIOD);
    velocity2 = 2 * PI * ((float)encoderVal2 / pulse_round) * r_wheel * (1000 / PERIOD);
    Publish();

    // 将encoderVal1, encoderVal2归零，方便下一次计算速度
    // encoderVal1 = 0;
    // encoderVal2 = 0;
    readTwist();

    int output1 = pidController1(target1, velocity1);
    int output2 = pidController2(target2, velocity2);
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
    delay(PERIOD);
}
