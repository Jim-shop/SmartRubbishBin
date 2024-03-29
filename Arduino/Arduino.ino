#include <MsTimer2.h>
#include <math.h>

#include "config.hpp"
#include "constants.hpp"

#include "console.hpp"

//-----------------------------------全局变量
float target1 = 1;            //左 保守
float target2 = -1;           //右 速度
volatile int encoderVal1 = 0; //编码器 1 值
volatile int encoderVal2 = 0; //编码器 2 值
float velocity1;              //转速 1
float velocity2;              //转速 2
volatile float x_final = 0;
volatile float y_final = 0;
volatile float th_final = PI / 2;

bool shouldPull = false; // 收到的，指示是否应该启动推杆
bool isSuccess = false;  // 发送的，指示Arduino是否已经完成动作

float linear_x = 1, linear_y = -1, angular_z = 0;

/////////////////////////////////////
////         程序代码部分          ////
/////////////////////////////////////

//-----------------------------------上位机数据接收
void read()
{
    enum DataType : uint8_t
    {
        Twist = 0xa0,
        Bool = 0xa1,
    };
    union TwistBuf
    {
        float f[3]; // arduino 的 double 就是 float
        uint8_t i[sizeof(f)];
    };

    static DataType currMod;
    static int_fast8_t currByteNo = -3;
    static bool needReset = false, needFlushInput = false;
    static uint8_t checkCode1 = 0x55 ^ 0xaa, checkCode2 = 0x55 + 0xaa;
    static TwistBuf twistBuf;
    static bool boolBuf;

    int byteBuf = Serial.read();
    for (uint8_t currByte = byteBuf; byteBuf >= 0; byteBuf = Serial.read(), currByte = byteBuf)
    {
        if (currByteNo < 0)
        {
            switch (currByteNo)
            {
            case -3:
                if (currByte == 0x55)
                    currByteNo++;
                break;

            case -2:
                if (currByte == 0xaa)
                    currByteNo++;
                else
                    currByteNo = -3;
                break;

            case -1:
                if (currByte == Twist || currByte == Bool)
                {
                    currMod = (DataType)currByte;
                    currByteNo++;
                    checkCode1 ^= currByte;
                    checkCode2 += currByte;
                }
                else
                    currByteNo = -3;
                break;
            }
        }
        else
        {
            if (currMod == Twist)
            {
                switch (currByteNo)
                {
                case sizeof(twistBuf):
                    if (checkCode1 == currByte)
                        currByteNo++;
                    else
                        needReset = true, needFlushInput = true;
                    break;

                case sizeof(twistBuf) + 1:
                    needReset = true;
                    if (checkCode2 == currByte)
                    {
                        linear_x = twistBuf.f[0];
                        linear_y = twistBuf.f[1];
                        angular_z = twistBuf.f[2];
                        target1 = linear_x - angular_z * d_wheel / 2;
                        target2 = linear_x + angular_z * d_wheel / 2;
                    }
                    else
                        needFlushInput = true;
                    break;

                default:
                    twistBuf.i[currByteNo] = currByte;
                    currByteNo++;
                    checkCode1 ^= currByte;
                    checkCode2 += currByte;
                    break;
                }
            }
            else
            {
                switch (currByteNo)
                {
                case 1:
                    if (checkCode1 == currByte)
                        currByteNo++;
                    else
                    {
                        needReset = true;
                        needFlushInput = true;
                    }
                    break;

                case 2:
                    needReset = true;
                    if (checkCode2 == currByte)
                        shouldPull = boolBuf;
                    else
                        needFlushInput = true;
                    break;

                case 0:
                    boolBuf = currByte;
                    currByteNo++;
                    checkCode1 ^= currByte;
                    checkCode2 += currByte;
                    break;
                }
            }
        }

        if (needReset)
        {
            currByteNo = -3;
            checkCode1 = 0x55 ^ 0xaa;
            checkCode2 = 0x55 + 0xaa;
            needReset = false;
        }
        if (needFlushInput)
        {
            while (Serial.available() > 0)
                Serial.read();
            needFlushInput = false;
        }
    }
}

//-----------------------------------Odom和tf数据发送
inline void Publish(void)
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

    union
    {
        float f[5]; // arduino 的 double 就是 float
        uint8_t i[sizeof(f)];
        // } buf{.f = {linear_x, y_final, th_final, velocity1, velocity2}};
    } buf{.f = {x_final, y_final, th_final, velocity1, velocity2}};
    uint8_t sentbuf[2 + sizeof(buf) + 2];
    sentbuf[0] = 0x55, sentbuf[1] = 0xaa;
    memcpy(&sentbuf[2], buf.i, sizeof(buf));
    uint8_t &checkCode1 = sentbuf[2 + sizeof(buf) + 0],
            &checkCode2 = sentbuf[2 + sizeof(buf) + 1];
    checkCode1 = checkCode2 = 0x55;
    for (int i = 1; i < 2 + sizeof(buf); i++)
        checkCode1 ^= sentbuf[i], checkCode2 += sentbuf[i];
    Serial.write(sentbuf, sizeof(sentbuf));
    /*
     uint8_t sentbuf[2 + sizeof(bool) + 2] = {
         0x55,
         0xaa,
         isSuccess,
         0x55 ^ 0xaa ^ isSuccess,
         uint8_t(0x55 + 0xaa + isSuccess),
     };
     Serial.write(sentbuf, sizeof(sentbuf));*/
}

//-------------------------------control
inline int pidController1(float targetVelocity, float currentVelocity)
{
    static float u1;
    static float u11, u12, u13;
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

inline int pidController2(float targetVelocity, float currentVelocity)
{
    static float u2;
    static float u21, u22, u23;
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
long forwards[5], backwards[5];
void getEncoder1(void) // the previous model is not very accurate
{
    static auto lastTime = millis();
    auto thisTime = millis();
    auto diffTime = thisTime - lastTime;
    if (diffTime > 4)
        diffTime = 4;
    if (digitalRead(ENCODER_A1) ^ digitalRead(ENCODER_B1))
        backwards[diffTime]++;
    else
        forwards[diffTime]++;
    lastTime = thisTime;
}
void getEncoder2(void)
{
    if (digitalRead(ENCODER_A2) ^ digitalRead(ENCODER_B2))
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
    interrupts();         // 恢复中断
}

//-----------------------------------主函数
void setup()
{
    // 改变9,10管脚PWM频率
    setTimer1();

    // 定时接收Twist信息
    MsTimer2::set(CONTROL_PERIOD, control);
    // MsTimer2::start();

    // 电机1
    pinMode(ENCODER_A1, INPUT);
    pinMode(ENCODER_B1, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B1), getEncoder1, CHANGE);

    // 电机2
    pinMode(ENCODER_A2, INPUT);
    pinMode(ENCODER_B2, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B2), getEncoder2, CHANGE);

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
void control()
{
    // con.parse();
    // 计算轮子1,2的速度
    velocity1 = 2 * PI * ((float)encoderVal1 / pulse_round) * r_wheel * (1000 / CONTROL_PERIOD);
    velocity2 = 2 * PI * ((float)encoderVal2 / pulse_round) * r_wheel * (1000 / CONTROL_PERIOD);
    Publish();

    // 将encoderVal1, encoderVal2归零，方便下一次计算速度
    encoderVal1 = 0;
    encoderVal2 = 0;
    read();

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
}

void loop()
{
    Serial.println("\nforward:");
    for (int i = 0; i < 5; i++)
    {
        Serial.print("\t[");
        Serial.print(i);
        Serial.print("]");
        Serial.print(forwards[i]);
    }
    Serial.println("\nbackward:");
    for (int i = 0; i < 5; i++)
    {
        Serial.print("\t[");
        Serial.print(i);
        Serial.print("]");
        Serial.print(backwards[i]);
    }
    encoderVal1 = forwards[0] * 0.9 + forwards[1] * 1 + forwards[2] * 1.3 + forwards[3] * 1.11 + forwards[4] * 1.12 -
                  backwards[0] * 1.3 - backwards[1] * 1.05 - backwards[2] * 0.9 - backwards[3] * 1 - backwards[4] * 1;
    // Serial.println(encoderVal1);
}
