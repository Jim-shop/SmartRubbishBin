class Motor
{
private: // 内部参数
    uint8_t encoder_A_pin, encoder_B_pin, pwm_pin;

public:
    Motor(uint8_t encoder_A_pin, uint8_t encoder_B_pin, uint8_t pwm_pin)
        : encoder_A_pin{encoder_A_pin}, encoder_B_pin{encoder_B_pin}, pwm_pin{pwm_pin}
    {
    }
    
};