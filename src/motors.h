#ifndef PI_LIB_MOTORS
#define PI_LIB_MOTORS

#define MOTORS_ACCELERATION 1

void init_motors()
{
    // configure for inverted fast PWM output on motor control pins:
    //  set OCxx on compare match, clear on timer overflow
    //  Timer0 and Timer2 counts up from 0 to 255 and then overflows directly to 0
    TCCR0A = TCCR2A = 0xF3;

    // use the system clock/8 (=2.5 MHz) as the timer clock,
    // which will produce a PWM frequency of 10 kHz
    TCCR0B = TCCR2B = 0x02;

    // use the system clock (=20 MHz) as the timer clock,
    // which will produce a PWM frequency of 78 kHz.  The Baby Orangutan B
    // and 3Pi can support PWM frequencies this high.  The
    // Orangutan LV-168 cannot support frequencies above 10 kHz.
    //TCCR0B = TCCR2B = 0x01;

    // initialize all PWMs to 0% duty cycle (braking)
    OCR0A = OCR0B = OCR2A = OCR2B = 0;

    TIMSK0 |= (1 << TOIE0);

    DDRD |= (1 << PIN5) | (1 << PIN6)| (1 << PIN3);
    DDRB |= (1 << PIN3);
    PORTD &= ~(1 << PIN5);
    PORTD &= ~(1 << PIN6);
    PORTD &= ~(1 << PIN3);
    PORTB &= ~(1 << PIN3);
}

void clean_motors()
{
    TCCR0A = 0;
    TCCR2A = 0;
    TCCR0B = 0;
    TCCR2B = 0;
}

namespace detail
{
    volatile int16_t g_speed[2] = {0, 0};
    volatile int16_t g_speed_cur[2] = {0, 0};
    volatile bool g_need_set_speed[2] = {false, false};
    volatile bool g_soft_speed_set = true;
    volatile bool g_speed_is_setted = false;
    volatile uint8_t g_sub_timer = 0;
    volatile uint32_t g_timer = 0;

    void setLeftMotor(int16_t speed)
    {
        bool reverse = false;
        if (speed < 0)
        {
            speed = -speed; // make speed a positive quantity
            reverse = true;    // preserve the direction
        }
        if (speed > 0xFF)   // 0xFF = 255
            speed = 0xFF;

        if (reverse)
        {
            OCR0B = 0;      // hold one driver input high
            OCR0A = speed;  // pwm the other input
        }
        else    // forward
        {
            OCR0B = speed;  // pwm one driver input
            OCR0A = 0;      // hold the other driver input high
        }
    }

    void setRightMotor(int16_t speed)
    {
        bool reverse = false;
        if (speed < 0)
        {
            speed = -speed; // make speed a positive quantity
            reverse = true;    // preserve the direction
        }
        if (speed > 0xFF)   // 0xFF = 255
            speed = 0xFF;

        if (reverse)
        {
            OCR2B = 0;      // hold one driver input high
            OCR2A = speed;  // pwm the other input
        }
        else    // forward
        {
            OCR2B = speed;  // pwm one driver input
            OCR2A = 0;      // hold the other driver input high
        }
    }
}

void setRightMotor(int16_t speed)
{
    detail::g_speed[1] = speed;

    if(detail::g_soft_speed_set)
        detail::g_need_set_speed[1] = true;
    else
        detail::setRightMotor(speed);
}

void setLeftMotor(int16_t speed)
{
    detail::g_speed[0] = speed;

    if(detail::g_soft_speed_set)
        detail::g_need_set_speed[0] = true;
    else
        detail::setLeftMotor(speed);
}

void setMotorPowerID(uint8_t motor, int16_t speed)
{
    detail::g_speed[motor] = speed;
    if(motor)
        detail::setRightMotor(speed);
    else
        detail::setLeftMotor(speed);
}

inline void setMotorPower(int16_t left, int16_t right)
{
    setLeftMotor(left);
    setRightMotor(right);
}

int16_t getMotorPowerID(uint8_t motor)
{
    return detail::g_speed[motor];
}

inline int16_t getLeftMotor()
{
    return getMotorPowerID(0);
}

inline int16_t getRightMotor()
{
    return getMotorPowerID(1);
}

inline void setSoftAccel(bool enabled)
{
    detail::g_soft_speed_set = enabled;
}

ISR(TIMER0_OVF_vect)
{
    if(++detail::g_sub_timer != 10)
        return;

    detail::g_sub_timer = 0;
    ++detail::g_timer;

    buzzer.update();

    if(detail::g_speed_is_setted)
        return;
    detail::g_speed_is_setted = true;
    for(uint8_t i = 0; i < 2; ++i)
    {
        if(!detail::g_need_set_speed[i])
            continue;

        if(detail::g_speed[i] == detail::g_speed_cur[i])
        {
            detail::g_need_set_speed[i] = false;
            continue;
        }

        int16_t val = abs(detail::g_speed[i]-detail::g_speed_cur[i]);
        if(val >= MOTORS_ACCELERATION)
            val = MOTORS_ACCELERATION;

        if(detail::g_speed[i] < detail::g_speed_cur[i])
            val *= -1;
        detail::g_speed_cur[i] += val;
        setMotorPowerID(i, detail::g_speed_cur[i]);
    }
    detail::g_speed_is_setted = false;
}


#endif
