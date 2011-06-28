#ifndef PI_LIB_MOTORS
#define PI_LIB_MOTORS

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

void setLeftMotor(int16_t speed)
{
    unsigned char reverse = 0;
    if (speed < 0)
    {
        speed = -speed; // make speed a positive quantity
        reverse = 1;    // preserve the direction
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
    unsigned char reverse = 0;
    if (speed < 0)
    {
        speed = -speed; // make speed a positive quantity
        reverse = 1;    // preserve the direction
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

void setMotorPower(int16_t left, int16_t right)
{
    setLeftMotor(left);
    setRightMotor(right);
}

#endif
