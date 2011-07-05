#ifndef PI_LIB_INIT
#define PI_LIB_INIT

void init()
{
#ifdef PI_LIB_TIME
    init_timer();
#endif

#ifdef PI_LIB_MOTORS
    init_motors();
#endif

#ifdef PI_LIB_SENSORS
    init_sensors();
#endif

#ifdef PI_LIB_DISPLAY
    init_display();
#endif

#ifdef PI_LIB_RS232
    init_rs232();
#endif

#ifdef PI_LIB_I2C
//    init_i2c(); TODO
#endif

#ifdef PI_LIB_BUTTONS
    init_buttons();
#endif   
}

void clean()
{
#ifdef PI_LIB_TIME
    clean_timer();
#endif

#ifdef PI_LIB_MOTORS
    clean_motors();
#endif

#ifdef PI_LIB_SENSORS
    clean_sensors();
#endif

#ifdef PI_LIB_DISPLAY
    clean_display();
#endif

#ifdef PI_LIB_RS232
    clean_rs232();
#endif

#ifdef PI_LIB_I2C
//    clean_i2c(); TODO
#endif

#ifdef PI_LIB_BUTTONS
    clean_buttons();
#endif   
}

void run();

int main()
{
    init();
    sei();
    run();
    cli();
    clean();
    return 0;
}

#endif