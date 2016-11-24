#ifndef PI_LIB_INIT
#define PI_LIB_INIT

void init()
{
#ifdef PI_LIB_BUZZER
    init_buzzer();
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
#ifdef PI_LIB_BUZZER
    clean_buzzer();
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

	if (isPressed(BUTTON_A)) {
		display.printNumToXY(getBatteryVoltage(), 4,0);		
		delay(2000);
	}
	
    run();
    
	cli();
    clean();
    
	// for working bootloader after end of program
    init_rs232();
    sei();
	while(true) {}

    return 0;
}

#endif