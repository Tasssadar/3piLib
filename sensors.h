#ifndef PI_LIB_SENSORS
#define PI_LIB_SENSORS

struct ground_sensors_t
{
    uint16_t value[6];
};

volatile struct ground_sensors_t g_sensors;
uint16_t g_threshold = 512;

ISR(ADC_vect)
{
    static uint8_t currentSensor = 0;
    static bool initSensor = false;
    static const uint8_t sensorMap[6] = { 0, 1, 2, 3, 4, 6 };
    
    if (initSensor)
    {
        uint8_t adcl = ADCL;
        uint8_t adch = ADCH;
        
        uint16_t value = (adch << 8) | (adcl);
        g_sensors.value[currentSensor++] = value;

        if(currentSensor == 6)
            currentSensor = 0;
        
        ADMUX = (1<<REFS0)|sensorMap[currentSensor];
    }
    
    initSensor = !initSensor;
    
    // Start the next conversion
    ADCSRA |= (1<<ADSC);
}

void init_sensors()
{
    DIDR0 = (1<<ADC0D)|(1<<ADC1D)|(1<<ADC2D)|(1<<ADC3D)|(1<<ADC4D)|(1<<ADC5D);
    ADCSRA = (1<<ADEN)|(1<<ADSC)|(1<<ADIF)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
    ADCSRB = 0;
    
    ADMUX = (1<<REFS0);
    PORTC |= (1 << 0)|(1 << 1)|(1 << 2)|(1 << 3)|(1 << 4)|(1 << 5);
    
    g_sensors.value[0] = 1024;
    g_sensors.value[1] = 1024;
    g_sensors.value[2] = 1024;
    g_sensors.value[3] = 1024;
    g_sensors.value[4] = 1024;
    g_sensors.value[5] = 1024;
}

void clean_sensors()
{
    ADCSRA = 0;
}

inline int16_t getSensorValue(uint8_t index, bool threshold = true)
{
    cli();
    while (g_sensors.value[index] == 1024)
    {
        sei();
        nop();
        cli();
    }
    int16_t res = g_sensors.value[index];
    if(threshold) res -= g_threshold;
    sei();
    nop();

    return res; 
}

inline void calibrate_sensors()
{
    uint32_t avg = 0;
    for(uint8_t i = 0; i < 5; ++i)
        avg += getSensorValue(i);
    
    g_threshold = (uint16_t) (avg / 5);
}

inline uint16_t getBatteryVoltage()
{
    return (((uint32_t(getSensorValue(5, false))*5000+511)/1023)*3+1)/2;
}

#endif
