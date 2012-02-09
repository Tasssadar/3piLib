#ifndef PI_LIB_SENSORS
#define PI_LIB_SENSORS

// LOW_BATTERY = (((low_voltage_in_mv*2-1)/3)*1023-511)/5000;
// Current setting is 4800mV
#define LOW_BATTERY 654

#define PI_GRND_SENSOR_COUNT 5
#define PI_TOTAL_SENSORS 7
#define PI_VCC 5000

struct ground_sensors_t
{
    uint16_t value[PI_TOTAL_SENSORS];
};

volatile struct ground_sensors_t g_sensors;

volatile int16_t g_calibratedMinimum[PI_GRND_SENSOR_COUNT];
volatile int16_t g_calibratedMaximum[PI_GRND_SENSOR_COUNT];

ISR(ADC_vect)
{
    static uint8_t currentSensor = 0;
    static bool initSensor = false;
    static const uint8_t sensorMap[PI_TOTAL_SENSORS] = { 0, 1, 2, 3, 4, 6, 7 };
    
    if (initSensor)
    {
        uint8_t adcl = ADCL;
        uint8_t adch = ADCH;
        
        uint16_t value = (adch << 8) | (adcl);
        g_sensors.value[currentSensor++] = value;

        if(currentSensor == PI_TOTAL_SENSORS)
        {
            currentSensor = 0;
            if(buzzer.isEmergency() && value > LOW_BATTERY)
                buzzer.emergency(false);
            else if(!buzzer.isEmergency() && value < LOW_BATTERY)
                buzzer.emergency(true);
        }
        
        ADMUX = (1<<REFS0)|sensorMap[currentSensor];
    }
    
    initSensor = !initSensor;
    
    // Start the next conversion
    ADCSRA |= (1<<ADSC);
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
    sei();
    nop();
    
    if(threshold)
    {
        int16_t denominator = g_calibratedMaximum[index] - g_calibratedMinimum[index];

        int16_t val = res;
        if(denominator)
            val = int32_t(res - g_calibratedMinimum[index])*1000/denominator;
        if(val < 0)
            res = 0;
        else if(val > 1024)
            res = 1024;
        else
            res = val;
    }

    return res; 
}

void resetCalibration()
{
    for(uint8_t i = 0; i < PI_GRND_SENSOR_COUNT; ++i)
    {
        g_calibratedMaximum[i] = 0;
        g_calibratedMinimum[i] = 1024;
    }
}

void calibrate_sensors()
{
    uint8_t i;
    int16_t sensor_value;
    int16_t max_sensor_values[PI_GRND_SENSOR_COUNT];
    int16_t min_sensor_values[PI_GRND_SENSOR_COUNT];

    for(uint8_t j = 0; j<10; ++j)
    {
        for(i = 0; i<PI_GRND_SENSOR_COUNT; ++i)
        {
            sensor_value = getSensorValue(i, false);
            // set the max we found THIS time
            if(j == 0 || max_sensor_values[i] < sensor_value)
                max_sensor_values[i] = sensor_value;

            // set the min we found THIS time
            if(j == 0 || min_sensor_values[i] > sensor_value)
                min_sensor_values[i] = sensor_value;
        }
    }

    // record the min and max calibration values
    for(i = 0; i < PI_GRND_SENSOR_COUNT; ++i)
    {
        if(min_sensor_values[i] > g_calibratedMaximum[i])
            g_calibratedMaximum[i] = min_sensor_values[i];
        if(max_sensor_values[i] < g_calibratedMinimum[i])
            g_calibratedMinimum[i] = max_sensor_values[i];
    }
}

void cal_round()
{
    resetCalibration();
    for(uint8_t counter = 0; counter < 80; ++counter)
    {
        if(counter < 20 || counter >= 60)
            setMotorPower(40,-40);
        else
            setMotorPower(-40,40);
        calibrate_sensors();

        // 80*20 = 1600 ms.
        delay(20);
    }
    setMotorPower(0, 0);
}

inline uint16_t getBatteryVoltage()
{
    return (((uint32_t(getSensorValue(5, false))*PI_VCC+511)/1023)*3+1)/2;
}

inline uint8_t getTrimPct()
{
    return (uint32_t(getSensorValue(6, false))*100)/1023;
}

inline uint16_t getTrimMV()
{
    return (uint32_t(getTrimPct()) * PI_VCC)/100;
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
    g_sensors.value[6] = 1024;

    resetCalibration();
}

void clean_sensors()
{
    ADCSRA = 0;
}

#endif
