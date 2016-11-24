#ifndef PI_LIB_SENSORS
#define PI_LIB_SENSORS

// LOW_BATTERY = (((low_voltage_in_mv*2-1)/3)*1023-511)/5000;
// Current setting is 4200mV
#define LOW_BATTERY 560

#define PI_GRND_SENSOR_COUNT 5
#define PI_TOTAL_SENSORS 7
#define PI_BAT_VOLTAGE_SENSOR 6
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

		if (currentSensor == PI_BAT_VOLTAGE_SENSOR)
		{
		    if(buzzer.isEmergency() && value > LOW_BATTERY)
                buzzer.emergency(false);
            else if(!buzzer.isEmergency() && value < LOW_BATTERY)
                buzzer.emergency(true);
		}

        if(currentSensor == PI_TOTAL_SENSORS)
        {
            currentSensor = 0;
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

void store_sensor_cal(uint16_t address)
{
    int32_t val = 0;
    uint8_t bits = 0;
    for(uint8_t i = 0; i < PI_GRND_SENSOR_COUNT*2;++i)
    {
        if(i < PI_GRND_SENSOR_COUNT)
            val |= (int32_t(g_calibratedMinimum[i] & 0x7FF) << bits);
        else
            val |= (int32_t(g_calibratedMaximum[i-5] & 0x7FF) << bits);

        bits += 11;

        while(bits >= 8)
        {
            store_eeprom(address++, uint8_t(val & 0xFF));
            val >>= 8;
            bits -= 8;
        }
    }

    if(bits)
        store_eeprom(address, uint8_t(val));
}

void load_sensor_cal(uint16_t address)
{
    int32_t val = 0;
    uint8_t bits = 0;
    for(uint8_t i = 0; i < PI_GRND_SENSOR_COUNT*2;)
    {
        val |= (int32_t(load_eeprom<uint8_t>(address++)) << bits);
        bits += 8;

        while(bits >= 11)
        {
            if(i < 5)
                g_calibratedMinimum[i] = (val & 0x7FF);
            else
                g_calibratedMaximum[i-5] = (val & 0x7FF);

            val >>= 11;
            bits -= 11;
            ++i;
        }
    }
}

// int PololuQTRSensors::readLine(unsigned int *sensor_values,
//                                unsigned char readMode, unsigned char white_line)
// from Pololu lib, src/PololuQTRSensors/PololuQTRSensors.cpp

// Operates the same as read calibrated, but also returns an
// estimated position of the robot with respect to a line. The
// estimate is made using a weighted average of the sensor indices
// multiplied by 1000, so that a return value of 0 indicates that
// the line is directly below sensor 0, a return value of 1000
// indicates that the line is directly below sensor 1, 2000
// indicates that it's below sensor 2000, etc.  Intermediate
// values indicate that the line is between two sensors.  The
// formula is:
//
//    0*value0 + 1000*value1 + 2000*value2 + ...
//   --------------------------------------------
//         value0  +  value1  +  value2 + ...
//
// By default, this function assumes a dark line (high values)
// surrounded by white (low values).  If your line is light on
// black, set the optional second argument white_line to true.  In
// this case, each sensor value will be replaced by (1000-value)
// before the averaging.
int16_t getLinePos(bool white_line = false)
{
    bool on_line = false;
    uint8_t i;
    uint32_t avg = 0; // this is for the weighted total, which is long
                      // before division
    uint16_t sum = 0; // this is for the denominator which is <= 64000
    static int16_t last_value = 0; // assume initially that the line is left.

    for(i = 0; i < PI_GRND_SENSOR_COUNT; ++i)
    {
        int16_t value = getSensorValue(i);
        if(white_line)
            value = 1024-value;

        // keep track of whether we see the line at all
        if(value > 200)
            on_line = 1;

        // only average in values that are above a noise threshold
        if(value > 50)
        {
            avg += (int32_t)(value) * (i * 1024);
            sum += value;
        }
    }

    if(!on_line)
    {
        // If it last read to the left of center, return 0.
        if(last_value < (PI_GRND_SENSOR_COUNT-1)*1024/2)
            return 0;
        else // If it last read to the right of center, return the max.
            return (PI_GRND_SENSOR_COUNT-1)*1024;

    }

    last_value = avg/sum;
    return last_value;
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
