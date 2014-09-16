#define PI_LIB_VERSION 29

#ifndef PI_LIB_COMMON
#define PI_LIB_COMMON

#define JUNIOR_F_CPU 20000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

#ifdef JUNIOR_ON_IDLE
#define JUNIOR_DO_IDLE() JUNIOR_ON_IDLE()
#else
#define JUNIOR_DO_IDLE()
#endif

inline void nop()
{
    asm volatile ("nop");
}

#ifndef __cplusplus
# error Nezapomente si v nastaveni AVR studia zapnout "-x c++" !
#endif

#if !defined(F_CPU) && !defined(JUNIOR_F_CPU)
# error Nemate nastavenou frekvenci procesoru!
#endif

#ifndef JUNIOR_F_CPU
# define JUNIOR_F_CPU F_CPU
#endif

#ifndef F_CPU
# define F_CPU JUNIOR_F_CPU
#endif

#if defined(F_CPU) && F_CPU != JUNIOR_F_CPU
# error Mate v nastaveni spatne nastavenou frekvenci procesoru !
#endif

#define JUNIOR_CONCAT2(x, y) x ## y
#define JUNIOR_CONCAT(x, y) JUNIOR_CONCAT2(x, y)

template <typename T>
inline T abs(T num)
{
    return (num < 0) ? -num : num;
}

template <typename T>
T load_eeprom(uint16_t address)
{
    T res;
    char * ptr = (char *) &res;
    char * pend = ptr + sizeof res;

    EEARH = (address >> 8);
    EEARL = uint8_t(address);

    while (ptr != pend)
    {
        EECR = (1<<EERE);
        *ptr++ = EEDR;
        ++EEARL;
    }

    return res;
}

template <typename T>
void store_eeprom(uint16_t address, T value)
{
    char * ptr = (char *) &value;
    char * pend = ptr + sizeof value;

    EEARH = (address >> 8);
    EEARL = uint8_t(address);

    while (ptr != pend)
    {
        EEDR = *ptr++;

        EECR = (1<<EEMPE);
        EECR = (1<<EEPE);

        while (EECR & (1<<EEPE))
        {
        }

        ++EEARL;
    }
}
#endif

#ifndef PI_LIB_BUZZER
#define PI_LIB_BUZZER

class buzzer_t
{
public:
    buzzer_t()
    {
        m_running = m_emergencyEnabled = m_started = false;
        m_freq = 2500;
        m_time_on = m_time_off = 0;
    }

    void set(uint16_t time_on = 0, uint16_t time_off = 0, bool run = true)
    {
        m_running = false;
        m_time_on = time_on;
        m_time_off = time_off;
        if(run)
            start();
    }

    void start()
    {
        if(!m_freq)
            return;

        m_running = m_started = true;

        if(!m_time_on)
            m_timer = -1;
        else
            m_timer = m_time_on;
        TCCR1B = (1 << WGM12) | (1 << CS11);
        OCR1AH = (m_freq >> 8);
        OCR1AL = (m_freq & 0xFF);
        TCCR1A |= (1 << COM1B0);
    }

    void stop()
    {
        m_running = m_started = false;
        TCCR1A = 0;
        TCCR1B = 0;
        TCCR1A &= ~(1 << COM1B0);
    }

    bool isStarted()
    {
        return m_started;
    }

    void setFreq(uint32_t hz)
    {
        m_freq = F_CPU/(hz*16);
        if(m_running)
        {
            OCR1AH = (m_freq >> 8);
            OCR1AL = (m_freq & 0xFF);
        }
    }

    void update()
    {
        if(!m_started || m_timer == -1)
            return;

        --m_timer;
        if(m_timer == 0)
        {
            if(m_running)
            {
                if(m_time_off == 0)
                    m_timer = -1;
                else
                    m_timer = m_time_off;

                TCCR1A &= ~(1 << COM1B0);
                OCR1AH = (m_freq >> 8);
                OCR1AL = (m_freq & 0xFF);
            }
            else
            {
                m_timer = m_time_on;
                TCCR1A |= (1 << COM1B0);
                OCR1AH = (m_freq >> 8);
                OCR1AL = (m_freq & 0xFF);
            }
            m_running = !m_running;
        }
    }

    void emergency(bool on)
    {
        if(!m_emergencyEnabled || m_emergency == on)
            return;
        m_emergency = on;
        if(on)
            set(1000, 500);
        else
            stop();
    }

    bool isEmergency() { return m_emergency; }
    void setEmergencyEnabled(bool enable) { m_emergencyEnabled = enable; }

private:
    uint16_t m_freq;
    int16_t m_time_on;
    int16_t m_time_off;
    int16_t m_timer;
    volatile bool m_running;
    volatile bool m_started;
    volatile bool m_emergency;
    volatile bool m_emergencyEnabled;
};

buzzer_t buzzer;

void init_buzzer()
{
    TCCR1B = (1 << WGM12) | (1 << CS11);

    OCR1BH = 0;
    OCR1BL = 0;
    DDRB |= (1 << 2);
    PORTB &= ~(1 << 2);
}

void clean_buzzer()
{
    buzzer.stop();
}

#endif
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

#ifndef PI_LIB_TIME
#define PI_LIB_TIME

// Delays for for the specified nubmer of microseconds.
inline void delayMicroseconds(uint16_t microseconds)
{
    __asm__ volatile (
        "1: push r22"     "\n\t"
        "   ldi  r22, 4"  "\n\t"
        "2: dec  r22"     "\n\t"
        "   brne 2b"      "\n\t"
        "   pop  r22"     "\n\t"   
        "   sbiw %0, 1"   "\n\t"
        "   brne 1b"
        : "=w" ( microseconds )  
        : "0" ( microseconds )
    );  
}

inline void delay(uint16_t ms)
{
    while (ms--)
      delayMicroseconds(1000);
}

uint32_t getTicksCount()
{
    cli();
    uint32_t time = detail::g_timer;
    sei();
    return time;
}

void resetTicks() __attribute__ ((deprecated));
void resetTicks()
{
    cli();
    detail::g_timer = 0;
    sei();
}

class stopwatch
{
public:
    stopwatch(bool running = true) : m_running(running)
    {
        clear();
    }

    void clear()
    {
        if(m_running)
            m_base = getTicksCount();
        else
            m_base = 0;
    }

    void cancel()
    {
        m_running = false;
        m_base = 0;
    }

    bool running() const
    {
        return m_running;
    }

    void restart()
    {
        m_running = true;
        m_base = getTicksCount();
    }

    void start()
    {
        if(!m_running)
        {
            m_running = true;
            m_base = getTicksCount() - m_base;
        }
    }

    void stop()
    {
        if(m_running)
        {
            m_running = false;
            m_base = getTicksCount() - m_base;
        }
    }

    uint32_t operator()() const
    {
        return get();
    }

    uint32_t get() const
    {
        if(m_running)
            return getTicksCount() - m_base;
        else
            return m_base;
    }

    void set(uint32_t val)
    {
        if(m_running)
            m_base = getTicksCount() - val;
        else
            m_base = val;
    }

    void decrease(uint32_t val)
    {
        if(m_running)
            m_base += val;
        else
            m_base -= val;
    }

private:
    volatile bool m_running;
    volatile uint32_t m_base;
};

#endif

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

#ifndef PI_LIB_DISPLAY
#define PI_LIB_DISPLAY

/*
  Mostly from OrangutanLCD.cpp - Library for using the LCD on the Orangutan LV, SV, SVP, X2, or 3pi robot.
  This library incorporates some code originally written by Tom Benedict as part of Orangutan-Lib.
  Released into the public domain.
*/

/*
 * Copyright (c) 2008 Pololu Corporation. For more information, see
 *
 *   http://www.pololu.com
 *   http://forum.pololu.com
 *   http://www.pololu.com/docs/0J18
 *
 * You may freely modify and share this code, as long as you keep this
 * notice intact (including the two links above).  Licensed under the
 * Creative Commons BY-SA 3.0 license:
 *
 *   http://creativecommons.org/licenses/by-sa/3.0/
 *
 * Disclaimer: To the extent permitted by law, Pololu provides this work
 * without any warranty.  It might be defective, in which case you agree
 * to be responsible for all resulting costs and damages.
 */

// Read and write timing:
// 
// Write                            Read
// 
// Set RS & R/W                 Set RS & R/W
// Wait >= 40ns                 Wait >= 40ns
// Set E High                   Set E High
// Wait >= 150ns                Wait >= 120ns
// [Data must be valid by now]  [Data is now valid for read]
// Wait >= 80ns                 Wait >= 210ns
// Set E Low                    Set E Low
// Wait >= 10ns                 RS & R/W can change
// [Data, RS, & R/W can change]
//
// In both cases, E cannot be brought high, then low, then high again
// in less than 500ns.
//
// Even though all that timing information is in nanoseconds, we need
// to be concerned about it.  At 20MHz, each AVR instruction executes
// in 50ns.  There are delays in there that must be longer than 50ns,
// so we must make sure we wait an appropriate amount of time.
//
// Initialization:
// 
// 4-Bit                            8-Bit
// 
// Wait >= 15ms                 Wait >= 15ms
// Send 0x3                     Send 0x30
// Wait >= 4.1ms                Wait >= 4.1ms
// Send 0x3                     Send 0x30
// Wait >= 100us                Wait >= 100us
// Send 0x3                     Send 0x30
// Wait >= 100us                Wait >= 100us
// Send 0x2
// Wait >= 100us                        
// Send 0x2 0x8                 Send 0x38
// [Busy Flag is now valid]     [Busy Flag is now valid]
// Check BF                     Check BF
// Send 0x0 0x8                 Send 0x08
// Check BF                     Check BF
// Send 0x0 0x1                 Send 0x01
// Check BF                     Check BF
// Send 0x0 0x6                 Send 0x06
// Check BF                     Check BF
// Send 0x0 0xC                 Send 0x0C
//
// A point to consider:  Prior to a write, the busy flag (BF)
// must be clear.  During the time BF is not valid, apparently
// it stays clear.  So the BF check can apparently operate
// even when BF is not valid.  It simply will not cause any
// delays in execution.  I do not know if this is universally
// true, but it appears to be true for the two LCD used on the
// Orangutan LV-168.
// 
// Another point:  On 4-bit interfaces, reading the busy flag
// is a little tricky.  The same command that returns the busy
// flag also returns the cursor's current memory address.  This 
// requires 8-bits of I/O, even if we're not interested in the 
// cursor's address.  So to check for the busy flag we have to
// do two 4-bit reads, complete with all the timing mentioned
// above, even though we're discarding the contents of the second
// 4-bit read.
// 
// The Orangutan LV, SV, SVP and 3pi use the LCD in 4-bit mode with E,
// R/W, and RS control lines.  The Orangutan X2 uses the LCD in 8-bit
// mode with E, R/W, and RS control lines.

#define LCD_WIDTH   8
#define LCD_HEIGHT  2

// On the Orangutan LV-168 and 3pi robot, the LCD control lines are split between
// ports B and D:

#define LCD_DB4                PORTB1        // PB1
#define LCD_DB5                PORTB4        // PB4
#define LCD_DB6                PORTB5        // PB5
#define LCD_DB7                PORTD7        // PD7

#define LCD_RS_DDR            DDRD
#define LCD_RS_PORT            PORTD
#define LCD_E_DDR            DDRD
#define LCD_E_PORT            PORTD
#define LCD_RW_DDR            DDRB
#define LCD_RW_PORT            PORTB

#define LCD_RW                PORTB0
#define LCD_RS                PORTD2
#define LCD_E                PORTD4

#define LCD_BF_DDR            DDRD
#define LCD_BF_PIN            PIND
#define LCD_BF_PORT            PORTD
#define LCD_BF_MASK            (1 << LCD_DB7)

// Since we're only using four data lines, and since the pins they're
// wired up to don't start with 0, we need to shift them into
// position in order to load their values into the LCD.  Port B uses
// bits 1, 4, and 5.  We need to make our data line up like this:
//
//    PortB:     7 6 5 4 3 2 1 0
//  LCD Data:      2 1     0
//
//  PortD:     7 6 5 4 3 2 1 0
//  LCD Data:  3
//
//  Pass your 4-bit LCD data value to the LCD_PORTB_DATA and LCD_PORTD_DATA
//  macros to get the respective PORTB and PORTD values.  You can use them
//  as follows:
//
//  PORTB &= ~LCD_PORTB_MASK;
//  PORTB |= LCD_PORTB_DATA(lcdData);
//  PORTD &= ~LCD_PORTD_MASK;
//  PORTD |= LCD_PORTD_DATA(lcdData);

#define LCD_PORTB_MASK            ((1 << LCD_DB4) | (1 << LCD_DB5) | (1 << LCD_DB6))
#define LCD_PORTD_MASK            (1 << LCD_DB7)
#define LCD_PORTB_DATA(data)    (((data & 0x01) <<1 ) | ((data & 0x06) << 3))
#define LCD_PORTD_DATA(data)    ((data & 0x08) << 4)

#define LCD_PORTB_DATA(data)    (((data & 0x01) <<1 ) | ((data & 0x06) << 3))
#define LCD_PORTD_DATA(data)    ((data & 0x08) << 4)

#define LCD_TIMEOUT 10000

#define LCD_CLEAR       0x01
#define LCD_SHOW_BLINK  0x0F
#define LCD_SHOW_SOLID  0x0E        
#define LCD_HIDE        0x0C
#define LCD_CURSOR_L    0x10
#define LCD_CURSOR_R    0x14
#define LCD_SHIFT_L     0x18
#define LCD_SHIFT_R     0x1C
#define LCD_LEFT        0
#define LCD_RIGHT       1
#define CURSOR_SOLID    0
#define CURSOR_BLINKING 1
#define LCD_CGRAM       6

namespace detail
{
class Display
{
public:    
    void init()
    {
        // Set up the DDR for the LCD control lines
        LCD_RS_DDR |= 1 << LCD_RS;
        LCD_RW_DDR |= 1 << LCD_RW;
        LCD_E_DDR |= 1 << LCD_E;

        // Wait >15ms
        delay(30);

        send_4bit_cmd(0x3);    // function set
        delay(6);    // wait >4.1ms
        send_4bit_cmd(0x3);    // function set
        delay(2);    // wait >100us
        send_4bit_cmd(0x3);    // function set
        delay(2);    // wait >100us
        send_4bit_cmd(0x2);    // 4-bit interface
        delay(2);
        send_cmd(0x28);    // 4-bit, 2 line, 5x8 dots char (busy flag is now valid)

        send_cmd(0x08);    // display off, cursor off, blinking off
        send_cmd(0x01);    // clear display
        send_cmd(0x06);    // set entry mode: cursor shifts right, no scrolling
        send_cmd(0x0C);    // display on, cursor off, blinking off
    }

    void showCursor(uint8_t cursorType)
    {
        if (cursorType == CURSOR_BLINKING)
            send_cmd(LCD_SHOW_BLINK);
        else
            send_cmd(LCD_SHOW_SOLID);
    }
    
    // shifts the cursor LEFT or RIGHT the given number of positions.
    // direction should be either LCD_LEFT or LCD_RIGHT
    void moveCursor(uint8_t direction, uint8_t num)
    {
        while(num-- > 0)
        {
            if (direction == LCD_LEFT)
                send_cmd(LCD_CURSOR_L);
            else
                send_cmd(LCD_CURSOR_R);
        }
    }
    
    // shifts the display LEFT or RIGHT the given number of
    // positions, delaying for delay_time milliseconds between each shift.
    // This is what you'd use for a scrolling display.
    // direction should be either LCD_LEFT or LCD_RIGHT
    void scroll(uint8_t direction, uint8_t num, uint16_t delay_time)
    {
        while(num--)
        {
            if (direction == LCD_LEFT)
                send_cmd(LCD_SHIFT_L);
            else
                send_cmd(LCD_SHIFT_R);
            delay(delay_time);
        }
    }
    
    // moves the cursor to the specified (x, y) position
    // x is a zero-based column indicator (0 <= x <= 7)
    // y is a zero-based row indicator (0 <= y <= LCD rows-1)
    void gotoXY(uint8_t x, uint8_t y)
    {
        // Memory locations for the start of each line
        // The actual memory locations are 0x00, and 0x40, but since
        // D7 needs to be high in order to set a new memory location, we can go
        // ahead and make the seventh bit of our memory location bytes to 1,
        // which makes the numbers 0x80 and 0xC0:

        unsigned char line_mem[] = {0x80, 0xC0, 0x94, 0xD4};

        // Grab the location in the LCD's memory of the start of line y,
        // and add X to it to get the right character location.
        send_cmd(line_mem[y] + x);
    }
    
    inline void print(const char *str)
    {
        while (*str != 0)
            send_data(*str++);
    }
    
    template <typename T>
    void printNumber(T n, uint8_t width = 0)
    {
        char buf[32];
        uint8_t len = 0;

        if (n != 0)
        {
            T a = abs(n);

            while (a > 0)
            {
                T b = a / 10;
                buf[len++] = '0' + (a - b * 10);
                a = b;
            }

            if (n < 0)
                buf[len++] = '-';
        }
        else
            buf[len++] = '0';

        for (; width > len; --width)
            send_data(' ');
        
        for (; len > 0; --len)
            send_data(buf[len-1]);
    }
    
    void loadCustomCharacter(const char *picture_p, uint8_t number)
    {
        // Each character takes up 8 bytes of the character memory, so we
        // multiply by 8 to get the address.
        number *= 8;

        for(uint8_t i = 0; i < 8; ++i)
        {
            // set CG RAM address
            send_cmd((1<<LCD_CGRAM) | (number+i));

            // write character data
            send_data(picture_p[i]);
        }
    }
    
    inline void clear() { send_cmd(LCD_CLEAR); }
    inline void send_cmd(uint8_t cmd) { send(cmd, 0, 2); }
    inline void send_4bit_cmd(uint8_t cmd) { send(cmd, 0, 1); }
    inline void send_data(uint8_t data) { send(data, 1, 2); }
    inline void hideCursor() { send_cmd(LCD_HIDE);}
    inline void printToXY(const char *str, uint8_t x, uint8_t y)
    {
        gotoXY(x, y);
        print(str);
    }
    template <typename T>
    inline void printNumToXY(T num, uint8_t x, uint8_t y)
    {
        gotoXY(x, y);
        printNumber(num);
    }

private:
    // Wait for the busy flag to clear.  The 4-bit interface is
    // more complicated than the 8-bit interface because E must
    // be strobed twice to get the full eight bits back from
    // the LCD, even though we're only interested in one of them.
    void busyWait()
    {
        uint8_t temp_ddr, data;

        // Save our DDR information
        temp_ddr = LCD_BF_DDR;

        // Set up the data DDR for input
        LCD_BF_DDR &= ~LCD_BF_MASK;

        // Set up RS and RW to read the state of the LCD's busy flag
        LCD_RS_PORT &= ~(1 << LCD_RS);
        LCD_RW_PORT |= 1 << LCD_RW;
        
        uint16_t usCounter = 0;

        do
        {
            delayMicroseconds(1);
            
            // Bring E high
            LCD_E_PORT |= 1 << LCD_E;

            // Wait at least 120ns (1us is excessive)
            delayMicroseconds(1);

            // Get the data back from the LCD
            data = LCD_BF_PIN;

            // That excessive delay means our cycle time on E cannot be
            // shorter than 1000ns (500ns being the spec), so no further
            // delays are required

            // Bring E low
            LCD_E_PORT &= ~(1 << LCD_E);
            
            usCounter += 2;


            // Wait a small bit
            delayMicroseconds(1);

            // When using the 4-bit interface, we still need to
            // strobe out the 4 bits we don't care about:

            // Bring E high
            LCD_E_PORT |= 1 << LCD_E;

            // Wait at least 120ns (1us is excessive)
            delayMicroseconds(1);

            // Bring E low
            LCD_E_PORT &= ~(1 << LCD_E);
            
            usCounter += 2;
        }
        while ((data & LCD_BF_MASK) && (usCounter < LCD_TIMEOUT));

        // To reach here our busy flag must be zero, meaning the LCD is free
        // or the 20ms timeout period has elapsed    


        // Restore our DDR information
        LCD_BF_DDR = temp_ddr;
    }

    // Send data via the 4- or 8-bit interface.  This assumes the busy flag
    // is clear, that our DDRs are all set, etc.  Basically all it does is
    // line up the bits and send them out the appropriate I/O lines while
    // strobing the E control line.
    void sendData(uint8_t data)
    {
        PORTB = (PORTB & ~LCD_PORTB_MASK) | LCD_PORTB_DATA(data);
        PORTD = (PORTD & ~LCD_PORTD_MASK) | LCD_PORTD_DATA(data);

        // At this point the four data lines are set, so the Enable pin 
        // is strobed to let the LCD latch them.

        // Bring E high
        LCD_E_PORT |= 1 << LCD_E;
        
        // Wait => 450ns (1us is excessive)
        delayMicroseconds(1);

        // Bring E low
        LCD_E_PORT &= ~(1 << LCD_E);

        delayMicroseconds(1);

        // Dropping out of the routine will take at least 10ns, the time
        // given by the datasheet for the LCD controller to read the
        // nibble on the falling edge of E

        // Our nibble has now been sent to the LCD.
    }
    
    void send(uint8_t data, uint8_t rs, uint8_t numSends)
    {
        // Wait until the busy flag clears
        busyWait();

        // Save our DDR and port information

        uint8_t temp_ddrb, temp_portb, temp_ddrd, temp_portd;
        temp_ddrb = DDRB;
        temp_portb = PORTB;
        temp_ddrd = DDRD;
        temp_portd = PORTD;

        // Clear RW and set or clear RS based on the rs argument
        LCD_RW_PORT &= ~(1 << LCD_RW);
        if (rs)
            LCD_RS_PORT |= 1 << LCD_RS;
        else
            LCD_RS_PORT &= ~(1 << LCD_RS);

        // Set the data pins as outputs

        DDRB |= LCD_PORTB_MASK;
        DDRD |= LCD_PORTD_MASK;

        if (numSends != 1)
            sendData(data >> 4);    // send high nibble via 4-bit interface
        sendData(data & 0x0F);    // send low nibble via 4-bit interface


        // Restore our DDR and port information
        DDRD = temp_ddrd;
        PORTD = temp_portd;
        DDRB = temp_ddrb;
        PORTB = temp_portb;
    } 
}; // class Display
} // namespace detail

detail::Display display;

void init_display()
{
    display.init();
}

void clean_display()
{
    display.clear();
}

#endif
#ifndef PI_LIB_RS232
#define PI_LIB_RS232

#ifndef JUNIOR_RS232_BPS
#define JUNIOR_RS232_BPS 115200
#endif

#ifndef JUNIOR_RS232_TXBUF
# define JUNIOR_RS232_TXBUF 96
#endif

#ifndef JUNIOR_RS232_RXBUF
# define JUNIOR_RS232_RXBUF 32
#endif

#ifndef JUNIOR_RS232_BPS
# error Nastavte symbol JUNIOR_RS232_BPS na rychlost, s jakou chcete komunikovat (napr. 115200).
#endif

inline void force_wd_reset()
{
    //TODO: THIS SHOULD NOT BE HERE, BUT BOOTLOADER WONT STOP PWM ON 3PI
    clean_motors();
    //display has to be cleared, too
    display.clear();

    cli();
#if defined(WDTCR)
    WDTCR = (1<<WDCE)|(1<<WDE);
    WDTCR = (1<<WDE);
#elif defined(WDTCSR)
    WDTCSR = (1<<WDCE)|(1<<WDE);
    WDTCSR = (1<<WDE);
#else
# error Unsupported Watchdog timer interface.
#endif
    for (;;)
    {
    }
}


namespace detail {

template <typename T, uint8_t size>
class queue
{
public:
    queue()
        : m_rd_ptr(0), m_wr_ptr(0)
    {
    }

    bool push(T t)
    {
        uint8_t wr = m_wr_ptr;

        uint8_t new_ptr = inc(wr);

        if (new_ptr == m_rd_ptr)
            return false;

        m_elems[wr] = t;
        m_wr_ptr = new_ptr;
        return true;
    }

    bool empty() const
    {
        return m_rd_ptr == m_wr_ptr;
    }

    bool full() const
    {
        return inc(m_wr_ptr) == m_rd_ptr;
    }

    T top() const
    {
        return m_elems[m_rd_ptr];
    }

    void pop()
    {
        m_rd_ptr = inc(m_rd_ptr);
    }

private:
    T m_elems[size];
    volatile uint8_t m_rd_ptr;
    volatile uint8_t m_wr_ptr;

    uint8_t inc(uint8_t v) const
    {
        ++v;
        return v == size? 0: v;
    }
};

}

namespace detail {

template <int rxmax, int txmax>
class rs232_t
{
public:
    void data_in(char ch)
    {
        m_rxbuf.push(ch);
    }

    bool data_out(char & ch)
    {
        if (m_txbuf.empty())
            return false;

        ch = m_txbuf.top();
        m_txbuf.pop();
        return true;
    }

    char get()
    {
        char ch;

        while (!this->peek(ch))
        {
            JUNIOR_DO_IDLE();
        }

        return ch;
    }

    bool peek(char & ch)
    {
        if (m_rxbuf.empty())
            return false;

        ch = m_rxbuf.top();
        m_rxbuf.pop();
        return true;
    }

    void sendCharacter(char ch)
    {
        while (m_txbuf.full())
            JUNIOR_DO_IDLE();
        m_txbuf.push(ch);
        UCSR0B |= (1<<UDRIE0);
    }

    void write(char ch)
    {
        sendCharacter(ch);
    }

    void send(const char * str)
    {
        for (; *str != 0; ++str)
            m_txbuf.push(*str);
        UCSR0B |= (1<<UDRIE0);
    }

    void sendHexByte(uint8_t byte)
    {
        static const char hexdigits[] = "0123456789ABCDEF";
        this->sendCharacter(hexdigits[byte >> 4]);
        this->sendCharacter(hexdigits[byte & 0x0f]);
    }

    template <typename T>
    void sendNumberHex(T n, uint8_t width = 0)
    {
        char buf[32];
        uint8_t len = 0;

        if (n != 0)
        {
            T a = abs(n);

            while (a > 0)
            {
                buf[len++] = '0' + (a & 0x0f);
                a = a >> 4;
            }

            if (n < 0)
                buf[len++] = '-';
        }
        else
            buf[len++] = '0';

        for (; width > len; --width)
            m_txbuf.push(' ');
        for (; len > 0; --len)
            m_txbuf.push(buf[len-1]);
        UCSR0B |= (1<<UDRIE0);
    }

    template <typename T>
    void sendNumber(T n, uint8_t width = 0)
    {
        char buf[32];
        uint8_t len = 0;

        if (n != 0)
        {
            T a = abs(n);

            while (a > 0)
            {
                T b = a / 10;
                buf[len++] = '0' + (a - b * 10);
                a = b;
            }

            if (n < 0)
                buf[len++] = '-';
        }
        else
            buf[len++] = '0';

        for (; width > len; --width)
            m_txbuf.push(' ');
        for (; len > 0; --len)
            m_txbuf.push(buf[len-1]);
        UCSR0B |= (1<<UDRIE0);
    }

    template <typename T>
    void dumpNumber(T n)
    {
        sendNumber(n);
        sendCharacter(' ');
    }

    void wait()
    {
        while (!m_txbuf.empty())
        {
            JUNIOR_DO_IDLE();
        }
    }

    bool txempty() const
    {
        return m_txbuf.empty();
    }

private:
    detail::queue<char, rxmax> m_rxbuf;
    detail::queue<char, txmax> m_txbuf;
};

}

detail::rs232_t<JUNIOR_RS232_RXBUF, JUNIOR_RS232_TXBUF> rs232;

inline void init_rs232()
{
    // Initialize the RS232 line
    UCSR0A = (1<<U2X0);
    UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
#define JUNIOR_UBRR ((F_CPU + 4 * JUNIOR_RS232_BPS) / (8 * JUNIOR_RS232_BPS) - 1)
    UBRR0H = JUNIOR_UBRR >> 8;
    UBRR0L = JUNIOR_UBRR;
    UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
}


inline void clean_rs232()
{
    UCSR0B = 0;
    UCSR0A = 0;
    UCSR0C = 0;
}

ISR(USART_RX_vect)
{
    unsigned char ch = UDR0;
    {
        static const unsigned char bootSeq[] = { 0x74, 0x7E, 0x7A, 0x33 };
        static uint8_t state = 0;

        if (ch == bootSeq[state])
        {
            if (++state == 4)
                force_wd_reset();
        }
        else
            state = 0;
    }
    rs232.data_in(ch);
}

ISR(USART_UDRE_vect)
{
    char ch;
    if (rs232.data_out(ch))
        UDR0 = ch;
    else
        UCSR0B &= ~(1<<UDRIE0);
}

#endif

#ifndef PI_LIB_FORMAT
#define PI_LIB_FORMAT

template <typename Stream, typename Unsigned>
void send_hex(Stream & s, Unsigned v, uint8_t width = 0, char fill = '0')
{
    static char const digits[] = "0123456789ABCDEF";

    char buf[32];
    uint8_t i = 0;

    if (v == 0)
    {
        buf[i++] = '0';
    }
    else if (v < 0)
    {
        buf[i++] = '*';
    }
    else
    {
        for (; v != 0; v >>= 4)
        {
            buf[i++] = digits[v & 0xF];
        }
    }

    while (i < width)
        buf[i++] = fill;

    for (; i > 0; --i)
        s.write(buf[i - 1]);
}

template <typename Stream, typename Integer>
void send_int(Stream & s, Integer v, uint8_t width = 0, char fill = ' ')
{
    char buf[32];
    uint8_t i = 0;
    bool negative = false;

    if (v == 0)
    {
        buf[i++] = '0';
    }
    else 
    {
        if (v < 0)
        {
            negative = true;
            v = -v;
        }

        for (; v != 0; v /= 10)
        {
            buf[i++] = (v % 10) + '0';
        }
    }

    if (negative)
        buf[i++] = '-';

    while (i < width)
        buf[i++] = fill;

    for (; i > 0; --i)
        s.write(buf[i - 1]);
}

template <typename Stream, typename T>
void send_bin(Stream & s, T const & t)
{
    char const * ptr = reinterpret_cast<char const *>(&t);
    for (uint8_t i = 0; i < sizeof t; ++i)
        s.write(ptr[i]);
}

namespace detail {

template <typename Stream, typename Pattern>
class format_impl
{
public:
    format_impl(Stream & out, Pattern const & pattern)
        : m_out(out), m_pattern(pattern)
    {
        this->write_literal();
    }

    format_impl & operator%(const char& ch)
    {
        m_out.write(ch);
        m_pattern.pop();
        this->write_literal();
        return *this;
    }

    format_impl & operator%(char const * str)
    {
        while (*str)
            m_out.write(*str++);
        m_pattern.pop();
        this->write_literal();
        return *this;
    }

    format_impl & operator%(const bool& v)
    {
        m_out.write(v ? '1' : '0');
        m_pattern.pop();
        this->write_literal();
        return *this;
    }

    template <typename T>
    format_impl & operator%(T const & t)
    {
        char f = m_pattern.top();
        bool hex =  false;
        uint8_t width = 0;
        if(f == 'x')
        {
            hex = true;
            m_pattern.pop();
            f = m_pattern.top();
        }
        if(f >='0' && f <= '9')
            width = f - '0';
        if (hex)
            send_hex(m_out, t, width);
        else
            send_int(m_out, t, width);
        if(!m_pattern.empty())
            m_pattern.pop();
        this->write_literal();
        return *this;
    }

private:
    void write_literal()
    {
        bool escape = false;
        for (; !m_pattern.empty(); m_pattern.pop())
        {
            char ch = m_pattern.top();

            if (ch == '%')
            {
                if (escape)
                {
                    m_out.write('%');
                    m_out.write('%');
                    escape = false;
                }
                else
                {
                    escape = true;
                }
            }
            else
            {
                if (escape)
                    break;
                m_out.write(ch);
            }
        }
    }

    Stream &m_out;
    Pattern m_pattern;
};

class string_literal_range
{
public:
    string_literal_range(char const * pattern) :
        m_pattern(pattern)
    {
    }

    bool empty() const
    {
        return *m_pattern == 0;
    }

    char top() const
    {
        return *m_pattern;
    }

    void pop()
    {
        ++m_pattern;
    }

private:
    char const * m_pattern;
};

} // namespace detail

template <typename Stream>
detail::format_impl<Stream, detail::string_literal_range> format(Stream & out, char const * pattern)
{
    return detail::format_impl<Stream, detail::string_literal_range>(out, detail::string_literal_range(pattern));
}

#endif

#ifndef PI_LIB_I2C
#define PI_LIB_I2C

//TODO

#endif

#ifndef PI_LIB_BUTTONS
#define PI_LIB_BUTTONS

#define BUTTON_C        (1 << PORTB5)
#define BUTTON_B        (1 << PORTB4)
#define BUTTON_A        (1 << PORTB1)
#define ALL_BUTTONS     (BUTTON_A | BUTTON_B | BUTTON_C)

void init_buttons()
{
    DDRB &= ~ALL_BUTTONS;
    PORTB |= ALL_BUTTONS;
}

void clean_buttons()
{
    PORTB &= ~ALL_BUTTONS;
}

inline bool isPressed(uint8_t buttons)
{
    return !(PINB & buttons);
}

inline uint8_t waitForPress(uint8_t buttons)
{
    while(PINB & buttons)
        JUNIOR_DO_IDLE();
    return ((~PINB) & buttons);       // return the pressed button(s)
}

inline uint8_t waitForRelease(uint8_t buttons)
{
    while(!(PINB & buttons))
        JUNIOR_DO_IDLE();
    return (PINB & buttons);          // return the pressed button(s)
}

inline void waitForButton(uint8_t buttons)
{
    do
    {
        while(!isPressed(buttons))
            JUNIOR_DO_IDLE();
        delay(5);
    } while(!isPressed(buttons));
    
    delay(50);
    
    do
    {
        while(isPressed(buttons))
            JUNIOR_DO_IDLE();
        delay(5);
    } while(isPressed(buttons));
}

#endif

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
    run();
    cli();
    clean();
    return 0;
}

#endif
