#ifndef PI_LIB_BUZZER
#define PI_LIB_BUZZER

class buzzer_t
{
public:
    buzzer_t()
    {
        m_running = m_emergencyEnabled = m_started = false;
        m_freq = 1250;
        m_time_on = m_time_off = 0;
    }

    void set(uint16_t time_on = 0, uint16_t time_off = 0, bool run = true)
    {
        m_running = false;
       // m_freq = F_CPU/(hz*16); // 2*prescaler(8)
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

        if(!m_time_on || !m_time_off)
            m_timer = -1;
        else
            m_timer = m_time_on;
        TCCR1A |= (1 << COM1B0);
        OCR1BH = (m_freq >> 8);
        OCR1BL = uint8_t(m_freq);
    }

    void stop()
    {
        m_running = m_started = false;
        TCCR1A &= ~(1 << COM1B0);
        OCR1BH = 0;
        OCR1BL = 0;
    }

    bool isStarted()
    {
        return m_started;
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
                m_timer = m_time_off;
                TCCR1A &= ~(1 << COM1B0);
                OCR1BH = 0;
                OCR1BL = 0;
            }
            else
            {
                m_timer = m_time_on;
                TCCR1A |= (1 << COM1B0);
                OCR1BH = (m_freq >> 8);
                OCR1BL = uint8_t(m_freq);
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
    void setEmergencyEnabled(bool enable) { return m_emergencyEnabled = enable; }

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