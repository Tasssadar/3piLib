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
