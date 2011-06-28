#define PI_LIB_VERSION 1

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

#endif

#ifndef PI_LIB_TIME
#define PI_LIB_TIME

// TODO

#endif

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

#ifndef PI_LIB_SENSORS
#define PI_LIB_SENSORS

#endif

#ifndef PI_LIB_RS232
#define PI_LIB_RS232

#ifndef JUNIOR_RS232_BPS
#define JUNIOR_RS232_BPS 38400
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


void setMotorPower(int16_t left, int16_t right);

inline void force_wd_reset()
{
    //TODO: THIS SHOULD NOT BE HERE, BUT BOOTLOADER WONT STOP PWM ON 3PI
    clean_motors();

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
        m_txbuf.push(ch);
        UCSR0B |= (1<<UDRIE0);
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
            T a = (n < 0)? -n: n;

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
            T a = (n < 0)? -n: n;

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

#ifndef PI_LIB_I2C
#define PI_LIB_I2C

//TODO

#endif

#ifndef PI_LIB_INIT
#define PI_LIB_INIT

void init()
{
#ifdef PI_LIB_TIME
//    init_timer(); TODO
#endif

#ifdef PI_LIB_MOTORS
    init_motors();
#endif

#ifdef PI_LIB_SENSORS
//    init_sensors(); TODO
#endif

#ifdef PI_LIB_RS232
    init_rs232();
#endif

#ifdef PI_LIB_I2C
//    init_i2c(); TODO
#endif
}

void clean()
{
#ifdef PI_LIB_TIME
//    clean_timer(); TODO
#endif

#ifdef PI_LIB_MOTORS
    clean_motors();
#endif

#ifdef PI_LIB_SENSORS
//    clean_sensors(); TODO
#endif

#ifdef PI_LIB_RS232
    clean_rs232();
#endif

#ifdef PI_LIB_I2C
//    clean_i2c(); TODO
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
