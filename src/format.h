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