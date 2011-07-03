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
            send_data(' ');
        
        for (; len > 0; --len)
            send_data(buf[len-1]);
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