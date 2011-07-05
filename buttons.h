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
