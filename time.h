#ifndef PI_LIB_TIME
#define PI_LIB_TIME

// Delays for for the specified nubmer of microseconds.
inline void delayMicroseconds(unsigned int microseconds)
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

// TODO

#endif
