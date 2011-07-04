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
    return (num < 0 ? -num : num);
}
#endif
