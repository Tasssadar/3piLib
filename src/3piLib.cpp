#include "3piLib.h"

void run()
{
    format(rs232, "test %1 , %2") % 4 % 5;
    while(true) {}
}