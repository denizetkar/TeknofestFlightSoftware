#ifndef PRINT_64BIT_H
#define PRINT_64BIT_H

#include <Arduino.h>

template<int base = 10>
void print_uint64_t(uint64_t number)
{
    unsigned char buf[int(ceil(64*log(2)/log(base)))];
    uint8_t i = 0;

    if (number == 0)
    {
        Serial.print((char)'0');
        return;
    }

    while (number > 0)
    {
        uint64_t q = number/base;
        buf[i++] = number - q*base;
        number = q;
    }

    for (; i > 0; i--)
    Serial.print((char) (buf[i - 1] < 10 ?
    '0' + buf[i - 1] :
    'A' + buf[i - 1] - 10));
}

template<int base = 10>
void print_int64_t(int64_t number)
{
    if (number < 0)
    {
        Serial.print('-');
        number = -number;
    }
    print_uint64_t<base>((uint64_t)number);
}

#endif
