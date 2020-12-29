#include "checksum.h"

uint16_t checksum_oc16_add(uint16_t sum, uint16_t value)
{
    if ((((uint32_t) sum) + ((uint32_t) value)) >= 0xFFFFU)
        sum += 1;

    sum += value;
    return sum;
}

uint16_t checksum_oc16(uint16_t *buffer, uint16_t len)
{
    uint16_t sum = 0x0000;

    for (uint16_t i = 0; i < len; ++i)
    {
        sum = checksum_oc16_add(sum, *buffer);
        ++buffer;
    }

    return ~sum;
}