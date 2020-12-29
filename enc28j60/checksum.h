#ifndef _ENC28J60_CHECKSUM_H
#define _ENC28J60_CHECKSUM_H

#include <stdint.h>

uint16_t checksum_oc16_add(uint16_t sum, uint16_t value);
uint16_t checksum_oc16(uint16_t *buffer, uint16_t len);

#endif
