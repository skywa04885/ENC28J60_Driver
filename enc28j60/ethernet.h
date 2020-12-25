#ifndef _ENC28J60_ETHERNET_H
#define _ENC28J60_ETHERNET_H

#include "stdint.h"

typedef struct __attribute__ (( packed )) {
    unsigned poverride  : 1;
    unsigned pcrcen     : 1;
    unsigned ppaden     : 1;
    unsigned phugeen    : 1;
    unsigned unused     : 4;
} enc28j60_packet_control_byte_t;

typedef struct __attribute__ (( packed ))
{
    uint8_t     da[6];
    uint8_t     sa[6];
    uint16_t    len;
    uint8_t     payload[0];
} enc28j60_packet_data_t;

typedef struct __attribute__ (( packed )) {
    enc28j60_packet_control_byte_t  cb;
    enc28j60_packet_data_t          data; /* Data & Status Vector */
} enc28j60_ethernet_packet_t;

#endif
