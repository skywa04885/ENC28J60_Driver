#ifndef _ENC28J60_UDP_H
#define _ENC28J60_UDP_H

/**************************************************
 * Cross-Platform Standard Library's
 **************************************************/

#include <stdio.h>
#include <string.h>

/**************************************************
 * Project Header Files
 **************************************************/

#include "ip.h"

/**************************************************
 * Datatypes
 **************************************************/

typedef struct __attribute__ (( packed )) {
    uint16_t sp;        /* Source Port */
    uint16_t dp;        /* Destination Port */
    uint16_t l;         /* Length */
    uint16_t cs;        /* Checksum */
    uint8_t payload[0];
} enc28j60_udp_packet_t;

/**************************************************
 * Prototypes
 **************************************************/

uint16_t enc28j60_udp_calc_cs(enc28j60_ip_hdr_t *ip_hdr, enc28j60_udp_packet_t *udp_hdr);

#endif
