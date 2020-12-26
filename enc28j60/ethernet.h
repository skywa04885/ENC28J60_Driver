#ifndef _ENC28J60_ETHERNET_H
#define _ENC28J60_ETHERNET_H

#include "stdint.h"

/**************************************************
 * OSI Layer 2 / Ethernet
 **************************************************/

#define ENC28J60_ETHERNET_PACKET_TYPE_IPV4              0x0800
#define ENC28J60_ETHERNET_PACKET_TYPE_ARP               0x0806

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
    uint16_t    type;
    uint8_t     payload[0];
} enc28j60_packet_data_t;

typedef struct __attribute__ (( packed )) {
    enc28j60_packet_control_byte_t  cb;
    enc28j60_packet_data_t          data; /* Data & Status Vector */
} enc28j60_ethernet_packet_t;

/**************************************************
 * OSI Layer 3 / ARP
 **************************************************/

typedef enum
{
    ENC28J60_ARP_PACKET_OP_REQUEST = 1,
    ENC28J60_ARP_PACKET_OP_REPLY = 2
} enc28j60_arp_packet_op_t;

typedef struct __attribute__ (( packed ))
{
    uint16_t    hrd;        /* Hardware Address Space */
    uint16_t    pro;        /* Protocol Address Space */
    uint8_t     hln;        /* Byte Length Of Hardware Address */
    uint8_t     pln;        /* Byte-Length Of Protocol Address */
    uint16_t    op;         /* Opcode */
    uint8_t     sha[6];     /* Hardware Address of sender */
    uint8_t     spa[4];     /* Protocol Address of sender */
    uint8_t     tha[6];     /* Hardware Address of target */
    uint8_t     tpa[4];     /* Protocol Address of target */
} enc28j60_arp_packet_t;

/**************************************************
 * OSI Layer 3 / IP
 **************************************************/

#endif
