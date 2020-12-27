#ifndef _ENC28J60_IP_H
#define _ENC28J60_IP_H

/**************************************************
 * Cross-Platform Standard Library's
 **************************************************/

#include <stdio.h>

/**************************************************
 * Datatypes
 **************************************************/

#define ENC28J60_IP_FLAGS_DONT_FRAGMENT 1

typedef enum
{
    ENC28J60_IP_PROTO_HOPOPT = 0,
    ENC28J60_IP_PROTO_ICMP,
    ENC28J60_IP_PROTO_IGMP,
    ENC28J60_IP_PROTO_GGP,
    ENC28J60_IP_PROTO_IPv4,
    ENC28J60_IP_PROTO_ST,
    ENC28J60_IP_PROTO_TCP,
    ENC28J60_IP_PROTO_CBT,
    ENC28J60_IP_PROTO_EGP,
    ENC28J60_IP_PROTO_IGP,
    ENC28J60_IP_PROTO_BBN_RCC_MON,
    ENC28J60_IP_PROTO_NVP_II,
    ENC28J60_IP_PROTO_PUP,
    ENC28J60_IP_PROTO_ARGUS,
    ENC28J60_IP_PROTO_EMCON,
    ENC28J60_IP_PROTO_XNET,
    ENC28J60_IP_PROTO_CHAOS,
    ENC28J60_IP_PROTO_UDP,
    ENC28J60_IP_PROTO_MUX,
    ENC28J60_IP_PROTO_DCN_MEAS,
    ENC28J60_IP_PROTO_HMP,
    ENC28J60_IP_PROTO_PRM,
    ENC28J60_IP_PROTO_XNS_IDP
} enc28j60_ip_proto_t;

typedef enum
{
    ENC28J60_IP_TOS_PRECEDENCE_ROUTINE = 0b000,
    ENC28J60_IP_TOS_PRECEDENCE_PRIORITY,
    ENC28J60_IP_TOS_PRECEDENCE_IMMEDIATE,
    ENC28J60_IP_TOS_PRECEDENCE_FLASH,
    ENC28J60_IP_TOS_PRECEDENCE_FLASH_OVR,
    ENC28J60_IP_TOS_PRECEDENCE_CRITIC_ECP,
    ENC28J60_IP_TOS_PRECEDENCE_INTERNETWORK_CTRL,
    ENC28J60_IP_TOS_PRECEDENCE_NETWORK_CTRL
} enc28j60_ip_tos_precedence_t;

typedef struct __attribute__ (( packed ))
{
    unsigned precedence : 3;
    unsigned d          : 1;
    unsigned t          : 1;
    unsigned r          : 1;
    unsigned zero       : 2;
} enc28j60_ip_tos_t;

typedef struct __attribute__ (( packed ))
{
    unsigned            ihl     : 4;    /* Internet Header Length */
    unsigned            ver     : 4;    /* Version */
    enc28j60_ip_tos_t   tos;            /* Type of service */
    uint16_t            tl;             /* Total Length */
    uint16_t            id;             /* Identification */
    unsigned            flags   : 3;    /* Flags */
    unsigned            fragof  : 13;   /* Fragment Offset */
    uint8_t             ttl;            /* Time To Live */
    uint8_t             proto;          /* Protocol */
    uint16_t            hdr_cs;         /* Header Checksum */
    uint8_t             sa[4];          /* Source Address */
    uint8_t             da[4];          /* Destination Address */
    uint8_t             payload[0];
} enc28j60_ip_hdr_t;

#endif
