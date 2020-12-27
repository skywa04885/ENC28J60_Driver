#ifndef _ENC28J60_DRIVER_H
#define _ENC28J60_DRIVER_H

#define _ENC28J60_AVR

/**************************************************
 * Configuration
 **************************************************/

#define ENC28J60_MAX_FRAME_LEN  512UL
#define ENC28J60_RXBUFF_END     4096UL
#define ENC28J60_TXBUFF_START   4098UL

#ifdef _ENC28J60_AVR

#define ENC28J60_DDR            DDRB
#define ENC28J60_PORT           PORTB
#define ENC28J60_PIN            PINB
#define ENC28J60_CS             PB0             /* D8 */

#endif

/**************************************************
 * Runtime/Compiler time conversiona
 **************************************************/

#define HTON16(A) (((A & 0x00FF) << 8) | ((A & 0xFF00) >> 8))
#define NTOH16(A) HTON16(A)

/**************************************************
 * Cross-Platform Standard Library's
 **************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/**************************************************
 * AVR Device Support
 **************************************************/

#ifdef _ENC28J60_AVR

#include <avr/io.h>
#include <util/delay.h>

#define SPI_DDR                 DDRB
#define SPI_PORT                PORTB
#define SPI_SCK                 PB5
#define SPI_MISO                PB4
#define SPI_MOSI                PB3
#define SPI_SS                  PB2

#define enc28j60_delay_us       _delay_us
#define enc28j60_delay_ms       _delay_ms

#endif

/**************************************************
 * GCC Headers
 **************************************************/

#include <string.h>

/**************************************************
 * Project headers
 **************************************************/

#include "ethernet.h"
#include "ip.h"
#include "udp.h"

/**************************************************
 * Hardware Types
 **************************************************/

typedef enum
{
    ENC28J60_RCR = 0b000,
    ENC28J60_RBM,
    ENC28J60_WCR,
    ENC28J60_WBM,
    ENC28J60_BFS,
    ENC28J60_BFC,
    ENC28J60_SRC = 0b111
} enc28j60_spi_opcode_t;

typedef enum
{
    ENC28J60_BANK_0 = 0b00,
    ENC28J60_BANK_1,
    ENC28J60_BANK_2,
    ENC28J60_BANK_3
} enc28j60_bank_t;

typedef enum {
    ENC28J60_PHLCON_LACFG_DISPLAY_TX_ACTIVITY = 0b0001,
    ENC28J60_PHLCON_LACFG_DISPLAY_RX_ACTIVITY,
    ENC28J60_PHLCON_LACFG_DISPLAY_COL_ACTIVITY,
    ENC28J60_PHLCON_LACFG_DISPLAY_LINK_STATUS,
    ENC28J60_PHLCON_LACFG_DISPLAY_DUPLEX_STATUS,
    ENC28J60_PHLCON_LACFG_DISPLAY_RX_TX_ACTIVITY = 0b0111,
    ENC28J60_PHLCON_LACFG_ON,
    ENC28J60_PHLCON_LACFG_OFF,
    ENC28J60_PHLCON_LACFG_BLINK_FAST,
    ENC28J60_PHLCON_LACFG_BLINK_SLOW,
    ENC28J60_PHLCON_LACFG_DISPLAY_LINK_STATUS_AND_RX_ACTIVITY,
    ENC28J60_PHLCON_LACFG_DISPLAY_LINK_STATUS_AND_RX_TX_ACTIVITY,
    ENC28J60_PHLCON_LACFG_DISPLAY_DUPLEX_STATUS_AND_COLL_ACTIVITY
} enc28j60_phlcon_lacfg_t;

typedef enum {
    ENC28J60_PHLCON_LFRQ_NSTRCH = 0b00,
    ENC28J60_PHLCON_LFRQ_MSTRCH,
    ENC28J60_PHLCON_LFRQ_LSTRCH
} enc28j60_phlcon_lfrq_t;

typedef enum
{
    ENC28J60_OK = 0,
    ENC28J60_NO_PKT_AVAILABLE,
} enc28j60_err_t;

typedef struct __attribute__ (( packed ))
{
    uint16_t rbc;               /* Receive Byte-Count */
    unsigned lede       : 1;    /* Logen Event / Drop Event */
    unsigned reserved1  : 1;
    unsigned ceps       : 1;    /* Carrier Event Previously Seen */
    unsigned reserved2  : 1;
    unsigned crcerr     : 1;    /* CRC Error */
    unsigned lce        : 1;    /* Length Check Error */
    unsigned loor       : 1;    /* Length Out of Range */
    unsigned rok        : 1;    /* Received OK */
    unsigned rmp        : 1;    /* Receive Multicast Packet */
    unsigned rbp        : 1;    /* Receive Broadcast Packet */
    unsigned dn         : 1;    /* Dribble Nibble ( LOL ) */
    unsigned rcf        : 1;    /* Receive Control Frame */
    unsigned rpcf       : 1;    /* Receive Pause Control Frame */
    unsigned ruo        : 1;    /* Receive Unknown Opcode */
    unsigned rvtd       : 1;    /* Receive VLAN Type Detected */
    unsigned zero       : 1;
} enc28j60_receive_status_vector_t;

/**************************************************
 * Configuration
 **************************************************/

typedef struct
{
    uint8_t mac[6];
    uint8_t ipv4[4];
    /* Callbaks */
    void (*udp_callback)(uint16_t, uint16_t, uint8_t *);
    /* Bit-Fields */
    unsigned full_duplex : 1;
} enc28j60_driver_cfg_t;

/**************************************************
 * Hardware Prototypes
 **************************************************/

void            spi_init(void);
uint8_t         spi_transceive(uint8_t b);
void            spi_select(void);
void            spi_deselect(void);

/**************************************************
 * Command Prototypes
 **************************************************/

void            enc28j60_wcr(uint8_t reg, uint8_t val);
void            enc28j60_wbm(uint8_t *data, uint16_t len);

void            enc28j60_bfs(uint8_t reg, uint8_t mask);
void            enc28j60_bfc(uint8_t reg, uint8_t mask);

uint8_t         enc28j60_mii_rcr(uint8_t reg);
uint8_t         enc28j60_eth_rcr(uint8_t reg);
void            enc28j60_rbm(uint8_t *data, uint16_t len);

void            enc28j60_src();

/**************************************************
 * PHY Writing / Reading Prototypes
 **************************************************/

void            enc28j60_phy_write(uint8_t reg, uint16_t value);
uint16_t        enc28j60_phy_read(uint8_t reg);

/**************************************************
 * Prototypes
 **************************************************/

uint8_t *       enc28j60_get_buffer();

void            enc28j60_init(enc28j60_driver_cfg_t *cfg);
void            enc28j60_wait_clkrdy();
void            enc28j60_mac_init();
void            enc28j60_rx_init();
void            enc28j60_filter_init();
void            enc28j60_phy_init(enc28j60_driver_cfg_t *cfg);
void            enc28j60_rx_disable();
void            enc28j60_rx_enable();

void            enc28j60_bank_select(enc28j60_bank_t bank);
void            enc28j60_write_mac(uint8_t *mac);
void            enc28j60_read_mac(uint8_t *mac);
void            enc28j60_mistat_wait_busy();
bool            enc28j60_is_link_up(void);

void            enc28j60_enable_led_stretch();
void            enc28j60_disable_led_stretch();
void            enc28j60_set_led_stretch_time(enc28j60_phlcon_lfrq_t time);
void            enc28j60_set_led_a_mode(enc28j60_phlcon_lacfg_t mode);
void            enc28j60_set_led_b_mode(enc28j60_phlcon_lacfg_t mode);

uint8_t         enc28j60_get_packet_count();
void            enc28j60_write_packet(enc28j60_ethernet_packet_t *packet, uint16_t len);
enc28j60_err_t  enc28j60_read_packet(enc28j60_ethernet_packet_t *packet);

void            enc28j60_prepare_ethernet_packet(enc28j60_driver_cfg_t *cfg, uint8_t *buffer);
void            enc28j60_send_arp(enc28j60_driver_cfg_t *cfg, uint8_t *ip);
void            enc28j60_send_udp(enc28j60_driver_cfg_t *cfg, uint16_t port, uint16_t len, uint8_t *data, uint8_t *ipv4, uint8_t *mac);
void            enc28j60_event_poll(enc28j60_driver_cfg_t *cfg);

/**************************************************
 * Data Printing Prototypes
 **************************************************/

void            print_mac(uint8_t *mac, FILE *ofstream);

#endif
