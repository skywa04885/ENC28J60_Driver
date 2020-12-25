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
 * Cross-Platform Standard Library's
 **************************************************/

#include <stdint.h>

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
 * Project headers
 **************************************************/

#include "ethernet.h"

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

/**************************************************
 * Configuration
 **************************************************/

typedef struct
{
    uint8_t mac[6];
    /* Bit-Fields */
    unsigned full_duplex : 1;
} enc28j60_driver_cfg_t;

/**************************************************
 * Hardware Prototypes
 **************************************************/

void        spi_init(void);
uint8_t     spi_transceive(uint8_t b);
void        spi_select(void);
void        spi_deselect(void);

/**************************************************
 * Command Prototypes
 **************************************************/

void        enc28j60_wcr(uint8_t reg, uint8_t val);
void        enc28j60_wbm(uint8_t *data, uint16_t len);

void        enc28j60_bfs(uint8_t reg, uint8_t mask);
void        enc28j60_bfc(uint8_t reg, uint8_t mask);

uint8_t     enc28j60_mii_rcr(uint8_t reg);
uint8_t     enc28j60_eth_rcr(uint8_t reg);

void        enc28j60_src();

/**************************************************
 * PHY Writing / Reading Prototypes
 **************************************************/

void        enc28j60_phy_write(uint8_t reg, uint16_t value);
uint16_t    enc28j60_phy_read(uint8_t reg);

/**************************************************
 * Prototypes
 **************************************************/

void        enc28j60_init(enc28j60_driver_cfg_t *cfg);
void        enc28j60_wait_clkrdy();
void        enc28j60_mac_init();
void        enc28j60_rx_init();
void        enc28j60_filter_init();
void        enc28j60_phy_init(enc28j60_driver_cfg_t *cfg);

void        enc28j60_bank_select(enc28j60_bank_t bank);
void        enc28j60_write_mac(uint8_t *mac);
void        enc28j60_read_mac(uint8_t *mac);
void        enc28j60_mistat_wait_busy();

void        enc28j60_enable_led_stretch();
void        enc28j60_disable_led_stretch();
void        enc28j60_set_led_stretch_time(enc28j60_phlcon_lfrq_t time);
void        enc28j60_set_led_a_mode(enc28j60_phlcon_lacfg_t mode);
void        enc28j60_set_led_b_mode(enc28j60_phlcon_lacfg_t mode);

void        enc28j60_write_packet(enc28j60_ethernet_packet_t *packet);

#endif
