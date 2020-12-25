#include "driver.h"
#include "registers.h"

/**************************************************
 * AVR Hardware Functions
 **************************************************/

#ifdef _ENC28J60_AVR

void spi_init(void)
{
    // Configure SPI GPIO's
    SPI_DDR |= _BV(SPI_MOSI) | _BV(SPI_SCK) | _BV(SPI_SS);

    // Configure SPI Hardware
    SPSR ^= SPSR;                                   // Clear SPCR
    SPCR = _BV(MSTR) | _BV(SPE) | _BV(SPR0);        // Master & Enable & 1 Mhz
}

uint8_t spi_transceive(uint8_t b)
{
    SPDR = b;
    while(!(SPSR & _BV(SPIF)));
    return SPDR;
}

void spi_select(void)
{
    ENC28J60_PORT &= ~_BV(ENC28J60_CS);     // CS: Low
}

void spi_deselect(void)
{
    ENC28J60_PORT |= _BV(ENC28J60_CS);      // CS: High
}

#endif

/**************************************************
 * Command Functions
 **************************************************/

void enc28j60_wcr(uint8_t reg, uint8_t val)
{
    spi_select();

    spi_transceive((ENC28J60_WCR << 5) | (reg & 0b00011111));
    spi_transceive(val);
    
    spi_deselect();
}

void enc28j60_wbm(uint8_t *data, uint16_t len)
{
    spi_select();

    spi_transceive((ENC28J60_WBM << 5) | 0x1A);
    for (uint16_t i = 0; i < len; ++i)
        spi_transceive(data[i]); 

    spi_deselect();
}

void enc28j60_bfs(uint8_t reg, uint8_t mask)
{
    spi_select();

    spi_transceive((ENC28J60_BFS << 5) | (reg & 0b00011111));
    spi_transceive(mask);

    spi_deselect();
}

void enc28j60_bfc(uint8_t reg, uint8_t mask)
{
    spi_select();

    spi_transceive((ENC28J60_BFC << 5) | (reg & 0b00011111));
    spi_transceive(mask);

    spi_deselect();
}

uint8_t enc28j60_mii_rcr(uint8_t reg)
{
    uint8_t res;

    spi_select();
    
    spi_transceive((ENC28J60_RCR << 5) | (reg & 0b00011111));
    spi_transceive(0xFF); // Dummy Byte
    res = spi_transceive(0xFF);

    spi_deselect();

    return res;
}

uint8_t enc28j60_eth_rcr(uint8_t reg)
{
    uint8_t res;

    spi_select();

    spi_transceive((ENC28J60_RCR << 5) | (reg & 0b00011111));
    res = spi_transceive(0xFF);

    spi_deselect();

    return res;
}

void enc28j60_src()
{
    spi_select();
    spi_transceive((ENC28J60_SRC << 5) | 0b00011111);
    spi_deselect();
}

/**************************************************
 * PHY Writing / Reading Prototypes
 **************************************************/

void enc28j60_phy_write(uint8_t reg, uint16_t value)
{
    enc28j60_bank_select(ENC28J60_BANK_2);

    // 1. Write the address of the PHY register to write to
    //  into the MIREGADR register.

    enc28j60_wcr(ENC28J60_BK2_MIREGADR, reg);

    // 2. Write the lower 8 bits of data to write into the
    //  MIWRL register.

    enc28j60_wcr(ENC28J60_BLK2_MIWRL, (uint8_t) value);

    // 3. Write the upper 8 bits of data to write into the
    //  MIWRH register. Writing to this register auto-
    //  matically begins the MIIM transaction, so it must
    //  be written to after MIWRL. The MISTAT.BUSY
    //  bit becomes set.
    
    enc28j60_wcr(ENC28J60_BLK2_MIWRH, (uint8_t) (value >> 8));

    enc28j60_mistat_wait_busy();
}

uint16_t enc28j60_phy_read(uint8_t reg)
{
    uint16_t res;

    enc28j60_bank_select(ENC28J60_BANK_2);

    // 1. Write the address of the PHY register to read
    //  from into the MIREGADR register.

    enc28j60_wcr(ENC28J60_BK2_MIREGADR, reg);

    // 2. Set the MICMD.MIIRD bit. The read operation
    //  begins and the MISTAT.BUSY bit is set.

    enc28j60_wcr(ENC28J60_BK2_MICMD, _BV(ENC28J60_BK2_MICMD_MIIRD));

    // 3. Wait 10.24 μs. Poll the MISTAT.BUSY bit to be
    //  certain that the operation is complete. While
    //  busy, the host controller should not start any
    //  MIISCAN operations or write to the MIWRH
    //  register.
    //  When the MAC has obtained the register
    //  contents, the BUSY bit will clear itself.

    _delay_us(11);

    // Polls until BUSY flag is cleared, after which we select bank 2 again
    //  since MISTAT is in bank 3
    enc28j60_mistat_wait_busy();
    enc28j60_bank_select(ENC28J60_BANK_2);

    // 4. Clear the MICMD.MIIRD bit.
    enc28j60_wcr(ENC28J60_BK2_MICMD, 0x00);

    // 5. Read the desired data from the MIRDL and
    //  MIRDH registers. The order that these bytes are
    //  accessed is unimportant.
    res = enc28j60_mii_rcr(ENC28J60_BLK2_MIRDH);
    res <<= 8;
    res |= enc28j60_mii_rcr(ENC28J60_BLK2_MIRDL);

    return res;
}

/**************************************************
 * Functions
 **************************************************/

void enc28j60_init(enc28j60_driver_cfg_t *cfg)
{
    // Configure ENC28J60 GPIO's
    ENC28J60_DDR |= _BV(ENC28J60_CS);

    // Sets CS high, and initializes SPI
    spi_deselect();
    spi_init();

    // Waits for the clock to be stable, and performs software
    //  reset, to make sure everything is default
    enc28j60_wait_clkrdy();
    enc28j60_src();

    // Performs ENC28J60 default initialization
    enc28j60_rx_init();
    enc28j60_filter_init();
    enc28j60_mac_init(cfg);
    enc28j60_phy_init(cfg);

    // Sets the default LED's to RX/TX, and stretch time
    enc28j60_enable_led_stretch();
    enc28j60_set_led_stretch_time(ENC28J60_PHLCON_LFRQ_MSTRCH);
    enc28j60_set_led_a_mode(ENC28J60_PHLCON_LACFG_DISPLAY_RX_ACTIVITY);
    enc28j60_set_led_b_mode(ENC28J60_PHLCON_LACFG_DISPLAY_TX_ACTIVITY);
}

void enc28j60_wait_clkrdy()
{
    register uint8_t reg;

    for (;;)
    {
        reg = enc28j60_eth_rcr(ESTAT);
        if (reg & _BV(ESTAT_CLKRDY)) break;
    }
}

void enc28j60_mac_init(enc28j60_driver_cfg_t *cfg)
{
    register uint8_t reg;

    // 1. Set the MARXEN bit in MACON1 to enable the
    //  MAC to receive frames. If using full duplex, most
    //  applications should also set TXPAUS and
    //  RXPAUS to allow IEEE defined flow control to
    //  function.

    enc28j60_bank_select(ENC28J60_BANK_2);

    reg = enc28j60_mii_rcr(ENC28J60_BK2_MACON1);

    reg |= _BV(ENC28J60_BK2_MACON1_MARXEN);

    if (cfg->full_duplex)
        reg |= _BV(ENC28J60_BK2_MACON1_TXPAUS) | _BV(ENC28J60_BK2_MACON1_RXPAUS);

    enc28j60_wcr(ENC28J60_BK2_MACON1, reg);

    // 2. Configure the PADCFG, TXCRCEN and
    //  FULDPX bits of MACON3. Most applications
    //  should enable automatic padding to at least
    //  60 bytes and always append a valid CRC. For
    //  convenience, many applications may wish to set
    //  the FRMLNEN bit as well to enable frame length
    //  status reporting. The FULDPX bit should be set
    //  if the application will be connected to a
    //  full-duplex configured remote node; otherwise, it
    //  should be left clear.

    reg = enc28j60_mii_rcr(ENC28J60_BK2_MACON3);
    
    reg |= _BV(ENC28J60_BK2_MACON3_TXCRCEN)
        | _BV(ENC28J60_BK2_MACON3_FRMLNEN)
        | ENC28J60_BK2_MACON3_PADCFG(0x1);

    if (cfg->full_duplex)
        reg |= _BV(ENC28J60_BK2_MACON3_FULDPX);
    
    enc28j60_wcr(ENC28J60_BK2_MACON3, reg);

    // 3. Configure the bits in MACON4. For conform-
    //  ance to the IEEE 802.3 standard, set the
    //  DEFER bit.

    if (!cfg->full_duplex) // 'Defer Transmission Enable bit (applies to half duplex only)'
    {
        reg = enc28j60_mii_rcr(ENC28J60_BK2_MACON4);
        reg |= _BV(ENC28J60_BK2_MACON4_DEFER);
        enc28j60_wcr(ENC28J60_BK2_MACON4, reg);
    }

    // 4. Program the MAMXFL registers with the maxi-
    //  mum frame length to be permitted to be received
    //  or transmitted. Normal network nodes are
    //  designed to handle packets that are 1518 bytes
    //  or less.

    enc28j60_wcr(ENC28J60_BK2_MAMXFLL, (uint8_t) ENC28J60_MAX_FRAME_LEN);
    enc28j60_wcr(ENC28J60_BK2_MAMXFLH, (uint8_t) (ENC28J60_MAX_FRAME_LEN >> 8));

    // 5. Configure the Back-to-Back Inter-Packet Gap
    //  register, MABBIPG. Most applications will pro-
    //  gram this register with 15h when Full-Duplex
    //  mode is used and 12h when Half-Duplex mode
    //  is used.

    if (cfg->full_duplex)
        enc28j60_wcr(ENC28J60_BK2_MABBIPG, 0x15);
    else
        enc28j60_wcr(ENC28J60_BK2_MABBIPG, 0x12);

    // 6. Configure the Non-Back-to-Back Inter-Packet
    //  Gap register low byte, MAIPGL. Most applications
    //  will program this register with 12h.

    enc28j60_wcr(ENC28J60_BK2_MAIPGL, 0x12);

    // 7. If half duplex is used, the Non-Back-to-Back
    //  Inter-Packet Gap register high byte, MAIPGH,
    //  should be programmed. Most applications will
    //  program this register to 0Ch.

    if (!cfg->full_duplex)
        enc28j60_wcr(ENC28J60_BK2_MAIPGH, 0x0C);

    // 8. If Half-Duplex mode is used, program the
    //  Retransmission and Collision Window registers,
    //  MACLCON1 and MACLCON2. Most applications
    //  will not need to change the default Reset values.
    //  If the network is spread over exceptionally long
    //  cables, the default value of MACLCON2 may
    //  need to be increased.

    // Ignoring for now ...

    // 9. Program the local MAC address into the 
    //  MAADR1:MAADR6 registers.
    enc28j60_write_mac(cfg->mac);
}

void enc28j60_rx_init()
{
    enc28j60_bank_select(ENC28J60_BANK_0);

    // Sets the RX Buffer start
    enc28j60_wcr(ENC28J60_BK0_ERXSTL, 0x00);
    enc28j60_wcr(ENC28J60_BK0_ERXSTH, 0x00);

    // Sets the RX BUffer end, half of buffer by default
    enc28j60_wcr(ENC28J60_BK0_ERXNDL, (uint8_t) ENC28J60_RXBUFF_END);
    enc28j60_wcr(ENC28J60_BK0_ERXNDH, (uint8_t) (ENC28J60_RXBUFF_END >> 8));
}

void enc28j60_filter_init()
{
    enc28j60_bank_select(ENC28J60_BANK_1);

    register uint8_t reg = _BV(ENC28J60_BK1_ERXFCON_ANDOR)  // Reject packet, unless all filters accept
        | _BV(ENC28J60_BK1_ERXFCON_CRCEN)                   // Reject packet, if CRC invalid
        | _BV(ENC28J60_BK1_ERXFCON_UCEN);                   // Reject packet, if we're not target MAC

    enc28j60_wcr(ENC28J60_BK1_ERXFCON, reg);
}

void enc28j60_phy_init(enc28j60_driver_cfg_t *cfg)
{
    uint16_t reg;

    // For proper duplex operation, the PHCON1.PDPXMD
    //  bit must also match the value of the MACON3.FULDPX
    //  bit.
    if (cfg->full_duplex)
    {
        reg = enc28j60_phy_read(ENC28J60_PHY_PHCON1);
        reg |= _BV(ENC28J60_PHY_PHCON1_PDPXMD);
        enc28j60_phy_write(ENC28J60_PHY_PHCON1, reg);

        reg = enc28j60_phy_read(ENC28J60_PHY_PHCON1);
    }

    // If using half duplex, the host controller may wish to set
    //  the PHCON2.HDLDIS bit to prevent automatic
    //  loopback of the data which is transmitted.
    if (!cfg->full_duplex)
    {
        reg = enc28j60_phy_read(ENC28J60_PHY_PHCON2);
        reg |= _BV(ENC28J60_PHY_PHCON2_HDLDIS);
        enc28j60_phy_write(ENC28J60_PHY_PHCON2, reg);
    }
}

void enc28j60_bank_select(enc28j60_bank_t bank)
{
    register uint8_t reg = enc28j60_eth_rcr(ECON1);      // Get previous value of ECON1
    reg &= ~(0x3);                              // Clear Bank-Select
    reg |= (bank & 0x3);                        // Write new value to Bank-Select
    enc28j60_wcr(ECON1, reg);                   // Write new value to ECON1
}

static uint8_t enc28j60_mac_registers[] = {
    ENC28J60_BK3_MAADR1,
    ENC28J60_BK3_MAADR2,
    ENC28J60_BK3_MAADR3,
    ENC28J60_BK3_MAADR4,
    ENC28J60_BK3_MAADR5,
    ENC28J60_BK3_MAADR6
};

void enc28j60_write_mac(uint8_t *mac)
{
    enc28j60_bank_select(ENC28J60_BANK_3);
    for (register uint8_t i = 0; i < 6; ++i)
        enc28j60_wcr(enc28j60_mac_registers[i], mac[i]);
}

void enc28j60_read_mac(uint8_t *mac)
{
    enc28j60_bank_select(ENC28J60_BANK_3);
    for (register uint8_t i = 0; i < 6; ++i)
        mac[i] = enc28j60_mii_rcr(enc28j60_mac_registers[i]);
}

void enc28j60_mistat_wait_busy()
{
    enc28j60_bank_select(ENC28J60_BANK_3);

    for (;;)
    {
        register uint8_t reg = enc28j60_mii_rcr(ENC28J60_BK3_MISTAT);
        if (!(reg & _BV(ENC28J60_BK3_MISTAT_BUSY))) break;
    }
}

void enc28j60_enable_led_stretch()
{
    uint16_t reg = enc28j60_phy_read(ENC28J60_PHY_PHLCON);
    reg |= _BV(ENC28J60_PHY_PHLCON_STRCH);
    enc28j60_phy_write(ENC28J60_PHY_PHLCON, reg);
}

void enc28j60_disable_led_stretch()
{
    uint16_t reg = enc28j60_phy_read(ENC28J60_PHY_PHLCON);
    reg &= ~_BV(ENC28J60_PHY_PHLCON_STRCH);
    enc28j60_phy_write(ENC28J60_PHY_PHLCON, reg);
}

void enc28j60_set_led_stretch_time(enc28j60_phlcon_lfrq_t time)
{
    uint16_t reg = enc28j60_phy_read(ENC28J60_PHY_PHLCON);
    reg &= ~ENC28J60_PHY_PHLCON_LFRQ(0x3);
    reg |= ENC28J60_PHY_PHLCON_LFRQ(time);
    enc28j60_phy_write(ENC28J60_PHY_PHLCON, reg);
}

void enc28j60_set_led_a_mode(enc28j60_phlcon_lacfg_t mode)
{
    uint16_t reg = enc28j60_phy_read(ENC28J60_PHY_PHLCON);
    reg &= ~ENC28J60_PHY_PHLCON_LACFG(0xF);
    reg |= ENC28J60_PHY_PHLCON_LACFG(mode);
    enc28j60_phy_write(ENC28J60_PHY_PHLCON, reg);
}

void enc28j60_set_led_b_mode(enc28j60_phlcon_lacfg_t mode)
{
    uint16_t reg = enc28j60_phy_read(ENC28J60_PHY_PHLCON);
    reg &= ~ENC28J60_PHY_PHLCON_LBCFG(0xF);
    reg |= ENC28J60_PHY_PHLCON_LBCFG(mode);
    enc28j60_phy_write(ENC28J60_PHY_PHLCON, reg);
}

void enc28j60_write_packet(enc28j60_ethernet_packet_t *packet)
{
    register uint8_t reg;
    uint16_t reg16;

    enc28j60_bank_select(ENC28J60_BANK_0);

    // 1. Appropriately program the ETXST Pointer to
    //  point to an unused location in memory. It will
    //  point to the per packet control byte. In the
    //  example, it would be programmed to 0120h. It is
    //  recommended that an even address be used for
    //  ETXST.

    enc28j60_wcr(ENC28J60_ETXSTL, (uint8_t) ENC28J60_TXBUFF_START);
    enc28j60_wcr(ENC28J60_ETXSTH, (uint8_t) (ENC28J60_TXBUFF_START >> 8));

    // 2. Use the WBM SPI command to write the per
    //  packet control byte, the destination address, the
    //  source MAC address, the type/length and the
    //  data payload.

    // Sets AUTOINC in ECON2
    reg = enc28j60_eth_rcr(ECON2);
    reg |= _BV(ECON2_AUTOINC);
    enc28j60_wcr(ECON2, reg);

    // Writes the Write-Start address to EWRPTL, this will increment each
    //  buffer write
    enc28j60_wcr(ENC28J60_EWRPTL, (uint8_t) ENC28J60_TXBUFF_START);
    enc28j60_wcr(ENC28J60_EWRPTH, (uint8_t) (ENC28J60_TXBUFF_START >> 8));

    // Writes the packet
    enc28j60_wbm((uint8_t *) &packet->cb, 1);                   // 1. Control Byte
    enc28j60_wbm((uint8_t *) &packet->data.da, 6);              // 2. Destination
    enc28j60_wbm((uint8_t *) &packet->data.sa, 6);              // 3. Source
    enc28j60_wbm((uint8_t *) &packet->data.len, 2);             // 4. Type/Len
    enc28j60_wbm(packet->data.payload, packet->data.len);       // 5. Payload ( Variable Length )

    // 3. Appropriately program the ETXND Pointer. It
    //  should point to the last byte in the data payload.
    //  In the example, it would be programmed to
    //  0156h.

    // Since the Write 'Buffer Memory Command', incremented the EWRPT register
    //  and by reading it's value, and removing 1 from it, we will get the address
    //  of the final byte in the packet

    reg16 = enc28j60_eth_rcr(ENC28J60_EWRPTH);
    reg16 <<= 8;
    reg16 |= enc28j60_eth_rcr(ENC28J60_EWRPTL);
    reg16 -= 1;

    /*
    printf("Start: %u\n", ENC28J60_TXBUFF_START);
    printf("End: %u\n", reg16);
    printf("Diff: %u\n", reg16 - ENC28J60_TXBUFF_START);
    */

    // Stores the address inside the ETXND pointer

    enc28j60_wcr(ENC28J60_ETXNDL, (uint8_t) reg16);
    enc28j60_wcr(ENC28J60_ETXNDH, (uint8_t) (reg16 >> 8));

    // 4. Clear EIR.TXIF, set EIE.TXIE and set EIE.INTIE
    //  to enable an interrupt when done (if desired).

    enc28j60_bfc(EIR, _BV(EIR_TXIF));

    // 5. Start the transmission process by setting
    //  ECON1.TXRTS.

    enc28j60_bfs(ECON1, _BV(ECON1_TXRTS));

    // Waits for the bit to be cleared

    for (;;)
    {
        reg = enc28j60_eth_rcr(ECON1);
        if (!(reg & _BV(ECON1_TXRTS))) break;
    }
}