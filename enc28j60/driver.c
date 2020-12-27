#include "driver.h"
#include "registers.h"

static uint8_t __driver_pkt_buff[256];
static uint8_t __driver_write_buff[256];

/**************************************************
 * AVR Hardware Functions
 **************************************************/

#ifdef _ENC28J60_AVR

void spi_init(void)
{
    // Configure SPI GPIO's
    SPI_DDR |= _BV(SPI_MOSI) | _BV(SPI_SCK) | _BV(SPI_SS);

    // Configure SPI Hardware
    SPSR = _BV(SPI2X);                  // Clear SPCR
    SPCR = _BV(MSTR) | _BV(SPE);        // Master & Enable & 1 Mhz
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

void enc28j60_rbm(uint8_t *data, uint16_t len)
{
    spi_select();

    spi_transceive((ENC28J60_RBM << 5) | 0x1A);
    for (uint16_t i = 0; i < len; ++i)
        data[i] = spi_transceive(0xFF);

    spi_deselect();
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

uint8_t *enc28j60_get_buffer()
{
    return __driver_pkt_buff;
}

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

    // Disable RX
    enc28j60_rx_disable();

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

    // Enables RX
    enc28j60_rx_enable();
}

bool enc28j60_is_link_up()
{
    return (enc28j60_phy_read(ENC28J60_PHY_PHSTAT2) & _BV(ENC28J60_PHY_PHSTAT2_LSTAT));
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

    // Sets the initial hardware write position
    enc28j60_wcr(ENC28J60_BK0_ERXRDPTH, 0x00);
    enc28j60_wcr(ENC28J60_BK0_ERXRDPTL, 0x00);
}

void enc28j60_filter_init()
{
    enc28j60_bank_select(ENC28J60_BANK_1);
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

void enc28j60_rx_disable()
{
    enc28j60_bfc(ECON1, _BV(ECON1_RXEN));
}

void enc28j60_rx_enable()
{
    enc28j60_bfs(ECON1, _BV(ECON1_RXEN));
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

void enc28j60_write_packet(enc28j60_ethernet_packet_t *packet, uint16_t len)
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

    enc28j60_wcr(ENC28J60_BK0_ETXSTL, (uint8_t) ENC28J60_TXBUFF_START);
    enc28j60_wcr(ENC28J60_BK0_ETXSTH, (uint8_t) (ENC28J60_TXBUFF_START >> 8));

    // 2. Use the WBM SPI command to write the per
    //  packet control byte, the destination address, the
    //  source MAC address, the type/length and the
    //  data payload.

    // Sets AUTOINC in ECON2
    enc28j60_bfs(ECON2, ECON2_AUTOINC);

    // Writes the Write-Start address to EWRPTL, this will increment each
    //  buffer write
    enc28j60_wcr(ENC28J60_BK0_EWRPTL, (uint8_t) ENC28J60_TXBUFF_START);
    enc28j60_wcr(ENC28J60_BK0_EWRPTH, (uint8_t) (ENC28J60_TXBUFF_START >> 8));

    // Writes the packet
    enc28j60_wbm((uint8_t *) &packet->cb, 1);                   // 1. Control Byte
    enc28j60_wbm((uint8_t *) &packet->data.da, 6);              // 2. Destination
    enc28j60_wbm((uint8_t *) &packet->data.sa, 6);              // 3. Source
    enc28j60_wbm((uint8_t *) &packet->data.type, 2);            // 4. Type
    enc28j60_wbm(packet->data.payload, len);                    // 5. Payload ( Variable Length )

    // 3. Appropriately program the ETXND Pointer. It
    //  should point to the last byte in the data payload.
    //  In the example, it would be programmed to
    //  0156h.

    // Since the Write 'Buffer Memory Command', incremented the EWRPT register
    //  and by reading it's value, and removing 1 from it, we will get the address
    //  of the final byte in the packet

    reg16 = enc28j60_eth_rcr(ENC28J60_BK0_EWRPTH);
    reg16 <<= 8;
    reg16 |= enc28j60_eth_rcr(ENC28J60_BK0_EWRPTL);
    reg16 -= 1;

    // Stores the address inside the ETXND pointer

    enc28j60_wcr(ENC28J60_BK0_ETXNDL, (uint8_t) reg16);
    enc28j60_wcr(ENC28J60_BK0_ETXNDH, (uint8_t) (reg16 >> 8));

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

uint8_t enc28j60_get_packet_count()
{
    register uint8_t reg;

    enc28j60_bank_select(ENC28J60_BANK_1);
    reg = enc28j60_eth_rcr(ENC28J60_BK1_EPKTCNT);

    return reg;
}

enc28j60_err_t enc28j60_read_packet(enc28j60_ethernet_packet_t *packet)
{
    register uint8_t reg;
    uint16_t reg16, rx_buffer_rp, next_packet_pointer;
    enc28j60_receive_status_vector_t status_vec;

    // Checks if there are any packets available
    if (enc28j60_get_packet_count() == 0) return ENC28J60_NO_PKT_AVAILABLE;

    // Selects bank 0
    enc28j60_bank_select(ENC28J60_BANK_0);

    // Gets the packet read position
    rx_buffer_rp = enc28j60_eth_rcr(ENC28J60_BK0_ERXRDPTH);
    rx_buffer_rp <<= 8;
    rx_buffer_rp |= enc28j60_eth_rcr(ENC28J60_BK0_ERXRDPTL);

    // Sets AUTOINC in ECON2
    enc28j60_bfs(ECON2, ECON2_AUTOINC);

    // Sets ERDPT this will increment each read
    enc28j60_wcr(ENC28J60_BK0_ERDPTL, (uint8_t) rx_buffer_rp);
    enc28j60_wcr(ENC28J60_BK0_ERDPTH, (uint8_t) (rx_buffer_rp >> 8));

    // Read the headers, the next packet pointer, and the status vector
    enc28j60_rbm((uint8_t *) &next_packet_pointer, 2);
    enc28j60_rbm((uint8_t *) &status_vec, sizeof (enc28j60_receive_status_vector_t));

    // Reads the body
    enc28j60_rbm((uint8_t *) &packet->data, status_vec.rbc);

    // Frees the buffer space we've just read, by incrementing the packet
    //  read pointer, and write 1 to ECON2.PKTDEC in order to decrease the packet count
    enc28j60_wcr(ENC28J60_BK0_ERXRDPTL, (uint8_t) next_packet_pointer);
    enc28j60_wcr(ENC28J60_BK0_ERXRDPTH, (uint8_t) (next_packet_pointer >> 8));

    enc28j60_bfs(ECON2, _BV(ECON2_PKTDEC));

    return ENC28J60_OK;
}


void enc28j60_send_udp(enc28j60_driver_cfg_t *cfg, uint16_t port, uint16_t len, uint8_t *data, uint8_t *ipv4, uint8_t *mac)
{
    // Creates the ethernet packet
    enc28j60_ethernet_packet_t *packet = (enc28j60_udp_packet_t *) __driver_write_buff;
    packet->data.type = HTON16(ENC28J60_ETHERNET_PACKET_TYPE_IPV4);
    memcpy(packet->data.da, mac, 6);
    memcpy(packet->data.sa, cfg->mac, 6);

    // Creates the IP header
    enc28j60_ip_hdr_t *ip_hdr = (enc28j60_ip_hdr_t *) packet->data.payload;
    ip_hdr->ihl = 5;
    ip_hdr->ttl = 60;
    ip_hdr->proto = HTON16(ENC28J60_IP_PROTO_UDP);
    ip_hdr->ver = 4;
    ip_hdr->tos.precedence = ENC28J60_IP_TOS_PRECEDENCE_ROUTINE;
    ip_hdr->flags = _BV(ENC28J60_IP_FLAGS_DONT_FRAGMENT);
    ip_hdr->tos.d = 0;
    ip_hdr->tos.r = 0;
    ip_hdr->tos.t = 0;
    ip_hdr->flags = 0;
    ip_hdr->tl = (ip_hdr->ihl * 4) + len;

    memcpy(ip_hdr->sa, cfg->ipv4, 4);
    memcpy(ip_hdr->da, ipv4, 4);

    // Creates the UDP packet
    enc28j60_udp_packet_t *udp_packet = (enc28j60_udp_packet_t *) &packet->data.payload[ip_hdr->ihl * 4];
    udp_packet->l = HTON16(len);
    udp_packet->dp = HTON16(port);
    udp_packet->sp = 0x0000;
    memcpy(udp_packet->payload, data, len);

    // Writes the UDP packet
    enc28j60_write_packet(packet, ip_hdr->tl);
}

void enc28j60_send_arp(enc28j60_driver_cfg_t *cfg, uint8_t *ip)
{
    enc28j60_ethernet_packet_t *ethernet_packet = (enc28j60_ethernet_packet_t *) __driver_pkt_buff;
    enc28j60_arp_packet_t *arp_packet = (enc28j60_arp_packet_t *) ethernet_packet->data.payload;

    // Configures the ETHERNET packet
    memcpy(ethernet_packet->data.sa, cfg->mac, 6);
    memset(ethernet_packet->data.da, 0xFF, 6);
    ethernet_packet->data.type = HTON16(ENC28J60_ETHERNET_PACKET_TYPE_ARP);

    // Configures the ARP Packet
    arp_packet->op = HTON16(ENC28J60_ARP_PACKET_OP_REQUEST);
    arp_packet->hrd = HTON16(0x0001);       /* Ethernet */
    arp_packet->pro = HTON16(0x0800);       /* IPv4 */

    arp_packet->hln = 6;                    /* MAC Addr */
    arp_packet->pln = 4;                    /* IPv4 Addr */

    memset(arp_packet->tha, 0xFF, 6);       /* Target Hardware Address ( Unknown, Broadcast ) */
    memcpy(arp_packet->tpa, ip, 4);         /* Target Protocol address ( Known ) */
    
    memcpy(arp_packet->sha, cfg->mac, 6);   /* Source Hardware Address */
    memcpy(arp_packet->spa, cfg->ipv4, 4);  /* Source Prototol Address */

    // Sends the packet
    enc28j60_write_packet(ethernet_packet, sizeof (enc28j60_arp_packet_t));
}

void enc28j60_event_poll(enc28j60_driver_cfg_t *cfg)
{
 // Reads an packet if possible
    enc28j60_ethernet_packet_t *packet = enc28j60_get_buffer();
    enc28j60_err_t e = enc28j60_read_packet(packet);
    if (e == ENC28J60_NO_PKT_AVAILABLE) return;

    // Checks the packet type
    switch (HTON16(packet->data.type))
    {
        // Handles ARP requests
        case ENC28J60_ETHERNET_PACKET_TYPE_ARP:
        {
            enc28j60_arp_packet_t *arp_packet = (enc28j60_arp_packet_t *) packet->data.payload; 

            // Checks if the ARP packet is an request, if so
            //  we will check if it is about us, and response
            //  with our MAC
            if (NTOH16(arp_packet->op) != ENC28J60_ARP_PACKET_OP_REQUEST) break;
            else if (arp_packet->hln != 6 || arp_packet->pln != 4) break;
            else if (NTOH16(arp_packet->hrd) != 0x0001 || NTOH16(arp_packet->pro) != 0x0800) break;
            else if (memcmp(arp_packet->tpa, cfg->ipv4, 4) != 0) break;
            
            printf("Responding to ARP Request of: ");
            print_mac(arp_packet->sha, stdout);
            printf("\n");

            // Prepares the ARP Response, by setting the opcode to reply
            //  after which we copy the source to target addresses, and
            //  put our data in the source ones
            arp_packet->op = HTON16(ENC28J60_ARP_PACKET_OP_REPLY);

            memcpy(arp_packet->tha, arp_packet->sha, 6);
            memcpy(arp_packet->tpa, arp_packet->spa, 4);

            memcpy(arp_packet->spa, cfg->ipv4, 4);
            memcpy(arp_packet->sha, cfg->mac, 6);

            // Sends the response packet
            enc28j60_write_packet(packet, sizeof (enc28j60_arp_packet_t));
            break;
        }
        case ENC28J60_ETHERNET_PACKET_TYPE_IPV4:
        {
            enc28j60_ip_hdr_t *ip_hdr = (enc28j60_ip_hdr_t *) packet->data.payload;
            switch (ip_hdr->proto)
            {
                case ENC28J60_IP_PROTO_UDP:
                {
                    enc28j60_udp_packet_t *udp_packet = (enc28j60_udp_packet_t *) &packet->data.payload[ip_hdr->ihl * 4];
                    cfg->udp_callback(NTOH16(udp_packet->dp), NTOH16(udp_packet->l), udp_packet->payload);
                    break;
                }
            }

            break;
        }
        default: break;
    }
}

/**************************************************
 * Data Printing Prototypes
 **************************************************/

void print_mac(uint8_t *mac, FILE *ofstream)
{
    fprintf(ofstream, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}