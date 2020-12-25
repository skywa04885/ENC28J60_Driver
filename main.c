#include <string.h>
#include "enc28j60/driver.h"
#include "enc28j60/registers.h"
#include "uart.h"

#define MAC { 0xAA, 0xAA, 0xA4, 0x52, 0x37, 0x3C }

static enc28j60_driver_cfg_t cfg = {
    .mac = MAC,
    .full_duplex = 1
};

static uint8_t broadcast[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

int main(void)
{
    // Initializes UART
    uart_init();

    // Initializes ENC28J60
    enc28j60_init(&cfg);

    uint8_t macbuff[6];
    enc28j60_read_mac(macbuff);
    printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", macbuff[0], macbuff[1], macbuff[2], macbuff[3], macbuff[4], macbuff[5]);

    // Starts writing packets
    uint8_t buffer[512];
    enc28j60_ethernet_packet_t *packet = (enc28j60_ethernet_packet_t *) buffer;

    packet->cb.poverride = 0;

    const char *message = "Who let the dogs out? WHoo WHoo WHoo!!";

    packet->data.len = strlen(message) + 1; // String + Null Term
    memcpy(packet->data.payload, message, packet->data.len);

    memcpy(packet->data.da, broadcast, 6);
    memcpy(packet->data.da, cfg.mac, 6);

    for (;;)
    {
        enc28j60_write_packet(packet);
        _delay_ms(250);
    }
}