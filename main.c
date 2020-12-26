#include <string.h>
#include "enc28j60/driver.h"
#include "enc28j60/registers.h"
#include "uart.h"

#define MAC { 0xAA, 0xAA, 0xA4, 0x52, 0x37, 0x3C }
#define IPV4 { 192, 168, 2, 188 }

static enc28j60_driver_cfg_t cfg = {
    .mac = MAC,
    .ipv4 = IPV4,
    .full_duplex = 1
};

static uint8_t broadcast[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

int main(void)
{
    // Initializes UART
    uart_init();

    // Initializes ENC28J60
    enc28j60_init(&cfg);

    // Resolves IP to mac
    uint8_t arp_macbuff[6];
    uint8_t arp_ipbuff[4] = { 192, 168, 2, 22 };

    _delay_ms(500);
    enc28j60_send_arp(&cfg, arp_ipbuff);

    uint8_t buffer[256];
    enc28j60_ethernet_packet_t *packet = (enc28j60_ethernet_packet_t *) buffer;

    uint16_t packet_count = 0;
    for (;;)
    {
        enc28j60_err_t e = enc28j60_read_packet(packet);
        if (e == ENC28J60_NO_PKT_AVAILABLE) continue;

        printf("\nNew packet[%u]:\n", packet_count++);
        printf("DA: ");
        print_mac(packet->data.da, stdout);
        printf("\nSA: ");
        print_mac(packet->data.sa, stdout);
        printf("\n");

        _delay_ms(250);
    }
}