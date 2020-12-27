#include <string.h>
#include <avr/wdt.h>
#include "enc28j60/driver.h"
#include "enc28j60/registers.h"
#include "uart.h"

#define MAC { 0xAA, 0xAA, 0xA4, 0x52, 0x37, 0x3C }
#define IPV4 { 192, 168, 2, 188 }

/**************************************************
 * Driver configuration
 **************************************************/

static enc28j60_driver_cfg_t cfg = {
    .mac = MAC,
    .ipv4 = IPV4,
    .full_duplex = 1
};

/**************************************************
 * Static variables
 **************************************************/

static volatile uint8_t target_mac[6];
static volatile uint8_t target_ip[4] = { 192, 168, 2, 1 };

/**************************************************
 * Functions
 **************************************************/

void main_setup(void)
{
    uart_init();
    enc28j60_init(&cfg);

    // Waits for the LINK UP
    printf("LINK awaiting ..\n");
    while (!enc28j60_is_link_up());
    printf("LINK up, proceeding ..\n");

    // Resolves the MAC Address of the target IP
    printf("ARP For %u.%u.%u.%u sent\n", target_ip[0], target_ip[1], target_ip[2], target_ip[3]);
    enc28j60_send_arp(&cfg, target_ip);
    enc28j60_ethernet_packet_t *packet = enc28j60_get_buffer();
    for (uint16_t i = 0; i < 500; ++i)
    {
        _delay_ms(10);

        // Attempts to read packet, if there is no packet available
        //  we will proceed to the next loop
        enc28j60_err_t e = enc28j60_read_packet(packet);
        if (e == ENC28J60_NO_PKT_AVAILABLE) continue;

        // Reads the packet, and checks if it is meant for us,
        //  and if the type is correct, else just continue to next iteration
        if (memcmp(packet->data.da, cfg.mac, 6) != 0) continue;
        else if (NTOH16(packet->data.type) != ENC28J60_ETHERNET_PACKET_TYPE_ARP) continue;

        // Checks the ARP Packet, and if the ARP thing is an reply, if so
        //  we know that we've received the mac
        enc28j60_arp_packet_t *arp_packet = (enc28j60_arp_packet_t *) packet->data.payload;
        if (NTOH16(arp_packet->op) != ENC28J60_ARP_PACKET_OP_REPLY) continue;

        // Reads the MAC from the packet, and returns from setup
        memcpy(target_mac, arp_packet->sha, 6);
        printf("ARP Reply MAC: ");
        print_mac(target_mac, stdout);
        printf("\n");
        return;
    }

    // Since the ARP response failed, print error and restart
    //  AVR device
    printf("ARP Failed, reboot\n");
    ((void (*)()) 0x0000)();
}

void main_loop(void)
{

}

int main(void)
{
    main_setup();
    for (;;) main_loop();
}