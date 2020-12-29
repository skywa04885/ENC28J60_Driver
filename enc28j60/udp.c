#include "udp.h"
#include "driver.h"
#include "checksum.h"

uint16_t enc28j60_udp_calc_cs(enc28j60_ip_hdr_t *ip_hdr, enc28j60_udp_packet_t *udp_hdr)
{
    uint16_t sum = 0x0000;

    // Add the Pseudo Header to the checksum
    sum = checksum_oc16_add(sum, ((uint16_t *) ip_hdr->sa)[0]);
    sum = checksum_oc16_add(sum, ((uint16_t *) ip_hdr->sa)[1]);

    sum = checksum_oc16_add(sum, ((uint16_t *) ip_hdr->da)[0]);
    sum = checksum_oc16_add(sum, ((uint16_t *) ip_hdr->da)[1]);

    sum = checksum_oc16_add(sum, HTON16((uint16_t) ip_hdr->proto));
    sum = checksum_oc16_add(sum, (uint16_t) udp_hdr->l);

    // Adds the UDP header to the sum
    uint16_t *p = (uint16_t *) udp_hdr;
    for (uint8_t i = 0; i < (sizeof (enc28j60_udp_packet_t) / 2); ++i)
    {
        sum = checksum_oc16_add(sum, *p);
        ++p;
    }

    // Adds the data to the sum
    p = (uint16_t *) udp_hdr->payload;
    for (uint16_t i = 0; i < (NTOH16(udp_hdr->l) / 2); ++i)
    {
        sum = checksum_oc16_add(sum, *p);
        ++p;
    }

    return ~sum;
}