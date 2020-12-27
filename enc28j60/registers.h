#ifndef _ENC28J60_REGISTERS_H
#define _ENC28J60_REGISTERS_H

/**************************************************
 * Control Registers ( Not Bank-Specific )
 **************************************************/

/* ECON1 */

#define ECON1                           0x1F

#define ECON1_TXRST                     7
#define ECON1_RXRST                     6
#define ECON1_DMAST                     5
#define ECON1_CSUMEN                    4
#define ECON1_TXRTS                     3
#define ECON1_RXEN                      2
#define ECON1_BSEL_SHIFT                0

/* ECON2 */

#define ECON2                           0x1E

#define ECON2_AUTOINC                   7
#define ECON2_PKTDEC                    6
#define ECON2_PWRSV                     5
#define ECON2_VRPS                      3

/* ESTAT */

#define ESTAT                           0x1D

#define ESTAT_INT                       7
#define ESTAT_BUFFER                    6
#define ESTAT_LATECOL                   4
#define ESTAT_RXBUSY                    2
#define ESTAT_TXABRT                    1
#define ESTAT_CLKRDY                    0

/* EIR */

#define EIR                             0x1C

#define EIR_PKTIE                       6
#define EIR_DMAIF                       5
#define EIR_LINKIF                      4
#define EIR_TXIF                        3
#define EIR_TXERIF                      1
#define EIR_RXERIF                      0

/* EIE */

#define EIE                             0x1B

#define EIE_INTIE                       7
#define EIE_PKTIE                       6
#define EIE_DMAIE                       5
#define EIE_LINKIE                      4
#define EIE_TXIE                        3
#define EIE_TXERIE                      1
#define EIE_RXERIE                      0

/**************************************************
 * Control Registers ( Bank 3 )
 **************************************************/

/* MAADR5 */

#define ENC28J60_BK3_MAADR5             0x00
#define ENC28J60_BK3_MAADR6             0x01
#define ENC28J60_BK3_MAADR3             0x02
#define ENC28J60_BK3_MAADR4             0x03
#define ENC28J60_BK3_MAADR1             0x04
#define ENC28J60_BK3_MAADR2             0x05

/* MISTAT */

#define ENC28J60_BK3_MISTAT             0x0A

#define ENC28J60_BK3_MISTAT_BUSY        0
#define ENC28J60_BK3_MISTAT_SCAN        1
#define ENC28J60_BK3_MISTAT_NVALID      2

/**************************************************
 * Control Registers ( Bank 2 )
 **************************************************/

/* MACON1 */

#define ENC28J60_BK2_MACON1             0x00

#define ENC28J60_BK2_MACON1_MARXEN      0
#define ENC28J60_BK2_MACON1_PASSALL     1
#define ENC28J60_BK2_MACON1_RXPAUS      2
#define ENC28J60_BK2_MACON1_TXPAUS      3

/* MACON2 */

#define ENC28J60_BK2_MACON3             0x02

#define ENC28J60_BK2_MACON3_FULDPX      0
#define ENC28J60_BK2_MACON3_FRMLNEN     1
#define ENC28J60_BK2_MACON3_HFRMEN      2
#define ENC28J60_BK2_MACON3_PHDREN      3
#define ENC28J60_BK2_MACON3_TXCRCEN     4

#define ENC28J60_BK2_MACON3_PADCFG(A)   (A << 5)

/* MACON4 */

#define ENC28J60_BK2_MACON4             0x03

#define ENC28J60_BK2_MACON4_NOBKOFF     4
#define ENC28J60_BK2_MACON4_BPEN        5
#define ENC28J60_BK2_MACON4_DEFER       6


/* MAMXFL */

#define ENC28J60_BK2_MAMXFLL            0x0A
#define ENC28J60_BK2_MAMXFLH            0x0B

/* MABBIPG */

#define ENC28J60_BK2_MABBIPG            0x04

/* MAIPG */

#define ENC28J60_BK2_MAIPGL             0x06
#define ENC28J60_BK2_MAIPGH             0x07

/* MIREGADR */

#define ENC28J60_BK2_MIREGADR           0x14

/* MICMD */

#define ENC28J60_BK2_MICMD              0x12

#define ENC28J60_BK2_MICMD_MIIRD        0
#define ENC28J60_BK2_MICMD_MIISCAN      1

/* MIRD */

#define ENC28J60_BLK2_MIRDL             0x18
#define ENC28J60_BLK2_MIRDH             0x19

/* MIWR */

#define ENC28J60_BLK2_MIWRL             0x16
#define ENC28J60_BLK2_MIWRH             0x17

/**************************************************
 * Control Registers ( Bank 1 )
 **************************************************/

/* ERXFCON */

#define ENC28J60_BK1_ERXFCON            0x18

#define ENC28J60_BK1_ERXFCON_UCEN       7
#define ENC28J60_BK1_ERXFCON_ANDOR      6
#define ENC28J60_BK1_ERXFCON_CRCEN      5
#define ENC28J60_BK1_ERXFCON_PMEN       4
#define ENC28J60_BK1_ERXFCON_MPEN       3
#define ENC28J60_BK1_ERXFCON_HTEN       2
#define ENC28J60_BK1_ERXFCON_MCEN       1
#define ENC28J60_BK1_ERXFCON_BCEN       0

/* EPKTCNT*/

#define ENC28J60_BK1_EPKTCNT            0x19

/**************************************************
 * Control Registers ( Bank 0 )
 **************************************************/

/* ERXST / ERXND */

#define ENC28J60_BK0_ERXSTL             0x08
#define ENC28J60_BK0_ERXSTH             0x09

#define ENC28J60_BK0_ERXNDL             0x0A
#define ENC28J60_BK0_ERXNDH             0x0B

/* ETXST */

#define ENC28J60_BK0_ETXSTL                 0x04
#define ENC28J60_BK0_ETXSTH                 0x05

/* ETXND */

#define ENC28J60_BK0_ETXNDL                 0x06
#define ENC28J60_BK0_ETXNDH                 0x07

/* EWRPT */

#define ENC28J60_BK0_EWRPTL                 0x02
#define ENC28J60_BK0_EWRPTH                 0x03

/* ERDPT */

#define ENC28J60_BK0_ERDPTL                 0x00
#define ENC28J60_BK0_ERDPTH                 0x01

/* ERXWRPT */

#define ENC28J60_BK0_ERXWRPTL               0x0E
#define ENC28J60_BK0_ERXWRPTH               0x0F

/* ERXRDPT */

#define ENC28J60_BK0_ERXRDPTL               0x0C
#define ENC28J60_BK0_ERXRDPTH               0x0D

/**************************************************
 * PHY Registers
 **************************************************/

/* PHCON1 */

#define ENC28J60_PHY_PHCON1             0x00

#define ENC28J60_PHY_PHCON1_PDPXMD      8
#define ENC28J60_PHY_PHCON1_PPWRSV      11
#define ENC28J60_PHY_PHCON1_PLOOPBK     14
#define ENC28J60_PHY_PHCON1_PRST        15

/* PHCON2 */

#define ENC28J60_PHY_PHCON2             0x10

#define ENC28J60_PHY_PHCON2_FRCLNK      14
#define ENC28J60_PHY_PHCON2_TXDIS       13
#define ENC28J60_PHY_PHCON2_JABBER      10
#define ENC28J60_PHY_PHCON2_HDLDIS      8

/* PHID1 */

#define ENC28J60_PHY_PHID1              0x02
#define ENC28J60_PHY_PHID2              0x03

/* PHLCON */

#define ENC28J60_PHY_PHLCON             0x14

#define ENC28J60_PHY_PHLCON_STRCH       1
#define ENC28J60_PHY_PHLCON_LFRQ(A)     ((A & 0x3) << 1)
#define ENC28J60_PHY_PHLCON_LBCFG(A)    ((A & 0xF) << 4)
#define ENC28J60_PHY_PHLCON_LACFG(A)    ((A & 0xF) << 8)

/* PHSTAT2 */

#define ENC28J60_PHY_PHSTAT2             0x11
#define ENC28J60_PHY_PHSTAT2_TXSTAT      13
#define ENC28J60_PHY_PHSTAT2_RXSTAT      12
#define ENC28J60_PHY_PHSTAT2_COLSTAT     11
#define ENC28J60_PHY_PHSTAT2_LSTAT       10
#define ENC28J60_PHY_PHSTAT2_PLRITY      5


#endif
