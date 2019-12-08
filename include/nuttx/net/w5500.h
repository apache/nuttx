/************************************************************************************
 * include/nuttx/net/w5500.h
 * WIZnet W5500 Ethernet Controller
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __INCLUDE_NUTTX_NET_W5500_H
#define __INCLUDE_NUTTX_NET_W5500_H

/************************************************************************************'
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#ifdef CONFIG_NET_W5500

/************************************************************************************
 * Included Files
 ************************************************************************************/

/* W5500 Register Addresses *********************************************************/

/* Common Register Block */

#define W5500_MR             0x0000  /* Mode */
#define W5500_GAR0           0x0001  /* Gateway Address */
#define W5500_GAR1           0x0002
#define W5500_GAR2           0x0003
#define W5500_GAR3           0x0004
#define W5500_SUBR0          0x0005  /* Subnet Mask Address */
#define W5500_SUBR1          0x0006
#define W5500_SUBR2          0x0007
#define W5500_SUBR3          0x0008
#define W5500_SHAR0          0x0009  /* Source Hardware Address */
#define W5500_SHAR1          0x000a
#define W5500_SHAR2          0x000b
#define W5500_SHAR3          0x000c
#define W5500_SHAR4          0x000d
#define W5500_SHAR5          0x000e
#define W5500_SIPR0          0x000f  /* Source IP Address */
#define W5500_SIPR1          0x0010
#define W5500_SIPR2          0x0011
#define W5500_SIPR3          0x0012
#define W5500_INTLEVEL0      0x0013  /* Interrupt Low Level Timer */
#define W5500_INTLEVEL1      0x0014
#define W5500_IR             0x0015  /* Interrupt */
#define W5500_IMR            0x0016  /* Interrupt Mask */
#define W5500_SIR            0x0017  /* Socket Interrupt */
#define W5500_SIMR           0x0018  /* Socket Interrupt Mask */
#define W5500_RTR0           0x0019  /* Retry Time */
#define W5500_RTR1           0x001a
#define W5500_RCR            0x001b  /* Retry Count */
#define W5500_PTIMER         0x001c  /* PPP LCP Request Timer */
#define W5500_PMAGIC         0x001d  /* PPP LCP Magic number */
#define W5500_PHAR0          0x001e  /* PPP Destination MAC Address */
#define W5500_PHAR1          0x001f
#define W5500_PHAR2          0x0020
#define W5500_PHAR3          0x0021
#define W5500_PHAR4          0x0022
#define W5500_PHAR5          0x0023
#define W5500_PSID0          0x0024  /* PPP Session Identification */
#define W5500_PSID1          0x0025
#define W5500_PMRU0          0x0026  /* PPP Maximum Segment Size */
#define W5500_PMRU1          0x0027
#define W5500_UIPR0          0x0028  /* Unreachable IP address */
#define W5500_UIPR1          0x0029
#define W5500_UIPR2          0x002a
#define W5500_UIPR3          0x002b
#define W5500_UPORTR0        0x002c  /* Unreachable Port */
#define W5500_UPORTR1        0x002d
#define W5500_PHYCFGR        0x002e  /* PHY Configuration */
                                     /* 0x002f-0x0038: Reserved */
#define W5500_VERSIONR       0x0039  /* Chip version */
                                     /* 0x003a-0xffff: Reserved */

/* Socket Register Block */

#define W5500_SN_MR          0x0000  /* Socket n Mode */
#define W5500_SN_CR          0x0001  /* Socket n Command */
#define W5500_SN_IR          0x0002  /* Socket n Interrupt */
#define W5500_SN_SR          0x0003  /* Socket n Status */
#define W5500_SN_PORT0       0x0004  /* Socket n Source Port */
#define W5500_SN_PORT1       0x0005
#define W5500_SN_DHAR0       0x0006  /* Socket n Destination Hardware Address */
#define W5500_SN_DHAR1       0x0007
#define W5500_SN_DHAR2       0x0008
#define W5500_SN_DHAR3       0x0009
#define W5500_SN_DHAR4       0x000a
#define W5500_SN_DHAR5       0x000b
#define W5500_SN_DIPR0       0x000c  /* Socket n Destination IP Address */
#define W5500_SN_DIPR1       0x000d
#define W5500_SN_DIPR2       0x000e
#define W5500_SN_DIPR3       0x000f
#define W5500_SN_DPORT0      0x0010  /* Socket n Destination Port */
#define W5500_SN_DPORT1      0x0011
#define W5500_SN_MSSR0       0x0012  /* Socket n Maximum Segment Size */
#define W5500_SN_MSSR1       0x0013
                                     /* 0x0014: Reserved */
#define W5500_SN_TOS         0x0015  /* Socket n IP TOS */
#define W5500_SN_TTL         0x0016  /* Socket n IP TTL */
                                     /* 0x0017-0x001d: Reserved */
#define W5500_SN_RXBUF_SIZE  0x001e  /* Socket n Receive Buffer Size */
#define W5500_SN_TXBUF_SIZE  0x001f  /* Socket n Transmit Buffer Size */
#define W5500_SN_TX_FSR0     0x0020  /* Socket n TX Free Size */
#define W5500_SN_TX_FSR1     0x0021
#define W5500_SN_TX_RD0      0x0022  /* Socket n TX Read Pointer */
#define W5500_SN_TX_RD1      0x0023
#define W5500_SN_TX_WR0      0x0024  /* Socket n TX Write Pointer */
#define W5500_SN_TX_WR1      0x0025
#define W5500_SN_RX_RSR0     0x0026  /* Socket n RX Received Size */
#define W5500_SN_RX_RSR1     0x0027
#define W5500_SN_RX_RD0      0x0028  /* Socket n RX Read Pointer */
#define W5500_SN_RX_RD1      0x0029
#define W5500_SN_RX_WR0      0x002a  /* Socket n RX Write Pointer */
#define W5500_SN_RX_WR1      0x002b
#define W5500_SN_IMR         0x002c  /* Socket n Interrupt Mask */
#define W5500_SN_FRAG0       0x002d  /* Socket n Fragment Offset in IP header */
#define W5500_SN_FRAG1       0x002e
#define W5500_SN_KPALVTR     0x002f  /* Keep alive timer */
                                     /* 0x0030-0xffff: Reserved */

/* W5500 Register Bitfield Definitions **********************************************/

/* Common Register Block */

/* Mode Register (MR) */

#define MR_FARP              (1 << 1)  /* Bit 1: Force ARP */
#define MR_PPPOE             (1 << 3)  /* Bit 3: PPPoE Mode */
#define MR_PB                (1 << 4)  /* Bit 4: Ping Block Mode */
#define MR_WOL               (1 << 5)  /* Bit 5: Wake on LAN */
#define MR_RST               (1 << 7)  /* Bit 7: Reset registers */

/* Interrupt Register (IR), Interrupt Mask Register (IMR) */

#define INT_MP               (1 << 4)  /* Bit 4:  Magic Packet */
#define INT_PPPOE            (1 << 5)  /* Bit 5:  PPPoE Connection Close */
#define INT_UNREACH          (1 << 6)  /* Bit 6:  Destination unreachable */
#define INT_CONFLICT         (1 << 7)  /* Bit 7:  IP Conflict */

/* Socket Interrupt Register (SIR) */

#define SIR(n)               (1 << (n))

/* Socket Interrupt Mask Register (SIMR)) */

#define SIMR(n)              (1 << (n))

/* PHY Configuration Register (PHYCFGR) */

#define PHYCFGR_LNK          (1 << 0)  /* Bit 0:  Link Status */
#define PHYCFGR_SPI          (1 << 1   /* Bit 2:  Speed Status */
#define PHYCFGR_DPX          (1 << 2)  /* Bit 3:  Duplex Status */
#define PHYCFGR_OPMDC_SHIFT  (3)       /* Bits 3-5: Operation Mode Configuration */
#define PHYCFGR_OPMDC_MASK   (7 << PHYCFGR_OPMDC_SHIFT)
#  define PHYCFGR_OPMDC_10BT_HD_NAN  (0 << PHYCFGR_OPMDC_SHIFT) /* 10BT Half-duplex */
#  define PHYCFGR_OPMDC_10BT_HFD_NAN (1 << PHYCFGR_OPMDC_SHIFT) /* 10BT Full-duplex */
#  define PHYCFGR_OPMDC_100BT_HD_NAN (2 << PHYCFGR_OPMDC_SHIFT) /* 100BT Half-duplex */
#  define PHYCFGR_OPMDC_10BT_FD_NAN  (3 << PHYCFGR_OPMDC_SHIFT) /* 100BT Full-duplex,
                                                                 * Auto-negotiation */
#  define PHYCFGR_OPMDC_100BT_HD_AN  (4 << PHYCFGR_OPMDC_SHIFT) /* 100BT Half-duplex,
                                                                 * Auto-negotiation */
#  define PHYCFGR_OPMDC_POWER_DOWN   (6 << PHYCFGR_OPMDC_SHIFT) /* Power Down mode */
#  define PHYCFGR_OPMDC_ALLCAP_AN    (7 << PHYCFGR_OPMDC_SHIFT) /* All capable,
                                                                 * Auto-negotiation */
#define PHYCFGR_OPMD         (1 << 6)  /* Bit 6:  Configure PHY Operation Mode */
#define PHYCFGR_RST          (1 << 7)  /* Bit 7:  Reset */

/* Socket Register Block */

/* Socket n Mode Register (SN_MR) */

#define SN_MR_PROTOCOL_SHIFT (0)       /* Bits 0-3:  Protocol */
#define SN_MR_PROTOCOL_MASK  (15 << SN_MR_PROTOCOL_SHIFT)
#  define SN_MR_P0           (1 << (SN_MR_PROTOCOL_SHIFT + 0))
#  define SN_MR_P1           (1 << (SN_MR_PROTOCOL_SHIFT + 1))
#  define SN_MR_P2           (1 << (SN_MR_PROTOCOL_SHIFT + 2))
#  define SN_MR_P3           (1 << (SN_MR_PROTOCOL_SHIFT + 3))
#  define SM_MR_CLOSED       0
#  define SM_MR_TCP          SN_MR_P0
#  define SM_MR_UDP          SN_MR_P1
#  define SM_MR_MACRAW       SN_MR_P2
#define SN_MR_UCASTB         (1 << 4)  /* Bit 4:  UNICAST Blocking in UDP mode */
#define SN_MR_MIP6B          (1 << 4)  /* Bit 4:  IPv6 packet Blocking in MACRAW mode */
#define SN_MR_ND             (1 << 5)  /* Bit 5:  Use No Delayed ACK */
#define SN_MR_MC             (1 << 5)  /* Bit 5:  Multicast */
#define SN_MR_MMB            (1 << 5)  /* Bit 5:  Multicast Blocking in MACRAW mode */
#define SN_MR_BCASTB         (1 << 6)  /* Bit 6:  Broadcast Blocking in MACRAW and
                                        *         UDP mode */
#define SN_MR_MULTI          (1 << 7)  /* Bit 7:  Multicasting in UDP mode */
#define SN_MR_MFEN           (1 << 7)  /* Bit 7:  MAC Filter Enable in MACRAW mode */

/* Socket n Command Register (SN_CR) */

#define SN_CR_OPEN           0x01      /* Socket n is initialized and opened according
                                        * to the protocol selected in SN_MR */
#define SN_CR_LISTEN         0x02      /* Socket n operates as a 'TCP server' and waits
                                        * for connection request from any 'TCP client' */
#define SN_CR_CONNECT        0x04      /* 'TCP client' connection request */
#define SN_CR_DISCON         0x08      /* TCP disconnection request */
#define SN_CR_CLOSE          0x10      /* Close socket n */
#define SN_CR_SEND           0x20      /* Transmit all data in Socket n TX buffer */
#define SN_CR_SEND_MAC       0x21      /* Transmit all UDP data (no ARP) */
#define SN_CR_SEND_KEEP      0x22      /* Send TCP keep-alive packet */
#define SN_CR_RECV           0x40      /* Complete received data in Socket n RX buffer */

/* Socket n Interrupt Register (SN_IR) and Socket n Interrupt Mask Register (SN_IMR)  */

#define SN_INT_CON           (1 << 0)  /* Bit 0:  Connection with peer successful */
#define SN_INT_DISCON        (1 << 1)  /* Bit 1:  FIN or FIN/ACK received from peer */
#define SN_INT_RECV          (1 << 2)  /* Bit 2:  Data received from peer */
#define SN_INT_TIMEOUT       (1 << 3)  /* Bit 3:  ARP or TCP timeout */
#define SN_INT_SEND_OK       (1 << 4)  /* Bit 4:  SEND command completed */

/* Socket n Status Register (SN_SR) */

#define SN_SR_SOCK_CLOSED      0x00
#define SN_SR_SOCK_INIT        0x13
#define SN_SR_SOCK_LISTEN      0x14
#define SN_SR_SOCK_ESTABLISHED 0x17
#define SN_SR_SOCK_CLOSE_WAIT  0x1c
#define SN_SR_SOCK_UDP         0x22
#define SN_SR_SOCK_MACRAW      0x42

#define SN_SR_SOCK_SYNSENT     0x15    /* Transitional status */
#define SN_SR_SOCK_SYNRECV     0x16
#define SN_SR_SOCK_FIN_WAIT    0x18
#define SN_SR_SOCK_CLOSING     0x1a
#define SN_SR_SOCK_TIME_WAIT   0x1b
#define SN_SR_SOCK_LAST_ACK    0x1d

/* Socket n RX Buffer Size Register (SN_RXBUF) */

#define SN_RXBUF_0KB          0
#define SN_RXBUF_1KB          1
#define SN_RXBUF_2KB          2
#define SN_RXBUF_4KB          4
#define SN_RXBUF_8KB          5
#define SN_RXBUF_16KB         16

/* Socket n TX Buffer Size Register (SN_TXBUF) */

#define SN_TXBUF_0KB          0
#define SN_TXBUF_1KB          1
#define SN_TXBUF_2KB          2
#define SN_TXBUF_4KB          4
#define SN_TXBUF_8KB          5
#define SN_TXBUF_16KB         16

/************************************************************************************'
 * Public Types
 ************************************************************************************/

/* This structure describes the lower-half interface provided by board-
 * specific logic.
 */

struct w5500_lower_s
{
  uint32_t frequency;         /* Frequency to use with SPI_SETFREQUENCY() */
  uint16_t spidevid;          /* Index used with SPIDEV_ETHERNET() macro */
  enum spi_mode_e mode mode;  /* SPI more for use with SPI_SETMODE() */

  /* Lower-half callbacks:
   *
   * attach() - Attach the W5500 interrupt to the driver interrupt handler.
   * enable() - Enable or disable the W5500 interrupt.
   * reset()  - Set the RSTn pin to the provided state.
   */

  int  (*attach)(FAR const struct w5500_lower_s *lower, xcpt_t handler,
                 FAR void *arg);
  void (*enable)(FAR const struct w5500_lower_s *lower, bool enable);
  void (*reset)(FAR const struct w5500_lower_s *lower, bool reset);
};

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: w5500_initialize
 *
 * Description:
 *   Initialize the Ethernet controller and driver
 *
 * Parameters:
 *   lower - The lower half driver instance for this W5500 chip.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ************************************************************************************/

int w5500_initialize(FAR struct w5500_lower_s *lower);

#endif /* CONFIG_NET_W5500 */
#endif /* __INCLUDE_NUTTX_NET_W5500_H */
