/****************************************************************************
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
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_NET_W5500_H
#define __INCLUDE_NUTTX_NET_W5500_H

/****************************************************************************'
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#ifdef CONFIG_NET_W5500

/****************************************************************************'
 * Included Files
 ****************************************************************************/

/* W5500 Register Addresses *************************************************/

/* Common Register Block */

#define W5500_MR            0x0000  /* Mode */
#define W5500_GAR0          0x0001  /* Gateway Address */
#define W5500_GAR1          0x0002
#define W5500_GAR2          0x0003
#define W5500_GAR3          0x0004
#define W5500_SUBR0         0x0005  /* Subnet Mask Address */
#define W5500_SUBR1         0x0006
#define W5500_SUBR2         0x0007
#define W5500_SUBR3         0x0008
#define W5500_SHAR0         0x0009  /* Source Hardware Address */
#define W5500_SHAR1         0x000a
#define W5500_SHAR2         0x000b
#define W5500_SHAR3         0x000c
#define W5500_SHAR4         0x000d
#define W5500_SHAR5         0x000e
#define W5500_SIPR0         0x000f  /* Source IP Address */
#define W5500_SIPR1         0x0010
#define W5500_SIPR2         0x0011
#define W5500_SIPR3         0x0012
#define W5500_INTLEVEL0     0x0013  /* Interrupt Low Level Timer */
#define W5500_INTLEVEL1     0x0014
#define W5500_IR            0x0015  /* Interrupt */
#define W5500_IMR           0x0016  /* Interrupt Mask */
#define W5500_SIR           0x0017  /* Socket Interrupt */
#define W5500_SIMR          0x0018  /* Socket Interrupt Mask */
#define W5500_RTR0          0x0019  /* Retry Time */
#define W5500_RTR1          0x001a
#define W5500_RCR           0x001b  /* Retry Count */
#define W5500_PTIMER        0x001c  /* PPP LCP Request Timer */
#define W5500_PMAGIC        0x001d  /* PPP LCP Magic number */
#define W5500_PHAR0         0x001e  /* PPP Destination MAC Address */
#define W5500_PHAR1         0x001f
#define W5500_PHAR2         0x0020
#define W5500_PHAR3         0x0021
#define W5500_PHAR4         0x0022
#define W5500_PHAR5         0x0023
#define W5500_PSID0         0x0024  /* PPP Session Identification */
#define W5500_PSID1         0x0025
#define W5500_PMRU0         0x0026  /* PPP Maximum Segment Size */
#define W5500_PMRU1         0x0027
#define W5500_UIPR0         0x0028  /* Unreachable IP address */
#define W5500_UIPR1         0x0029
#define W5500_UIPR2         0x002a
#define W5500_UIPR3         0x002b
#define W5500_UPORTR0       0x002c  /* Unreachable Port */
#define W5500_UPORTR1       0x002d
#define W5500_PHYCFGR       0x002e  /* PHY Configuration */
                                    /* 0x002f-0x0038: Reserved */
#define W5500_VERSIONR      0x0039  /* Chip version */
                                    /* 0x003a-0xffff: Reserved */

/* Socket Register Block */

#define W5500_SN_MR         0x0000  /* Socket n Mode */
#define W5500_SN_CR         0x0001  /* Socket n Command */
#define W5500_SN_IR         0x0002  /* Socket n Interrupt */
#define W5500_SN_SR         0x0003  /* Socket n Status */
#define W5500_SN_PORT0      0x0004  /* Socket n Source Port */
#define W5500_SN_PORT1      0x0005
#define W5500_SN_DHAR0      0x0006  /* Socket n Destination Hardware Address */
#define W5500_SN_DHAR1      0x0007
#define W5500_SN_DHAR2      0x0008
#define W5500_SN_DHAR3      0x0009
#define W5500_SN_DHAR4      0x000a
#define W5500_SN_DHAR5      0x000b
#define W5500_SN_DIPR0      0x000c  /* Socket n Destination IP Address */
#define W5500_SN_DIPR1      0x000d
#define W5500_SN_DIPR2      0x000e
#define W5500_SN_DIPR3      0x000f
#define W5500_SN_DPORT0     0x0010  /* Socket n Destination Port */
#define W5500_SN_DPORT1     0x0011
#define W5500_SN_MSSR0      0x0012  /* Socket n Maximum Segment Size */
#define W5500_SN_MSSR1      0x0013
                                    /* 0x0014: Reserved */
#define W5500_SN_TOS        0x0015  /* Socket n IP TOS */
#define W5500_SN_TTL        0x0016  /* Socket n IP TTL */
                                    /* 0x0017-0x001d: Reserved */
#define W5500_SN_RXBUF_SIZE 0x001e  /* Socket n Receive Buffer Size */
#define W5500_SN_TXBUF_SIZE 0x001f  /* Socket n Transmit Buffer Size */
#define W5500_SN_TX_FSR0    0x0020  /* Socket n TX Free Size */
#define W5500_SN_TX_FSR1    0x0021
#define W5500_SN_TX_RD0     0x0022  /* Socket n TX Read Pointer */
#define W5500_SN_TX_RD1     0x0023
#define W5500_SN_TX_WR0     0x0024  /* Socket n TX Write Pointer */
#define W5500_SN_TX_WR1     0x0025
#define W5500_SN_RX_RSR0    0x0026  /* Socket n RX Received Size */
#define W5500_SN_RX_RSR1    0x0027
#define W5500_SN_RX_RD0     0x0028  /* Socket n RX Read Pointer */
#define W5500_SN_RX_RD1     0x0029
#define W5500_SN_RX_WR0     0x002a  /* Socket n RX Write Pointer */
#define W5500_SN_RX_WR1     0x002b
#define W5500_SN_IMR        0x002c  /* Socket n Interrupt Mask */
#define W5500_SN_FRAG0      0x002d  /* Socket n Fragment Offset in IP header */
#define W5500_SN_FRAG1      0x002e
#define W5500_SN_KPALVTR    0x002f  /* Keep alive timer */
                                    /* 0x0030-0xffff: Reserved */

/* W5500 Register Bitfield Definitions **************************************/

/* Common Register Block */


/* Socket Register Block */


/****************************************************************************'
 * Public Types
 ****************************************************************************/

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

/****************************************************************************'
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
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
 ****************************************************************************/

int w5500_initialize(FAR struct w5500_lower_s *lower);

#endif /* CONFIG_NET_W5500 */
#endif /* __INCLUDE_NUTTX_NET_W5500_H */
