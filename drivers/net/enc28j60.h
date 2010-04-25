/****************************************************************************
 * drivers/net/enc28j60.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __DRIVERS_NET_ENC28J60_H
#define __DRIVERS_NET_ENC28J60_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* A total of seven instructions are implemented on the ENC28J60 */

#define ENC28J60_RCR      (0x00)    /* Read Control Register
                                     * 000 | aaaaa | (Registe value returned)) */
#define ENC28J60_RBM      (0x3a)    /* Read Buffer Memory
                                     * 001 | 11010 | (Read buffer data follows) */
#define ENC28J60_WCR      (0x40)    /* Write Control Register
                                     * 010 | aaaaa | dddddddd */
#define ENC28J60_WBM      (0x7a)    /* Write Buffer Memory
                                     * 011 | 11010 | (Write buffer data follows) */
#define ENC28J60_BFS      (0x80)    /* Bit Field Set
                                     * 100 | aaaaa | dddddddd */
#define ENC28J60_BFC      (0xa0)    /* Bit Field Clear
                                     * 101 | aaaaa | dddddddd */
#define ENC28J60_SRC      (0xff)    /* System Reset
                                     * 111 | 11111 | (No data) */

/* Control registers are accessed with the RCR, RBM, WCR, BFS, and BFC commands.
 * The following identifies all ENC28J60 control registers.  The Control register
 * memory is partitioned into four banks, selectable by the bank select bits,
 * BSEL1:BSEL0, in the ECON1 register.
 *
 * The last five locations (0x1b to 0x1f) of all banks point to a common set of
 * registers: EIE, EIR, ESTAT, ECON2 and ECON1. These are key registers used
 * in controlling and monitoring the operation of the device. Their common
 * mapping allows easy access without switching the bank.
 *
 * Control registers for the ENC28J60 are generically grouped as ETH, MAC and
 * MII registers. Register names starting with E belong to the ETH group. Similarly,
 * registers names starting with MA belong to the MAC group and registers prefixed
 * with MI belong to the MII group.
 */

#define EIE                (0x1b)   /* Ethernet Interrupt Enable Register */
#define EIR                (0x1c)   /* Ethernet Interupt Request Register */
#define ESTAT              (0x1d)   /* Ethernet Status Register */
#define ECON2              (0x1e)   /* Ethernet Control 2 Register */
#define ECON1              (0x1f)   /* Ethernet Control 1 Register */

/* Ethernet Interrupt Enable Register Bit Definitions */

#define EIE_RXERIE         (1 << 0) /* Bit 0: Receive Error Interrupt Enable */
#define EIE_TXERIE         (1 << 1) /* Bit 1: Transmit Error Interrupt Enable */
                                    /* Bit 2: Reserved */
#define EIE_TXIE           (1 << 3) /* Bit 3: Transmit Enable */
#define EIE_LINKIE         (1 << 4) /* Bit 4: Link Status Change Interrupt Enable */
#define EIE_DMAIE          (1 << 5) /* Bit 5: DMA Interrupt Enable */
#define EIE_PKTIE          (1 << 6) /* Bit 6: Receive Packet Pending Interrupt Enable */
#define EIE_INTIE          (1 << 7) /* Bit 7: Global INT Interrupt Enable */

/* Ethernet Interupt Request Register Bit Definitions */

#define EIR_RXERIF         (1 << 0) /* Bit 0: Receive Error Interrupt */
#define EIR_TXERIF         (1 << 1) /* Bit 1: Transmit Error Interrupt */
                                    /* Bit 2: Reserved */
#define EIR_TXIF           (1 << 3) /* Bit 3: Transmit Interrupt */
#define EIR_LINKIF         (1 << 4) /* Bit 4: Link Change Interrupt */
#define EIR_DMAIF          (1 << 5) /* Bit 5: DMA Interrupt */
#define EIR_PKTIF          (1 << 6) /* Bit 6: Receive Packet Pending Interrupt */
                                    /* Bit 7: Reserved */

/* Ethernet Status Register Bit Definitions */

#define ESTAT_CLKRDY       (1 << 0) /* Bit 0: Clock Ready */
#define ESTAT_TXABRT       (1 << 1) /* Bit 1: Transmit Abort Error */
#define ESTAT_RXBUSY       (1 << 2) /* Bit 2: Receive Busy */
                                    /* Bit 3: Reserved */
#define ESTAT_LATECOL      (1 << 4) /* Bit 4: Late Collision Error */
                                    /* Bit 5: Reserved */
#define ESTAT_BUFER        (1 << 6) /* Bit 6: Ethernet Buffer Error Status */
#define ESTAT_INT          (1 << 7) /* Bit 7: INT Interrupt */

/* Ethernet Control 1 Register Bit Definitions */

#define ECON1_BSEL_SHIFT   (0)      /* Bits 0-1: Bank select */
#define ECON1_BSEL_MASK    (3 << ECON1_BSEL_SHIFT)
#  define ECON1_BSEL_BANK0 (0 << 0) /* Bank 0 */
#  define ECON1_BSEL_BANK1 (1 << 1) /* Bank 1 */
#  define ECON1_BSEL_BANK2 (2 << 0) /* Bank 2 */
#  define ECON1_BSEL_BANK3 (3 << 0) /* Bank 3 */
#define ECON1_RXEN         (1 << 2) /* Bit 2: Receive Enable */
#define ECON1_TXRTS        (1 << 3) /* Bit 3: Transmit Request to Send */
#define ECON1_CSUMEN       (1 << 4) /* Bit 4: DMA Checksum Enable */
#define ECON1_DMAST        (1 << 5) /* Bit 5: DMA Start and Busy Status */
#define ECON1_RXRST        (1 << 6) /* Bit 6: Receive Logic Reset */
#define ECON1_TXRST        (1 << 7) /* Bit 7: Transmit Logic Reset */

/* Ethernet Control 2 Register */
                                    /* Bits 0-2: Reserved */
#define ECON2_VRPS         (1 << 3) /* Bit 3: Voltage Regulator Power Save Enable */
                                    /* Bit 4: Reserved */
#define ECON2_PWRSV        (1 << 5) /* Bit 5: Power Save Enable */
#define ECON2_PKTDEC       (1 << 6) /* Bit 6: Packet Decrement */
#define ECON2_AUTOINC      (1 << 7) /* Bit 7: Automatic Buffer Pointer Increment Enable */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_NET_ENC28J60_H */
