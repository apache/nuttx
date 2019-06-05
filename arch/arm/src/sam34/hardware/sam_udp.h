/****************************************************************************************
 * arch/arm/src/sam34/hardware/sam_udp.h
 * USB Device Port (UDP) definitions for the SAM4E
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_UDP_H
#define __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_UDP_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/
/* General Definitions ******************************************************************/
/* Capabilities and characteristics of endpoints
 *
 *   EP  EP BANKS  EP SIZE   EP TYPE
 *   --- --------- --------- ---------
 *    0      1        64     Control/Bulk/Interrupt
 *    1      2        64     Bulk/Iso/Interrupt
 *    2      2        64     Bulk/Iso/Interrupt
 *    3      1        64     Control/Bulk/Interrupt
 *    4      2       512     Bulk/Iso/Interrupt
 *    5      2       512     Bulk/Iso/Interrupt
 *    6      2        64     Bulk/Iso/Interrupt
 *    7      2        64     Bulk/Iso/Interrupt
 */

#define SAM_UDP_NENDPOINTS                  8     /* EP0-7 */
#define SAM_UDP_MAXPACKETSIZE(ep)           ((((unsigned)(ep) & 6) == 4) ? 512 : 64)
#define SAM_UDP_NBANKS(ep)                  (((unsigned)(ep) == 0 || (unsigned)(ep) == 3) ? 1 : 2)
#define SAM_UDP_CONTROL(ep)                 (((unsigned)(ep) == 0 || (unsigned)(ep) == 3))
#define SAM_UDP_BULK(ep)                    (true)
#define SAM_UDP_ISOCHRONOUS(ep)             (((unsigned)(ep) != 0 && (unsigned)(ep) != 3))
#define SAM_UDP_INTERRUPT(ep)               (true)

/* UDP register offsets *****************************************************************/

/* Global Registers */

#define SAM_UDP_FRMNUM_OFFSET               0x0000 /* UDP Frame Number Register */
#define SAM_UDP_GLBSTAT_OFFSET              0x0004 /* UDP Global State Register */
#define SAM_UDP_FADDR_OFFSET                0x0008 /* UDP Function Address Register */
                                                   /* 0x000c: Reserved */
#define SAM_UDP_IER_OFFSET                  0x0010 /* UDP Interrupt Enable Register */
#define SAM_UDP_IDR_OFFSET                  0x0014 /* UDP Interrupt Disable Register */
#define SAM_UDP_IMR_OFFSET                  0x0018 /* UDP Interrupt Mask Register */
#define SAM_UDP_ISR_OFFSET                  0x001c /* UDP Interrupt Status Register */
#define SAM_UDP_ICR_OFFSET                  0x0020 /* UDP Interrupt Clear Register */
                                                   /* 0x0024: Reserved */
#define SAM_UDP_RSTEP_OFFSET                0x0028 /* UDP Reset Endpoint Register */
                                                   /* 0x002c: Reserved */
/* Endpoint registers */

#define SAM_UDPEP_CSR_OFFSET(n)             (0x0030+((n)<<2))
#  define SAM_UDPEP_CSR0_OFFSET             0x0030 /* Endpoint Control and Status Register 0 */
#  define SAM_UDPEP_CSR1_OFFSET             0x0034 /* Endpoint Control and Status Register 1 */
#  define SAM_UDPEP_CSR2_OFFSET             0x0038 /* Endpoint Control and Status Register 2 */
#  define SAM_UDPEP_CSR3_OFFSET             0x003c /* Endpoint Control and Status Register 3 */
#  define SAM_UDPEP_CSR4_OFFSET             0x0040 /* Endpoint Control and Status Register 4 */
#  define SAM_UDPEP_CSR5_OFFSET             0x0044 /* Endpoint Control and Status Register 5 */
#  define SAM_UDPEP_CSR6_OFFSET             0x0048 /* Endpoint Control and Status Register 6 */
#  define SAM_UDPEP_CSR7_OFFSET             0x004c /* Endpoint Control and Status Register 7 */
#define SAM_UDPEP_FDR_OFFSET(n)             (0x0050+((n)<<2))
#  define SAM_UDPEP_FDR0_OFFSET             0x0050 /* Endpoint FIFO Data Register 0 */
#  define SAM_UDPEP_FDR1_OFFSET             0x0054 /* Endpoint FIFO Data Register 1 */
#  define SAM_UDPEP_FDR2_OFFSET             0x0058 /* Endpoint FIFO Data Register 2 */
#  define SAM_UDPEP_FDR3_OFFSET             0x005c /* Endpoint FIFO Data Register 3 */
#  define SAM_UDPEP_FDR4_OFFSET             0x0060 /* Endpoint FIFO Data Register 4 */
#  define SAM_UDPEP_FDR5_OFFSET             0x0064 /* Endpoint FIFO Data Register 5 */
#  define SAM_UDPEP_FDR6_OFFSET             0x0068 /* Endpoint FIFO Data Register 6 */
#  define SAM_UDPEP_FDR7_OFFSET             0x006c /* Endpoint FIFO Data Register 7 */
                                                   /* 0x0070: Reserved */
#define SAM_UDP_TXVC_OFFSET                 0x0074 /* Transceiver Control Register */
                                                   /* 0x0078-0x00fc: Reserved */

/* UDP register addresses ***************************************************************/

/* Global Registers */

#define SAM_UDP_FRMNUM                      (SAM_UDP_BASE+SAM_UDP_FRMNUM_OFFSET)
#define SAM_UDP_GLBSTAT                     (SAM_UDP_BASE+SAM_UDP_GLBSTAT_OFFSET)
#define SAM_UDP_FADDR                       (SAM_UDP_BASE+SAM_UDP_FADDR_OFFSET)
#define SAM_UDP_IER                         (SAM_UDP_BASE+SAM_UDP_IER_OFFSET)
#define SAM_UDP_IDR                         (SAM_UDP_BASE+SAM_UDP_IDR_OFFSET)
#define SAM_UDP_IMR                         (SAM_UDP_BASE+SAM_UDP_IMR_OFFSET)
#define SAM_UDP_ISR                         (SAM_UDP_BASE+SAM_UDP_ISR_OFFSET)
#define SAM_UDP_ICR                         (SAM_UDP_BASE+SAM_UDP_ICR_OFFSET)
#define SAM_UDP_RSTEP                       (SAM_UDP_BASE+SAM_UDP_RSTEP_OFFSET)

/* Endpoint registers */

#define SAM_UDPEP_CSR(n)                    (SAM_UDP_BASE+SAM_UDPEP_CSR_OFFSET(n))
#  define SAM_UDPEP_CSR0                    (SAM_UDP_BASE+SAM_UDPEP_CSR0_OFFSET)
#  define SAM_UDPEP_CSR1                    (SAM_UDP_BASE+SAM_UDPEP_CSR1_OFFSET)
#  define SAM_UDPEP_CSR2                    (SAM_UDP_BASE+SAM_UDPEP_CSR2_OFFSET)
#  define SAM_UDPEP_CSR3                    (SAM_UDP_BASE+SAM_UDPEP_CSR3_OFFSET)
#  define SAM_UDPEP_CSR4                    (SAM_UDP_BASE+SAM_UDPEP_CSR4_OFFSET)
#  define SAM_UDPEP_CSR5                    (SAM_UDP_BASE+SAM_UDPEP_CSR5_OFFSET)
#  define SAM_UDPEP_CSR6                    (SAM_UDP_BASE+SAM_UDPEP_CSR6_OFFSET)
#  define SAM_UDPEP_CSR7                    (SAM_UDP_BASE+SAM_UDPEP_CSR7_OFFSET)
#define SAM_UDPEP_FDR(n)                    (SAM_UDP_BASE+SAM_UDPEP_FDR_OFFSET(n))
#  define SAM_UDPEP_FDR0                    (SAM_UDP_BASE+SAM_UDPEP_FDR0_OFFSET)
#  define SAM_UDPEP_FDR1                    (SAM_UDP_BASE+SAM_UDPEP_FDR1_OFFSET)
#  define SAM_UDPEP_FDR2                    (SAM_UDP_BASE+SAM_UDPEP_FDR2_OFFSET)
#  define SAM_UDPEP_FDR3                    (SAM_UDP_BASE+SAM_UDPEP_FDR3_OFFSET)
#  define SAM_UDPEP_FDR4                    (SAM_UDP_BASE+SAM_UDPEP_FDR4_OFFSET)
#  define SAM_UDPEP_FDR5                    (SAM_UDP_BASE+SAM_UDPEP_FDR5_OFFSET)
#  define SAM_UDPEP_FDR6                    (SAM_UDP_BASE+SAM_UDPEP_FDR6_OFFSET)
#  define SAM_UDPEP_FDR7                    (SAM_UDP_BASE+SAM_UDPEP_FDR7_OFFSET)

#define SAM_UDP_TXVC                        (SAM_UDP_BASE+SAM_UDP_TXVC_OFFSET)

/* UDP register bit definitions *********************************************************/

/* Global Registers */

/* UDP Frame Number Register */

#define UDP_FRMNUM_SHIFT                    (0)       /* Bits 0-10: Frame Number in Packet Field Formats */
#define UDP_FRMNUM_MASK                     (0x000007ff)
#define UDP_FRMNUM_FRMERR                   (1 << 16) /* Bit 16: Frame Error */
#define UDP_FRMNUM_FRMOK                    (1 << 17) /* Bit 17: Frame OK */

/* UDP Global State Register */

#define UDP_GLBSTAT_FADDEN                  (1 << 0)  /* Bit 0:  Function Address Enable */
#define UDP_GLBSTAT_CONFG                   (1 << 1)  /* Bit 1:  Configured */
#define UDP_GLBSTAT_ESR                     (1 << 2)  /* Bit 2:  Enable Send Resume */
#define UDP_GLBSTAT_RSMINPR                 (1 << 3)  /* Bit 3:  */
#define UDP_GLBSTAT_RMWUPE                  (1 << 4)  /* Bit 4:  Remote Wake-up Enable */

/* UDP Function Address Register */

#define UDP_FADDR_SHIFT                     (0)       /* Bits 0-6:  Function Address Value */
#define UDP_FADDR_MASK                      (0x0000007f)
#  define UDP_FADDR(n)                      ((uint32_t)(n))
#define UDP_FADDR_FEN                       (1 << 8)  /* Bit 8:  Function Enable */

/* UDP Interrupt Enable, UDP Interrupt Disable, UDP Interrupt Mask, UDP Interrupt
 * Status, and UDP Interrupt Clear Registers.
 */

#define UDP_INT_EP_MASK                     (0x000000ff)
#define UDP_INT_EP(n)                       (1 << (n))
#  define UDP_INT_EP0                       (1 << 0)  /* Bit 0:  Endpoint 0 Interrupt (Not ICR) */
#  define UDP_INT_EP1                       (1 << 1)  /* Bit 1:  Endpoint 1 Interrupt (Not ICR) */
#  define UDP_INT_EP2                       (1 << 2)  /* Bit 2:  Endpoint 2 Interrupt (Not ICR) */
#  define UDP_INT_EP3                       (1 << 3)  /* Bit 3:  Endpoint 3 Interrupt (Not ICR) */
#  define UDP_INT_EP4                       (1 << 4)  /* Bit 4:  Endpoint 4 Interrupt (Not ICR) */
#  define UDP_INT_EP5                       (1 << 5)  /* Bit 5:  Endpoint 5 Interrupt (Not ICR) */
#  define UDP_INT_EP6                       (1 << 6)  /* Bit 6:  Endpoint 6 Interrupt (Not ICR) */
#  define UDP_INT_EP7                       (1 << 7)  /* Bit 7:  Endpoint 7 Interrupt (Not ICR) */
#define UDP_INT_RXSUSP                      (1 << 8)  /* Bit 8:  UDP Suspend Interrupt */
#define UDP_INT_RXRSM                       (1 << 9)  /* Bit 9:  UDP Resume Interrupt */
#define UDP_INT_EXTRSM                      (1 << 10) /* Bit 10: */
#define UDP_INT_SOF                         (1 << 11) /* Bit 11: Start Of Frame Interrupt */
#define UDP_ISR_ENDBUSRES                   (1 << 12) /* Bit 12: End of BUS Reset Interrupt Status (ISR and ICR only) */
#define UDP_INT_WAKEUP                      (1 << 13) /* Bit 13: UDP bus Wake-up Interrupt */

#define UDP_INT_ALL                         (0x00003fff)

/* UDP Reset Endpoint Register */

#define UDP_RSTEP(n)                        (1 << (n))
#  define UDP_RSTEP0                        (1 << 0)  /* Bit 0:  Reset Endpoint 0 */
#  define UDP_RSTEP1                        (1 << 1)  /* Bit 1:  Reset Endpoint 1 */
#  define UDP_RSTEP2                        (1 << 2)  /* Bit 2:  Reset Endpoint 2 */
#  define UDP_RSTEP3                        (1 << 3)  /* Bit 3:  Reset Endpoint 3 */
#  define UDP_RSTEP4                        (1 << 4)  /* Bit 4:  Reset Endpoint 4 */
#  define UDP_RSTEP5                        (1 << 5)  /* Bit 5:  Reset Endpoint 5 */
#  define UDP_RSTEP6                        (1 << 6)  /* Bit 6:  Reset Endpoint 6 */
#  define UDP_RSTEP7                        (1 << 7)  /* Bit 7:  Reset Endpoint 7 */

/* Endpoint registers */
/* Endpoint Control and Status Registers */

#define UDPEP_CSR_TXCOMP                    (1 << 0)  /* Bit 0:  Generates an IN packet with data */
#define UDPEP_CSR_RXDATABK0                 (1 << 1)  /* Bit 1:  Receive Data Bank 0 */
#define UDPEP_CSR_RXSETUP                   (1 << 2)  /* Bit 2:  Received Setup */
#define UDPEP_CSR_STALLSENT                 (1 << 3)  /* Bit 3:  Stall Sent */
#define UDPEP_CSR_ISOERROR                  (1 << 3)  /* Bit 3:  CRC error in isochronous transfer */
#define UDPEP_CSR_TXPKTRDY                  (1 << 4)  /* Bit 4:  Transmit Packet Ready */
#define UDPEP_CSR_FORCESTALL                (1 << 5)  /* Bit 5:  Force Stall */
#define UDPEP_CSR_RXDATABK1                 (1 << 6)  /* Bit 6:  Receive Data Bank 1 */
#define UDPEP_CSR_DIR_SHIFT                 (7)       /* Bit 7:  Transfer Direction */
#define UDPEP_CSR_DIR                       (1 << 7)  /* Bit 7:  Transfer Direction */
#define UDPEP_CSR_EPTYPE_SHIFT              (8)       /* Bit 8-10: Endpoint type */
#define UDPEP_CSR_EPTYPE_MASK               (7 << UDPEP_CSR_EPTYPE_SHIFT)
#  define UDPEP_CSR_EPTYPE_CTRL             (0 << UDPEP_CSR_EPTYPE_SHIFT) /* Control */
#  define UDPEP_CSR_EPTYPE_ISOOUT           (1 << UDPEP_CSR_EPTYPE_SHIFT) /* Isochronous OUT */
#  define UDPEP_CSR_EPTYPE_ISOIN            (5 << UDPEP_CSR_EPTYPE_SHIFT) /* Isochronous IN */
#  define UDPEP_CSR_EPTYPE_BULKOUT          (2 << UDPEP_CSR_EPTYPE_SHIFT) /* Bulk OUT */
#  define UDPEP_CSR_EPTYPE_BULKIN           (6 << UDPEP_CSR_EPTYPE_SHIFT) /* Bulk IN */
#  define UDPEP_CSR_EPTYPE_INTOUT           (3 << UDPEP_CSR_EPTYPE_SHIFT) /* Interrupt OUT */
#  define UDPEP_CSR_EPTYPE_INTIN            (7 << UDPEP_CSR_EPTYPE_SHIFT) /* Interrupt IN */
#define UDPEP_CSR_DTGLE                     (1 << 11) /* Bit 11:  Data Toggle */
#define UDPEP_CSR_EPEDS                     (1 << 15) /* Bit 15:  Endpoint Enable Disable */
#define UDPEP_CSR_RXBYTECNT_SHIFT           (16)      /* Bits 16-26: Number of Bytes Available in the FIFO */
#define UDPEP_CSR_RXBYTECNT_MASK            (0x7ff << UDPEP_CSR_RXBYTECNT_SHIFT)

/* Endpoint FIFO Data Registers */

#define UDPEP_FDR_MASK                      (0xff)    /* Bits 0-7: FIFO data value */

/* Transceiver Control Register */

#define UDP_TXVC_TXVDIS                     (1 << 8)  /* Bit 8:  Transceiver Disable */
#define UDP_TXVC_PUON                       (1 << 9)  /* Bit 9:  Pull-up On */

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_UDP_H */
