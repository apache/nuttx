/************************************************************************************************
 * arch/arm/src/sama5/chip/sam3u_uart.h
 * Debug Unit (DBGU) definitions for the SAMA5D3
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
 ************************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMA5_CHIP_SAM_DBGU_H
#define __ARCH_ARM_SRC_SAMA5_CHIP_SAM_DBGU_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/sam_memorymap.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* DBGU register offsets ************************************************************************/

#define SAM_DBGU_CR_OFFSET           0x0000 /* Control Register */
#define SAM_DBGU_MR_OFFSET           0x0004 /* Mode Register */
#define SAM_DBGU_IER_OFFSET          0x0008 /* Interrupt Enable Register */
#define SAM_DBGU_IDR_OFFSET          0x000c /* Interrupt Disable Register */
#define SAM_DBGU_IMR_OFFSET          0x0010 /* Interrupt Mask Register */
#define SAM_DBGU_SR_OFFSET           0x0014 /* [Channel] Status Register */
#define SAM_DBGU_RHR_OFFSET          0x0018 /* Receive Holding Register */
#define SAM_DBGU_THR_OFFSET          0x001c /* Transmit Holding Register */
#define SAM_DBGU_BRGR_OFFSET         0x0020 /* Baud Rate Generator Register */
                                            /* 0x0024-0x003c: Reserved */
#define SAM_DBGU_CIDR_OFFSET         0x0040 /* Chip ID Register */
#define SAM_DBGU_EXID_OFFSET         0x0044 /* Chip ID Extension Register */
#define SAM_DBGU_FNR_OFFSET          0x0048 /* Force NTRST Register */
                                            /* 0x004c-0x00fc: Reserved */

/* DBGU register addresses **********************************************************************/

#define SAM_DBGU_CR                  (SAM_DBGU_VBASE+SAM_DBGU_CR_OFFSET)
#define SAM_DBGU_MR                  (SAM_DBGU_VBASE+SAM_DBGU_MR_OFFSET)
#define SAM_DBGU_IER                 (SAM_DBGU_VBASE+SAM_DBGU_IER_OFFSET)
#define SAM_DBGU_IDR                 (SAM_DBGU_VBASE+SAM_DBGU_IDR_OFFSET)
#define SAM_DBGU_IMR                 (SAM_DBGU_VBASE+SAM_DBGU_IMR_OFFSET)
#define SAM_DBGU_SR                  (SAM_DBGU_VBASE+SAM_DBGU_SR_OFFSET)
#define SAM_DBGU_RHR                 (SAM_DBGU_VBASE+SAM_DBGU_RHR_OFFSET)
#define SAM_DBGU_THR                 (SAM_DBGU_VBASE+SAM_DBGU_THR_OFFSET)
#define SAM_DBGU_BRGR                (SAM_DBGU_VBASE+SAM_DBGU_BRGR_OFFSET)
#define SAM_DBGU_CIDR                (SAM_DBGU_VBASE+SAM_DBGU_CIDR_OFFSET)
#define SAM_DBGU_EXID                (SAM_DBGU_VBASE+SAM_DBGU_EXID_OFFSET)
#define SAM_DBGU_FNR                 (SAM_DBGU_VBASE+SAM_DBGU_FNR_OFFSET)

/* DBGU register bit definitions ****************************************************************/

/* DBGU Control Register */

#define DBGU_CR_RSTRX                (1 << 2)  /* Bit 2:  Reset Receiver */
#define DBGU_CR_RSTTX                (1 << 3)  /* Bit 3:  Reset Transmitter */
#define DBGU_CR_RXEN                 (1 << 4)  /* Bit 4:  Receiver Enable */
#define DBGU_CR_RXDIS                (1 << 5)  /* Bit 5:  Receiver Disable */
#define DBGU_CR_TXEN                 (1 << 6)  /* Bit 6:  Transmitter Enable */
#define DBGU_CR_TXDIS                (1 << 7)  /* Bit 7:  Transmitter Disable */
#define DBGU_CR_RSTSTA               (1 << 8)  /* Bit 8:  Reset Status Bits */

/* DBGU Mode Register */

#define DBGU_MR_PAR_SHIFT            (9)       /* Bits 9-11: Parity Type (2) */
#define DBGU_MR_PAR_MASK             (7 << DBGU_MR_PAR_SHIFT)
#  define DBGU_MR_PAR_EVEN           (0 << DBGU_MR_PAR_SHIFT) /* Even parity (1) */
#  define DBGU_MR_PAR_ODD            (1 << DBGU_MR_PAR_SHIFT) /* Odd parity (1) */
#  define DBGU_MR_PAR_SPACE          (2 << DBGU_MR_PAR_SHIFT) /* Space: parity forced to 0 (1) */
#  define DBGU_MR_PAR_MARK           (3 << DBGU_MR_PAR_SHIFT) /* Mark: parity forced to 1 (1) */
#  define DBGU_MR_PAR_NONE           (4 << DBGU_MR_PAR_SHIFT) /* No parity (1) */
#define DBGU_MR_CHMODE_SHIFT         (14)      /* Bits 14-15: Channel Mode (2) */
#define DBGU_MR_CHMODE_MASK          (3 << DBGU_MR_CHMODE_SHIFT)
#  define DBGU_MR_CHMODE_NORMAL      (0 << DBGU_MR_CHMODE_SHIFT) /* Normal Mode */
#  define DBGU_MR_CHMODE_ECHO        (1 << DBGU_MR_CHMODE_SHIFT) /* Automatic Echo */
#  define DBGU_MR_CHMODE_LLPBK       (2 << DBGU_MR_CHMODE_SHIFT) /* Local Loopback */
#  define DBGU_MR_CHMODE_RLPBK       (3 << DBGU_MR_CHMODE_SHIFT) /* Remote Loopback */

/* DBGU Interrupt Enable Register, DBGU Interrupt Disable Register, DBGU Interrupt Mask
 * Register, and DBGU Status Register common bit field definitions
 */

#define DBGU_INT_RXRDY               (1 << 0)  /* Bit 0:  RXRDY Interrupt */
#define DBGU_INT_TXRDY               (1 << 1)  /* Bit 1:  TXRDY Interrupt */
#define DBGU_INT_OVRE                (1 << 5)  /* Bit 5:  Overrun Error Interrupt */
#define DBGU_INT_FRAME               (1 << 6)  /* Bit 6:  Framing Error Interrupt */
#define DBGU_INT_PARE                (1 << 7)  /* Bit 7:  Parity Error Interrupt */
#define DBGU_INT_TXEMPTY             (1 << 9)  /* Bit 9:  TXEMPTY Interrupt */
#define DBGU_INT_COMMTX              (1 << 30) /* Bit 30: COMMTX (from ARM) Interrupt */
#define DBGU_INT_COMMRX              (1 << 31) /* Bit 31: COMMRX (from ARM) Interrupt */

#define DBGU_INT_ALLINTS             (0xc00002e3)

/* DBGU Receiver Holding Register */

#define DBGU_RHR_RXCHR_SHIFT         (0)       /* Bits 0-7: Received Character */
#define DBGU_RHR_RXCHR_MASK          (0xff << DBGU_RHR_RXCHR_SHIFT)

/* DBGU Transmit Holding Register */

#define DBGU_THR_TXCHR_SHIFT         (0)       /* Bits 0-7: Character to be Transmitted (DBGU only) */
#define DBGU_THR_TXCHR_MASK          (0xff << DBGU_THR_TXCHR_SHIFT)

/* DBGU Baud Rate Generator Register */

#define DBGU_BRGR_CD_SHIFT           (0)      /* Bits 0-15: Clock Divisor */
#define DBGU_BRGR_CD_MASK            (0xffff << DBGU_BRGR_CD_SHIFT)
#  define DBGU_BRGR_CD(n)            ((uint32_t)(n) << DBGU_BRGR_CD_SHIFT)

/* Chip ID Register */

#define DBGU_CIDR_VERSION_SHIFT      (0)      /* Bits 0-4: Version of the Device */
#define DBGU_CIDR_VERSION_MASK       (31 << DBGU_CIDR_VERSION_SHIFT)
#define DBGU_CIDR_EPROC_SHIFT        (5)      /* Bits 5-xx7 Embedded Processor */
#define DBGU_CIDR_EPROC_MASK         (7 << DBGU_CIDR_EPROC_SHIFT)
#  define DBGU_CIDR_EPROC_CA5        (6 << DBGU_CIDR_EPROC_SHIFT) /* Cortex-A5 */
#define DBGU_CIDR_NVPSIZ_SHIFT       (8)      /* Bits 8-11: Nonvolatile Program Memory Size */
#define DBGU_CIDR_NVPSIZ_MASK        (15 << DBGU_CIDR_NVPSIZ_SHIFT)
#  define DBGU_CIDR_NVPSIZ_NONE      (0 << DBGU_CIDR_NVPSIZ_SHIFT)  /* None */
#  define DBGU_CIDR_NVPSIZ_8K        (1 << DBGU_CIDR_NVPSIZ_SHIFT)  /* 8 Kbytes */
#  define DBGU_CIDR_NVPSIZ_16K       (2 << DBGU_CIDR_NVPSIZ_SHIFT)  /* 16 Kbytes */
#  define DBGU_CIDR_NVPSIZ_32K       (3 << DBGU_CIDR_NVPSIZ_SHIFT)  /* 32 Kbytes */
#  define DBGU_CIDR_NVPSIZ_64K       (5 << DBGU_CIDR_NVPSIZ_SHIFT)  /* 64 Kbytes */
#  define DBGU_CIDR_NVPSIZ_128K      (7 << DBGU_CIDR_NVPSIZ_SHIFT)  /* 128 Kbytes */
#  define DBGU_CIDR_NVPSIZ_256K      (9 << DBGU_CIDR_NVPSIZ_SHIFT)  /* 256 Kbytes */
#  define DBGU_CIDR_NVPSIZ_512K      (10 << DBGU_CIDR_NVPSIZ_SHIFT) /* 512 Kbytes */
#  define DBGU_CIDR_NVPSIZ_1M        (12 << DBGU_CIDR_NVPSIZ_SHIFT) /* 1024 Kbytes */
#  define DBGU_CIDR_NVPSIZ_2M        (14 << DBGU_CIDR_NVPSIZ_SHIFT) /* 2048 Kbytes */
#define DBGU_CIDR_NVPSIZ2_SHIFT      (12)     /* Bits 12-15: Second Nonvolatile Program Memory Size */
#define DBGU_CIDR_NVPSIZ2_MASK       (15 << DBGU_CIDR_NVPSIZ2_SHIFT)
#  define DBGU_CIDR_NVPSIZ2_NONE     (0 << DBGU_CIDR_NVPSIZ2_SHIFT)  /* None */
#  define DBGU_CIDR_NVPSIZ2_8K       (1 << DBGU_CIDR_NVPSIZ2_SHIFT)  /* 8 Kbytes */
#  define DBGU_CIDR_NVPSIZ2_16K      (2 << DBGU_CIDR_NVPSIZ2_SHIFT)  /* 16 Kbytes */
#  define DBGU_CIDR_NVPSIZ2_32K      (3 << DBGU_CIDR_NVPSIZ2_SHIFT)  /* 32 Kbytes */
#  define DBGU_CIDR_NVPSIZ2_64K      (5 << DBGU_CIDR_NVPSIZ2_SHIFT)  /* 64 Kbytes */
#  define DBGU_CIDR_NVPSIZ2_128K     (7 << DBGU_CIDR_NVPSIZ2_SHIFT)  /* 128 Kbytes */
#  define DBGU_CIDR_NVPSIZ2_256K     (9 << DBGU_CIDR_NVPSIZ2_SHIFT)  /* 256 Kbytes */
#  define DBGU_CIDR_NVPSIZ2_512K     (10 << DBGU_CIDR_NVPSIZ2_SHIFT) /* 512 Kbytes */
#  define DBGU_CIDR_NVPSIZ2_1M       (12 << DBGU_CIDR_NVPSIZ2_SHIFT) /* 1024 Kbytes */
#  define DBGU_CIDR_NVPSIZ2_2M       (14 << DBGU_CIDR_NVPSIZ2_SHIFT) /* 2048 Kbytes */
#define DBGU_CIDR_SRAMSIZ_SHIFT      (16)     /* Bits 16-19: Internal SRAM Size */
#define DBGU_CIDR_SRAMSIZ_MASK       (15 << DBGU_CIDR_SRAMSIZ_SHIFT)
#  define DBGU_CIDR_SRAMSIZ_1K       (1 << DBGU_CIDR_SRAMSIZ_SHIFT)  /* 1 Kbytes */
#  define DBGU_CIDR_SRAMSIZ_2K       (2 << DBGU_CIDR_SRAMSIZ_SHIFT)  /* 2 Kbytes */
#  define DBGU_CIDR_SRAMSIZ_6K       (3 << DBGU_CIDR_SRAMSIZ_SHIFT)  /* 6 Kbytes */
#  define DBGU_CIDR_SRAMSIZ_112K     (4 << DBGU_CIDR_SRAMSIZ_SHIFT)  /* 112 Kbytes */
#  define DBGU_CIDR_SRAMSIZ_4K       (5 << DBGU_CIDR_SRAMSIZ_SHIFT)  /* 4 Kbytes */
#  define DBGU_CIDR_SRAMSIZ_80K      (6 << DBGU_CIDR_SRAMSIZ_SHIFT)  /* 80 Kbytes */
#  define DBGU_CIDR_SRAMSIZ_160K     (7 << DBGU_CIDR_SRAMSIZ_SHIFT)  /* 160 Kbytes */
#  define DBGU_CIDR_SRAMSIZ_8K       (8 << DBGU_CIDR_SRAMSIZ_SHIFT)  /* 8 Kbytes */
#  define DBGU_CIDR_SRAMSIZ_16K      (9 << DBGU_CIDR_SRAMSIZ_SHIFT)  /* 16 Kbytes */
#  define DBGU_CIDR_SRAMSIZ_32K      (10 << DBGU_CIDR_SRAMSIZ_SHIFT) /* 32 Kbytes */
#  define DBGU_CIDR_SRAMSIZ_64K      (11 << DBGU_CIDR_SRAMSIZ_SHIFT) /* 64 Kbytes */
#  define DBGU_CIDR_SRAMSIZ_128K     (12 << DBGU_CIDR_SRAMSIZ_SHIFT) /* 128 Kbytes */
#  define DBGU_CIDR_SRAMSIZ_256K     (13 << DBGU_CIDR_SRAMSIZ_SHIFT) /* 256 Kbytes */
#  define DBGU_CIDR_SRAMSIZ_96K      (14 << DBGU_CIDR_SRAMSIZ_SHIFT) /* 96 Kbytes */
#  define DBGU_CIDR_SRAMSIZ_512K     (15 << DBGU_CIDR_SRAMSIZ_SHIFT) /* 512 Kbytes */
#define DBGU_CIDR_ARCH_SHIFT         (20)     /* Bits 20-23: Architecture Identifier */
#define DBGU_CIDR_ARCH_MASK          (15 << DBGU_CIDR_ARCH_SHIFT)
#  define DBGU_CIDR_ARCH_ATSAMA5xx   (0xa5 << DBGU_CIDR_ARCH_SHIFT) /* ATSAMA5xx Series */
#define DBGU_CIDR_NVPTYP_SHIFT       (28)     /* Bits 28-30: Nonvolatile Program Memory Type */
#define DBGU_CIDR_NVPTYP_MASK        (7 << DBGU_CIDR_NVPTYP_SHIFT)
#  define DBGU_CIDR_NVPTYP_ROM       (0 << DBGU_CIDR_NVPTYP_SHIFT) /* ROM */
#  define DBGU_CIDR_NVPTYP_ROMLESS   (1 << DBGU_CIDR_NVPTYP_SHIFT) /* ROMless or on-chip Flash */
#  define DBGU_CIDR_NVPTYP_SRAM      (4 << DBGU_CIDR_NVPTYP_SHIFT) /* SRAM emulating ROM */
#  define DBGU_CIDR_NVPTYP_FLASH     (2 << DBGU_CIDR_NVPTYP_SHIFT) /* Embedded Flash Memory */
#  define DBGU_CIDR_NVPTYP_ROMFLASH  (3 << DBGU_CIDR_NVPTYP_SHIFT) /* ROM and Embedded Flash Memory */
#define DBGU_CIDR_EXT                (1 << 31) /* Bit 31:  Extension Flag */

/* Chip ID Extension Register (32-bit ID */

/* Force NTRST Register */

#define DBGU_FNR_FNTRST              (1 << 0)  /* Bit 0:  Force NTRST */

/************************************************************************************************
 * Public Types
 ************************************************************************************************/

/************************************************************************************************
 * Public Data
 ************************************************************************************************/

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMA5_CHIP_SAM_DBGU_H */
