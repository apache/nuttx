/****************************************************************************************************
 * arch/arm/src/lpc54xx/lpc54_dma.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC54XX_CHIP_LPC54_DMA_H
#define __ARCH_ARM_SRC_LPC54XX_CHIP_LPC54_DMA_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include "chip/lpc54_memorymap.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

#define LPC54_DMA_NCHANNELS   29  /* 29 DMA channels, 0..28 */

/* Register offsets *********************************************************************************/
/* Global control and status registers */

#define LPC54_DMA_CTRL_OFFSET         0x0000  /* DMA control */
#define LPC54_DMA_INTSTA_OFFSET       0x0004  /* Interrupt status */
#define LPC54_DMA_SRAMBASE_OFFSET     0x0008  /* SRAM address of the channel configuration table */

/* Shared registers */

#define LPC54_DMA_ENABLESET0_OFFSET   0x0020  /* Channel enable read and Set for all DMA channels */
#define LPC54_DMA_ENABLECLR0_OFFSET   0x0028  /* Channel enable clear for all DMA channels */
#define LPC54_DMA_ACTIVE0_OFFSET      0x0030  /* Channel active status for all DMA channels */
#define LPC54_DMA_BUSY0_OFFSET        0x0038  /* Channel busy status for all DMA channels */
#define LPC54_DMA_ERRINT0_OFFSET      0x0040  /* Error interrupt status for all DMA channels */
#define LPC54_DMA_INTENSET0_OFFSET    0x0048  /* Interrupt enable read and Set for all DMA channels */
#define LPC54_DMA_INTENCLR0_OFFSET    0x0050  /* Interrupt enable clear for all DMA channels */
#define LPC54_DMA_INTA0_OFFSET        0x0058  /* Interrupt A status for all DMA channels */
#define LPC54_DMA_INTB0_OFFSET        0x0060  /* Interrupt B status for all DMA channels */
#define LPC54_DMA_SETVALID0_OFFSET    0x0068  /* Set ValidPending control bits for all DMA channels */
#define LPC54_DMA_SETTRIG0_OFFSET     0x0070  /* Set trigger control bits for all DMA channels */
#define LPC54_DMA_ABORT0_OFFSET       0x0078  /* Channel abort control for all DMA channels */

/* Channel registers 
 *
 
 * CFGn      Configuration register for DMA channel n
 * CTLSTATn  Control and status register for DMA channel n
 * XFERCFGn  Transfer configuration register for DMA channel n
 */

#define LPC54_DMA_CHAN_OFFSET(n)      ((n) << 4)
#define LPC54_DMA_CHAN_BASE_OFFSET    0x0400

#define LPC54_DMA_CFG_OFFSET          0x0000
#define LPC54_DMA_CTLSTAT_OFFSET      0x0004
#define LPC54_DMA_XFERCFG_OFFSET      0x0008

#define LPC54_DMA_CFGn_OFFSET(n)      (LPC54_DMA_CHAN_BASE_OFFSET + LPC54_DMA_CHAN_OFFSET(n) + LPC54_DMA_CFG_OFFSET)
#define LPC54_DMA_CTLSTATn_OFFSET(n)  (LPC54_DMA_CHAN_BASE_OFFSET + LPC54_DMA_CHAN_OFFSET(n) + LPC54_DMA_CTLSTAT_OFFSET)
#define LPC54_DMA_XFERCFGn_OFFSET(n)  (LPC54_DMA_CHAN_BASE_OFFSET + LPC54_DMA_CHAN_OFFSET(n) + LPC54_DMA_CTLSTAT_OFFSET)

/* Register addresses *******************************************************************************/
/* Global control and status registers */

#define LPC54_DMA_CTRL                (LPC54_DMA_BASE + LPC54_DMA_CTRL_OFFSET)
#define LPC54_DMA_INTSTA              (LPC54_DMA_BASE + LPC54_DMA_INTSTA_OFFSET)
#define LPC54_DMA_SRAMBASE            (LPC54_DMA_BASE + LPC54_DMA_SRAMBASE_OFFSET)

/* Shared registers */

#define LPC54_DMA_ENABLESET0          (LPC54_DMA_BASE + LPC54_DMA_ENABLESET0_OFFSET)
#define LPC54_DMA_ENABLECLR0          (LPC54_DMA_BASE + LPC54_DMA_ENABLECLR0_OFFSET)
#define LPC54_DMA_ACTIVE0             (LPC54_DMA_BASE + LPC54_DMA_ACTIVE0_OFFSET)
#define LPC54_DMA_BUSY0               (LPC54_DMA_BASE + LPC54_DMA_BUSY0_OFFSET)
#define LPC54_DMA_ERRINT0             (LPC54_DMA_BASE + LPC54_DMA_ERRINT0_OFFSET)
#define LPC54_DMA_INTENSET0           (LPC54_DMA_BASE + LPC54_DMA_INTENSET0_OFFSET)
#define LPC54_DMA_INTENCLR0           (LPC54_DMA_BASE + LPC54_DMA_INTENCLR0_OFFSET)
#define LPC54_DMA_INTA0               (LPC54_DMA_BASE + LPC54_DMA_INTA0_OFFSET)
#define LPC54_DMA_INTB0               (LPC54_DMA_BASE + LPC54_DMA_INTB0_OFFSET)
#define LPC54_DMA_SETVALID0           (LPC54_DMA_BASE + LPC54_DMA_SETVALID0_OFFSET)
#define LPC54_DMA_SETTRIG0            (LPC54_DMA_BASE + LPC54_DMA_SETTRIG0_OFFSET)
#define LPC54_DMA_ABORT0              (LPC54_DMA_BASE + LPC54_DMA_ABORT0_OFFSET)

/* Channel registers */

#define LPC54_DMA_CFG(n)              (LPC54_DMA_BASE + LPC54_DMA_CFGn_OFFSET(n))
#define LPC54_DMA_CTLSTAT(n)          (LPC54_DMA_BASE + LPC54_DMA_CTLSTATn_OFFSET(n))
#define LPC54_DMA_XFERCFG(n)          (LPC54_DMA_BASE + LPC54_DMA_XFERCFGn_OFFSET(n))

/* Register bit definitions *************************************************************************/

/* DMA control */

#define DMA_CTRL_ENABLE               (1 << 0)  /* Bit 0: DMA controller master enable */

/* Interrupt status */

#define DMA_INTSTA_ACTIVEINT          (1 << 1)  /* Bit 1:  Interrupt pending */
#define DMA_INTSTA_ACTIVEERRINTT      (1 << 2)  /* Bit 2:  error interruptpending */

/* SRAM address of the channel configuration table (Bits 9-31) */

/* Channel enable read and Set for all DMA channels */

#define DMA_ENABLESET0(n)             (1 << (n)) /* Bit n: Enable/disable DMA channel n */

/* Channel enable clear for all DMA channels */

#define DMA_ENABLECLR0(n)             (1 << (n)) /* Bit n: Disable DMA channel n */

/* Channel active status for all DMA channels */

#define DMA_ACTIVE0(n)                (1 << (n)) /* Bit n: Channel n active */

/* Channel busy status for all DMA channels */

#define DMA_BUSY0(n)                  (1 << (n)) /* Bit n: Channel n busy */

/* Error interrupt status for all DMA channels */

#define DMA_ERRINT0                   (1 << (n)) /* Bit n: Error interrupt active*/

/* Interrupt enable read and Set for all DMA channels */

#define DMA_INTENSET0                 (1 << (n)) /* Bit n: Enable channel n interrupt */

/* Interrupt enable clear for all DMA channels */

#define DMA_INTENCLR0                 (1 << (n)) /* Bit n: Disable channel n interrupt */

/* Interrupt A status for all DMA channels */

#define DMA_INTA0                     (1 << (n)) /* Bit n: DMA channel n interrupt A active */

/* Interrupt B status for all DMA channels */

#define DMA_INTB0                     (1 << (n)) /* Bit n: DMA channel n interrupt B active */

/* Set ValidPending control bits for all DMA channels */

#define DMA_SETVALID0                 (1 << (n)) /* Bit n: SETVALID control for DMA channel n */

/* Set trigger control bits for all DMA channels */

#define DMA_SETTRIG0                  (1 << (n)) /* Bit n: Set Trigger control bit for DMA channel n */

/* Channel abort control for all DMA channels */

#define DMA_ABORT0                    (1 << (n)) /* Bit n: Abort control for DMA channel n */

/* Configuration register for DMA channel n=0..28 */

#define DMA_CFG_PERIPHREQEN           (1 << 0)  /* Bit 0:  Peripheral request Enable */
#define DMA_CFG_HWTRIGEN              (1 << 1)  /* Bit 1:  Hardware Triggering Enable for this channel */
#define DMA_CFG_TRIGPOL               (1 << 4)  /* Bit 4:  Trigger Polarity */
#define DMA_CFG_TRIGTYPE              (1 << 5)  /* Bit 5:  Trigger Type */
#  define DMA_CFG_TRIGTYPE_EDGE       (0)
#  define DMA_CFG_TRIGTYPE_LEVEL      DMA_CFG_TRIGTYPE
#define DMA_CFG_TRIGBURST             (1 << 6)  /* Bit 6:  Trigger Burst */
#define DMA_CFG_BURSTPOWER_SHIFT      (8)       /* Bits 8-11: Burst Power */
#define DMA_CFG_BURSTPOWER_MASK       (15 << DMA_CFG_BURSTPOWER_SHIFT)
#  define DMA_CFG_BURSTPOWER_SIZE1    (0  << DMA_CFG_BURSTPOWER_SHIFT) /* Burst size = 1 */
#  define DMA_CFG_BURSTPOWER_SIZE2    (1  << DMA_CFG_BURSTPOWER_SHIFT) /* Burst size = 2) */ 
#  define DMA_CFG_BURSTPOWER_SIZE4    (2  << DMA_CFG_BURSTPOWER_SHIFT) /* Burst size = 4) */
#  define DMA_CFG_BURSTPOWER_SIZE8    (3  << DMA_CFG_BURSTPOWER_SHIFT) /* Burst size = 8) */
#  define DMA_CFG_BURSTPOWER_SIZE16   (4  << DMA_CFG_BURSTPOWER_SHIFT) /* Burst size = 16 */
#  define DMA_CFG_BURSTPOWER_SIZE32   (5  << DMA_CFG_BURSTPOWER_SHIFT) /* Burst size = 32 */
#  define DMA_CFG_BURSTPOWER_SIZE64   (6  << DMA_CFG_BURSTPOWER_SHIFT) /* Burst size = 64 */
#  define DMA_CFG_BURSTPOWER_SIZE128  (7  << DMA_CFG_BURSTPOWER_SHIFT) /* Burst size = 128 */
#  define DMA_CFG_BURSTPOWER_SIZE256  (8  << DMA_CFG_BURSTPOWER_SHIFT) /* Burst size = 256 */
#  define DMA_CFG_BURSTPOWER_SIZE512  (9  << DMA_CFG_BURSTPOWER_SHIFT) /* Burst size = 512 */
#  define DMA_CFG_BURSTPOWER_SIZE1024 (10 << DMA_CFG_BURSTPOWER_SHIFT) /* Burst size = 1024 */
#define DMA_CFG_SRCBURSTWRAP          (1 << 14) /* Bit 14: Source Burst Wrap */
#define DMA_CFG_DSTBURSTWRAP          (1 << 15) /* Bit 15: Destination Burst Wrap */
#define DMA_CFG_CHPRIORITY_SHIFT      (16)      /* Bits 16-18: Priority of this channel */
#define DMA_CFG_CHPRIORITY_MASK       (7 << DMA_CFG_CHPRIORITY_MASK)
#  define DMA_CFG_CHPRIORITY(n)       ((uint32_t)(n) << DMA_CFG_CHPRIORITY_MASK)
#  define DMA_CFG_CHPRIORITY_MAX      (0 << DMA_CFG_CHPRIORITY_MASK) /* highest priority */
#  define DMA_CFG_CHPRIORITY_MIn      (7 << DMA_CFG_CHPRIORITY_MASK) /* lowest priority */

/* Control and status register for DMA channel n=0..28 */

#define DMA_CTLSTAT_VALIDPENDING      (1 << 0)  /* Bit 0:  Valid pending flag for this channel */
#define DMA_CTLSTAT_TRIG              (1 << 2)  /* Bit 2:  Trigger flag */

/* Transfer configuration register for DMA channel n=0..28 */

#define DMA_XFERCFG_CFGVALID          (1 << 0)  /* Bit 0:  Configuration Valid flag */
#define DMA_XFERCFG_RELOAD            (1 << 1)  /* Bit 1:  Channel control structure will be reloaded */
#define DMA_XFERCFG_SWTRIG            (1 << 2   /* Bit 2:  Software Trigger */
#define DMA_XFERCFG_CLRTRIG           (1 << 3)  /* Bit 3:  Clear Trigger. 0 */
#define DMA_XFERCFG_SETINTA           (1 << 4)  /* Bit 4:  Set Interrupt flag A for this channel */
#define DMA_XFERCFG_SETINTB           (1 << 5)  /* Bit 5:  Set Interrupt flag B for this channel */
#define DMA_XFERCFG_WIDTH_SHIFT       (8)       /* Bits 8-9: Transfer width used for this DMA channel */
#define DMA_XFERCFG_WIDTH_MASK        (3 << DMA_XFERCFG_WIDTH_SHIFT)
#  define DMA_XFERCFG_WIDTH _MASK     (0 << DMA_XFERCFG_WIDTH_SHIFT) /* 8-bit */
#  define DMA_XFERCFG_WIDTH _MASK     (1 << DMA_XFERCFG_WIDTH_SHIFT) /* 16-bit */
#  define DMA_XFERCFG_WIDTH _MASK     (2 << DMA_XFERCFG_WIDTH_SHIFT) /* 32-bit */
#define DMA_XFERCFG_SRCINC_SHIFT      (12)      /*  Bits 12-13: Source address increment control */
#define DMA_XFERCFG_SRCINC_MASK       (3 << DMA_XFERCFG_SRCINC_SHIFT)
#  define DMA_XFERCFG_SRCINC_NONE     (0 << DMA_XFERCFG_SRCINC_SHIFT) /* None */
#  define DMA_XFERCFG_SRCINC_WIDTH    (1 << DMA_XFERCFG_SRCINC_SHIFT) /* 1 x width */
#  define DMA_XFERCFG_SRCINC_2xWIDTH  (2 << DMA_XFERCFG_SRCINC_SHIFT) /* 2 x width */
#  define DMA_XFERCFG_SRCINC_4xWIDTH  (3 << DMA_XFERCFG_SRCINC_SHIFT) /* 4 x width */
#define DMA_XFERCFG_DSTINC_SHIFT      (14)      /*  Bits 14:15: Destination address increment control */
#define DMA_XFERCFG_DSTINC_MASK       (3 << DMA_XFERCFG_DSTINC_SHIFT)
#  define DMA_XFERCFG_SRCINC_NONE     (0 << DMA_XFERCFG_DSTINC_SHIFT) /* None */
#  define DMA_XFERCFG_DSTINC_WIDTH    (1 << DMA_XFERCFG_DSTINC_SHIFT) /* 1 x width */
#  define DMA_XFERCFG_DSTINC_2xWIDTH  (2 << DMA_XFERCFG_DSTINC_SHIFT) /* 2 x width */
#  define DMA_XFERCFG_DSTINC_4xWIDTH  (3 << DMA_XFERCFG_DSTINC_SHIFT) /* 4 x width */
#define DMA_XFERCFGXFERCOUNT_SHIFT    (16)    /*  Bits 16:25: Total number of transfers to be performed -1 */
#define DMA_XFERCFGXFERCOUNT_MASK     (0x3ff << DMA_XFERCFGXFERCOUNT_SHIFT)
#  define DMA_XFERCFGXFERCOUNT(n)     ((uint32_t)((n)-1) << DMA_XFERCFGXFERCOUNT_SHIFT)

#endif /* __ARCH_ARM_SRC_LPC54XX_CHIP_LPC54_DMA_H */
