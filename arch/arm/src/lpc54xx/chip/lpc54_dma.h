/********************************************************************************************
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC54XX_CHIP_LPC54_DMA_H
#define __ARCH_ARM_SRC_LPC54XX_CHIP_LPC54_DMA_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>
#include "chip/lpc54_memorymap.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

#define LPC54_DMA_NCHANNELS         30      /* Channels 0..29 */

/* Register offsets *************************************************************************/

/* Global control and status registers */

#define LPC54_DMA_CTRL_OFFSET       0x0000  /* DMA control */
#define LPC54_DMA_INTSTAT_OFFSET    0x0004  /* Interrupt status */
#define LPC54_DMA_SRAMBASE_OFFSET   0x0008  /* SRAM address of the channel configuration table */

/* Shared registers */

#define LPC54_DMA_ENABLESET0_OFFSET 0x0020  /* Channel enable read and Set for all DMA channels */
#define LPC54_DMA_ENABLECLR0_OFFSET 0x0028  /* Channel enable clear for all DMA channels */
#define LPC54_DMA_ACTIVE0_OFFSET    0x0030  /* Channel active status for all DMA channels */
#define LPC54_DMA_BUSY0_OFFSET      0x0038  /* Channel busy status for all DMA channels */
#define LPC54_DMA_ERRINT0_OFFSET    0x0040  /* Error interrupt status for all DMA channels */
#define LPC54_DMA_INTENSET0_OFFSET  0x0048  /* Interrupt enable read and Set for all DMA channels */
#define LPC54_DMA_INTENCLR0_OFFSET  0x0050  /* Interrupt enable clear for all DMA channels */
#define LPC54_DMA_INTA0_OFFSET      0x0058  /* Interrupt A status for all DMA channels */
#define LPC54_DMA_INTB0_OFFSET      0x0060  /* Interrupt B status for all DMA channels */
#define LPC54_DMA_SETVALID0_OFFSET  0x0068  /* Set ValidPending control bits for all DMA channels */
#define LPC54_DMA_SETTRIG0_OFFSET   0x0070  /* Set trigger control bits for all DMA channels */
#define LPC54_DMA_ABORT0_OFFSET     0x0078  /* Channel abort control for all DMA channels */

/* Channel registers */

#define LPC54_DMA_CHAN_OFFSET(n)    (0x0400 + ((n) << 4))
#define LPC54_DMA_CFG_OFFSET        0x0000  /* Configuration register for DMA channel n */
#define LPC54_DMA_CTLSTAT_OFFSET    0x0004  /* Control and status register for DMA channel n */
#define LPC54_DMA_XFERCFG_OFFSET    0x0008  /* Transfer configuration register for DMA channel n */

/* Register addresses ***********************************************************************/

/* Global control and status registers */

#define LPC54_DMA_CTRL              (LPC54_DMA_BASE + LPC54_DMA_CTRL_OFFSET)
#define LPC54_DMA_INTSTAT           (LPC54_DMA_BASE + LPC54_DMA_INTSTAT_OFFSET)
#define LPC54_DMA_SRAMBASE          (LPC54_DMA_BASE + LPC54_DMA_SRAMBASE_OFFSET)

/* Shared registers */

#define LPC54_DMA_ENABLESET0        (LPC54_DMA_BASE + LPC54_DMA_ENABLESET0_OFFSET)
#define LPC54_DMA_ENABLECLR0        (LPC54_DMA_BASE + LPC54_DMA_ENABLECLR0_OFFSET)
#define LPC54_DMA_ACTIVE0           (LPC54_DMA_BASE + LPC54_DMA_ACTIVE0_OFFSET)
#define LPC54_DMA_BUSY0             (LPC54_DMA_BASE + LPC54_DMA_BUSY0_OFFSET)
#define LPC54_DMA_ERRINT0           (LPC54_DMA_BASE + LPC54_DMA_ERRINT0_OFFSET)
#define LPC54_DMA_INTENSET0         (LPC54_DMA_BASE + LPC54_DMA_INTENSET0_OFFSET)
#define LPC54_DMA_INTENCLR0         (LPC54_DMA_BASE + LPC54_DMA_INTENCLR0_OFFSET)
#define LPC54_DMA_INTA0             (LPC54_DMA_BASE + LPC54_DMA_INTA0_OFFSET)
#define LPC54_DMA_INTB0             (LPC54_DMA_BASE + LPC54_DMA_INTB0_OFFSET)
#define LPC54_DMA_SETVALID0         (LPC54_DMA_BASE + LPC54_DMA_SETVALID0_OFFSET)
#define LPC54_DMA_SETTRIG0          (LPC54_DMA_BASE + LPC54_DMA_SETTRIG0_OFFSET)
#define LPC54_DMA_ABORT0            (LPC54_DMA_BASE + LPC54_DMA_ABORT0_OFFSET)

/* Channel registers */

#define LPC54_DMA_CHAN_BASE(n)      (LPC54_DMA_BASE + LPC54_DMA_CHAN_OFFSET(n))
#define LPC54_DMA_CFG(n)            (LPC54_DMA_CHAN_BASE(n) + LPC54_DMA_CFG_OFFSET)
#define LPC54_DMA_CTLSTAT(n)        (LPC54_DMA_CHAN_BASE(n) + LPC54_DMA_CTLSTAT_OFFSET)
#define LPC54_DMA_XFERCFG(n)        (LPC54_DMA_CHAN_BASE(n) + LPC54_DMA_XFERCFG_OFFSET)

/* Register bit definitions *****************************************************************/

/* DMA control */

#define DMA_CTRL_ENABLE             (1 << 0)  /* Bit 0:  DMA controller master enable */

/* Interrupt status */

#define DMA_INTSTAT_ACTIVEINT       (1 << 1)  /* Bit 1:  Summarizes pending enabled interrupts */
#define DMA_INTSTAT_ACTIVEERRINT    (1 << 2)  /* Bit 2:  Summarizes pending error interrupts */

/* SRAM address of the channel configuration table */

#define DMA_SRAMBASE_MASK           0xfffffe00

/* The remaining shared registers are all 32 bit encoded fieldss with bit n corresponding to
 * Channel n.
 */

#define DMA_CHANNEL(n)              (1 << (n))
#define DMA_ALL_CHANNELS            0x3fffffff

/* Channel registers */

/* Configuration register for DMA channel n */

#define DMA_CFG_PERIPHREQEN         (1 << 0)  /* Bit 0:  Peripheral request Enable */
#define DMA_CFG_HWTRIGEN            (1 << 1)  /* Bit 1:  Hardware Triggering Enable */
#define DMA_CFG_TRIGPOL             (1 << 4)  /* Bit 4:  Trigger Polarity */
#  define DMA_CFG_ACTIVE_LOW        (0)
#  define DMA_CFG_FALLING_EDGE      (0)
#  define DMA_CFG_ACTIVE_HIGH       DMA_CFG_TRIGPOL
#  define DMA_CFG_RISING_EDGE       DMA_CFG_TRIGPOL
#define DMA_CFG_TRIGTYPE            (1 << 5)  /* Bit 5:  Trigger Type */
#  define DMA_CFG_EDGE_TRIGGER      (0)
#  define DMA_CFG_LEVEL_TRIGGER     DMA_CFG_TRIGTYPE
#define DMA_CFG_TRIGBURST           (1 << 6)  /* Bit 6:  Trigger Burst */
#define DMA_CFG_BURSTPOWER_SHIFT    (8)       /* Bits 8-11: Burst Power */
#define DMA_CFG_BURSTPOWER_MASK     (15 << DMA_CFG_BURSTPOWER_SHIFT)
# define DMA_CFG_BURSTPOWER_1       (0 << DMA_CFG_BURSTPOWER_SHIFT) /* Burst size = 1 (2^0) */
# define DMA_CFG_BURSTPOWER_2       (1 << DMA_CFG_BURSTPOWER_SHIFT) /* Burst size = 2 (2^1) */
# define DMA_CFG_BURSTPOWER_3       (2 << DMA_CFG_BURSTPOWER_SHIFT) /* Burst size = 4 (2^2) */
# define DMA_CFG_BURSTPOWER_8       (3 << DMA_CFG_BURSTPOWER_SHIFT) /* Burst size = 8 (2^2) */
# define DMA_CFG_BURSTPOWER_16      (4 << DMA_CFG_BURSTPOWER_SHIFT) /* Burst size = 16 (2^2) */
# define DMA_CFG_BURSTPOWER_32      (5 << DMA_CFG_BURSTPOWER_SHIFT) /* Burst size = 32 (2^2) */
# define DMA_CFG_BURSTPOWER_64      (6 << DMA_CFG_BURSTPOWER_SHIFT) /* Burst size = 64 (2^2) */
# define DMA_CFG_BURSTPOWER_128     (7 << DMA_CFG_BURSTPOWER_SHIFT) /* Burst size = 128 (2^2) */
# define DMA_CFG_BURSTPOWER_256     (8 << DMA_CFG_BURSTPOWER_SHIFT) /* Burst size = 256 (2^2) */
# define DMA_CFG_BURSTPOWER_512     (9 << DMA_CFG_BURSTPOWER_SHIFT) /* Burst size = 256 (2^2) */
# define DMA_CFG_BURSTPOWER_1024    (10 << DMA_CFG_BURSTPOWER_SHIFT) /* Burst size = 1024 (2^10) */
#define DMA_CFG_SRCBURSTWRAP        (1 << 14) /* Bit 14: Source Burst Wrap */
#define DMA_CFG_DSTBURSTWRAP        (1 << 15) /* Bit 15: Destination Burst Wrap */
#define DMA_CFG_CHPRIORITY_SHIFT    (16)      /* Bits 16-18: Priority of this channel */
#define DMA_CFG_CHPRIORITY_MASK     (7 << DMA_CFG_CHPRIORITY_SHIFT)
#  define DMA_CFG_CHPRIORITY(n)     ((uint32_t)(n) << DMA_CFG_CHPRIORITY_SHIFT)
#  define DMA_CFG_CHPRIORITY_HIGH   (0 << DMA_CFG_CHPRIORITY_SHIFT) /* Highest priority */
#  define DMA_CFG_CHPRIORITY_LOW    (7 << DMA_CFG_CHPRIORITY_SHIFT) /* Lowest priority */

/* Control and status register for DMA channel n */

#define DMA_CTLSTAT_VALIDPENDING    (1 << 0)  /* Bit 0:  Valid pending flag */
#define DMA_CTLSTAT_TRIG            (1 << 2)  /* Bit 2:  Trigger flag */

/* Transfer configuration register for DMA channel n */

#define DMA_XFERCFG_CFGVALID        (1 << 0)  /* Bit 0:  Configuration Valid flag */
#define DMA_XFERCFG_RELOAD          (1 << 1)  /* Bit 1:  Reload channel’s control structure */
#define DMA_XFERCFG_SWTRIG          (1 << 2)  /* Bit 2:  Software Trigger */
#define DMA_XFERCFG_CLRTRIG         (1 << 3)  /* Bit 3:  Clear Trigger */
#define DMA_XFERCFG_SETINTA         (1 << 4)  /* Bit 4:  Set Interrupt flag A */
#define DMA_XFERCFG_SETINTB         (1 << 5)  /* Bit 5:  Set Interrupt flag B */
#define DMA_XFERCFG_WIDTH_SHIFT     (8)       /* Bits 8-9: Transfer width */
#define DMA_XFERCFG_WIDTH_MASK      (3 << DMA_XFERCFG_WIDTH_SHIFT)
#  define DMA_XFERCFG_WIDTH_8BIT    (0 << DMA_XFERCFG_WIDTH_SHIFT) /* 8-bit transfers */
#  define DMA_XFERCFG_WIDTH_16BIT   (1 << DMA_XFERCFG_WIDTH_SHIFT) /* 16-bit transfers */
#  define DMA_XFERCFG_WIDTH_32BIT   (2 << DMA_XFERCFG_WIDTH_SHIFT) /* 32-bit transfers */
#define DMA_XFERCFG_SRCINC_SHIFT    (12)      /* Bits 12-13: Source address increment */
#define DMA_XFERCFG_SRCINC_MASK     (3 << DMA_XFERCFG_SRCINC_SHIFT)
#  define DMA_XFERCFG_SRCINC_NONE   (0 << DMA_XFERCFG_SRCINC_SHIFT) /* No increment */
#  define DMA_XFERCFG_SRCINC_1X     (1 << DMA_XFERCFG_SRCINC_SHIFT) /* 1 x width */
#  define DMA_XFERCFG_SRCINC_2X     (2 << DMA_XFERCFG_SRCINC_SHIFT) /* 2 x width */
#  define DMA_XFERCFG_SRCINC_4X     (3 << DMA_XFERCFG_SRCINC_SHIFT) /* 4 x width */
#define DMA_XFERCFG_DSTINC_SHIFT    (14)      /* Bits 14-15: Destination address increment */
#define DMA_XFERCFG_DSTINC_MASK     (3 << DMA_XFERCFG_DSTINC_SHIFT)
#  define DMA_XFERCFG_DSTINC_NONE   (0 << DMA_XFERCFG_DSTINC_SHIFT) /* No increment */
#  define DMA_XFERCFG_DSTINC_1X     (1 << DMA_XFERCFG_DSTINC_SHIFT) /* 1 x width */
#  define DMA_XFERCFG_DSTINC_2X     (2 << DMA_XFERCFG_DSTINC_SHIFT) /* 2 x width */
#  define DMA_XFERCFG_DSTINC_4X     (3 << DMA_XFERCFG_DSTINC_SHIFT) /* 4 x width */
#define DMA_XFERCFG_XFERCOUNT_SHIFT (16)      /* Bits 16-25: Total number of transfers to be performed */
#define DMA_XFERCFG_XFERCOUNT_MASK  (0x3ff << DMA_XFERCFG_XFERCOUNT_SHIFT)
#  define DMA_XFERCFG_XFERCOUNT(n)  ((uint32_t)((n)-1) << DMA_XFERCFG_XFERCOUNT_SHIFT)

#endif /* __ARCH_ARM_SRC_LPC54XX_CHIP_LPC54_DMA_H */
