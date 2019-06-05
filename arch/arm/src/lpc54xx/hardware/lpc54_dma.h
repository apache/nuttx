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

#ifndef __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_DMA_H
#define __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_DMA_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>
#include "hardware/lpc54_memorymap.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

#define LPC54_DMA_NCHANNELS         30      /* Channels 0..29 */
#define LPC54_DMA_MAXXFRS           1024    /* Maximum number of transfers per DMA */

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

/* DMA requests *****************************************************************************/
/* DMA requests are directly connected to the peripherals. Each channel supports one DMA
 * request line and one trigger input. Some DMA requests allow a selection of requests
 * sources. DMA triggers are selected from many possible input sources.
 */

/* Peripheral request inputs to DMA channel.  For DMA channel 'n', the corresponding DMA
 * trigger input is provided by the setting of the INPUT MUX register DMA_ITRIG_INMUXn
 */

#define FLEXCOMM0_RX_DMACHAN        (0)       /* Flexcomm Interface 0 RX */
#define FLEXCOMM0_I2CSLAVE_DMACHAN  (0)       /* Flexcomm Interface 0 I2C Slave */
#define FLEXCOMM0_TX_DMACHAN        (1)       /* Flexcomm Interface 0 TX */
#define FLEXCOMM0_I2CMASTER_DMACHAN (1)       /* Flexcomm Interface 0 I2C Master */
#define FLEXCOMM1_RX_DMACHAN        (2)       /* Flexcomm Interface 1 RX */
#define FLEXCOMM1_I2CSLAVE_DMACHAN  (2)       /* Flexcomm Interface 1 I2C Slave */
#define FLEXCOMM1_TX_DMACHAN        (3)       /* Flexcomm Interface 1 TX */
#define FLEXCOMM1_I2CMASTER_DMACHAN (3)       /* Flexcomm Interface 1 I2C Master */
#define FLEXCOMM2_RX_DMACHAN        (4)       /* Flexcomm Interface 2 RX */
#define FLEXCOMM2_I2CSLAVE_DMACHAN  (4)       /* Flexcomm Interface 2 I2C Slave */
#define FLEXCOMM2_TX_DMACHAN        (5)       /* Flexcomm Interface 2 TX */
#define FLEXCOMM2_I2CMASTER_DMACHAN (5)       /* Flexcomm Interface 2 I2C Master */
#define FLEXCOMM3_RX_DMACHAN        (6)       /* Flexcomm Interface 3 RX */
#define FLEXCOMM3_I2CSLAVE_DMACHAN  (6)       /* Flexcomm Interface 3 I2C Slave */
#define FLEXCOMM3_TX_DMACHAN        (7)       /* Flexcomm Interface 3 TX */
#define FLEXCOMM3_I2CMASTER_DMACHAN (7)       /* Flexcomm Interface 3 I2C Master */
#define FLEXCOMM4_RX_DMACHAN        (8)       /* Flexcomm Interface 4 RX */
#define FLEXCOMM4_I2CSLAVE_DMACHAN  (8)       /* Flexcomm Interface 4 I2C Slave */
#define FLEXCOMM4_TX_DMACHAN        (9)       /* Flexcomm Interface 4 TX */
#define FLEXCOMM4_I2CMASTER_DMACHAN (9)       /* Flexcomm Interface 4 I2C Master */
#define FLEXCOMM5_RX_DMACHAN        (10)      /* Flexcomm Interface 5 RX */
#define FLEXCOMM5_I2CSLAVE_DMACHAN  (10)      /* Flexcomm Interface 5 I2C Slave */
#define FLEXCOMM5_TX_DMACHAN        (11)      /* Flexcomm Interface 5 TX */
#define FLEXCOMM5_I2CMASTER_DMACHAN (11)      /* Flexcomm Interface 5 I2C Master */
#define FLEXCOMM6_RX_DMACHAN        (12)      /* Flexcomm Interface 6 RX */
#define FLEXCOMM6_I2CSLAVE_DMACHAN  (12)      /* Flexcomm Interface 6 I2C Slave */
#define FLEXCOMM6_TX_DMACHAN        (13)      /* Flexcomm Interface 6 TX */
#define FLEXCOMM6_I2CMASTER_DMACHAN (13)      /* Flexcomm Interface 6 I2C Master */
#define FLEXCOMM7_RX_DMACHAN        (14)      /* Flexcomm Interface 7 RX */
#define FLEXCOMM7_I2CSLAVE_DMACHAN  (14)      /* Flexcomm Interface 7 I2C Slave */
#define FLEXCOMM7_TX_DMACHAN        (15)      /* Flexcomm Interface 7 TX */
#define FLEXCOMM7_I2CMASTER_DMACHAN (15)      /* Flexcomm Interface 7 I2C Master */
#define DMIC0_DMACHAN               (16)      /* DMIC0 */
#define DMIC1_DMACHAN               (17)      /* DMIC1 */
#define SPIFI_DMACHAN               (18)      /* SPIFI */
#define SHA_DMACHAN                 (19)      /* SHA  */
#define FLEXCOMM8_RX_DMACHAN        (20)      /* Flexcomm Interface 8 RX */
#define FLEXCOMM8_I2CSLAVE_DMACHAN  (20)      /* Flexcomm Interface 8 I2C Slave */
#define FLEXCOMM8_TX_DMACHAN        (21)      /* Flexcomm Interface 8 TX */
#define FLEXCOMM8_I2CMASTER_DMACHAN (21)      /* Flexcomm Interface 8 I2C Slave (?) */
#define FLEXCOMM9_RX_DMACHAN        (22)      /* Flexcomm Interface 9 RX */
#define FLEXCOMM9_I2CSLAVE_DMACHAN  (22)      /* Flexcomm Interface 9 I2C Slave */
#define FLEXCOMM9_TX_DMACHAN        (23)      /* Flexcomm Interface 9 TX */
#define FLEXCOMM9_I2CMASTER_DMACHAN (23)      /* Flexcomm Interface 9 I2C Slave (?) */
#define SMARTCARD0_RX_DMACHAN       (24)      /* SMARTCARD0_RX */
#define SMARTCARD0_TX_DMACHAN       (25)      /* SMARTCARD0_TX */
#define SMARTCARD1_RX_DMACHAN       (26)      /* SMARTCARD1_RX */
#define SMARTCARD1_TX_DMACHAN       (27)      /* SMARTCARD1_TX */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/* DMA channel descriptor */

struct lpc54_dmachan_desc_s
{
  uint32_t reserved;
  uint32_t srcend;    /* Source data end address */
  uint32_t dstend;    /* Destination end address */
  uint32_t link;      /* Link to next descriptor */
};

#endif /* __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_DMA_H */
