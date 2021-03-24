/****************************************************************************
 * arch/arm/src/max326xx/hardware/max32660_spimss.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_SPIMSS_H
#define __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_SPIMSS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/max326_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define MAX326_SPIMSS_DATA_OFFSET     0x0000  /* SPIMSS Data Register */
#define MAX326_SPIMSS_CTRL_OFFSET     0x0004  /* SPIMSS Control Register */
#define MAX326_SPIMSS_INTFL_OFFSET    0x0008  /* SPIMSS Interrupt Flag Register */
#define MAX326_SPIMSS_MODE_OFFSET     0x000c  /* SPIMSS Mode Register */
#define MAX326_SPIMSS_BRG_OFFSET      0x0014  /* SPIMSS Bit Rate Register */
#define MAX326_SPIMSS_DMA_OFFSET      0x0018  /* SPIMSS DMA Register */
#define MAX326_SPIMSS_I2SCTRL_OFFSET  0x001c  /* SPIMSS I2S Control Register */

/* Register Addresses *******************************************************/

#define MAX326_SPIMSS_DATA            (MAX326_SPIMSS_BASE + MAX326_SPIMSS_DATA_OFFSET)
#define MAX326_SPIMSS_CTRL            (MAX326_SPIMSS_BASE + MAX326_SPIMSS_CTRL_OFFSET)
#define MAX326_SPIMSS_INTFL           (MAX326_SPIMSS_BASE + MAX326_SPIMSS_INTFL_OFFSET)
#define MAX326_SPIMSS_MODE            (MAX326_SPIMSS_BASE + MAX326_SPIMSS_MODE_OFFSET)
#define MAX326_SPIMSS_BRG             (MAX326_SPIMSS_BASE + MAX326_SPIMSS_BRG_OFFSET)
#define MAX326_SPIMSS_DMA             (MAX326_SPIMSS_BASE + MAX326_SPIMSS_DMA_OFFSET)
#define MAX326_SPIMSS_I2SCTRL         (MAX326_SPIMSS_BASE + MAX326_SPIMSS_I2SCTRL_OFFSET)

/* Register Bit-field Definitions *******************************************/

/* SPIMSS Data Register */

#define SPIMSS_DATA_MASK              (0xffff)  /* Bits 0-15: SPIMSS Data */

/* SPIMSS Control Register */

#define SPIMSS_CTRL_ENABLE            (1 << 0)  /* Bit 0:  SPI1/I2S Enable */
#define SPIMSS_CTRL_MMEN              (1 << 1)  /* Bit 1:  SPI Master Mode Enable */
#define SPIMSS_CTRL_WOR               (1 << 2)  /* Bit 2:  Wired OR (Open Drain) Enable */
#define SPIMSS_CTRL_CLKPOL            (1 << 3)  /* Bit 3:  Clock Polarity */
#define SPIMSS_CTRL_PHASE             (1 << 4)  /* Bit 4:  Phase Select */
#define SPIMSS_CTRL_BIRQ              (1 << 5)  /* Bit 5:  Bit Rate Generator Timer
                                                 *         Interrupt Request */
#define SPIMSS_CTRL_STR               (1 << 6)  /* Bit 6:  Start SPI Interrupt */
#define SPIMSS_CTRL_IRQE              (1 << 7)  /* Bit 7:  Interrupt Request Enable */

/* SPIMSS Interrupt Flag Register */

#define SPIMSS_INTFL_SLAS             (1 << 0)  /* Bit 0:  Slave Select */
#define SPIMSS_INTFL_TXST             (1 << 1)  /* Bit 1:  Transmit Status */
#define SPIMSS_INTFL_TUND             (1 << 2)  /* Bit 2:  Transmit Underrun Flag */
#define SPIMSS_INTFL_ROVR             (1 << 3)  /* Bit 3:  Receive Overrun Flag */
#define SPIMSS_INTFL_ABT              (1 << 4)  /* Bit 4:  Slave Mode Transaction Abort
                                                 *         Flag */
#define SPIMSS_INTFL_COL              (1 << 5)  /* Bit 5:  Collision Flag */
#define SPIMSS_INTFL_TOVR             (1 << 6)  /* Bit 6:  Transmit Overrun Flag */
#define SPIMSS_INTFL_IRQ              (1 << 7)  /* Bit 7:  SPIMSS Interrupt Request Flag */

/* SPIMSS Mode Register */

#define SPIMSS_MODE_SSV               (1 << 0)  /* Bit 0:  Slave Select Value */
#define SPIMSS_MODE_SSIO              (1 << 1)  /* Bit 1:  Slave Select Input/Output Mode */
#define SPIMSS_MODE_NUMBITS_SHIFT     (2)       /* Bits 2-5:  Number of Data Bits per
                                                 *            Character to Transfer */
#define SPIMSS_MODE_NUMBITS_MASK      (15 << SPIMSS_MODE_NUMBITS_SHIFT)
#  define SPIMSS_MODE_NUMBITS(n)      ((uint32_t)((n) & 15) << SPIMSS_MODE_NUMBITS_SHIFT)
#define SPIMSS_MODE_TXLJ              (1 << 7)  /* Bit 7:  Transmit Data Alignment */

/* SPIMSS Bit Rate Register */

#define SPIMSS_BRG_MASK               (0xffff)  /* Bits 0-15: Bit Rate Reload Value */

/* SPIMSS DMA Register */

#define SPIMSS_DMA_TXFIFOLVL_SHIFT    (0)       /* Bits 0-2: Transmit FIFO Level */
#define SPIMSS_DMA_TXFIFOLVL_MASK     (7 << SPIMSS_DMA_TXFIFOLVL_SHIFT)
#  define SPIMSS_DMA_TXFIFOLVL(n)     ((uint32_t)((n) - 1) << SPIMSS_DMA_TXFIFOLVL_SHIFT)
#define SPIMSS_DMA_TXFIFOCLR          (1 << 4)  /* Bit 4:  Transmit FIFO Clear */
#define SPIMSS_DMA_TXFIFOCNT_SHIFT    (8)       /* Bits 8-11: Transmit FIFO Count */
#define SPIMSS_DMA_TXFIFOCNT_MASK     (15 << SPIMSS_DMA_TXFIFOCNT_SHIFT)
#  define SPIMSS_DMA_TXFIFOCNT(n)     ((uint32_t)(n) << SPIMSS_DMA_TXFIFOCNT_SHIFT)
#define SPIMSS_DMA_TXDMAEN            (1 << 15) /* Bit 15: Transmit DMA Enable */
#define SPIMSS_DMA_RXFIFOLVL_SHIFT    (16)      /* Bits 16-18: Receive FIFO Level */
#define SPIMSS_DMA_RXFIFOLVL_MASK     (7 << SPIMSS_DMA_RXFIFOLVL_SHIFT)
#  define SPIMSS_DMA_RXFIFOLVL(n)     ((uint32_t)((n) - 1) << SPIMSS_DMA_RXFIFOLVL_SHIFT)
#define SPIMSS_DMA_RXFIFOCLR          (1 << 20) /* Bit 20: Receive FIFO Clear */
#define SPIMSS_DMA_RXFIFOCNT_SHIFT    (24)      /* Bits 24-27: Receive FIFO Count */
#define SPIMSS_DMA_RXFIFOCNT_MASK     (15 << SPIMSS_DMA_RXFIFOCNT_SHIFT)
#  define SPIMSS_DMA_RXFIFOCNT(n)     ((uint32_t)(n) << SPIMSS_DMA_RXFIFOCNT_SHIFT)
#define SPIMSS_DMA_RXDMAEN            (1 << 31) /* Bit 31: Receive DMA Enable */

/* SPIMSS I2S Control Register */

#define SPIMSS_I2SCTRL_I2SEN          (1 << 0)  /* Bit 0:  I2S Mode Enable */
#define SPIMSS_I2SCTRL_I2SMUTE        (1 << 1)  /* Bit 1:  I2S Mute Transmit */
#define SPIMSS_I2SCTRL_I2SPAUSE       (1 << 2)  /* Bit 2:  I2S Pause Transmit/Receive */
#define SPIMSS_I2SCTRL_I2SMONO        (1 << 3)  /* Bit 3:  I2S Monophonic Audio Mode */
#define SPIMSS_I2SCTRL_I2SLJ          (1 << 4)  /* Bit 4:  I2S Left Justify */

#endif /* __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_SPIMSS_H */
