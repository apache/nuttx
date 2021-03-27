/****************************************************************************
 * arch/arm/src/max326xx/hardware/max326_dma.h
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

#ifndef __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX326_DMA_H
#define __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX326_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/max326_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DMA Channels */

#define MAX326_DMA_CHAN0            0  /* DMA Channel 0 */
#define MAX326_DMA_CHAN1            1  /* DMA Channel 1 */
#define MAX326_DMA_CHAN2            2  /* DMA Channel 2 */
#define MAX326_DMA_CHAN3            3  /* DMA Channel 3 */
#define MAX326_DMA_NCHAN            4  /* Four DMA Channels */

/* DMA priorities */

#define MAX326_DMAPRIO_LO           3  /* Lowest priority */
#define MAX326_DMAPRIO_MEDLO        2
#define MAX326_DMAPRIO_MEDHI        1
#define MAX326_DMAPRIO_HI           0  /* Highest priority */

/* DMA Prescaler */

#define MAX326_DMA_PS_DISABLE       0  /* Disable timer */
#define MAX326_DMA_PS_DIV256        1  /* fhclk / 256 */
#define MAX326_DMA_PS_DIV64K        2  /* fhclk / 64K */
#define MAX326_DMA_PS_DIV16M        3  /* fhclk / 16M */

/* DMA Timeouts */

#define MAX326_DMATO_3to4           0  /* 3-4 prescaler clocks */
#define MAX326_DMATO_7to8           1  /* 7-8 prescaler clocks */
#define MAX326_DMATO_15to16         2  /* 15-16 prescaler clocks */
#define MAX326_DMATO_31to82         3  /* 31-32 prescaler clocks */
#define MAX326_DMATO_63to64         4  /* 63-64 prescaler clocks */
#define MAX326_DMATO_127to128       5  /* 127-128 prescaler clocks */
#define MAX326_DMATO_255to256       6  /* 255-256 prescaler clocks */
#define MAX326_DMATO_511to512       7  /* 511-512 prescaler clocks */

/* Register Offsets *********************************************************/

/* DMA Control Registers */

#define MAX326_DMA_INTEN_OFFSET     0x0000 /* DMA Control register */
#define MAX326_DMA_INTFL_OFFSET     0x0004 /* DMA Interrupt Status register */

/* DMA Channel Registers */

#define MAX326_DMACH_OFFSET(n)      (0x0100 + ((unsigned int)(n) << 5))
#define MAX326_DMACH0_OFFSET        0x0100
#define MAX326_DMACH1_OFFSET        0x0120
#define MAX326_DMACH2_OFFSET        0x0140
#define MAX326_DMACH3_OFFSET        0x0160

#define MAX326_DMACH_CFG_OFFSET     0x0000  /* DMA Channel Configuration Register */
#define MAX326_DMACH_STAT_OFFSET    0x0004  /* DMA Channel Status Register */
#define MAX326_DMACH_SRC_OFFSET     0x0008  /* DMA Channel Source Register */
#define MAX326_DMACH_DST_OFFSET     0x000c  /* DMA Channel Destination Register */
#define MAX326_DMACH_CNT_OFFSET     0x0010  /* DMA Channel Count Register */
#define MAX326_DMACH_SRCRLD_OFFSET  0x0014  /* DMA Channel Source Reload Register */
#define MAX326_DMACH_DSTRLD_OFFSET  0x0018  /* DMA Channel Destination Reload Register */
#define MAX326_DMACH_CNTRLD_OFFSET  0x001c  /* DMA Channel Count Reload Register */

/* Register Addresses *******************************************************/

/* DMA Control Registers */

#define MAX326_DMA_INTEN            (MAX326_DMA_BASE + MAX326_DMA_INTEN_OFFSET)
#define MAX326_DMA_INTFL            (MAX326_DMA_BASE + MAX326_DMA_INTFL_OFFSET)

/* DMA Channel Registers */

#define MAX326_DMACH_BASE(n)        (MAX326_DMA_BASE + MAX326_DMACH_OFFSET(n))
#define MAX326_DMACH0_BASE          (MAX326_DMA_BASE + MAX326_DMACH0_OFFSET)
#define MAX326_DMACH1_BASE          (MAX326_DMA_BASE + MAX326_DMACH1_OFFSET)
#define MAX326_DMACH2_BASE          (MAX326_DMA_BASE + MAX326_DMACH2_OFFSET)
#define MAX326_DMACH3_BASE          (MAX326_DMA_BASE + MAX326_DMACH3_OFFSET)

#define MAX326_DMACH_CFG(n)         (MAX326_DMACH_BASE(n) + MAX326_DMACH_CFG_OFFSET)
#define MAX326_DMACH_STAT(n)        (MAX326_DMACH_BASE(n) + MAX326_DMACH_STAT_OFFSET)
#define MAX326_DMACH_SRC(n)         (MAX326_DMACH_BASE(n) + MAX326_DMACH_SRC_OFFSET)
#define MAX326_DMACH_DST(n)         (MAX326_DMACH_BASE(n) + MAX326_DMACH_DST_OFFSET)
#define MAX326_DMACH_CNT(n)         (MAX326_DMACH_BASE(n) + MAX326_DMACH_CNT_OFFSET)
#define MAX326_DMACH_SRCRLD(n)      (MAX326_DMACH_BASE(n) + MAX326_DMACH_SRCRLD_OFFSET)
#define MAX326_DMACH_DSTRLD(n)      (MAX326_DMACH_BASE(n) + MAX326_DMACH_DSTRLD_OFFSET)
#define MAX326_DMACH_CNTRLD(n)      (MAX326_DMACH_BASE(n) + MAX326_DMACH_CNTRLD_OFFSET)

/* Register Bit-field Definitions *******************************************/

/* DMA Control register */

#define DMA_INTEN(n)                (1 << (n)) /* Channel n interrupt enable */

/* DMA Interrupt Status register */

#define DMA_INTFL(n)                (1 << (n)) /* Channel n interrupt pending */

/* DMA Channel Registers */

/* DMA Channel Configuration Register */

#define DMACH_CFG_CHEN              (1 << 0)  /* Bit nn:  Channel Enable */
#define DMACH_CFG_RLDEN             (1 << 1)  /* Bit nn:  Reload Enable */
#define DMACH_CFG_PRI_SHIFT         (2)       /* Bits 2-3: DMA priority */
#define DMACH_CFG_PRI_MASK          (3 << DMACH_CFG_PRI_SHIFT)
#  define DMACH_CFG_PRI(n)          ((uint32_t)(n) << DMACH_CFG_PRI_SHIFT)
#  define DMACH_CFG_PRI_LO          (3 << DMACH_CFG_PRI_SHIFT)
#  define DMACH_CFG_PRI_MEDLO       (2 << DMACH_CFG_PRI_SHIFT)
#  define DMACH_CFG_PRI_MEDHI       (1 << DMACH_CFG_PRI_SHIFT)
#  define DMACH_CFG_PRI_HI          (0 << DMACH_CFG_PRI_SHIFT)
#define DMACH_CFG_REQSEL_SHIFT      (4)       /* Bits 4-9: Request Select
                                               *           See values below */
#define DMACH_CFG_REQSEL_MASK       (0x3f << DMACH_CFG_REQSEL_SHIFT)
#  define DMACH_CFG_REQSEL(n)       ((uint32_t)(n) << DMACH_CFG_REQSEL_SHIFT)
#define DMACH_CFG_REQWAIT           (1 << 10) /* Bit 10: Request Wait Enable */
#define DMACH_CFG_TOSEL_SHIFT       (11)      /* Bits 11-13: Time-Out Select */
#define DMACH_CFG_TOSEL_MASK        (7 << DMACH_CFG_TOSEL_SHIFT)
#  define DMACH_CFG_TOSEL(n)        ((uint32_t)(n) << DMACH_CFG_TOSEL_SHIFT)
#  define DMACH_CFG_TOSEL_3to4      (0 << DMACH_CFG_TOSEL_SHIFT) /* 3-4 */
#  define DMACH_CFG_TOSEL_7to8      (1 << DMACH_CFG_TOSEL_SHIFT) /* 7-8 */
#  define DMACH_CFG_TOSEL_15to16    (2 << DMACH_CFG_TOSEL_SHIFT) /* 15-16 */
#  define DMACH_CFG_TOSEL_31to82    (3 << DMACH_CFG_TOSEL_SHIFT) /* 31-32 */
#  define DMACH_CFG_TOSEL_63to64    (4 << DMACH_CFG_TOSEL_SHIFT) /* 63-64 */
#  define DMACH_CFG_TOSEL_127to128  (5 << DMACH_CFG_TOSEL_SHIFT) /* 127-128 */
#  define DMACH_CFG_TOSEL_255to256  (6 << DMACH_CFG_TOSEL_SHIFT) /* 255-256 */
#  define DMACH_CFG_TOSEL_511to512  (7 << DMACH_CFG_TOSEL_SHIFT) /* 511-512 */

#define DMACH_CFG_PSSEL_SHIFT       (14)      /* Bits 14-15: Pre-Scale Select */
#define DMACH_CFG_PSSEL_MASK        (3 << DMACH_CFG_PSSEL_SHIFT)
#  define DMACH_CFG_PSSEL(n)        ((uint32_t)(n) << DMACH_CFG_PSSEL_SHIFT)
#  define DMACH_CFG_PSSEL_DISABLE   (0 << DMACH_CFG_PSSEL_SHIFT) /* Disable timer */
#  define DMACH_CFG_PSSEL_DIV256    (1 << DMACH_CFG_PSSEL_SHIFT) /* fhclk / 256 */
#  define DMACH_CFG_PSSEL_DIV64K    (2 << DMACH_CFG_PSSEL_SHIFT) /* fhclk / 64K */
#  define DMACH_CFG_PSSEL_DIV16M    (3 << DMACH_CFG_PSSEL_SHIFT) /* fhclk / 16M */

#define DMACH_CFG_SRCWD_SHIFT       (16)      /* Bits 16-17: Source Width */
#define DMACH_CFG_SRCWD_MASK        (3 << DMACH_CFG_SRCWD_SHIFT)
#  define DMACH_CFG_SRCWD_1BYTE     (0 << DMACH_CFG_SRCWD_SHIFT)
#  define DMACH_CFG_SRCWD_2BYTES    (1 << DMACH_CFG_SRCWD_SHIFT)
#  define DMACH_CFG_SRCWD_4BYTES    (2 << DMACH_CFG_SRCWD_SHIFT)
#define DMACH_CFG_SRCINC            (1 << 18) /* Bit 18: Source Increment
                                               *         Enable */
#define DMACH_CFG_DSTWD_SHIFT       (20)      /* Bits 20-21: Destination Width */
#define DMACH_CFG_DSTWD_MASK        (3 << DMACH_CFG_DSTWD_SHIFT)
#  define DMACH_CFG_DSTWD_1BYTE     (0 << DMACH_CFG_DSTWD_SHIFT)
#  define DMACH_CFG_DSTWD_2BYTES    (1 << DMACH_CFG_DSTWD_SHIFT)
#  define DMACH_CFG_DSTWD_4BYTES    (2 << DMACH_CFG_DSTWD_SHIFT)
#define DMACH_CFG_DSTINC            (1 << 22) /* Bit 22: Destination Increment
                                               *         Enable */
#define DMACH_CFG_BRST_SHIFT        (24)      /* Bits 24-28: Burst Size */
#define DMACH_CFG_BRST_MASK         (31 << DMACH_CFG_BRST_SHIFT)
#  define DMACH_CFG_BRST(n)         ((uin32_t)((n) - 1) << DMACH_CFG_BRST_SHIFT) /* n=1..32 */

#define DMACH_CFG_CHDIEN            (1 << 30) /* Bit 30: Channel Disable
                                               *         Interrupt Enable */
#define DMACH_CFG_CTZIEN            (1 << 31) /* Bit 31: CTZ Interrupt Enable */

/* Values for the CFG register Request Select field */

#define DMACH_REQSEL_MEMTOMEM        0x00
#define DMACH_REQSEL_SPI0RX          0x01
#define DMACH_REQSEL_SPI1RX          0x02
#define DMACH_REQSEL_SPI2RX          0x03
#define DMACH_REQSEL_UART0RX         0x04
#define DMACH_REQSEL_UART1RX         0x05
#define DMACH_REQSEL_I2C0RX          0x07
#define DMACH_REQSEL_I2C1RX          0x08
#define DMACH_REQSEL_ADC             0x09
#define DMACH_REQSEL_UART2RX         0x0e
#define DMACH_REQSEL_SPI3RX          0x0f
#define DMACH_REQSEL_SPI_MSS0RX      0x10
#define DMACH_REQSEL_USBRXEP1        0x11
#define DMACH_REQSEL_USBRXEP2        0x12
#define DMACH_REQSEL_USBRXEP3        0x13
#define DMACH_REQSEL_USBRXEP4        0x14
#define DMACH_REQSEL_USBRXEP5        0x15
#define DMACH_REQSEL_USBRXEP6        0x16
#define DMACH_REQSEL_USBRXEP7        0x17
#define DMACH_REQSEL_USBRXEP8        0x18
#define DMACH_REQSEL_USBRXEP9        0x19
#define DMACH_REQSEL_USBRXEP10       0x1a
#define DMACH_REQSEL_USBRXEP11       0x1b
#define DMACH_REQSEL_SPI0TX          0x21
#define DMACH_REQSEL_SPI1TX          0x22
#define DMACH_REQSEL_SPI2TX          0x23
#define DMACH_REQSEL_UART0TX         0x24
#define DMACH_REQSEL_UART1TX         0x25
#define DMACH_REQSEL_I2C0TX          0x27
#define DMACH_REQSEL_I2C1TX          0x28
#define DMACH_REQSEL_UART2TX         0x2e
#define DMACH_REQSEL_SPI3TX          0x2f
#define DMACH_REQSEL_SPI_MSS0TX      0x30
#define DMACH_REQSEL_USBTXEP1        0x31
#define DMACH_REQSEL_USBTXEP2        0x32
#define DMACH_REQSEL_USBTXEP3        0x33
#define DMACH_REQSEL_USBTXEP4        0x34
#define DMACH_REQSEL_USBTXEP5        0x35
#define DMACH_REQSEL_USBTXEP6        0x36
#define DMACH_REQSEL_USBTXEP7        0x37
#define DMACH_REQSEL_USBTXEP8        0x38
#define DMACH_REQSEL_USBTXEP9        0x39
#define DMACH_REQSEL_USBTXEP10       0x3a
#define DMACH_REQSEL_USBTXEP11       0x3b

/* DMA Channel Status Register */

#define DMACH_STAT_CHST             (1 << 0)  /* Bit 0:  Channel Status */
#define DMACH_STAT_IPEND            (1 << 1)  /* Bit 1:  Channel Interrupt */
#define DMACH_STAT_CTZST            (1 << 2)  /* Bit 2:  CTZ Status */
#define DMACH_STAT_RLDST            (1 << 3)  /* Bit 3:  Reload Status */
#define DMACH_STAT_BUSERR           (1 << 4)  /* Bit 4:  Bus Error */
#define DMACH_STAT_TOST             (1 << 6)  /* Bit 6:  Time-Out Status */

/* DMA Channel Source Register (32-bit Source Device Address) */

/* DMA Channel Destination Register (32-bit Destination Device Address) */

/* DMA Channel Count Register */

#define DMACH_CNT_MASK              (0x00ffffff) /* Bits 0-23: DMA counter */

/* DMA Channel Source Reload Register */

#define DMACH_SRCRLD_MASK           (0x7fffffff) /* Bits 0-30: Source Address
                                                  *            Reload Value */

/* DMA Channel Destination Reload Register */

#define DMACH_DSTRLD_MASK           (0x7fffffff) /* Bits 0-30: Destination
                                                  *            Address Reload
                                                  *            Value */

/* DMA Channel Count Reload Register */

#define DMACH_CNTRLD_MASK           (0x00ffffff) /* Bits 0-23: Count Reload
                                                  *            Value */
#define DMACH_CNTRLD_RLDEN          (1 << 31)    /* Bit 31: Reload Enable */

#endif /* __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX326_DMA_H */
