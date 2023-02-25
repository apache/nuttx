/****************************************************************************
 * arch/arm/src/gd32f4/hardware/gd32f4xx_dma.h
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

#ifndef __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_DMA_H
#define __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DMA definitions */
#define GD32_DMA0_BASE               (GD32_DMA_BASE+0x00000000)          /* DMA0 base address */
#define GD32_DMA1_BASE               (GD32_DMA_BASE+0x00000400)          /* DMA1 base address */

/* Register Offsets *********************************************************/
#define GD32_DMA_INTF0_OFFSET          0x0000            /* DMA interrupt flag register 0 offset */
#define GD32_DMA_INTF1_OFFSET          0x0004            /* DMA interrupt flag register 1 offset */
#define GD32_DMA_INTC0_OFFSET          0x0008            /* DMA interrupt flag clear register 1 offset */
#define GD32_DMA_INTC1_OFFSET          0x000C            /* DMA interrupt flag clear register 1 offset */

#define GD32_DMA_CH0CTL_OFFSET         0x0010            /* DMA channel 0 control register offset */
#define GD32_DMA_CH0CNT_OFFSET         0x0014            /* DMA channel 0 counter register offset */
#define GD32_DMA_CH0PADDR_OFFSET       0x0018            /* DMA channel 0 peripheral base address register offset */
#define GD32_DMA_CH0M0ADDR_OFFSET      0x001C            /* DMA channel 0 memory 0 base address register offset */
#define GD32_DMA_CH0M1ADDR_OFFSET      0x0020            /* DMA channel 0 memory 1 base address register offset */
#define GD32_DMA_CH0FCTL_OFFSET        0x0024            /* DMA channel 0 FIFO control register offset */

#define GD32_DMA_CH1CTL_OFFSET         0x0028            /* DMA channel 1 control register offset */
#define GD32_DMA_CH1CNT_OFFSET         0x002C            /* DMA channel 1 counter register offset */
#define GD32_DMA_CH1PADDR_OFFSET       0x0030            /* DMA channel 1 peripheral base address register offset */
#define GD32_DMA_CH1M0ADDR_OFFSET      0x0034            /* DMA channel 1 memory 0 base address register offset */
#define GD32_DMA_CH1M1ADDR_OFFSET      0x0038            /* DMA channel 1 memory 1 base address register offset */
#define GD32_DMA_CH1FCTL_OFFSET        0x003C            /* DMA channel 1 FIFO control register offset */

#define GD32_DMA_CH2CTL_OFFSET         0x0040            /* DMA channel 2 control register offset */
#define GD32_DMA_CH2CNT_OFFSET         0x0044            /* DMA channel 2 counter register offset */
#define GD32_DMA_CH2PADDR_OFFSET       0x0048            /* DMA channel 2 peripheral base address register offset */
#define GD32_DMA_CH2M0ADDR_OFFSET      0x004C            /* DMA channel 2 memory 0 base address register offset */
#define GD32_DMA_CH2M1ADDR_OFFSET      0x0050            /* DMA channel 2 memory 1 base address register offset */
#define GD32_DMA_CH2FCTL_OFFSET        0x0054            /* DMA channel 2 FIFO control register offset */

#define GD32_DMA_CH3CTL_OFFSET         0x0058            /* DMA channel 3 control register offset */
#define GD32_DMA_CH3CNT_OFFSET         0x005C            /* DMA channel 3 counter register offset */
#define GD32_DMA_CH3PADDR_OFFSET       0x0060            /* DMA channel 3 peripheral base address register offset */
#define GD32_DMA_CH3M0ADDR_OFFSET      0x0064            /* DMA channel 3 memory 0 base address register offset */
#define GD32_DMA_CH3M1ADDR_OFFSET      0x0068            /* DMA channel 3 memory 1 base address register offset */
#define GD32_DMA_CH3FCTL_OFFSET        0x006C            /* DMA channel 3 FIFO control register offset */

#define GD32_DMA_CH4CTL_OFFSET         0x0070            /* DMA channel 4 control register offset */
#define GD32_DMA_CH4CNT_OFFSET         0x0074            /* DMA channel 4 counter register offset */
#define GD32_DMA_CH4PADDR_OFFSET       0x0078            /* DMA channel 4 peripheral base address register offset */
#define GD32_DMA_CH4M0ADDR_OFFSET      0x007C            /* DMA channel 4 memory 0 base address register offset */
#define GD32_DMA_CH4M1ADDR_OFFSET      0x0080            /* DMA channel 4 memory 1 base address register offset */
#define GD32_DMA_CH4FCTL_OFFSET        0x0084            /* DMA channel 4 FIFO control register offset */

#define GD32_DMA_CH5CTL_OFFSET         0x0088            /* DMA channel 5 control register offset */
#define GD32_DMA_CH5CNT_OFFSET         0x008C            /* DMA channel 5 counter register offset */
#define GD32_DMA_CH5PADDR_OFFSET       0x0090            /* DMA channel 5 peripheral base address register offset */
#define GD32_DMA_CH5M0ADDR_OFFSET      0x0094            /* DMA channel 5 memory 0 base address register offset */
#define GD32_DMA_CH5M1ADDR_OFFSET      0x0098            /* DMA channel 5 memory 1 base address register offset */
#define GD32_DMA_CH5FCTL_OFFSET        0x009C            /* DMA channel 5 FIFO control register offset */

#define GD32_DMA_CH6CTL_OFFSET         0x00A0            /* DMA channel 6 control register offset */
#define GD32_DMA_CH6CNT_OFFSET         0x00A4            /* DMA channel 6 counter register offset */
#define GD32_DMA_CH6PADDR_OFFSET       0x00A8            /* DMA channel 6 peripheral base address register offset */
#define GD32_DMA_CH6M0ADDR_OFFSET      0x00AC            /* DMA channel 6 memory 0 base address register offset */
#define GD32_DMA_CH6M1ADDR_OFFSET      0x00B0            /* DMA channel 6 memory 1 base address register offset */
#define GD32_DMA_CH6FCTL_OFFSET        0x00B4            /* DMA channel 6 FIFO control register offset */

#define GD32_DMA_CH7CTL_OFFSET         0x00B8            /* DMA channel 7 control register offset */
#define GD32_DMA_CH7CNT_OFFSET         0x00BC            /* DMA channel 7 counter register offset */
#define GD32_DMA_CH7PADDR_OFFSET       0x00C0            /* DMA channel 7 peripheral base address register offset */
#define GD32_DMA_CH7M0ADDR_OFFSET      0x00C4            /* DMA channel 7 memory 0 base address register offset */
#define GD32_DMA_CH7M1ADDR_OFFSET      0x00C8            /* DMA channel 7 memory 1 base address register offset */
#define GD32_DMA_CH7FCTL_OFFSET        0x00CC            /* DMA channel 7 FIFO control register offset */

/* Register Addresses *******************************************************/

#define GD32_DMA_INTF0(dmax)           ((dmax)+GD32_DMA_INTF0_OFFSET)            /* DMA interrupt flag register 0 */
#define GD32_DMA_INTF1(dmax)           ((dmax)+GD32_DMA_INTF1_OFFSET)            /* DMA interrupt flag register 1 */
#define GD32_DMA_INTC0(dmax)           ((dmax)+GD32_DMA_INTC0_OFFSET)            /* DMA interrupt flag clear register 1 */
#define GD32_DMA_INTC1(dmax)           ((dmax)+GD32_DMA_INTC1_OFFSET)            /* DMA interrupt flag clear register 1 */

#define GD32_DMA_CH0CTL(dmax)          ((dmax)+GD32_DMA_CH0CTL_OFFSET)           /* DMA channel 0 control register */
#define GD32_DMA_CH0CNT(dmax)          ((dmax)+GD32_DMA_CH0CNT_OFFSET)           /* DMA channel 0 counter register */
#define GD32_DMA_CH0PADDR(dmax)        ((dmax)+GD32_DMA_CH0PADDR_OFFSET)         /* DMA channel 0 peripheral base address register */
#define GD32_DMA_CH0M0ADDR(dmax)       ((dmax)+GD32_DMA_CH0M0ADDR_OFFSET)        /* DMA channel 0 memory 0 base address register */
#define GD32_DMA_CH0M1ADDR(dmax)       ((dmax)+GD32_DMA_CH0M1ADDR_OFFSET)        /* DMA channel 0 memory 1 base address register */
#define GD32_DMA_CH0FCTL(dmax)         ((dmax)+GD32_DMA_CH0FCTL_OFFSET)          /* DMA channel 0 FIFO control register */

#define GD32_DMA_CH1CTL(dmax)          ((dmax)+GD32_DMA_CH1CTL_OFFSET)           /* DMA channel 1 control register */
#define GD32_DMA_CH1CNT(dmax)          ((dmax)+GD32_DMA_CH1CNT_OFFSET)           /* DMA channel 1 counter register */
#define GD32_DMA_CH1PADDR(dmax)        ((dmax)+GD32_DMA_CH1PADDR_OFFSET)         /* DMA channel 1 peripheral base address register */
#define GD32_DMA_CH1M0ADDR(dmax)       ((dmax)+GD32_DMA_CH1M0ADDR_OFFSET)        /* DMA channel 1 memory 0 base address register */
#define GD32_DMA_CH1M1ADDR(dmax)       ((dmax)+GD32_DMA_CH1M1ADDR_OFFSET)        /* DMA channel 1 memory 1 base address register */
#define GD32_DMA_CH1FCTL(dmax)         ((dmax)+GD32_DMA_CH1FCTL_OFFSET)          /* DMA channel 1 FIFO control register */

#define GD32_DMA_CH2CTL(dmax)          ((dmax)+GD32_DMA_CH2CTL_OFFSET)           /* DMA channel 2 control register */
#define GD32_DMA_CH2CNT(dmax)          ((dmax)+GD32_DMA_CH2CNT_OFFSET)           /* DMA channel 2 counter register */
#define GD32_DMA_CH2PADDR(dmax)        ((dmax)+GD32_DMA_CH2PADDR_OFFSET)         /* DMA channel 2 peripheral base address register */
#define GD32_DMA_CH2M0ADDR(dmax)       ((dmax)+GD32_DMA_CH2M0ADDR_OFFSET)        /* DMA channel 2 memory 0 base address register */
#define GD32_DMA_CH2M1ADDR(dmax)       ((dmax)+GD32_DMA_CH2M1ADDR_OFFSET)        /* DMA channel 2 memory 1 base address register */
#define GD32_DMA_CH2FCTL(dmax)         ((dmax)+GD32_DMA_CH2FCTL_OFFSET)          /* DMA channel 2 FIFO control register */

#define GD32_DMA_CH3CTL(dmax)          ((dmax)+GD32_DMA_CH3CTL_OFFSET)           /* DMA channel 3 control register */
#define GD32_DMA_CH3CNT(dmax)          ((dmax)+GD32_DMA_CH3CNT_OFFSET)           /* DMA channel 3 counter register */
#define GD32_DMA_CH3PADDR(dmax)        ((dmax)+GD32_DMA_CH3PADDR_OFFSET)         /* DMA channel 3 peripheral base address register */
#define GD32_DMA_CH3M0ADDR(dmax)       ((dmax)+GD32_DMA_CH3M0ADDR_OFFSET)        /* DMA channel 3 memory 0 base address register */
#define GD32_DMA_CH3M1ADDR(dmax)       ((dmax)+GD32_DMA_CH3M1ADDR_OFFSET)        /* DMA channel 3 memory 1 base address register */
#define GD32_DMA_CH3FCTL(dmax)         ((dmax)+GD32_DMA_CH3FCTL_OFFSET)          /* DMA channel 3 FIFO control register */

#define GD32_DMA_CH4CTL(dmax)          ((dmax)+GD32_DMA_CH4CTL_OFFSET)           /* DMA channel 4 control register */
#define GD32_DMA_CH4CNT(dmax)          ((dmax)+GD32_DMA_CH4CNT_OFFSET)           /* DMA channel 4 counter register */
#define GD32_DMA_CH4PADDR(dmax)        ((dmax)+GD32_DMA_CH4PADDR_OFFSET)         /* DMA channel 4 peripheral base address register */
#define GD32_DMA_CH4M0ADDR(dmax)       ((dmax)+GD32_DMA_CH4M0ADDR_OFFSET)        /* DMA channel 4 memory 0 base address register */
#define GD32_DMA_CH4M1ADDR(dmax)       ((dmax)+GD32_DMA_CH4M1ADDR_OFFSET)        /* DMA channel 4 memory 1 base address register */
#define GD32_DMA_CH4FCTL(dmax)         ((dmax)+GD32_DMA_CH4FCTL_OFFSET)          /* DMA channel 4 FIFO control register */

#define GD32_DMA_CH5CTL(dmax)          ((dmax)+GD32_DMA_CH5CTL_OFFSET)           /* DMA channel 5 control register */
#define GD32_DMA_CH5CNT(dmax)          ((dmax)+GD32_DMA_CH5CNT_OFFSET)           /* DMA channel 5 counter register */
#define GD32_DMA_CH5PADDR(dmax)        ((dmax)+GD32_DMA_CH5PADDR_OFFSET)         /* DMA channel 5 peripheral base address register */
#define GD32_DMA_CH5M0ADDR(dmax)       ((dmax)+GD32_DMA_CH5M0ADDR_OFFSET)        /* DMA channel 5 memory 0 base address register */
#define GD32_DMA_CH5M1ADDR(dmax)       ((dmax)+GD32_DMA_CH5M1ADDR_OFFSET)        /* DMA channel 5 memory 1 base address register */
#define GD32_DMA_CH5FCTL(dmax)         ((dmax)+GD32_DMA_CH5FCTL_OFFSET)          /* DMA channel 5 FIFO control register */

#define GD32_DMA_CH6CTL(dmax)          ((dmax)+GD32_DMA_CH6CTL_OFFSET)           /* DMA channel 6 control register */
#define GD32_DMA_CH6CNT(dmax)          ((dmax)+GD32_DMA_CH6CNT_OFFSET)           /* DMA channel 6 counter register */
#define GD32_DMA_CH6PADDR(dmax)        ((dmax)+GD32_DMA_CH6PADDR_OFFSET)         /* DMA channel 6 peripheral base address register */
#define GD32_DMA_CH6M0ADDR(dmax)       ((dmax)+GD32_DMA_CH6M0ADDR_OFFSET)        /* DMA channel 6 memory 0 base address register */
#define GD32_DMA_CH6M1ADDR(dmax)       ((dmax)+GD32_DMA_CH6M1ADDR_OFFSET)        /* DMA channel 6 memory 1 base address register */
#define GD32_DMA_CH6FCTL(dmax)         ((dmax)+GD32_DMA_CH6FCTL_OFFSET)          /* DMA channel 6 FIFO control register */

#define GD32_DMA_CH7CTL(dmax)          ((dmax)+GD32_DMA_CH7CTL_OFFSET)           /* DMA channel 7 control register */
#define GD32_DMA_CH7CNT(dmax)          ((dmax)+GD32_DMA_CH7CNT_OFFSET)           /* DMA channel 7 counter register */
#define GD32_DMA_CH7PADDR(dmax)        ((dmax)+GD32_DMA_CH7PADDR_OFFSET)         /* DMA channel 7 peripheral base address register */
#define GD32_DMA_CH7M0ADDR(dmax)       ((dmax)+GD32_DMA_CH7M0ADDR_OFFSET)        /* DMA channel 7 memory 0 base address register */
#define GD32_DMA_CH7M1ADDR(dmax)       ((dmax)+GD32_DMA_CH7M1ADDR_OFFSET)        /* DMA channel 7 memory 1 base address register */
#define GD32_DMA_CH7FCTL(dmax)         ((dmax)+GD32_DMA_CH7FCTL_OFFSET)          /* DMA channel 7 FIFO control register */

/* DMA channelx register address */
#define GD32_DMA_CHCTL(dma, channelx)            ((dma+0x10) + 0x18*(channelx))  /* The address of DMA channel CHXCTL register  */
#define GD32_DMA_CHCNT(dma, channelx)            ((dma+0x14) + 0x18*(channelx))  /* The address of DMA channel CHXCNT register */
#define GD32_DMA_CHPADDR(dma, channelx)          ((dma+0x18) + 0x18*(channelx))  /* The address of DMA channel CHXPADDR register */
#define GD32_DMA_CHM0ADDR(dma, channelx)         ((dma+0x1C) + 0x18*(channelx))  /* The address of DMA channel CHXM0ADDR register */
#define GD32_DMA_CHM1ADDR(dma, channelx)         ((dma+0x20) + 0x18*(channelx))  /* The address of DMA channel CHXM1ADDR register */
#define GD32_DMA_CHFCTL(dma, channelx)           ((dma+0x24) + 0x18*(channelx))  /* The address of DMA channel CHXMADDR register */

/* bits definitions */

/* DMA_INTF */
#define DMA_INTF_FEEIF                 (1 << 0)                        /* Bit 0: FIFO error and exception flag */
#define DMA_INTF_SDEIF                 (1 << 2)                        /* Bit 2: single data mode exception flag */
#define DMA_INTF_TAEIF                 (1 << 3)                        /* Bit 3: transfer access error flag */
#define DMA_INTF_HTFIF                 (1 << 4)                        /* Bit 4: half transfer finish flag */
#define DMA_INTF_FTFIF                 (1 << 5)                        /* Bit 5: full transger finish flag */

/* DMA_INTC */
#define DMA_INTC_FEEIFC                (1 << 0)                        /* Bit 0: clear FIFO error and exception flag */
#define DMA_INTC_SDEIFC                (1 << 2)                        /* Bit 2: clear single data mode exception flag */
#define DMA_INTC_TAEIFC                (1 << 3)                        /* Bit 3: clear single data mode exception flag */
#define DMA_INTC_HTFIFC                (1 << 4)                        /* Bit 4: clear half transfer finish flag */
#define DMA_INTC_FTFIFC                (1 << 5)                        /* Bit 5: clear full transger finish flag */

/* DMA_CHxCTL,x=0..7 */
#define DMA_CHXCTL_CHEN                (1 << 0)                        /* Bit 0: channel x enable */
#define DMA_CHXCTL_SDEIE               (1 << 1)                        /* Bit 1: enable bit for channel x single data mode exception interrupt */
#define DMA_CHXCTL_TAEIE               (1 << 2)                        /* Bit 2: enable bit for channel x tranfer access error interrupt */
#define DMA_CHXCTL_HTFIE               (1 << 3)                        /* Bit 3: enable bit for channel x half transfer finish interrupt */
#define DMA_CHXCTL_FTFIE               (1 << 4)                        /* Bit 4: enable bit for channel x full transfer finish interrupt */
#define DMA_CHXCTL_TFCS                (1 << 5)                        /* Bit 5: transfer flow controller select */
#define DMA_CHXCTL_TM_SHIFT            (6)                             /* Bit 6-7: transfer mode */
#define DMA_CHXCTL_TM_MASK             (3 << DMA_CHXCTL_TM_SHIFT)
#define DMA_CHXCTL_TM(n)               ((n) << DMA_CHXCTL_TM_SHIFT)
#  define DMA_PERIPH_TO_MEMORY         DMA_CHXCTL_TM(0)                /* 00: read from peripheral and write to memory */
#  define DMA_MEMORY_TO_PERIPH         DMA_CHXCTL_TM(1)                /* 01: read from peripheral and write to memory */
#  define DMA_MEMORY_TO_MEMORY         DMA_CHXCTL_TM(2)                /* 02: read from peripheral and write to memory */

#define DMA_CHXCTL_CMEN                (1 << 8)                        /* Bit 8: circulation mode */
#define DMA_CHXCTL_PNAGA               (1 << 9)                        /* Bit 9: next address generation algorithm of peripheral */
#define DMA_CHXCTL_MNAGA               (1 << 10)                       /* Bit 10: next address generation algorithm of memory */
#define DMA_CHXCTL_PWIDTH_SHIFT        (11)                            /* Bit 11-12: transfer width of peipheral */
#define DMA_CHXCTL_PWIDTH_MASK         (3 << DMA_CHXCTL_PWIDTH_SHIFT)
#define DMA_CHXCTL_PWIDTH(n)           ((n) << DMA_CHXCTL_PWIDTH_SHIFT)
#  define DMA_PERIPH_WIDTH_8BIT        DMA_CHXCTL_PWIDTH(0)            /* 00: transfer data width of peripheral is 8-bit */
#  define DMA_PERIPH_WIDTH_16BIT       DMA_CHXCTL_PWIDTH(1)            /* 01: transfer data width of peripheral is 16-bit */
#  define DMA_PERIPH_WIDTH_32BIT       DMA_CHXCTL_PWIDTH(2)            /* 02: transfer data width of peripheral is 32-bit */

#define DMA_CHXCTL_MWIDTH_SHIFT        (13)                            /* Bit 13-14: transfer width of memory */
#define DMA_CHXCTL_MWIDTH_MASK         (3 << DMA_CHXCTL_MWIDTH_SHIFT)
#define DMA_CHXCTL_MWIDTH(n)           ((n) << DMA_CHXCTL_MWIDTH_SHIFT)
#  define DMA_MEMORY_WIDTH_8BIT        DMA_CHXCTL_MWIDTH(0)            /* 00: transfer data width of memory is 8-bit */
#  define DMA_MEMORY_WIDTH_16BIT       DMA_CHXCTL_MWIDTH(1)            /* 01: transfer data width of memory is 16-bit */
#  define DMA_MEMORY_WIDTH_32BIT       DMA_CHXCTL_MWIDTH(2)            /* 02: transfer data width of memory is 32-bit */

#define DMA_CHXCTL_PAIF                (1 << 15)                       /* Bit 15: peripheral address increment fixed */

#define DMA_CHXCTL_PRIO_SHIFT          (16)                            /* Bit 16-17: priority level */
#define DMA_CHXCTL_PRIO_MASK           (3 << DMA_CHXCTL_PRIO_SHIFT)
#define DMA_CHXCTL_PRIO(n)             ((n) << DMA_CHXCTL_PRIO_SHIFT)
#  define DMA_PRIORITY_LOW             DMA_CHXCTL_PRIO(0)              /* 00: low priority */
#  define DMA_PRIORITY_MEDIUM          DMA_CHXCTL_PRIO(1)              /* 01: medium priority */
#  define DMA_PRIORITY_HIGH            DMA_CHXCTL_PRIO(2)              /* 10: high priority */
#  define DMA_PRIORITY_ULTRA_HIGH      DMA_CHXCTL_PRIO(3)              /* 11: ultra high priority */

#define DMA_CHXCTL_SBMEN               (1 << 18)                       /* Bit 18: switch-buffer mode enable */
#define DMA_CHXCTL_MBS                 (1 << 19)                       /* Bit19: memory buffer select */

#define DMA_CHXCTL_PBURST_SHIFT        (21)                            /* Bit 21-22: transfer burst type of peripheral */
#define DMA_CHXCTL_PBURST_MASK         (3 << DMA_CHXCTL_PBURST_SHIFT)
#define DMA_CHXCTL_PBURST(n)           ((n) << DMA_CHXCTL_PBURST_SHIFT)
#  define DMA_PERIPH_BURST_SINGLE      DMA_CHXCTL_PBURST(0)            /* single burst */
#  define DMA_PERIPH_BURST_4_BEAT      DMA_CHXCTL_PBURST(1)            /* 4-beat burst */
#  define DMA_PERIPH_BURST_8_BEAT      DMA_CHXCTL_PBURST(2)            /* 8-beat burst */
#  define DMA_PERIPH_BURST_16_BEAT     DMA_CHXCTL_PBURST(3)            /* 16-beat burst */

#define DMA_CHXCTL_MBURST_SHIFT        (23)                            /* Bit 23-24: transfer burst type of memory */
#define DMA_CHXCTL_MBURST_MASK         (3 << DMA_CHXCTL_MBURST_SHIFT)
#define DMA_CHXCTL_MBURST(n)           ((n) << DMA_CHXCTL_MBURST_SHIFT)
#  define DMA_MEMORY_BURST_SINGLE      DMA_CHXCTL_MBURST(0)            /* single burst */
#  define DMA_MEMORY_BURST_4_BEAT      DMA_CHXCTL_MBURST(1)            /* 4-beat burst */
#  define DMA_MEMORY_BURST_8_BEAT      DMA_CHXCTL_MBURST(2)            /* 8-beat burst */
#  define DMA_MEMORY_BURST_16_BEAT     DMA_CHXCTL_MBURST(3)            /* 16-beat burst */

#define DMA_CHXCTL_PERIEN_SHIFT        (25)                            /* Bit 25-27: peripheral enable */
#define DMA_CHXCTL_PERIEN_MASK         (7 << DMA_CHXCTL_PERIEN_SHIFT)
#define DMA_CHXCTL_PERIEN(n)           ((n) << DMA_CHXCTL_PERIEN_SHIFT)
#  define DMA_PERIPH_0_SELECT           DMA_CHXCTL_PERIEN(0)           /* peripheral 0 select */
#  define DMA_PERIPH_1_SELECT           DMA_CHXCTL_PERIEN(1)           /* peripheral 1 select */
#  define DMA_PERIPH_2_SELECT           DMA_CHXCTL_PERIEN(2)           /* peripheral 2 select */
#  define DMA_PERIPH_3_SELECT           DMA_CHXCTL_PERIEN(3)           /* peripheral 3 select */
#  define DMA_PERIPH_4_SELECT           DMA_CHXCTL_PERIEN(4)           /* peripheral 4 select */
#  define DMA_PERIPH_5_SELECT           DMA_CHXCTL_PERIEN(5)           /* peripheral 5 select */
#  define DMA_PERIPH_6_SELECT           DMA_CHXCTL_PERIEN(6)           /* peripheral 6 select */
#  define DMA_PERIPH_7_SELECT           DMA_CHXCTL_PERIEN(7)           /* peripheral 7 select */

/* DMA_CHxCNT, x=0..7 */
#define DMA_CHXCNT_CNT_MASK            (0xffff << 0)                   /* Bit 0-15: transfer counter */

/* DMA_CHxPADDR,x=0..7 */
#define DMA_CHXPADDR_PADDR_MASK        (0xffffffff << 0)               /* Bit 0-31: peripheral base address */

/* DMA_CHxM0ADDR,x=0..7 */
#define DMA_CHXM0ADDR_M0ADDR_MASK      (0xffffffff << 0)               /* Bit 0-31: memory 0 base address */

/* DMA_CHxM1ADDR,x=0..7 */
#define DMA_CHXM1ADDR_M0ADDR_MASK      (0xffffffff << 0)               /* Bit 0-31: memory 1 base address */

/* DMA_CHxFCTL,x=0..7 */
#define DMA_CHXFCTL_FCCV_SHIFT         (0)                             /* Bit 0-1: FIFO counter critical value */
#define DMA_CHXFCTL_FCCV_MASK          (3 << DMA_CHXFCTL_FCCV_SHIFT)
#define DMA_CHXFCTL_FCCV(n)            ((n) << DMA_CHXFCTL_FCCV_SHIFT)
#  define DMA_FIFO_1_WORD              DMA_CHXFCTL_FCCV(0)             /* critical value 1 word */
#  define DMA_FIFO_2_WORD              DMA_CHXFCTL_FCCV(1)             /* critical value 2 word */
#  define DMA_FIFO_3_WORD              DMA_CHXFCTL_FCCV(2)             /* critical value 3 word */
#  define DMA_FIFO_4_WORD              DMA_CHXFCTL_FCCV(3)             /* critical value 4 word */

#define DMA_CHXFCTL_MDMEN              (1 << 2)                        /* Bit 2: multi-data mode enable */
#define DMA_CHXFCTL_FCNT_SHIFT         (3)                             /* Bit 3-5: FIFO counter */
#define DMA_CHXFCTL_FCNT_MASK          (7 << DMA_CHXFCTL_FCNT_SHIFT)
#define DMA_CHXFCTL_FCNT(n)            ((n) << DMA_CHXFCTL_FCNT_SHIFT)

#define DMA_CHXFCTL_FEEIE              (1 << 7)                        /* Bit 3: FIFO exception interrupt enable */

#define GD32_DMA_FLAG_ADD(flag,channel)        ((uint32_t)((flag)<<((((uint32_t)(channel)*6U))+((uint32_t)(((uint32_t)(channel)) >> 1U)&0x01U)*4U)))   /* DMA channel flag shift */

/* peripheral increasing mode */
#define DMA_PERIPH_INCREASE_ENABLE       (0x00000000)          /* Next address of peripheral is increasing address mode */
#define DMA_PERIPH_INCREASE_DISABLE      (0x00000001)          /* Next address of peripheral is fixed address mode */
#define DMA_PERIPH_INCREASE_FIX          (0x00000002)          /* Next address of peripheral is increasing fixed */

/* memory increasing mode */
#define DMA_MEMORY_INCREASE_ENABLE       (0x00000000)          /* Next address of memory is increasing address mode */
#define DMA_MEMORY_INCREASE_DISABLE      (0x00000001)          /* Next address of memory is fixed address mode */

/* DMA circular mode */
#define DMA_CIRCULAR_MODE_ENABLE         (0x00000000)          /* Circular mode enable */
#define DMA_CIRCULAR_MODE_DISABLE        (0x00000001)          /* Circular mode disable */

/* DMA width selection */
#define DMA_WIDTH_8BITS_SELECT           (0x00000000)          /* Select 8 bits width */
#define DMA_WIDTH_16BITS_SELECT          (0x00000001)          /* Select 16 bits width */
#define DMA_WIDTH_32BITS_SELECT          (0x00000002)          /* Select 16 bits width */

/* DMA priority level selection */
#define DMA_PRIO_LOW_SELECT              (0x00000000)          /* Select low priority level */
#define DMA_PRIO_MEDIUM_SELECT           (0x00000001)          /* Select medium priority level */
#define DMA_PRIO_HIGH_SELECT             (0x00000002)          /* Select high priority level */
#define DMA_PRIO_ULTRA_HIGHSELECT        (0x00000003)          /* Select ultra high priority level */

/* DMA channel select */

#define GD32_DMA_CH0                 (0)        /* DMA Channel 0 */
#define GD32_DMA_CH1                 (1)        /* DMA Channel 1 */
#define GD32_DMA_CH2                 (2)        /* DMA Channel 2 */
#define GD32_DMA_CH3                 (3)        /* DMA Channel 3 */
#define GD32_DMA_CH4                 (4)        /* DMA Channel 4 */
#define GD32_DMA_CH5                 (5)        /* DMA Channel 5 */
#define GD32_DMA_CH6                 (6)        /* DMA Channel 6 */
#define GD32_DMA_CH7                 (7)        /* DMA Channel 7 */

/* DMA peripheral select */

#define GD32_DMA_SUBPERI0            (0)        /* DMA Peripheral 0 */
#define GD32_DMA_SUBPERI1            (1)        /* DMA Peripheral 1 */
#define GD32_DMA_SUBPERI2            (2)        /* DMA Peripheral 2 */
#define GD32_DMA_SUBPERI3            (3)        /* DMA Peripheral 3 */
#define GD32_DMA_SUBPERI4            (4)        /* DMA Peripheral 4 */
#define GD32_DMA_SUBPERI5            (5)        /* DMA Peripheral 5 */
#define GD32_DMA_SUBPERI6            (6)        /* DMA Peripheral 6 */
#define GD32_DMA_SUBPERI7            (7)        /* DMA Peripheral 7 */

/* DMA peripheral requests mapping */

#define CHANNEL_SHIFT               (0)
#define CHANNEL_MASK                 (0xf)
#define PERIPH_SHIFT                 (4)
#define PERIPH_MASK                  (0x7)

/* Peripheral requests to DMA0 */

#define DMA_REQ_SPI2_RX_1            ((GD32_DMA_SUBPERI0 << PERIPH_SHIFT) | (GD32_DMA_CH0 << CHANNEL_SHIFT))
#define DMA_REQ_I2C0_RX_1            ((GD32_DMA_SUBPERI1 << PERIPH_SHIFT) | (GD32_DMA_CH0 << CHANNEL_SHIFT))
#define DMA_REQ_TIMER3_CH0           ((GD32_DMA_SUBPERI2 << PERIPH_SHIFT) | (GD32_DMA_CH0 << CHANNEL_SHIFT))
#define DMA_REQ_I2S2_ADD_RX_1        ((GD32_DMA_SUBPERI3 << PERIPH_SHIFT) | (GD32_DMA_CH0 << CHANNEL_SHIFT))
#define DMA_REQ_UART4_RX             ((GD32_DMA_SUBPERI4 << PERIPH_SHIFT) | (GD32_DMA_CH0 << CHANNEL_SHIFT))
#define DMA_REQ_UART7_TX             ((GD32_DMA_SUBPERI5 << PERIPH_SHIFT) | (GD32_DMA_CH0 << CHANNEL_SHIFT))
#define DMA_REQ_TIMER4_CH2           ((GD32_DMA_SUBPERI6 << PERIPH_SHIFT) | (GD32_DMA_CH0 << CHANNEL_SHIFT))
#define DMA_REQ_TIMER4_UP_1          ((GD32_DMA_SUBPERI6 << PERIPH_SHIFT) | (GD32_DMA_CH0 << CHANNEL_SHIFT))
#define DMA_REQ_I2S2_ADD_RX          ((GD32_DMA_SUBPERI7 << PERIPH_SHIFT) | (GD32_DMA_CH0 << CHANNEL_SHIFT))

#define DMA_REQ_TIMER1_UP_1          ((GD32_DMA_SUBPERI3 << PERIPH_SHIFT) | (GD32_DMA_CH1 << CHANNEL_SHIFT))
#define DMA_REQ_TIMER1_CH2           ((GD32_DMA_SUBPERI3 << PERIPH_SHIFT) | (GD32_DMA_CH1 << CHANNEL_SHIFT))
#define DMA_REQ_USART2_RX            ((GD32_DMA_SUBPERI4 << PERIPH_SHIFT) | (GD32_DMA_CH1 << CHANNEL_SHIFT))
#define DMA_REQ_UART6_TX             ((GD32_DMA_SUBPERI5 << PERIPH_SHIFT) | (GD32_DMA_CH1 << CHANNEL_SHIFT))
#define DMA_REQ_TIMER4_CH3_1         ((GD32_DMA_SUBPERI6 << PERIPH_SHIFT) | (GD32_DMA_CH1 << CHANNEL_SHIFT))
#define DMA_REQ_TIMER4_TG            ((GD32_DMA_SUBPERI6 << PERIPH_SHIFT) | (GD32_DMA_CH1 << CHANNEL_SHIFT))
#define DMA_REQ_TIMER5_UP            ((GD32_DMA_SUBPERI7 << PERIPH_SHIFT) | (GD32_DMA_CH1 << CHANNEL_SHIFT))

#define DMA_REQ_SPI2_RX_2            ((GD32_DMA_SUBPERI0 << PERIPH_SHIFT) | (GD32_DMA_CH2 << CHANNEL_SHIFT))
#define DMA_REQ_TIMER6_UP_1          ((GD32_DMA_SUBPERI1 << PERIPH_SHIFT) | (GD32_DMA_CH2 << CHANNEL_SHIFT))
#define DMA_REQ_I2S2_ADD_RX_2        ((GD32_DMA_SUBPERI2 << PERIPH_SHIFT) | (GD32_DMA_CH2 << CHANNEL_SHIFT))
#define DMA_REQ_I2C2_RX              ((GD32_DMA_SUBPERI3 << PERIPH_SHIFT) | (GD32_DMA_CH2 << CHANNEL_SHIFT))
#define DMA_REQ_UART3_RX             ((GD32_DMA_SUBPERI4 << PERIPH_SHIFT) | (GD32_DMA_CH2 << CHANNEL_SHIFT))
#define DMA_REQ_TIMER2_CH3           ((GD32_DMA_SUBPERI5 << PERIPH_SHIFT) | (GD32_DMA_CH2 << CHANNEL_SHIFT))
#define DMA_REQ_TIMER2_UP_1          ((GD32_DMA_SUBPERI5 << PERIPH_SHIFT) | (GD32_DMA_CH2 << CHANNEL_SHIFT))
#define DMA_REQ_TIMER4_CH0           ((GD32_DMA_SUBPERI6 << PERIPH_SHIFT) | (GD32_DMA_CH2 << CHANNEL_SHIFT))
#define DMA_REQ_I2C1_RX_1            ((GD32_DMA_SUBPERI7 << PERIPH_SHIFT) | (GD32_DMA_CH2 << CHANNEL_SHIFT))

#define DMA_REQ_SPI1_RX              ((GD32_DMA_SUBPERI0 << PERIPH_SHIFT) | (GD32_DMA_CH3 << CHANNEL_SHIFT))
#define DMA_REQ_TIMER3_CH1           ((GD32_DMA_SUBPERI2 << PERIPH_SHIFT) | (GD32_DMA_CH3 << CHANNEL_SHIFT))
#define DMA_REQ_I2S1_ADD_RX          ((GD32_DMA_SUBPERI3 << PERIPH_SHIFT) | (GD32_DMA_CH3 << CHANNEL_SHIFT))
#define DMA_REQ_USART2_TX_1          ((GD32_DMA_SUBPERI4 << PERIPH_SHIFT) | (GD32_DMA_CH3 << CHANNEL_SHIFT))
#define DMA_REQ_UART6_RX             ((GD32_DMA_SUBPERI5 << PERIPH_SHIFT) | (GD32_DMA_CH3 << CHANNEL_SHIFT))
#define DMA_REQ_TIMER4_CH3_2         ((GD32_DMA_SUBPERI6 << PERIPH_SHIFT) | (GD32_DMA_CH3 << CHANNEL_SHIFT))
#define DMA_REQ_TIMER4_UP_2          ((GD32_DMA_SUBPERI6 << PERIPH_SHIFT) | (GD32_DMA_CH3 << CHANNEL_SHIFT))
#define DMA_REQ_I2C1_RX_2            ((GD32_DMA_SUBPERI7 << PERIPH_SHIFT) | (GD32_DMA_CH3 << CHANNEL_SHIFT))

#define DMA_REQ_SPI1_TX              ((GD32_DMA_SUBPERI0 << PERIPH_SHIFT) | (GD32_DMA_CH4 << CHANNEL_SHIFT))
#define DMA_REQ_TIMER6_UP_2          ((GD32_DMA_SUBPERI1 << PERIPH_SHIFT) | (GD32_DMA_CH4 << CHANNEL_SHIFT))
#define DMA_REQ_I2S1_ADD_TX          ((GD32_DMA_SUBPERI2 << PERIPH_SHIFT) | (GD32_DMA_CH4 << CHANNEL_SHIFT))
#define DMA_REQ_I2C2_TX              ((GD32_DMA_SUBPERI3 << PERIPH_SHIFT) | (GD32_DMA_CH4 << CHANNEL_SHIFT))
#define DMA_REQ_UART3_TX             ((GD32_DMA_SUBPERI4 << PERIPH_SHIFT) | (GD32_DMA_CH4 << CHANNEL_SHIFT))
#define DMA_REQ_TIMER2_CH0           ((GD32_DMA_SUBPERI5 << PERIPH_SHIFT) | (GD32_DMA_CH4 << CHANNEL_SHIFT))
#define DMA_REQ_TIMER2_UP_2          ((GD32_DMA_SUBPERI5 << PERIPH_SHIFT) | (GD32_DMA_CH4 << CHANNEL_SHIFT))
#define DMA_REQ_TIMER4_CH1           ((GD32_DMA_SUBPERI6 << PERIPH_SHIFT) | (GD32_DMA_CH4 << CHANNEL_SHIFT))
#define DMA_REQ_USART2_TX_2          ((GD32_DMA_SUBPERI7 << PERIPH_SHIFT) | (GD32_DMA_CH4 << CHANNEL_SHIFT))

#define DMA_REQ_SPI2_TX_1            ((GD32_DMA_SUBPERI0 << PERIPH_SHIFT) | (GD32_DMA_CH5 << CHANNEL_SHIFT))
#define DMA_REQ_I2C0_RX              ((GD32_DMA_SUBPERI1 << PERIPH_SHIFT) | (GD32_DMA_CH5 << CHANNEL_SHIFT))
#define DMA_REQ_I2S2_ADD_TX          ((GD32_DMA_SUBPERI2 << PERIPH_SHIFT) | (GD32_DMA_CH5 << CHANNEL_SHIFT))
#define DMA_REQ_TIMER1_CH0           ((GD32_DMA_SUBPERI3 << PERIPH_SHIFT) | (GD32_DMA_CH5 << CHANNEL_SHIFT))
#define DMA_REQ_USART1_RX            ((GD32_DMA_SUBPERI4 << PERIPH_SHIFT) | (GD32_DMA_CH5 << CHANNEL_SHIFT))
#define DMA_REQ_TIMER2_CH1           ((GD32_DMA_SUBPERI5 << PERIPH_SHIFT) | (GD32_DMA_CH5 << CHANNEL_SHIFT))
#define DMA_REQ_DAC0                 ((GD32_DMA_SUBPERI7 << PERIPH_SHIFT) | (GD32_DMA_CH5 << CHANNEL_SHIFT))

#define DMA_REQ_I2C0_TX_1            ((GD32_DMA_SUBPERI1 << PERIPH_SHIFT) | (GD32_DMA_CH6 << CHANNEL_SHIFT))
#define DMA_REQ_TIMER3_UP            ((GD32_DMA_SUBPERI2 << PERIPH_SHIFT) | (GD32_DMA_CH6 << CHANNEL_SHIFT))
#define DMA_REQ_TIMER1_CH1           ((GD32_DMA_SUBPERI3 << PERIPH_SHIFT) | (GD32_DMA_CH6 << CHANNEL_SHIFT))
#define DMA_REQ_TIMER1_CH3_1         ((GD32_DMA_SUBPERI3 << PERIPH_SHIFT) | (GD32_DMA_CH6 << CHANNEL_SHIFT))
#define DMA_REQ_USART1_TX            ((GD32_DMA_SUBPERI4 << PERIPH_SHIFT) | (GD32_DMA_CH6 << CHANNEL_SHIFT))
#define DMA_REQ_UART7_RX             ((GD32_DMA_SUBPERI5 << PERIPH_SHIFT) | (GD32_DMA_CH6 << CHANNEL_SHIFT))
#define DMA_REQ_TIMER4_UP            ((GD32_DMA_SUBPERI6 << PERIPH_SHIFT) | (GD32_DMA_CH6 << CHANNEL_SHIFT))
#define DMA_REQ_DAC1                 ((GD32_DMA_SUBPERI7 << PERIPH_SHIFT) | (GD32_DMA_CH6 << CHANNEL_SHIFT))

#define DMA_REQ_SPI2_TX_2            ((GD32_DMA_SUBPERI0 << PERIPH_SHIFT) | (GD32_DMA_CH7 << CHANNEL_SHIFT))
#define DMA_REQ_I2C0_TX_2            ((GD32_DMA_SUBPERI1 << PERIPH_SHIFT) | (GD32_DMA_CH7 << CHANNEL_SHIFT))
#define DMA_REQ_TIMER3_CH2           ((GD32_DMA_SUBPERI2 << PERIPH_SHIFT) | (GD32_DMA_CH7 << CHANNEL_SHIFT))
#define DMA_REQ_TIMER1_UP_2          ((GD32_DMA_SUBPERI3 << PERIPH_SHIFT) | (GD32_DMA_CH7 << CHANNEL_SHIFT))
#define DMA_REQ_TIMER1_CH3_2         ((GD32_DMA_SUBPERI3 << PERIPH_SHIFT) | (GD32_DMA_CH7 << CHANNEL_SHIFT))
#define DMA_REQ_UART4_TX             ((GD32_DMA_SUBPERI4 << PERIPH_SHIFT) | (GD32_DMA_CH7 << CHANNEL_SHIFT))
#define DMA_REQ_TIMER2_CH2           ((GD32_DMA_SUBPERI5 << PERIPH_SHIFT) | (GD32_DMA_CH7 << CHANNEL_SHIFT))
#define DMA_REQ_I2C1_TX              ((GD32_DMA_SUBPERI7 << PERIPH_SHIFT) | (GD32_DMA_CH7 << CHANNEL_SHIFT))

/* Peripheral requests to DMA1 */

#define DMA_REQ_ADC0_1               ((GD32_DMA_SUBPERI0 << PERIPH_SHIFT) | ((GD32_DMA_CH0 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_ADC2_1               ((GD32_DMA_SUBPERI2 << PERIPH_SHIFT) | ((GD32_DMA_CH0 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_SPI0_RX_1            ((GD32_DMA_SUBPERI3 << PERIPH_SHIFT) | ((GD32_DMA_CH0 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_SPI3_RX_1            ((GD32_DMA_SUBPERI4 << PERIPH_SHIFT) | ((GD32_DMA_CH0 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_TIMER0_TG_1          ((GD32_DMA_SUBPERI6 << PERIPH_SHIFT) | ((GD32_DMA_CH0 + 8) << CHANNEL_SHIFT))

#define DMA_REQ_DCI_1                ((GD32_DMA_SUBPERI1 << PERIPH_SHIFT) | ((GD32_DMA_CH1 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_ADC2_2               ((GD32_DMA_SUBPERI2 << PERIPH_SHIFT) | ((GD32_DMA_CH1 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_SPI3_TX_1            ((GD32_DMA_SUBPERI4 << PERIPH_SHIFT) | ((GD32_DMA_CH1 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_USART5_RX_1          ((GD32_DMA_SUBPERI5 << PERIPH_SHIFT) | ((GD32_DMA_CH1 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_TIMER0_CH0_1         ((GD32_DMA_SUBPERI6 << PERIPH_SHIFT) | ((GD32_DMA_CH1 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_TIMER7_UP            ((GD32_DMA_SUBPERI7 << PERIPH_SHIFT) | ((GD32_DMA_CH1 + 8) << CHANNEL_SHIFT))

#define DMA_REQ_TIMER7_CH0_1         ((GD32_DMA_SUBPERI0 << PERIPH_SHIFT) | ((GD32_DMA_CH2 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_TIMER7_CH1_1         ((GD32_DMA_SUBPERI0 << PERIPH_SHIFT) | ((GD32_DMA_CH2 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_TIMER7_CH2_1         ((GD32_DMA_SUBPERI0 << PERIPH_SHIFT) | ((GD32_DMA_CH2 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_ADC1_1               ((GD32_DMA_SUBPERI1 << PERIPH_SHIFT) | ((GD32_DMA_CH2 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_SPI0_RX_2            ((GD32_DMA_SUBPERI3 << PERIPH_SHIFT) | ((GD32_DMA_CH2 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_USART0_RX_1          ((GD32_DMA_SUBPERI4 << PERIPH_SHIFT) | ((GD32_DMA_CH2 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_USART5_RX_2          ((GD32_DMA_SUBPERI5 << PERIPH_SHIFT) | ((GD32_DMA_CH2 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_TIMER0_CH1_1         ((GD32_DMA_SUBPERI6 << PERIPH_SHIFT) | ((GD32_DMA_CH2 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_TIMER7_CH0_2         ((GD32_DMA_SUBPERI7 << PERIPH_SHIFT) | ((GD32_DMA_CH2 + 8) << CHANNEL_SHIFT))

#define DMA_REQ_ADC1_2               ((GD32_DMA_SUBPERI1 << PERIPH_SHIFT) | ((GD32_DMA_CH3 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_SPI4_RX_1            ((GD32_DMA_SUBPERI2 << PERIPH_SHIFT) | ((GD32_DMA_CH3 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_SPI0_TX_1            ((GD32_DMA_SUBPERI3 << PERIPH_SHIFT) | ((GD32_DMA_CH3 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_SDIO_1               ((GD32_DMA_SUBPERI4 << PERIPH_SHIFT) | ((GD32_DMA_CH3 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_SPI3_RX_2            ((GD32_DMA_SUBPERI5 << PERIPH_SHIFT) | ((GD32_DMA_CH3 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_TIMER0_CH0_2         ((GD32_DMA_SUBPERI6 << PERIPH_SHIFT) | ((GD32_DMA_CH3 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_TIMER7_CH1_2         ((GD32_DMA_SUBPERI7 << PERIPH_SHIFT) | ((GD32_DMA_CH3 + 8) << CHANNEL_SHIFT))

#define DMA_REQ_ADC0_2               ((GD32_DMA_SUBPERI0 << PERIPH_SHIFT) | ((GD32_DMA_CH4 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_SPI4_TX_1            ((GD32_DMA_SUBPERI2 << PERIPH_SHIFT) | ((GD32_DMA_CH4 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_SPI3_TX_2            ((GD32_DMA_SUBPERI5 << PERIPH_SHIFT) | ((GD32_DMA_CH4 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_TIMER0_CH3           ((GD32_DMA_SUBPERI6 << PERIPH_SHIFT) | ((GD32_DMA_CH4 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_TIMER0_TG_2          ((GD32_DMA_SUBPERI6 << PERIPH_SHIFT) | ((GD32_DMA_CH4 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_TIMER0_CMT           ((GD32_DMA_SUBPERI6 << PERIPH_SHIFT) | ((GD32_DMA_CH4 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_TIMER7_CH2_2         ((GD32_DMA_SUBPERI7 << PERIPH_SHIFT) | ((GD32_DMA_CH4 + 8) << CHANNEL_SHIFT))

#define DMA_REQ_SPI5_TX              ((GD32_DMA_SUBPERI1 << PERIPH_SHIFT) | ((GD32_DMA_CH5 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_SPI0_TX_2            ((GD32_DMA_SUBPERI3 << PERIPH_SHIFT) | ((GD32_DMA_CH5 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_USART0_RX_2          ((GD32_DMA_SUBPERI4 << PERIPH_SHIFT) | ((GD32_DMA_CH5 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_TIMER0_UP            ((GD32_DMA_SUBPERI6 << PERIPH_SHIFT) | ((GD32_DMA_CH5 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_SPI4_RX_2            ((GD32_DMA_SUBPERI7 << PERIPH_SHIFT) | ((GD32_DMA_CH5 + 8) << CHANNEL_SHIFT))

#define DMA_REQ_TIMER0_CH0_3         ((GD32_DMA_SUBPERI0 << PERIPH_SHIFT) | ((GD32_DMA_CH6 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_TIMER0_CH1_2         ((GD32_DMA_SUBPERI0 << PERIPH_SHIFT) | ((GD32_DMA_CH6 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_TIMER0_CH2_1         ((GD32_DMA_SUBPERI0 << PERIPH_SHIFT) | ((GD32_DMA_CH6 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_SPI5_RX              ((GD32_DMA_SUBPERI1 << PERIPH_SHIFT) | ((GD32_DMA_CH6 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_SDIO_2               ((GD32_DMA_SUBPERI4 << PERIPH_SHIFT) | ((GD32_DMA_CH6 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_USART5_TX_1          ((GD32_DMA_SUBPERI5 << PERIPH_SHIFT) | ((GD32_DMA_CH6 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_TIMER0_CH2_2         ((GD32_DMA_SUBPERI6 << PERIPH_SHIFT) | ((GD32_DMA_CH6 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_SPI4_TX_2            ((GD32_DMA_SUBPERI7 << PERIPH_SHIFT) | ((GD32_DMA_CH6 + 8) << CHANNEL_SHIFT))

#define DMA_REQ_DCI_2                ((GD32_DMA_SUBPERI1 << PERIPH_SHIFT) | ((GD32_DMA_CH7 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_USART0_TX            ((GD32_DMA_SUBPERI4 << PERIPH_SHIFT) | ((GD32_DMA_CH7 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_USART5_TX_2          ((GD32_DMA_SUBPERI5 << PERIPH_SHIFT) | ((GD32_DMA_CH7 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_TIMER7_CH3           ((GD32_DMA_SUBPERI7 << PERIPH_SHIFT) | ((GD32_DMA_CH7 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_TIMER7_TG            ((GD32_DMA_SUBPERI7 << PERIPH_SHIFT) | ((GD32_DMA_CH7 + 8) << CHANNEL_SHIFT))
#define DMA_REQ_TIMER7_CMT           ((GD32_DMA_SUBPERI7 << PERIPH_SHIFT) | ((GD32_DMA_CH7 + 8) << CHANNEL_SHIFT))

#endif /* __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_DMA_H */
