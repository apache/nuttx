/****************************************************************************
 * arch/risc-v/src/gd32vw55x/hardware/gd32vw55x_dma.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_DMA_H
#define __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "gd32vw55x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The GD32VW55x has a single DMA controller with 8 channels.  Each channel
 * can be connected to one of 8 sub-peripheral requests.
 */

#define GD32VW55X_NDMA               1   /* One DMA controller */
#define GD32VW55X_NDMA_CHANNELS      8   /* Eight channels */
#define GD32VW55X_NDMA_SUBPERIPHS    8   /* Eight sub-peripherals/channel */

/* Register Offsets *********************************************************/

#define GD32_DMA_INTF0_OFFSET        0x0000  /* Interrupt flag register 0 */
#define GD32_DMA_INTF1_OFFSET        0x0004  /* Interrupt flag register 1 */
#define GD32_DMA_INTC0_OFFSET        0x0008  /* Interrupt flag clear reg 0 */
#define GD32_DMA_INTC1_OFFSET        0x000c  /* Interrupt flag clear reg 1 */

#define GD32_DMA_CH0CTL_OFFSET       0x0010  /* Channel 0 control */
#define GD32_DMA_CH0CNT_OFFSET       0x0014  /* Channel 0 counter */
#define GD32_DMA_CH0PADDR_OFFSET     0x0018  /* Channel 0 periph address */
#define GD32_DMA_CH0M0ADDR_OFFSET    0x001c  /* Channel 0 memory 0 address */
#define GD32_DMA_CH0M1ADDR_OFFSET    0x0020  /* Channel 0 memory 1 address */
#define GD32_DMA_CH0FCTL_OFFSET      0x0024  /* Channel 0 FIFO control */

#define GD32_DMA_CH1CTL_OFFSET       0x0028  /* Channel 1 control */
#define GD32_DMA_CH1CNT_OFFSET       0x002c  /* Channel 1 counter */
#define GD32_DMA_CH1PADDR_OFFSET     0x0030  /* Channel 1 periph address */
#define GD32_DMA_CH1M0ADDR_OFFSET    0x0034  /* Channel 1 memory 0 address */
#define GD32_DMA_CH1M1ADDR_OFFSET    0x0038  /* Channel 1 memory 1 address */
#define GD32_DMA_CH1FCTL_OFFSET      0x003c  /* Channel 1 FIFO control */

#define GD32_DMA_CH2CTL_OFFSET       0x0040  /* Channel 2 control */
#define GD32_DMA_CH2CNT_OFFSET       0x0044  /* Channel 2 counter */
#define GD32_DMA_CH2PADDR_OFFSET     0x0048  /* Channel 2 periph address */
#define GD32_DMA_CH2M0ADDR_OFFSET    0x004c  /* Channel 2 memory 0 address */
#define GD32_DMA_CH2M1ADDR_OFFSET    0x0050  /* Channel 2 memory 1 address */
#define GD32_DMA_CH2FCTL_OFFSET      0x0054  /* Channel 2 FIFO control */

#define GD32_DMA_CH3CTL_OFFSET       0x0058  /* Channel 3 control */
#define GD32_DMA_CH3CNT_OFFSET       0x005c  /* Channel 3 counter */
#define GD32_DMA_CH3PADDR_OFFSET     0x0060  /* Channel 3 periph address */
#define GD32_DMA_CH3M0ADDR_OFFSET    0x0064  /* Channel 3 memory 0 address */
#define GD32_DMA_CH3M1ADDR_OFFSET    0x0068  /* Channel 3 memory 1 address */
#define GD32_DMA_CH3FCTL_OFFSET      0x006c  /* Channel 3 FIFO control */

#define GD32_DMA_CH4CTL_OFFSET       0x0070  /* Channel 4 control */
#define GD32_DMA_CH4CNT_OFFSET       0x0074  /* Channel 4 counter */
#define GD32_DMA_CH4PADDR_OFFSET     0x0078  /* Channel 4 periph address */
#define GD32_DMA_CH4M0ADDR_OFFSET    0x007c  /* Channel 4 memory 0 address */
#define GD32_DMA_CH4M1ADDR_OFFSET    0x0080  /* Channel 4 memory 1 address */
#define GD32_DMA_CH4FCTL_OFFSET      0x0084  /* Channel 4 FIFO control */

#define GD32_DMA_CH5CTL_OFFSET       0x0088  /* Channel 5 control */
#define GD32_DMA_CH5CNT_OFFSET       0x008c  /* Channel 5 counter */
#define GD32_DMA_CH5PADDR_OFFSET     0x0090  /* Channel 5 periph address */
#define GD32_DMA_CH5M0ADDR_OFFSET    0x0094  /* Channel 5 memory 0 address */
#define GD32_DMA_CH5M1ADDR_OFFSET    0x0098  /* Channel 5 memory 1 address */
#define GD32_DMA_CH5FCTL_OFFSET      0x009c  /* Channel 5 FIFO control */

#define GD32_DMA_CH6CTL_OFFSET       0x00a0  /* Channel 6 control */
#define GD32_DMA_CH6CNT_OFFSET       0x00a4  /* Channel 6 counter */
#define GD32_DMA_CH6PADDR_OFFSET     0x00a8  /* Channel 6 periph address */
#define GD32_DMA_CH6M0ADDR_OFFSET    0x00ac  /* Channel 6 memory 0 address */
#define GD32_DMA_CH6M1ADDR_OFFSET    0x00b0  /* Channel 6 memory 1 address */
#define GD32_DMA_CH6FCTL_OFFSET      0x00b4  /* Channel 6 FIFO control */

#define GD32_DMA_CH7CTL_OFFSET       0x00b8  /* Channel 7 control */
#define GD32_DMA_CH7CNT_OFFSET       0x00bc  /* Channel 7 counter */
#define GD32_DMA_CH7PADDR_OFFSET     0x00c0  /* Channel 7 periph address */
#define GD32_DMA_CH7M0ADDR_OFFSET    0x00c4  /* Channel 7 memory 0 address */
#define GD32_DMA_CH7M1ADDR_OFFSET    0x00c8  /* Channel 7 memory 1 address */
#define GD32_DMA_CH7FCTL_OFFSET      0x00cc  /* Channel 7 FIFO control */

/* Channel register block geometry.  The channel register blocks start at
 * offset 0x0010 and every block is 0x18 bytes long.
 */

#define GD32_DMA_CHBASE_OFFSET       0x0010  /* Channel 0 block offset */
#define GD32_DMA_CHSIZE              0x0018  /* Channel block size */

/* Register Addresses *******************************************************/

#define GD32_DMA_INTF0               (GD32VW55X_DMA_BASE + \
                                      GD32_DMA_INTF0_OFFSET)
#define GD32_DMA_INTF1               (GD32VW55X_DMA_BASE + \
                                      GD32_DMA_INTF1_OFFSET)
#define GD32_DMA_INTC0               (GD32VW55X_DMA_BASE + \
                                      GD32_DMA_INTC0_OFFSET)
#define GD32_DMA_INTC1               (GD32VW55X_DMA_BASE + \
                                      GD32_DMA_INTC1_OFFSET)

/* DMA channel x register addresses, x=0..7 */

#define GD32_DMA_CHBASE(ch)          (GD32VW55X_DMA_BASE + \
                                      GD32_DMA_CHBASE_OFFSET + \
                                      GD32_DMA_CHSIZE * (ch))
#define GD32_DMA_CHCTL(ch)           (GD32_DMA_CHBASE(ch) + 0x00)
#define GD32_DMA_CHCNT(ch)           (GD32_DMA_CHBASE(ch) + 0x04)
#define GD32_DMA_CHPADDR(ch)         (GD32_DMA_CHBASE(ch) + 0x08)
#define GD32_DMA_CHM0ADDR(ch)        (GD32_DMA_CHBASE(ch) + 0x0c)
#define GD32_DMA_CHM1ADDR(ch)        (GD32_DMA_CHBASE(ch) + 0x10)
#define GD32_DMA_CHFCTL(ch)          (GD32_DMA_CHBASE(ch) + 0x14)

/* Register Bit Definitions *************************************************/

/* DMA interrupt flag register 0/1 (INTF0/INTF1) and
 * DMA interrupt flag clear register 0/1 (INTC0/INTC1).
 *
 * Both registers hold the flags of 4 channels.  INTF0/INTC0 hold the flags
 * of channels 0-3 and INTF1/INTC1 the flags of channels 4-7.  The per
 * channel flag field is 6 bits wide, see GD32_DMA_FLAG_ADD below.
 */

#define DMA_INTF_FEEIF               (1 << 0)  /* FIFO error/exception */
#define DMA_INTF_SDEIF               (1 << 2)  /* Single data mode error */
#define DMA_INTF_TAEIF               (1 << 3)  /* Transfer access error */
#define DMA_INTF_HTFIF               (1 << 4)  /* Half transfer finish */
#define DMA_INTF_FTFIF               (1 << 5)  /* Full transfer finish */

#define DMA_INTC_FEEIFC              (1 << 0)  /* Clear FIFO error */
#define DMA_INTC_SDEIFC              (1 << 2)  /* Clear single data error */
#define DMA_INTC_TAEIFC              (1 << 3)  /* Clear transfer access err */
#define DMA_INTC_HTFIFC              (1 << 4)  /* Clear half transfer flag */
#define DMA_INTC_FTFIFC              (1 << 5)  /* Clear full transfer flag */

/* Shift the per channel 6-bit flag field to its position in the INTFx/INTCx
 * register.  'ch' must be relative to the register, i.e. 0-3.
 */

#define GD32_DMA_FLAG_ADD(flag, ch) \
  ((uint32_t)(flag) << (((uint32_t)(ch) * 6) + \
                        ((((uint32_t)(ch)) >> 1) & 1) * 4))

/* DMA channel x control register (CHxCTL), x=0..7 */

#define DMA_CHXCTL_CHEN              (1 << 0)  /* Bit 0: Channel enable */
#define DMA_CHXCTL_SDEIE             (1 << 1)  /* Bit 1: Single data mode
                                                * exception interrupt en */
#define DMA_CHXCTL_TAEIE             (1 << 2)  /* Bit 2: Transfer access
                                                * error interrupt enable */
#define DMA_CHXCTL_HTFIE             (1 << 3)  /* Bit 3: Half transfer
                                                * finish interrupt enable */
#define DMA_CHXCTL_FTFIE             (1 << 4)  /* Bit 4: Full transfer
                                                * finish interrupt enable */
#define DMA_CHXCTL_TFCS              (1 << 5)  /* Bit 5: Transfer flow
                                                * controller select */

#define DMA_CHXCTL_TM_SHIFT          (6)       /* Bits 6-7: Transfer mode */
#define DMA_CHXCTL_TM_MASK           (3 << DMA_CHXCTL_TM_SHIFT)
#define DMA_CHXCTL_TM(n)             ((uint32_t)(n) << DMA_CHXCTL_TM_SHIFT)
#  define DMA_PERIPH_TO_MEMORY       DMA_CHXCTL_TM(0)  /* Periph to memory */
#  define DMA_MEMORY_TO_PERIPH       DMA_CHXCTL_TM(1)  /* Memory to periph */
#  define DMA_MEMORY_TO_MEMORY       DMA_CHXCTL_TM(2)  /* Memory to memory */

#define DMA_CHXCTL_CMEN              (1 << 8)  /* Bit 8: Circulation mode */
#define DMA_CHXCTL_PNAGA             (1 << 9)  /* Bit 9: Peripheral next
                                                * address generation algo */
#define DMA_CHXCTL_MNAGA             (1 << 10) /* Bit 10: Memory next
                                                * address generation algo */

#define DMA_CHXCTL_PWIDTH_SHIFT      (11)      /* Bits 11-12: Peripheral
                                                * transfer width */
#define DMA_CHXCTL_PWIDTH_MASK       (3 << DMA_CHXCTL_PWIDTH_SHIFT)
#define DMA_CHXCTL_PWIDTH(n) \
  ((uint32_t)(n) << DMA_CHXCTL_PWIDTH_SHIFT)
#  define DMA_PERIPH_WIDTH_8BIT      DMA_CHXCTL_PWIDTH(0)
#  define DMA_PERIPH_WIDTH_16BIT     DMA_CHXCTL_PWIDTH(1)
#  define DMA_PERIPH_WIDTH_32BIT     DMA_CHXCTL_PWIDTH(2)

#define DMA_CHXCTL_MWIDTH_SHIFT      (13)      /* Bits 13-14: Memory
                                                * transfer width */
#define DMA_CHXCTL_MWIDTH_MASK       (3 << DMA_CHXCTL_MWIDTH_SHIFT)
#define DMA_CHXCTL_MWIDTH(n) \
  ((uint32_t)(n) << DMA_CHXCTL_MWIDTH_SHIFT)
#  define DMA_MEMORY_WIDTH_8BIT      DMA_CHXCTL_MWIDTH(0)
#  define DMA_MEMORY_WIDTH_16BIT     DMA_CHXCTL_MWIDTH(1)
#  define DMA_MEMORY_WIDTH_32BIT     DMA_CHXCTL_MWIDTH(2)

#define DMA_CHXCTL_PAIF              (1 << 15) /* Bit 15: Peripheral address
                                                * increment fixed */

#define DMA_CHXCTL_PRIO_SHIFT        (16)      /* Bits 16-17: Priority */
#define DMA_CHXCTL_PRIO_MASK         (3 << DMA_CHXCTL_PRIO_SHIFT)
#define DMA_CHXCTL_PRIO(n)           ((uint32_t)(n) << DMA_CHXCTL_PRIO_SHIFT)
#  define DMA_PRIORITY_LOW           DMA_CHXCTL_PRIO(0)
#  define DMA_PRIORITY_MEDIUM        DMA_CHXCTL_PRIO(1)
#  define DMA_PRIORITY_HIGH          DMA_CHXCTL_PRIO(2)
#  define DMA_PRIORITY_ULTRA_HIGH    DMA_CHXCTL_PRIO(3)

#define DMA_CHXCTL_SBMEN             (1 << 18) /* Bit 18: Switch-buffer
                                                * mode enable */
#define DMA_CHXCTL_MBS               (1 << 19) /* Bit 19: Memory buffer
                                                * select */

#define DMA_CHXCTL_PBURST_SHIFT      (21)      /* Bits 21-22: Peripheral
                                                * burst type */
#define DMA_CHXCTL_PBURST_MASK       (3 << DMA_CHXCTL_PBURST_SHIFT)
#define DMA_CHXCTL_PBURST(n) \
  ((uint32_t)(n) << DMA_CHXCTL_PBURST_SHIFT)
#  define DMA_PERIPH_BURST_SINGLE    DMA_CHXCTL_PBURST(0)
#  define DMA_PERIPH_BURST_4_BEAT    DMA_CHXCTL_PBURST(1)
#  define DMA_PERIPH_BURST_8_BEAT    DMA_CHXCTL_PBURST(2)
#  define DMA_PERIPH_BURST_16_BEAT   DMA_CHXCTL_PBURST(3)

#define DMA_CHXCTL_MBURST_SHIFT      (23)      /* Bits 23-24: Memory burst
                                                * type */
#define DMA_CHXCTL_MBURST_MASK       (3 << DMA_CHXCTL_MBURST_SHIFT)
#define DMA_CHXCTL_MBURST(n) \
  ((uint32_t)(n) << DMA_CHXCTL_MBURST_SHIFT)
#  define DMA_MEMORY_BURST_SINGLE    DMA_CHXCTL_MBURST(0)
#  define DMA_MEMORY_BURST_4_BEAT    DMA_CHXCTL_MBURST(1)
#  define DMA_MEMORY_BURST_8_BEAT    DMA_CHXCTL_MBURST(2)
#  define DMA_MEMORY_BURST_16_BEAT   DMA_CHXCTL_MBURST(3)

#define DMA_CHXCTL_PERIEN_SHIFT      (25)      /* Bits 25-27: Peripheral
                                                * (sub-peripheral) select */
#define DMA_CHXCTL_PERIEN_MASK       (7 << DMA_CHXCTL_PERIEN_SHIFT)
#define DMA_CHXCTL_PERIEN(n) \
  ((uint32_t)(n) << DMA_CHXCTL_PERIEN_SHIFT)
#  define DMA_PERIPH_0_SELECT        DMA_CHXCTL_PERIEN(0)
#  define DMA_PERIPH_1_SELECT        DMA_CHXCTL_PERIEN(1)
#  define DMA_PERIPH_2_SELECT        DMA_CHXCTL_PERIEN(2)
#  define DMA_PERIPH_3_SELECT        DMA_CHXCTL_PERIEN(3)
#  define DMA_PERIPH_4_SELECT        DMA_CHXCTL_PERIEN(4)
#  define DMA_PERIPH_5_SELECT        DMA_CHXCTL_PERIEN(5)
#  define DMA_PERIPH_6_SELECT        DMA_CHXCTL_PERIEN(6)
#  define DMA_PERIPH_7_SELECT        DMA_CHXCTL_PERIEN(7)

/* DMA channel x counter register (CHxCNT), x=0..7 */

#define DMA_CHXCNT_CNT_SHIFT         (0)       /* Bits 0-15: Counter */
#define DMA_CHXCNT_CNT_MASK          (0xffff << DMA_CHXCNT_CNT_SHIFT)

/* DMA channel x peripheral base address register (CHxPADDR), x=0..7 */

#define DMA_CHXPADDR_PADDR_MASK      (0xffffffff)

/* DMA channel x memory 0/1 base address register (CHxM0ADDR/CHxM1ADDR) */

#define DMA_CHXM0ADDR_M0ADDR_MASK    (0xffffffff)
#define DMA_CHXM1ADDR_M1ADDR_MASK    (0xffffffff)

/* DMA channel x FIFO control register (CHxFCTL), x=0..7 */

#define DMA_CHXFCTL_FCCV_SHIFT       (0)       /* Bits 0-1: FIFO counter
                                                * critical value */
#define DMA_CHXFCTL_FCCV_MASK        (3 << DMA_CHXFCTL_FCCV_SHIFT)
#define DMA_CHXFCTL_FCCV(n)          ((uint32_t)(n) << DMA_CHXFCTL_FCCV_SHIFT)
#  define DMA_FIFO_1_WORD            DMA_CHXFCTL_FCCV(0)
#  define DMA_FIFO_2_WORD            DMA_CHXFCTL_FCCV(1)
#  define DMA_FIFO_3_WORD            DMA_CHXFCTL_FCCV(2)
#  define DMA_FIFO_4_WORD            DMA_CHXFCTL_FCCV(3)

#define DMA_CHXFCTL_MDMEN            (1 << 2)  /* Bit 2: Multi data mode */
#define DMA_CHXFCTL_FCNT_SHIFT       (3)       /* Bits 3-5: FIFO counter */
#define DMA_CHXFCTL_FCNT_MASK        (7 << DMA_CHXFCTL_FCNT_SHIFT)
#define DMA_CHXFCTL_FEEIE            (1 << 7)  /* Bit 7: FIFO exception
                                                * interrupt enable */

/* Reset values of the DMA channel registers */

#define DMA_CHCTL_RESET_VALUE        (0x00000000)
#define DMA_CHCNT_RESET_VALUE        (0x00000000)
#define DMA_CHPADDR_RESET_VALUE      (0x00000000)
#define DMA_CHMADDR_RESET_VALUE      (0x00000000)
#define DMA_CHFCTL_RESET_VALUE       (0x00000021)
#define DMA_CHINTF_RESET_VALUE       (0x0000003d)

/* Configuration selections used by struct gd32_dma_config_s ****************/

/* Peripheral address generation algorithm */

#define DMA_PERIPH_INCREASE_ENABLE   (0)  /* Peripheral address increments */
#define DMA_PERIPH_INCREASE_DISABLE  (1)  /* Peripheral address is fixed */
#define DMA_PERIPH_INCREASE_FIX      (2)  /* Increment step is fixed to the
                                           * memory transfer width */

/* Memory address generation algorithm */

#define DMA_MEMORY_INCREASE_ENABLE   (0)  /* Memory address increments */
#define DMA_MEMORY_INCREASE_DISABLE  (1)  /* Memory address is fixed */

/* DMA circular mode.  NOTE that, unlike the GD32F4 header, the values match
 * the GigaDevice SPL so that a zeroed configuration means "not circular".
 */

#define DMA_CIRCULAR_MODE_DISABLE    (0)  /* Circular mode disabled */
#define DMA_CIRCULAR_MODE_ENABLE     (1)  /* Circular mode enabled */

/* DMA transfer width selection */

#define DMA_WIDTH_8BITS_SELECT       (0)  /* Select 8 bits width */
#define DMA_WIDTH_16BITS_SELECT      (1)  /* Select 16 bits width */
#define DMA_WIDTH_32BITS_SELECT      (2)  /* Select 32 bits width */

/* DMA priority level selection */

#define DMA_PRIO_LOW_SELECT          (0)  /* Select low priority level */
#define DMA_PRIO_MEDIUM_SELECT       (1)  /* Select medium priority level */
#define DMA_PRIO_HIGH_SELECT         (2)  /* Select high priority level */
#define DMA_PRIO_ULTRA_HIGHSELECT    (3)  /* Select ultra high priority */

/* DMA channel select */

#define GD32_DMA_CH0                 (0)  /* DMA channel 0 */
#define GD32_DMA_CH1                 (1)  /* DMA channel 1 */
#define GD32_DMA_CH2                 (2)  /* DMA channel 2 */
#define GD32_DMA_CH3                 (3)  /* DMA channel 3 */
#define GD32_DMA_CH4                 (4)  /* DMA channel 4 */
#define GD32_DMA_CH5                 (5)  /* DMA channel 5 */
#define GD32_DMA_CH6                 (6)  /* DMA channel 6 */
#define GD32_DMA_CH7                 (7)  /* DMA channel 7 */

/* DMA sub-peripheral select */

#define GD32_DMA_SUBPERI0            (0)  /* DMA sub-peripheral 0 */
#define GD32_DMA_SUBPERI1            (1)  /* DMA sub-peripheral 1 */
#define GD32_DMA_SUBPERI2            (2)  /* DMA sub-peripheral 2 */
#define GD32_DMA_SUBPERI3            (3)  /* DMA sub-peripheral 3 */
#define GD32_DMA_SUBPERI4            (4)  /* DMA sub-peripheral 4 */
#define GD32_DMA_SUBPERI5            (5)  /* DMA sub-peripheral 5 */
#define GD32_DMA_SUBPERI6            (6)  /* DMA sub-peripheral 6 */
#define GD32_DMA_SUBPERI7            (7)  /* DMA sub-peripheral 7 */

/* DMA peripheral request encoding ******************************************/

/* The 'periph_req' argument of gd32_dma_channel_alloc() is a bit encoded
 * value that holds both the channel number and the sub-peripheral number.
 * Since the GD32VW55x has a single DMA controller with 8 channels, 3 bits
 * are enough to hold the channel number:
 *
 *   Bits 0-2: DMA channel number (0-7)
 *   Bits 3-5: DMA sub-peripheral number (0-7)
 *   Bits 6-7: Reserved
 */

#define CHANNEL_SHIFT                (0)
#define CHANNEL_MASK                 (7)
#define PERIPH_SHIFT                 (3)
#define PERIPH_MASK                  (7)

#define GD32_DMA_REQUEST(subperi, channel) \
  ((uint8_t)((((subperi) & PERIPH_MASK) << PERIPH_SHIFT) | \
             (((channel) & CHANNEL_MASK) << CHANNEL_SHIFT)))

/* Peripheral requests to the DMA controller.
 *
 * Only the requests that are used by the peripherals that are supported by
 * this port are listed here.  Any other request of the peripheral request
 * mapping table of the GD32VW55x user manual can be encoded with the
 * GD32_DMA_REQUEST() macro above.
 */

#define DMA_REQ_UART1_RX      GD32_DMA_REQUEST(GD32_DMA_SUBPERI4, \
                                               GD32_DMA_CH0)
#define DMA_REQ_UART1_TX      GD32_DMA_REQUEST(GD32_DMA_SUBPERI4, \
                                               GD32_DMA_CH1)
#define DMA_REQ_USART0_RX     GD32_DMA_REQUEST(GD32_DMA_SUBPERI4, \
                                               GD32_DMA_CH2)
#define DMA_REQ_UART2_RX      GD32_DMA_REQUEST(GD32_DMA_SUBPERI4, \
                                               GD32_DMA_CH5)
#define DMA_REQ_UART2_TX      GD32_DMA_REQUEST(GD32_DMA_SUBPERI4, \
                                               GD32_DMA_CH6)
#define DMA_REQ_USART0_TX     GD32_DMA_REQUEST(GD32_DMA_SUBPERI4, \
                                               GD32_DMA_CH7)

#define DMA_REQ_SPI_RX        GD32_DMA_REQUEST(GD32_DMA_SUBPERI3, \
                                               GD32_DMA_CH2)
#define DMA_REQ_SPI_TX        GD32_DMA_REQUEST(GD32_DMA_SUBPERI3, \
                                               GD32_DMA_CH3)
#define DMA_REQ_I2S_ADD_RX    GD32_DMA_REQUEST(GD32_DMA_SUBPERI3, \
                                               GD32_DMA_CH0)
#define DMA_REQ_I2S_ADD_TX    GD32_DMA_REQUEST(GD32_DMA_SUBPERI3, \
                                               GD32_DMA_CH4)

#define DMA_REQ_QSPI_1        GD32_DMA_REQUEST(GD32_DMA_SUBPERI5, \
                                               GD32_DMA_CH0)
#define DMA_REQ_QSPI_2        GD32_DMA_REQUEST(GD32_DMA_SUBPERI5, \
                                               GD32_DMA_CH1)

#define DMA_REQ_HAU_IN        GD32_DMA_REQUEST(GD32_DMA_SUBPERI2, \
                                               GD32_DMA_CH7)

#endif /* __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_DMA_H */
