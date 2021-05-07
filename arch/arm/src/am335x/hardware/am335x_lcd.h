/****************************************************************************
 * arch/arm/src/am335x/hardware/am335x_lcd.h
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

#ifndef __ARCH_ARM_SRC_AM335X_HARDWARE_AM335X_LCD_H
#define __ARCH_ARM_SRC_AM335X_HARDWARE_AM335X_LCD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/am335x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define AM335X_LCD_PID_OFFSET                   0x0000
#define AM335X_LCD_CTRL_OFFSET                  0x0004
#define AM335X_LCD_LIDD_CTRL_OFFSET             0x000c
#define AM335X_LCD_LIDD_CS0_CONF_OFFSET         0x0010
#define AM335X_LCD_LIDD_CS0_ADDR_OFFSET         0x0014
#define AM335X_LCD_LIDD_CS0_DATA_OFFSET         0x0018
#define AM335X_LCD_LIDD_CS1_CONF_OFFSET         0x001c
#define AM335X_LCD_LIDD_CS1_ADDR_OFFSET         0x0020
#define AM335X_LCD_LIDD_CS1_DATA_OFFSET         0x0024
#define AM335X_LCD_RASTER_CTRL_OFFSET           0x0028
#define AM335X_LCD_RASTER_TIMING_0_OFFSET       0x002c
#define AM335X_LCD_RASTER_TIMING_1_OFFSET       0x0030
#define AM335X_LCD_RASTER_TIMING_2_OFFSET       0x0034
#define AM335X_LCD_RASTER_SUBPANEL_OFFSET       0x0038
#define AM335X_LCD_RASTER_SUBPANEL2_OFFSET      0x003c
#define AM335X_LCD_DMA_CTRL_OFFSET              0x0040
#define AM335X_LCD_DMA_FB0_BASE_OFFSET          0x0044
#define AM335X_LCD_DMA_FB0_CEIL_OFFSET          0x0048
#define AM335X_LCD_DMA_FB1_BASE_OFFSET          0x004c
#define AM335X_LCD_DMA_FB1_CEIL_OFFSET          0x0050
#define AM335X_LCD_SYSC_OFFSET                  0x0054
#define AM335X_LCD_IRQ_STAT_RAW_OFFSET          0x0058
#define AM335X_LCD_IRQ_STAT_OFFSET              0x005c
#define AM335X_LCD_IRQ_EN_SET_OFFSET            0x0060
#define AM335X_LCD_IRQ_EN_CLEAR_OFFSET          0x0064
#define AM335X_LCD_END_INT_OFFSET               0x0068
#define AM335X_LCD_CLKC_ENABLE_OFFSET           0x006c
#define AM335X_LCD_CLKC_RESET_OFFSET            0x0070

#define AM335X_LCD_LIDD_CS_CONF_OFFSET(n)       (0x0010 + (unsigned int)(n) * 0x0c)
#define AM335X_LCD_LIDD_CS_ADDR_OFFSET(n)       (0x0014 + (unsigned int)(n) * 0x0c)
#define AM335X_LCD_LIDD_CS_DATA_OFFSET(n)       (0x0018 + (unsigned int)(n) * 0x0c)
#define AM335X_LCD_DMA_FB_BASE_OFFSET(n)        (0x0044 + (unsigned int)(n) * 0x08)
#define AM335X_LCD_DMA_FB_CEIL_OFFSET(n)        (0x0048 + (unsigned int)(n) * 0x08)

/* Register virtual addresses ***********************************************/

#define AM335X_LCD_PID                          (AM335X_LCD_VADDR + AM335X_LCD_PID_OFFSET)
#define AM335X_LCD_CTRL                         (AM335X_LCD_VADDR + AM335X_LCD_CTRL_OFFSET)
#define AM335X_LCD_LIDD_CTRL                    (AM335X_LCD_VADDR + AM335X_LCD_LIDD_CTRL_OFFSET)
#define AM335X_LCD_LIDD_CS0_CONF                (AM335X_LCD_VADDR + AM335X_LCD_LIDD_CS0_CONF_OFFSET)
#define AM335X_LCD_LIDD_CS0_ADDR                (AM335X_LCD_VADDR + AM335X_LCD_LIDD_CS0_ADDR_OFFSET)
#define AM335X_LCD_LIDD_CS0_DATA                (AM335X_LCD_VADDR + AM335X_LCD_LIDD_CS0_DATA_OFFSET)
#define AM335X_LCD_LIDD_CS1_CONF                (AM335X_LCD_VADDR + AM335X_LCD_LIDD_CS1_CONF_OFFSET)
#define AM335X_LCD_LIDD_CS1_ADDR                (AM335X_LCD_VADDR + AM335X_LCD_LIDD_CS1_ADDR_OFFSET)
#define AM335X_LCD_LIDD_CS1_DATA                (AM335X_LCD_VADDR + AM335X_LCD_LIDD_CS1_DATA_OFFSET)
#define AM335X_LCD_RASTER_CTRL                  (AM335X_LCD_VADDR + AM335X_LCD_RASTER_CTRL_OFFSET)
#define AM335X_LCD_RASTER_TIMING_0              (AM335X_LCD_VADDR + AM335X_LCD_RASTER_TIMING_0_OFFSET)
#define AM335X_LCD_RASTER_TIMING_1              (AM335X_LCD_VADDR + AM335X_LCD_RASTER_TIMING_1_OFFSET)
#define AM335X_LCD_RASTER_TIMING_2              (AM335X_LCD_VADDR + AM335X_LCD_RASTER_TIMING_2_OFFSET)
#define AM335X_LCD_RASTER_SUBPANEL              (AM335X_LCD_VADDR + AM335X_LCD_RASTER_SUBPANEL_OFFSET)
#define AM335X_LCD_RASTER_SUBPANEL2             (AM335X_LCD_VADDR + AM335X_LCD_RASTER_SUBPANEL2_OFFSET)
#define AM335X_LCD_DMA_CTRL                     (AM335X_LCD_VADDR + AM335X_LCD_DMA_CTRL_OFFSET)
#define AM335X_LCD_DMA_FB0_BASE                 (AM335X_LCD_VADDR + AM335X_LCD_DMA_FB0_BASE_OFFSET)
#define AM335X_LCD_DMA_FB0_CEIL                 (AM335X_LCD_VADDR + AM335X_LCD_DMA_FB0_CEIL_OFFSET)
#define AM335X_LCD_DMA_FB1_BASE                 (AM335X_LCD_VADDR + AM335X_LCD_DMA_FB1_BASE_OFFSET)
#define AM335X_LCD_DMA_FB1_CEIL                 (AM335X_LCD_VADDR + AM335X_LCD_DMA_FB1_CEIL_OFFSET)
#define AM335X_LCD_SYSC                         (AM335X_LCD_VADDR + AM335X_LCD_SYSC_OFFSET)
#define AM335X_LCD_IRQ_STAT_RAW                 (AM335X_LCD_VADDR + AM335X_LCD_IRQ_STAT_RAW_OFFSET)
#define AM335X_LCD_IRQ_STAT                     (AM335X_LCD_VADDR + AM335X_LCD_IRQ_STAT_OFFSET)
#define AM335X_LCD_IRQ_EN_SET                   (AM335X_LCD_VADDR + AM335X_LCD_IRQ_EN_SET_OFFSET)
#define AM335X_LCD_IRQ_EN_CLEAR                 (AM335X_LCD_VADDR + AM335X_LCD_IRQ_EN_CLEAR_OFFSET)
#define AM335X_LCD_END_INT                      (AM335X_LCD_VADDR + AM335X_LCD_END_INT_OFFSET)
#define AM335X_LCD_CLKC_ENABLE                  (AM335X_LCD_VADDR + AM335X_LCD_CLKC_ENABLE_OFFSET)
#define AM335X_LCD_CLKC_RESET                   (AM335X_LCD_VADDR + AM335X_LCD_CLKC_RESET_OFFSET)

#define AM335X_LCD_LIDD_CS_CONF(n)              (AM335X_LCD_VADDR + AM335X_LCD_LIDD_CS_CONF_OFFSET(n))
#define AM335X_LCD_LIDD_CS_ADDR(n)              (AM335X_LCD_VADDR + AM335X_LCD_LIDD_CS_ADDR_OFFSET(n))
#define AM335X_LCD_LIDD_CS_DATA(n)              (AM335X_LCD_VADDR + AM335X_LCD_LIDD_CS_DATA_OFFSET(n))
#define AM335X_LCD_DMA_FB_BASE(n)               (AM335X_LCD_VADDR + AM335X_LCD_DMA_FB_BASE_OFFSET(n))
#define AM335X_LCD_DMA_FB_CEIL(n)               (AM335X_LCD_VADDR + AM335X_LCD_DMA_FB_CEIL_OFFSET(n))

/* Register bit field definitions *******************************************/

#define LCD_CTRL_MODE_SEL                       (1 << 0)  /* Bit 0:  LCD Mode select */
#  define LCD_CTRL_MODE_LIDD                    (0)
#  define LCD_CTRL_MODE_RASTER                  LCD_CTRL_MODE_SEL
#define LCD_CTRL_AUTO_UFLOW_RESTART             (1 << 1)  /* Bit 1:  Underflow restart selection */
#define LCD_CTRL_CLKDIV_SHIFT                   (8)       /* Bits 8-15: Clock divisor */
#define LCD_CTRL_CLKDIV_MASK                    (255 << LCD_CTRL_CLKDIV_SHIFT)

#define LCD_LIDD_CTRL_MODE_SEL_SHIFT            (0)  /* Bits 0-2:  LIDD Mode Select */
#define LCD_LIDD_CTRL_MODE_SEL_MASK             (7 << LCD_LIDD_CTRL_MODE_SEL_SHIFT)
#  define LCD_LIDD_CTRL_SYNC_MPU68              (0 << LCD_LIDD_CTRL_MODE_SEL_SHIFT) /* Sync MPU68 */
#  define LCD_LIDD_CTRL_ASYNC_MPU68             (1 << LCD_LIDD_CTRL_MODE_SEL_SHIFT) /* Async MPU68 */
#  define LCD_LIDD_CTRL_SYNC_MPU80              (2 << LCD_LIDD_CTRL_MODE_SEL_SHIFT) /* Sync MPU80 */
#  define LCD_LIDD_CTRL_ASYNC_MPU80             (3 << LCD_LIDD_CTRL_MODE_SEL_SHIFT) /* Async MPU80 */
#  define LCD_LIDD_CTRL_HITACHI                 (4 << LCD_LIDD_CTRL_MODE_SEL_SHIFT) /* Hitachi (Async) */

#define LCD_LIDD_CTRL_ALEPOL                    (1 << 3)  /* Bit 3:  Address Latch Enable (ALE) Polarity Control */
#define LCD_LIDD_CTRL_RS_EN_POL                 (1 << 4)  /* Bit 4:  Read Strobe/Direction Polarity Control */
#define LCD_LIDD_CTRL_WS_DIR_POL                (1 << 5)  /* Bit 5:  Write Strobe/Direction Polarity Control */
#define LCD_LIDD_CTRL_CS0_E0_POL                (1 << 6)  /* Bit 6:  Chip Select 0/Enable 0 (Secondary) Polarity Control */
#define LCD_LIDD_CTRL_CS1_E1_POL                (1 << 7)  /* Bit 7:  Chip Select 1/Enable 1 (Secondary) Polarity Control */
#define LCD_LIDD_CTRL_DMA_EN                    (1 << 8)  /* Bit 8:  LIDD DMA Enable */
#define LCD_LIDD_CTRL_DMA_CS0_CS1               (1 << 9)  /* Bit 9:  CS0/CS1 Select for LIDD DMA writes */

#define LCD_LIDD_CS_CONF_TA_SHIFT               (0)  /* Bits 0-1:  Turn-around Access */
#define LCD_LIDD_CS_CONF_TA_MASK                (3 << LCD_LIDD_CS0_CONF_TA_SHIFT)
#define LCD_LIDD_CS_CONF_R_HOLD_SHIFT           (2)  /* Bits 2-5:  Read Strobe Hold cycles */
#define LCD_LIDD_CS_CONF_R_HOLD_MASK            (15 << LCD_LIDD_CS0_CONF_R_HOLD_SHIFT)
#define LCD_LIDD_CS_CONF_R_STROBE_SHIFT         (6)  /* Bits 6-11:  Read Strobe Duration cycles */
#define LCD_LIDD_CS_CONF_R_STROBE_MASK          (63 << LCD_LIDD_CS0_CONF_R_STROBE_SHIFT)
#define LCD_LIDD_CS_CONF_R_SU_SHIFT             (12)  /* Bits 12-16:  Read Strobe Set-Up cycles */
#define LCD_LIDD_CS_CONF_R_SU_MASK              (31 << LCD_LIDD_CS0_CONF_R_SU_SHIFT)
#define LCD_LIDD_CS_CONF_W_HOLD_SHIFT           (17)  /* Bits 17-20:  Write Strobe Hold cycles */
#define LCD_LIDD_CS_CONF_W_HOLD_MASK            (15 << LCD_LIDD_CS0_CONF_W_HOLD_SHIFT)
#define LCD_LIDD_CS_CONF_W_STROBE_SHIFT         (21)  /* Bits 21-26:  Write Strobe Duration cycles */
#define LCD_LIDD_CS_CONF_W_STROBE_MASK          (63 << LCD_LIDD_CS0_CONF_W_STROBE_SHIFT)
#define LCD_LIDD_CS_CONF_W_SU_SHIFT             (27)  /* Bits 27-31:  Write Strobe Set-Up cycles */
#define LCD_LIDD_CS_CONF_W_SU_MASK              (31 << LCD_LIDD_CS0_CONF_W_SU_SHIFT)

#define LCD_LIDD_CS_ADDR_SHIFT                  (0)  /* Bits 0-15:  Address index */
#define LCD_LIDD_CS_ADDR_MASK                   (0xffff << LCD_LIDD_CS_ADDR_SHIFT)

#define LCD_LIDD_CS_DATA_SHIFT                  (0)  /* Bits 0-15:  Data */
#define LCD_LIDD_CS_DATA_MASK                   (0xffff << LCD_LIDD_CS_DATA_SHIFT)

#define LCD_RASTER_CTRL_LCD_EN                  (1 << 0)  /* Bit 0:  LCD Controller Enable */
#define LCD_RASTER_CTRL_LCD_BW                  (1 << 1)  /* Bit 1:  Only Applies for Passive Matrix Panels LCD Monochrome */
#define LCD_RASTER_CTRL_LCD_TFT                 (1 << 7)  /* Bit 7:  Active/Passive or display operation selection */
#define LCD_RASTER_CTRL_RD_ORDER                (1 << 8)  /* Bit 8:  Raster Data Order Select */
#define LCD_RASTER_CTRL_MONO_8B                 (1 << 9)  /* Bit 9:  Mono 8 bit */

#define LCD_RASTER_CTRL_REQDLY_SHIFT            (12)  /* Bits 12-19:  Palette Loading Delay When loading the Palette from DDR */
#define LCD_RASTER_CTRL_REQDLY_MASK             (255 << LCD_RASTER_CTRL_REQDLY_SHIFT)
#define LCD_RASTER_CTRL_PALMODE_SHIFT           (20)  /* Bits 20-21:  Palette Loading Mode */
#define LCD_RASTER_CTRL_PALMODE_MASK            (3 << LCD_RASTER_CTRL_PALMODE_SHIFT)
#  define LCD_RASTER_CTRL_PALETTE_DATA          (0 << LCD_RASTER_CTRL_PALMODE_SHIFT) /* Palette and data loading */
#  define LCD_RASTER_CTRL_PALETTE               (1 << LCD_RASTER_CTRL_PALMODE_SHIFT) /* Palette loading only */
#  define LCD_RASTER_CTRL_DATA                  (2 << LCD_RASTER_CTRL_PALMODE_SHIFT) /* Data loading only For Raw Data (12/16/24 bpp) framebuffers, no palette lookup is employed */

#define LCD_RASTER_CTRL_NIB_MODE                (1 << 22)  /* Bit 22:  Nibble Mode */
#define LCD_RASTER_CTRL_TFT_MAP                 (1 << 23)  /* Bit 23:  TFT Mode Alternate Signal Mapping for Palettized framebuffer */
#define LCD_RASTER_CTRL_STN565                  (1 << 24)  /* Bit 24:  Selects whether the framebuffer format is 16 bpp 565 or 12 bpp. */
#define LCD_RASTER_CTRL_TFT24                   (1 << 25)  /* Bit 25:  24 bit mode */
#define LCD_RASTER_CTRL_TFT24_UNPACKED          (1 << 26)  /* Bit 26:  24 bit Mode Packing */

#define LCD_RASTER_TIMING_0_PPLMSB              (1 << 3)  /* Bit 3:  Pixels-per-line MSB[10] */

#define LCD_RASTER_TIMING_0_PPLLSB_SHIFT        (4)   /* Bits 4-9:  Pixels-per-line LSB */
#define LCD_RASTER_TIMING_0_PPLLSB_MASK         (63 << LCD_RASTER_TIMING_0_PPLLSB_SHIFT)
#define LCD_RASTER_TIMING_0_HSW_SHIFT           (10)  /* Bits 10-15:  Horizontal Sync Pulse Width Lowbits*/
#define LCD_RASTER_TIMING_0_HSW_MASK            (63 << LCD_RASTER_TIMING_0_HSW_SHIFT)
#define LCD_RASTER_TIMING_0_HFP_SHIFT           (16)  /* Bits 16-23:  Horizontal Front Porch Lowbits */
#define LCD_RASTER_TIMING_0_HFP_MASK            (255 << LCD_RASTER_TIMING_0_HFP_SHIFT)
#define LCD_RASTER_TIMING_0_HBP_SHIFT           (24)  /* Bits 24-31:  Horizontal Back Porch Lowbits */
#define LCD_RASTER_TIMING_0_HBP_MASK            (255 << LCD_RASTER_TIMING_0_HBP_SHIFT)

#define LCD_RASTER_TIMING_1_LPP_SHIFT           (0)  /* Bits 0-9:  Lines Per Panel */
#define LCD_RASTER_TIMING_1_LPP_MASK            (255 << LCD_RASTER_TIMING_1_LPP_SHIFT)
#define LCD_RASTER_TIMING_1_VSW_SHIFT           (10)  /* Bits 10-15:  Vertical Sync Width Pulse */
#define LCD_RASTER_TIMING_1_VSW_MASK            (255 << LCD_RASTER_TIMING_1_VSW_SHIFT)
#define LCD_RASTER_TIMING_1_VFP_SHIFT           (16)  /* Bits 16-23:  Vertical Front Porch */
#define LCD_RASTER_TIMING_1_VFP_MASK            (255 << LCD_RASTER_TIMING_1_VFP_SHIFT)
#define LCD_RASTER_TIMING_1_VBP_SHIFT           (24)  /* Bits 24-31:  Vertical Back Porch */
#define LCD_RASTER_TIMING_1_VBP_MASK            (255 << LCD_RASTER_TIMING_1_VBP_SHIFT)

#define LCD_RASTER_TIMING_2_HFP_HBITS_SHIFT     (0)  /* Bits 0-1:  Bits 9:8 of the horizontal front porch field */
#define LCD_RASTER_TIMING_2_HFP_HBITS_MASK      (3 << LCD_RASTER_TIMING_2_HFP_HBITS_SHIFT)
#define LCD_RASTER_TIMING_2_HBP_HBITS_SHIFT     (4)  /* Bits 4-5:  Bits 9:8 of the horizontal back porch field */
#define LCD_RASTER_TIMING_2_HBP_HBITS_MASK      (3 << LCD_RASTER_TIMING_2_HBP_HBITS_SHIFT)
#define LCD_RASTER_TIMING_2_ACB_SHIFT           (8)  /* Bits 8-15:  AC Bias Pin Frequency */
#define LCD_RASTER_TIMING_2_ACB_MASK            (255 << LCD_RASTER_TIMING_2_ACB_SHIFT)
#define LCD_RASTER_TIMING_2_ACBI_SHIFT          (16)  /* Bits 16-19:  AC Bias Pins Transitions per Interrupt */
#define LCD_RASTER_TIMING_2_ACBI_MASK           (15  << LCD_RASTER_TIMING_2_ACBI_SHIFT)
#define LCD_RASTER_TIMING_2_IVS                 (1 << 20)  /* Bit 20:  Invert Vsync */
#define LCD_RASTER_TIMING_2_IHS                 (1 << 21)  /* Bit 21:  Invert Hsync */
#define LCD_RASTER_TIMING_2_IPC                 (1 << 22)  /* Bit 22:  Invert Pixel Clock */
#define LCD_RASTER_TIMING_2_IEO                 (1 << 23)  /* Bit 23:  Invert Output Enable */
#define LCD_RASTER_TIMING_2_PHSVS_RF            (1 << 24)  /* Bit 24:  Program HSYNC/VSYNC Rise or Fall */
#define LCD_RASTER_TIMING_2_PHSVS_ON            (1 << 25)  /* Bit 25:  Hsync/Vsync Pixel Clock Control On/Off */
#define LCD_RASTER_TIMING_2_LPP_B10_SHIFT       (26)       /* Bit 26:  Lines Per Panel Bit 10 */

#define LCD_RASTER_TIMING_2_LPP_B10_MASK        (1 << LCD_RASTER_TIMING_2_LPP_B10_SHIFT)
#define LCD_RASTER_TIMING_2_HSW_HBITS_SHIFT     (27)  /* Bits 27-30:  Bits 9 to 6 of the horizontal sync width field */
#define LCD_RASTER_TIMING_2_HSW_HBITS_MASK      (15 << LCD_RASTER_TIMING_2_HSW_HBITS_SHITF)

#define LCD_RASTER_SUBPANEL_DPDLSB_SHIFT        (0)  /* Bits 0-15:  Default Pixel Data LSB */
#define LCD_RASTER_SUBPANEL_DPDLSB_MASK         (65535 << LCD_RASTER_SUBPANEL_DPDLSB_SHIFT)
#define LCD_RASTER_SUBPANEL_LPPT_SHIFT          (16)  /* Bits 16-25:  Line Per Panel Threshold */
#define LCD_RASTER_SUBPANEL_LPPT_MASK           (1023 << LCD_RASTER_SUBPANEL_LPPT_SHIFT)
#define LCD_RASTER_SUBPANEL_HOLS                (1 << 29)  /* Bit 29:  High or Low Signal */
#define LCD_RASTER_SUBPANEL_SPEN                (1 << 31)  /* Bit 31:  Sub Panel Enable */

#define LCD_RASTER_SUBPANEL2_DPDMSB_SHIFT       (0)  /* Bits 0-7:  Default Pixel Data MSB */
#define LCD_RASTER_SUBPANEL2_DPDMSB_MASK        (255 << LCD_RASTER_SUBPANEL2_DPDMSB_SHIFT)
#define LCD_RASTER_SUBPANEL2_LPPT_B10           (1 << 8)  /* Bit 8:  Lines Per Panel Threshold Bit 10 */

#define LCD_DMA_CTRL_FRAME_MODE                 (1 << 0)  /* Bit 0:  Frame Mode */
#define LCD_DMA_CTRL_BE                         (1 << 1)  /* Bit 1:  Big Endian Enable */
#define LCD_DMA_CTRL_BYTE_SWAP                  (1 << 3)  /* Bit 3:  Bytelane Ordering */

#define LCD_DMA_CTRL_BURST_SIZE_SHIFT           (4)  /* Bits 4-6:  Burst Size setting for DMA transfers */
#define LCD_DMA_CTRL_BURST_SIZE_MASK            (7 << LCD_DMA_CTRL_BURST_SIZE_SHIFT)
#  define LCD_DMA_CTRL_BURST_SIZE_1             (0 << LCD_DMA_CTRL_BURST_SIZE_SHIFT) /* Burst size of 1 */
#  define LCD_DMA_CTRL_BURST_SIZE_2             (1 << LCD_DMA_CTRL_BURST_SIZE_SHIFT) /* Burst size of 2 */
#  define LCD_DMA_CTRL_BURST_SIZE_4             (2 << LCD_DMA_CTRL_BURST_SIZE_SHIFT) /* Burst size of 4 */
#  define LCD_DMA_CTRL_BURST_SIZE_8             (3 << LCD_DMA_CTRL_BURST_SIZE_SHIFT) /* Burst size of 8 */
#  define LCD_DMA_CTRL_BURST_SIZE_16            (4 << LCD_DMA_CTRL_BURST_SIZE_SHIFT) /* Burst size of 6 */

#define LCD_DMA_CTRL_TH_FIFO_RDY_SHIFT          (8)  /* Bits 8-10:  DMA FIFO threshold */
#define LCD_DMA_CTRL_TH_FIFO_RDY_MASK           (7 << LCD_DMA_CTRL_TH_FIFO_RDY_SHIFT)
#  define LCD_DMA_CTRL_TH_FIFO_RDY_8            (0 << LCD_DMA_CTRL_TH_FIFO_RDY_MASK) /* 8 words have been loaded */
#  define LCD_DMA_CTRL_TH_FIFO_RDY_16           (1 << LCD_DMA_CTRL_TH_FIFO_RDY_MASK) /* 16 words have been loaded */
#  define LCD_DMA_CTRL_TH_FIFO_RDY_32           (2 << LCD_DMA_CTRL_TH_FIFO_RDY_MASK) /* 32 words have been loaded */
#  define LCD_DMA_CTRL_TH_FIFO_RDY_64           (3 << LCD_DMA_CTRL_TH_FIFO_RDY_MASK) /* 64 words have been loaded */
#  define LCD_DMA_CTRL_TH_FIFO_RDY_128          (4 << LCD_DMA_CTRL_TH_FIFO_RDY_MASK) /* 128 words have been loaded */
#  define LCD_DMA_CTRL_TH_FIFO_RDY_256          (5 << LCD_DMA_CTRL_TH_FIFO_RDY_MASK) /* 256 words have been loaded */
#  define LCD_DMA_CTRL_TH_FIFO_RDY_512          (6 << LCD_DMA_CTRL_TH_FIFO_RDY_MASK) /* 512 words have been loaded */

#define LCD_DMA_CTRL_MASTER_PRIO_SHIFT          (16)  /* Bits 16-18:  Priority for the L3 OCP Master Bus */
#define LCD_DMA_CTRL_MASTER_PRIO_MASK           (7 << LCD_DMA_CTRL_MASTER_PRIO_SHIFT)

#define LCD_DMA_FB_BASE_SHIFT                   (2)  /* Bits 2-31:  Frame Buffer Base Address pointer */
#define LCD_DMA_FB_BASE_MASK                    (0x3fffffff << LCD_DMA_FB_BASE_SHIFT)

#define LCD_DMA_FB_CEIL_SHIFT                   (2)  /* Bits 2-31:  Frame Buffer Ceiling Address pointer */
#define LCD_DMA_FB_CEIL_MASK                    (0x3fffffff << LCD_DMA_FB_BASE_SHIFT)

#define LCD_SYSC_IDLE_SHIFT                     (2)  /* Bits 2-3:  Configuration of the local target state management mode */
#define LCD_SYSC_IDLE_MASK                      (3 << LCD_SYSC_IDLE_SHIFT)
#  define LCD_SYSC_IDLE_FORCE                   (0 << LCD_SYSC_IDLE_SHIFT) /* Force-idle mode */
#  define LCD_SYSC_IDLE_NO                      (1 << LCD_SYSC_IDLE_SHIFT) /* No-idle mode */
#  define LCD_SYSC_IDLE_SMART                   (2 << LCD_SYSC_IDLE_SHIFT) /* Smart-idle mode */

#define LCD_SYSC_STANDBY_SHIFT                  (4)  /* Bits 4-5:  Configuration of the local initiator state management mode */
#define LCD_SYSC_STANDBY_MASK                   (3 << LCD_SYSC_STANDBY_SHIFT)
#  define LCD_SYSC_STANDBY_FORCE                (0 << LCD_SYSC_STANDBY_SHIFT) /* Force-standby mode */
#  define LCD_SYSC_STANDBY_NO                   (1 << LCD_SYSC_STANDBY_SHIFT) /* No-standby mode */
#  define LCD_SYSC_STANDBY_SMART                (2 << LCD_SYSC_STANDBY_SHIFT) /* Smart-standby mode */

#define LCD_IRQ_DONE                            (1 << 0)  /* Bit 0:  Raster or LIDD Frame Done */
#define LCD_IRQ_RR_DONE                         (1 << 1)  /* Bit 1:  Raster Mode Frame Done */
#define LCD_IRQ_SYNC                            (1 << 2)  /* Bit 2:  Frame Synchronization Lost */
#define LCD_IRQ_ACB                             (1 << 3)  /* Bit 3:  For Passive Matrix Panels Only AC Bias Count */
#define LCD_IRQ_FUF                             (1 << 5)  /* Bit 5:  DMA FIFO Underflow */
#define LCD_IRQ_PL                              (1 << 6)  /* Bit 6:  DMA Palette Loaded */
#define LCD_IRQ_EOF0                            (1 << 8)  /* Bit 8:  DMA End-of-Frame 0 */
#define LCD_IRQ_EOF1                            (1 << 9)  /* Bit 9:  DMA End-of-Frame 1 */

#define LCD_CLKC_ENABLE_CORE                    (1 << 0)  /* Bit 0:  Software Clock Enable for the DMA submodule */
#define LCD_CLKC_ENABLE_LIDD                    (1 << 1)  /* Bit 1:  Software Clock Enable for the LIDD submodule */
#define LCD_CLKC_ENABLE_DMA                     (1 << 2)  /* Bit 2:  Software Clock Enable for the Core */

#define LCD_CLKC_RESET_CORE                     (1 << 0)  /* Bit 0:  Software Reset for the Core */
#define LCD_CLKC_RESET_LIDD                     (1 << 1)  /* Bit 1:  Software Reset for the LIDD submodule */
#define LCD_CLKC_RESET_DMA                      (1 << 2)  /* Bit 2:  Software Reset for the DMA submodule */
#define LCD_CLKC_RESET_MAIN                     (1 << 3)  /* Bit 3:  Software Reset for the entire LCD module */

#endif /* __ARCH_ARM_SRC_AM335X_HARDWARE_AM335X_LCD_H */
