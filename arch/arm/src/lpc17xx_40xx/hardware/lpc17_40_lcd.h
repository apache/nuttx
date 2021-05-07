/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/hardware/lpc17_40_lcd.h
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

#ifndef __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_LCD_H
#define __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_LCD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/lpc17_40_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define LPC17_40_LCD_TIMH_OFFSET            (0x0000) /* Horizontal Timing Control register */
#define LPC17_40_LCD_TIMV_OFFSET            (0x0004) /* Vertical Timing Control register */
#define LPC17_40_LCD_POL_OFFSET             (0x0008) /* Clock & Signal Polarity Control register */
#define LPC17_40_LCD_LE_OFFSET              (0x000c) /* Line End Control register */
#define LPC17_40_LCD_UPBASE_OFFSET          (0x0010) /* Upper Panel Frame Base Address register */
#define LPC17_40_LCD_LPBASE_OFFSET          (0x0014) /* Lower Panel Frame Base Address register */
#define LPC17_40_LCD_CTRL_OFFSET            (0x0018) /* LCD Control register */
#define LPC17_40_LCD_INTMSK_OFFSET          (0x001c) /* Interrupt Mask register */
#define LPC17_40_LCD_INTRAW_OFFSET          (0x0020) /* Raw Interrupt Status register */
#define LPC17_40_LCD_INTSTAT_OFFSET         (0x0024) /* Masked Interrupt Status register */
#define LPC17_40_LCD_INTCLR_OFFSET          (0x0028) /* Interrupt Clear register */
#define LPC17_40_LCD_UPCURR_OFFSET          (0x002c) /* Upper Panel Current Address Value register */
#define LPC17_40_LCD_LPCURR_OFFSET          (0x0030) /* Lower Panel Current Address Value register */

/* 256x16-bit Color Palette registers, n=0-127 */

#define LPC17_40_LCD_PAL_OFFSET(n)          (0x0200 + ((n) << 2))

/* Cursor Image registers, n=0-255 */

#define LPC17_40_LCD_CRSR_IMG_OFFSET(n)     (0x0800 + ((n) << 2))

#define LPC17_40_LCD_CRSR_CRTL_OFFSET       (0x0c00) /* Cursor Control register */
#define LPC17_40_LCD_CRSR_CFG_OFFSET        (0x0c04) /* Cursor Configuration register */
#define LPC17_40_LCD_CRSR_PAL0_OFFSET       (0x0c08) /* Cursor Palette register 0 */
#define LPC17_40_LCD_CRSR_PAL1_OFFSET       (0x0c0c) /* Cursor Palette register 1 */
#define LPC17_40_LCD_CRSR_XY_OFFSET         (0x0c10) /* Cursor XY Position register */
#define LPC17_40_LCD_CRSR_CLIP_OFFSET       (0x0c14) /* Cursor Clip Position register */
#define LPC17_40_LCD_CRSR_INTMSK_OFFSET     (0x0c20) /* Cursor Interrupt Mask register */
#define LPC17_40_LCD_CRSR_INTCLR_OFFSET     (0x0c24) /* Cursor Interrupt Clear register */
#define LPC17_40_LCD_CRSR_INTRAW_OFFSET     (0x0c28) /* Cursor Raw Interrupt Status register */
#define LPC17_40_LCD_CRSR_INTSTAT_OFFSET    (0x0c2c) /* Cursor Masked Interrupt Status register */

/* Register Addresses *******************************************************/

#define LPC17_40_LCD_TIMH                   (LPC17_40_LCD_BASE+LPC17_40_LCD_TIMH_OFFSET)
#define LPC17_40_LCD_TIMV                   (LPC17_40_LCD_BASE+LPC17_40_LCD_TIMV_OFFSET)
#define LPC17_40_LCD_POL                    (LPC17_40_LCD_BASE+LPC17_40_LCD_POL_OFFSET)
#define LPC17_40_LCD_LE                     (LPC17_40_LCD_BASE+LPC17_40_LCD_LE_OFFSET)
#define LPC17_40_LCD_UPBASE                 (LPC17_40_LCD_BASE+LPC17_40_LCD_UPBASE_OFFSET)
#define LPC17_40_LCD_LPBASE                 (LPC17_40_LCD_BASE+LPC17_40_LCD_LPBASE_OFFSET)
#define LPC17_40_LCD_CTRL                   (LPC17_40_LCD_BASE+LPC17_40_LCD_CTRL_OFFSET)
#define LPC17_40_LCD_INTMSK                 (LPC17_40_LCD_BASE+LPC17_40_LCD_INTMSK_OFFSET)
#define LPC17_40_LCD_INTRAW                 (LPC17_40_LCD_BASE+LPC17_40_LCD_INTRAW_OFFSET)
#define LPC17_40_LCD_INTSTAT                (LPC17_40_LCD_BASE+LPC17_40_LCD_INTSTAT_OFFSET)
#define LPC17_40_LCD_INTCLR                 (LPC17_40_LCD_BASE+ LPC17_40_LCD_INTCLR_OFFSET)
#define LPC17_40_LCD_UPCURR                 (LPC17_40_LCD_BASE+LPC17_40_LCD_UPCURR_OFFSET)
#define LPC17_40_LCD_LPCURR                 (LPC17_40_LCD_BASE+LPC17_40_LCD_LPCURR_OFFSET)

#define LPC17_40_LCD_PAL(n)                 (LPC17_40_LCD_BASE+LPC17_40_LCD_PAL_OFFSET(n))
#define LPC17_40_LCD_CRSR_IMG(n)            (LPC17_40_LCD_BASE+LPC17_40_LCD_CRSR_IMG_OFFSET(n))

#define LPC17_40_LCD_CRSR_CRTL              (LPC17_40_LCD_BASE+LPC17_40_LCD_CRSR_CRTL_OFFSET)
#define LPC17_40_LCD_CRSR_CFG               (LPC17_40_LCD_BASE+LPC17_40_LCD_CRSR_CFG_OFFSET)
#define LPC17_40_LCD_CRSR_PAL0              (LPC17_40_LCD_BASE+LPC17_40_LCD_CRSR_PAL0_OFFSET)
#define LPC17_40_LCD_CRSR_PAL1              (LPC17_40_LCD_BASE+LPC17_40_LCD_CRSR_PAL1_OFFSET)
#define LPC17_40_LCD_CRSR_XY                (LPC17_40_LCD_BASE+LPC17_40_LCD_CRSR_XY_OFFSET)
#define LPC17_40_LCD_CRSR_CLIP              (LPC17_40_LCD_BASE+LPC17_40_LCD_CRSR_CLIP_OFFSET)
#define LPC17_40_LCD_CRSR_INTMSK            (LPC17_40_LCD_BASE+LPC17_40_LCD_CRSR_INTMSK_OFFSET)
#define LPC17_40_LCD_CRSR_INTCLR            (LPC17_40_LCD_BASE+LPC17_40_LCD_CRSR_INTCLR_OFFSET)
#define LPC17_40_LCD_CRSR_INTRAW            (LPC17_40_LCD_BASE+LPC17_40_LCD_CRSR_INTRAW_OFFSET)
#define LPC17_40_LCD_CRSR_INTSTAT           (LPC17_40_LCD_BASE+LPC17_40_LCD_CRSR_INTSTAT_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* LCD_TIMH - Horizontal Timing Register */

                                                      /* Bits 0-1: Reserved */
#define LCD_TIMH_PPL_SHIFT                  (2)       /* Bits 2-7: Pixels Per Line - 16-1024ppl */
#define LCD_TIMH_PPL_MASK                   (0x3f << LCD_TIMH_PPL_SHIFT)
#define LCD_TIMH_HSW_SHIFT                  (8)       /* Bits 8-15: Horizontal Sync Pulse Width */
#define LCD_TIMH_HWS_MASK                   (0xff << LCD_TIMH_HSW_SHIFT)
#define LCD_TIMH_HFP_SHIFT                  (16)      /* Bits 16-23: Horizontal Front Porch */
#define LCD_TIMH_HFP_MASK                   (0xff << LCD_TIMH_HFP_SHIFT)
#define LCD_TIMH_HBP_SHIFT                  (24)      /* Bits 24-31: Horizontal Back Porch */
#define LCD_TIMH_HBP_MASK                   (0xff << LCD_TIMH_HBP_SHIFT)

/* LCD_TIMV - Vertical Timing Register */

#define LCD_TIMV_LPP_SHIFT                  (0)       /* Bits 0-9: Lines Per Panel 1-1024 lpp*/
#define LCD_TIMV_LPP_MASK                   (0x3ff << LCD_TIMV_LPP_SHIFT)
#define LCD_TIMV_VSW_SHIFT                  (10)      /* Bits 10-15: Vertical Synch Pulse Width */
#define LCD_TIMV_VSW_MASK                   (0x3f << LCD_TIMV_VSW_SHIFT)
#define LCD_TIMV_VFP_SHIFT                  (16)      /* Bits 16-23: Vertical Front Porch */
#define LCD_TIMV_VFP_MASK                   (0xff << LCD_TIMV_VFP_SHIFT)
#define LCD_TIMV_VBP_SHIFT                  (24)      /* Bits 24-31: Vertical Back Porch */
#define LCD_TIMV_VBP_MASK                   (0xff << LCD_TIMV_VBP_SHIFT)

/* LCD_POL - Clock and Signal Polarity Register */

#define LCD_POL_PCDLO_SHIFT                 (0)       /* Bits 0-4: Lower 5 bits of panel clock divisor */
#define LCD_POL_PCDLO_MASK                  (0x1f << LCD_POL_PCDLO_SHIFT)
#define LCD_POL_CLKSEL                      (1 << 5)  /* Bit 5: Clock select- 0=PCLK, 1=LCD_CLKIN */
#define LCD_POL_ACB_SHIFT                   (6)       /* Bits 6-10: AC bias pin frequency */
#define LCD_POL_ACB_MASK                    (0x1f << LCD_POL_ACB_SHIFT)
#define LCD_POL_IVS                         (1 << 11) /* Bit 11: Invert vertical sync */
#define LCD_POL_IHS                         (1 << 12) /* Bit 12: Invert horizontal sync */
#define LCD_POL_IPC                         (1 << 13) /* Bit 13: Invert panel clock */
#define LCD_POL_IOE                         (1 << 14) /* Bit 14: Invert output enable */
                                                      /* Bit 15: Reserved */
#define LCD_POL_CPL_SHIFT                   (16)      /* Bit 16-25: Clocks per line */
#define LCD_POL_CPL_MASK                    (0x3ff << LCD_POL_CPL_SHIFT)
#define LCD_POL_BCD                         (1 << 26) /* Bit 26: Bypass pixel clock divider */
#define LCD_POL_PCDHI_SHIFT                 (27)      /* Bits 27-31: Upper 5 bits of panel clock divisor */
#define LCD_POL_PCDHI_MASK                  (0x1f << LCD_POL_PCDHI_SHIFT)

/* LCD_LE - Line End Control Register */

#define LCD_LE_LED_SHIFT                    (0)       /* Bits 0-6: Line End delay */
#define LCD_LE_LED_MASK                     (0x7f << LCD_LE_LED_SHIFT)
                                                      /* Bits 7-15: Reserved */
#define LCD_LE_LEE                          (1 << 16) /* Bit 16: LCD line end enable */
                                                      /* Bit 17-31: Reserved */

/* LCD_UPBASE - Upper Panel Frame Base Address Register */

                                                      /* Bits 0-2: Reserved */
#define LCD_UPBASE_LCDUPBASE_SHIFT          (3)       /* Bits 3-31: LCD upper panel base address */
#define LCD_UPBASE_LCDUPBASE_MASK           (0x1FFFFFFF << LCD_UPBASE_LCDUPBASE_SHIFT)

/* LCD_UPBASE - Lower Panel Frame Base Address Register */

                                                      /* Bits 0-2: Reserved */
#define LCD_UPBASE_LCDLPBASE_SHIFT          (3)       /* Bits 3-31: LCD lower panel base address */
#define LCD_UPBASE_LCDLPBASE_MASK           (0x1FFFFFFF << LCD_UPBASE_LCDUPBASE_SHIFT)

/* LCD_CTRL - Control Register */

#define LCD_CTRL_LCDEN                      (1 << 0)  /* Bit 0: LCD enable control bit */
#define LCD_CTRL_LCDBPP_SHIFT               (1)       /* Bits 1-3: LCD bits per pixel */
#define LCD_CTRL_LCDBPP_MASK                (7 << LCD_CTRL_LCDBPP_SHIFT)
#  define LCD_CTRL_LCDBPP_1                 (0 << LCD_CTRL_LCDBPP_SHIFT) /* 1 bpp */
#  define LCD_CTRL_LCDBPP_2                 (1 << LCD_CTRL_LCDBPP_SHIFT) /* 2 bpp */
#  define LCD_CTRL_LCDBPP_4                 (2 << LCD_CTRL_LCDBPP_SHIFT) /* 4 bpp */
#  define LCD_CTRL_LCDBPP_8                 (3 << LCD_CTRL_LCDBPP_SHIFT) /* 8 bpp */
#  define LCD_CTRL_LCDBPP_16                (4 << LCD_CTRL_LCDBPP_SHIFT) /* 16 bpp */
#  define LCD_CTRL_LCDBPP_24                (5 << LCD_CTRL_LCDBPP_SHIFT) /* 24 bpp (TFT panel only) */
#  define LCD_CTRL_LCDBPP_565               (6 << LCD_CTRL_LCDBPP_SHIFT) /* 16 bpp, 5:6:5 mode */
#  define LCD_CTRL_LCDBPP_444               (7 << LCD_CTRL_LCDBPP_SHIFT) /* 12 bpp, 4:4:4 mode */

#define LCD_CTRL_LCDBW                      (1 << 4)  /* Bit 4: STN LCD monochrome/color selection */
#define LCD_CTRL_LCDTFT                     (1 << 5)  /* Bit 5: LCD TFT type selection */
#define LCD_CTRL_LCDMONO8                   (1 << 6)  /* Bit 6: Monochrome LCD interface bit */
#define LCD_CTRL_LCDDUAL                    (1 << 7)  /* Bit 7: Single or Dual LCD panel selection */
#define LCD_CTRL_BGR                        (1 << 8)  /* Bit 8: Color format */
#define LCD_CTRL_BEBO                       (1 << 9)  /* Bit 9:  Big-Endian Byte Order */
#define LCD_CTRL_BEPO                       (1 << 10) /* Bit 10: Big-Endian Pixel Ordering */
#define LCD_CTRL_LCDPWR                     (1 << 11) /* Bit 11: LCD Power enable */
#define LCD_CTRL_LCDVCOMP_SHIFT             (12)      /* Bits 12-13: LCD Vertical compare interrupt */
#define LCD_CTRL_LCDVCOMP_MASK              (3 << LCD_CTRL_LCDVCOMP_SHIFT)
                                                      /* Bits 14-15: Reserved */
#define LCD_CTRL_WATERMARK                  (1 << 16) /* Bit 16: LCD DMA FIFO watermark level */
                                                      /* Bits 17-31: Reserved */

/* LCD_INTMSK - Interrupt Mask Register */

                                                      /* Bits 0: Reserved */
#define LCD_INTMSK_FUFIM                    (1 << 1)  /* Bit 1: FIFO underflow interrupt enable */
#define LCD_INTMSK_LNBUIM                   (1 << 2)  /* Bit 2: LCD next base address interrupt enable */
#define LCD_INTMSK_VCOMPIM                  (1 << 3)  /* Bit 3: Vertical compare interrupt enable */
#define LCD_INTMSK_BERIM                    (1 << 4)  /* Bit 4: AHB Master error interrupt enable */
                                                      /* Bits 5-31: Reserved */
#define LCD_INTMSK_ALL                      (0x1e)

/* LCD_INTRAW - Raw Interrupt Status Register */

                                                      /* Bits 0: Reserved */
#define LCD_INTRAW_FUFRIS                   (1 << 1)  /* Bit 1: FIFO Underflow raw interrupt status */
#define LCD_INTRAW_LNBURIS                  (1 << 2)  /* Bit 2: LCD Next address base update intterupt */
#define LCD_INTRAW_VCOMPRIS                 (1 << 3)  /* Bit 3: Vertical compare interrupt status */
#define LCD_INTRAW_BERRAW                   (1 << 4)  /* Bit 4: AHB Master bus error interrupt status */
                                                      /* Bits 5-31: Reserved */
#define LCD_INTRAW_ALL                      (0x1e)

/* LCD_INTSTAT - Masked Interrupt Status Register */

                                                      /* Bits 0: Reserved */
#define LCD_INTSTAT_FUFMIS                  (1 << 1)  /* Bit 1: FIFO Underflow raw interrupt status */
#define LCD_INTSTAT_LNBUMIS                 (1 << 2)  /* Bit 2: LCD Next address base update intterupt */
#define LCD_INTSTAT_VCOMPMIS                (1 << 3)  /* Bit 3: Vertical compare interrupt status */
#define LCD_INTSTAT_BERMIS                  (1 << 4)  /* Bit 4: AHB Master bus error interrupt status */
                                                      /* Bits 15-31: Reserved */
#define LCD_INTSTAT_ALL                     (0x1e)

/* LCD_INTCLR - Interrupt Clear Register */

                                                      /* Bits 0: Reserved */
#define LCD_INTCLR_FUFIC                    (1 << 1)  /* Bit 1: FIFO Underflow raw interrupt clear */
#define LCD_INTCLR_LNBUIC                   (1 << 2)  /* Bit 2: LCD Next address base update intterupt */
#define LCD_INTCLR_VCOMPIC                  (1 << 3)  /* Bit 3: Vertical compare interrupt clear */
#define LCD_INTCLR_BERIC                    (1 << 4)  /* Bit 4: AHB Master bus error interrupt clear */
                                                      /* Bits 15-31: Reserved */
#define LCD_INTCLR_ALL                      (0x1e)

/* Upper and Lower Panel Address register has no bitfields */

/*   Upper Panel Current Address register (LCDUPCURR)
 *   Lower Panel Current Address register (LCDLPCURR)
 */

/* LCD_PAL - Color Palette Registers */

#define LCD_PAL_R0_SHIFT                    (0)       /* Bits 0-4: Red palette data */
#define LCD_PAL_R0_MASK                     (0x1f << LCD_PAL_R0_SHIFT)
#define LCD_PAL_G0_SHIFT                    (5)       /* Bits 5-9: Green palette data */
#define LCD_PAL_G0_MASK                     (0x1f << LCD_PAL_G0_SHIFT)
#define LCD_PAL_B0_SHIFT                    (10)      /* Bits 10-14: Blue paletted data */
#define LCD_PAL_B0_MASK                     (0x1f << LCD_PAL_B0_SHIFT)
#define LCD_PAL_I0                          (1 << 15) /* Bit 15: Intensity/Unused bit */
#define LCD_PAL_R1_SHIFT                    (16)      /* Bits 16-20: Red palette data */
#define LCD_PAL_R1_MASK                     (0x1f << LCD_PAL_R1_SHIFT)
#define LCD_PAL_G1_SHIFT                    (21)      /* Bits 21-25: Green palette data */
#define LCD_PAL_G1_MASK                     (0x1f << LCD_PAL_G1_SHIFT)
#define LCD_PAL_B1_SHIFT                    (26)      /* Bits 26-30: Blue palette data */
#define LCD_PAL_B1_MASK                     (0x1f << LCD_PAL_B1_SHIFT)
#define LCD_PAL_I1                          (1 << 31) /* Bit 31: Intensity/Unused bit */

/* LCD_CRSR_IMG - Cursor Image Register - has no bitfields */

/* The 256 words of the cursor image register defines the appearance
 * of either one 64x64 cursor, or 4 32x32 cursors.
 */

/* LCD CRSR_CTRL - Cursor Control Register */

#define LCD_CRSR_CTRL_CRSON                 (1 << 0)  /* Bit 0: Cursor enable */
                                                      /* Bits 1-3: Reserved */
#define LCD_CRSR_CTRL_CRSRNUM_SHIFT         (4)       /* Bits 4-5: Cursor image number */
#define LCD_CRSR_CTRL_CRSRNUM_MASK          (3 << LCD_CRSR_CTRL_CRSRNUM1_0_SHIFT)
                                                      /* Bits 6-31: Reserved */

/* If the selected cursor is 32x32 */

#define LCD_CURSOR0                         (0)
#define LCD_CURSOR1                         (1)
#define LCD_CURSOR2                         (2)
#define LCD_CURSOR3                         (3)

/* LCD CRSR_CFG - Cursor Configuration Register */

#define LCD_CRSR_CFG_CRSRSIZE               (1 << 0)  /* Bit 0: Cursor size selection */
#define LCD_CRSR_CFG_FRAMESYNC              (1 << 1)  /* Bit 1: Cursor frame sync type */
                                                      /* Bits 2-31: Reserved */

#define LCD_CURSOR_SIZE32                   (0)       /* 32x32 */
#define LCD_CURSOR_SIZE64                   (1)       /* 64x64 */
#define LCD_CURSOR_FRAMEASYNC               (0)       /* Cursor coordinates are asynchronous */
#define LCD_CURSOR_FRAMESYNC                (1)       /* coordinates are synchronize to framesync pulse */

/* LCD CRSR_PAL0/1 - Cursor Palette Registers */

#define LCD_CRSR_PAL_RED_SHIFT              (0)       /* Bits 0-7: Red color componnent */
#define LCD_CRSR_PAL_RED_MASK               (0xff << LCD_CRSR_PAL0_RED_SHIFT)
#define LCD_CRSR_PAL_GREEN_SHIFT            (8)       /* Bits 8-15: Green color component */
#define LCD_CRSR_PAL_GREEN_MASK             (0xff << LCD_CRSR_PAL0_GREEN_SHIFT)
#define LCD_CRSR_PAL_BLUE_SHIFT             (16)      /* Bits 16-23: Blue color component */
#define LCD_CRSR_PAL_BLUE_MASK              (0xff << LCD_CRSR_PAL0_BLUE_SHIFT)
                                                      /* Bits 24-31: Reserved */

/* LCD CRSR_XY - Cursor XY Position Register */

#define LCD_CRSR_CRSRX_SHIFT                (0)       /* Bits 0-9: X ordinate */
#define LCD_CRSR_CRSRX_MASK                 (0x3ff << LCD_CRSR_CRSRX_SHIFT)
                                                      /* Bits 10-15: Reserved */
#define LCD_CRSR_CRSRY_SHIFT                (16)      /* Bits 16-25: Y ordinate */
#define LCD_CRSR_CRSRY_MASK                 (0x3ff << LCD_CRSR_CRSRY_SHIFT)
                                                      /* Bits 26-31: Reserved */

/* LCD CRSR_CLIP - Cursor Clip Position Register */

#define LCD_CRSR_CRSRCLIPX_SHIFT            (0)       /* Bits 0-5: X clip position */
#define LCD_CRSR_CRSRCLIPX_MASK             (0x3f << LCD_CRSR_CRSRCLIPX_SHIFT)
                                                      /* Bits 6-7: Reserved */
#define LCD_CRSR_CRSRCLIPY_SHIFT            (8)       /* Bits 8-13: Reserved */
#define LCD_CRSR_CRSRCLIPY_MASK             (0x3f << LCD_CRSR_CRSRCLIPY_SHIFT)
                                                      /* Bits 14-31: Reserved */

/* LCD CRSR_INTMSK - Cursor Interrupt Mask Register */

#define LCD_CRSR_INTMSK_CRSRIM              (1 << 0)  /* Bit 0: Cursor interrupt mask */
                                                      /* Bits 1-31: Reserved */

/* LCD CRSR_INTCLR - Cursor Interrupt Clear Register */

#define LCD_CRSR_INTCLR_CRSRIC              (1 << 0)  /* Bit 0: Cursor interrupt clear */
                                                      /* Bits 1-31: Reserved */

/* LCD CRSR_INTRAW - Cursor Raw Interrupt Status Register */

#define LCD_CRSR_INTRAW_CRSRRIS             (1 << 0)  /* Bit 0: Cursor raw interrupt status */
                                                      /* Bits 1-31: Reserved */

/* LCD CRSR_INTSTAT - Mask Interrupt Status Register */

#define LCD_CRSR_INTSTAT_CRSRMIS            (1 << 0)  /* Bit 0: Cursor mask interrupt status */
                                                      /* Bits 1-31: Reserved */

#endif /* __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_LCD_H */
