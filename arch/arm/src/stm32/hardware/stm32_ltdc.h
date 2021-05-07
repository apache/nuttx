/****************************************************************************
 * arch/arm/src/stm32/hardware/stm32_ltdc.h
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

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32_LTDC_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32_LTDC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/stm32_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STM32_LTDC_NCLUT            256    /* Number of entries in the CLUTs */

/* LCDC Register Offsets ****************************************************/

#define STM32_LTDC_SSCR_OFFSET      0x0008 /* LTDC Synchronization Size Config Register */
#define STM32_LTDC_BPCR_OFFSET      0x000c /* LTDC Back Porch Configuration Register */
#define STM32_LTDC_AWCR_OFFSET      0x0010 /* LTDC Active Width Configuration Register */
#define STM32_LTDC_TWCR_OFFSET      0x0014 /* LTDC Total Width Configuration Register */
#define STM32_LTDC_GCR_OFFSET       0x0018 /* LTDC Global Control Register */
                                           /* 0x0020 Reserved */
#define STM32_LTDC_SRCR_OFFSET      0x0024 /* LTDC Shadow Reload Configuration Register */
                                           /* 0x0028 Reserved */
#define STM32_LTDC_BCCR_OFFSET      0x002c /* LTDC Background Color Configuration Register */
                                           /* 0x0030 Reserved */
#define STM32_LTDC_IER_OFFSET       0x0034 /* LTDC Interrupt Enable Register */
#define STM32_LTDC_ISR_OFFSET       0x0038 /* LTDC Interrupt Status Register */
#define STM32_LTDC_ICR_OFFSET       0x003c /* LTDC Interrupt Clear Register */
#define STM32_LTDC_LIPCR_OFFSET     0x0040 /* LTDC Line Interrupt Position Config Register */
#define STM32_LTDC_CPSR_OFFSET      0x0044 /* LTDC Current Position Status Register */
#define STM32_LTDC_CDSR_OFFSET      0x0048 /* LTDC Current Display Status Register */
                                           /* 0x004c-0x0080 Reserved */

#define STM32_LTDC_L1CR_OFFSET      0x0084 /* LTDC Layer 1 Control Register */
#define STM32_LTDC_L1WHPCR_OFFSET   0x0088 /* LTDC Layer 1 Window Horiz Pos Config Register */
#define STM32_LTDC_L1WVPCR_OFFSET   0x008c /* LTDC Layer 1 Window Vert Pos Config Register */
#define STM32_LTDC_L1CKCR_OFFSET    0x0090 /* LTDC Layer 1 Color Keying Config Register */
#define STM32_LTDC_L1PFCR_OFFSET    0x0094 /* LTDC Layer 1 Pixel Format Configuration Register */
#define STM32_LTDC_L1CACR_OFFSET    0x0098 /* LTDC Layer 1 Constant Alpha Config Register */
#define STM32_LTDC_L1DCCR_OFFSET    0x009c /* LTDC Layer 1 Default Color Config Register */
#define STM32_LTDC_L1BFCR_OFFSET    0x00a0 /* LTDC Layer 1 Blending Factors Config Register */
                                           /* 0x00A4-0x00A8 Reserved */
#define STM32_LTDC_L1CFBAR_OFFSET   0x00ac /* LTDC Layer 1 Color Frame Buffer Address Register */
#define STM32_LTDC_L1CFBLR_OFFSET   0x00b0 /* LTDC Layer 1 Color Frame Buffer Length Register */
#define STM32_LTDC_L1CFBLNR_OFFSET  0x00b4 /* LTDC Layer 1 Color Frame Buffer Line Number Register */
                                           /* 0x00B8-0x00C0 Reserved */
#define STM32_LTDC_L1CLUTWR_OFFSET  0x00c4 /* LTDC Layer 1 CLUT Write Register */
                                           /* 0x00C8-0x0100 Reserved */
#define STM32_LTDC_L2CR_OFFSET      0x0104 /* LTDC Layer 2 Control Register */
#define STM32_LTDC_L2WHPCR_OFFSET   0x0108 /* LTDC Layer 2 Window Horiz Pos Config Register */
#define STM32_LTDC_L2WVPCR_OFFSET   0x010c /* LTDC Layer 2 Window Vert Pos Config Register */
#define STM32_LTDC_L2CKCR_OFFSET    0x0110 /* LTDC Layer 2 Color Keying Config Register */
#define STM32_LTDC_L2PFCR_OFFSET    0x0114 /* LTDC Layer 2 Pixel Format Configuration Register */
#define STM32_LTDC_L2CACR_OFFSET    0x0118 /* LTDC Layer 2 Constant Alpha Config Register */
#define STM32_LTDC_L2DCCR_OFFSET    0x011c /* LTDC Layer 2 Default Color Config Register */
#define STM32_LTDC_L2BFCR_OFFSET    0x0120 /* LTDC Layer 2 Blending Factors Config Register */
                                           /* 0x0124-0x0128 Reserved */
#define STM32_LTDC_L2CFBAR_OFFSET   0x012c /* LTDC Layer 2 Color Frame Buffer Address Register */
#define STM32_LTDC_L2CFBLR_OFFSET   0x0130 /* LTDC Layer 2 Color Frame Buffer Length Register */
#define STM32_LTDC_L2CFBLNR_OFFSET  0x0134 /* LTDC Layer 2 Color Frame Buffer Line Number Register */
                                           /* 0x0138-0x0130 Reserved */
#define STM32_LTDC_L2CLUTWR_OFFSET  0x0144 /* LTDC Layer 2 CLUT Write Register */
                                           /* 0x0148-0x03ff Reserved */

/* LTDC Register Addresses **************************************************/

#define STM32_LTDC_SSCR             (STM32_LTDC_BASE + STM32_LTDC_SSCR_OFFSET)
#define STM32_LTDC_BPCR             (STM32_LTDC_BASE + STM32_LTDC_BPCR_OFFSET)
#define STM32_LTDC_AWCR             (STM32_LTDC_BASE + STM32_LTDC_AWCR_OFFSET)
#define STM32_LTDC_TWCR             (STM32_LTDC_BASE + STM32_LTDC_TWCR_OFFSET)
#define STM32_LTDC_GCR              (STM32_LTDC_BASE + STM32_LTDC_GCR_OFFSET)
#define STM32_LTDC_SRCR             (STM32_LTDC_BASE + STM32_LTDC_SRCR_OFFSET)
#define STM32_LTDC_BCCR             (STM32_LTDC_BASE + STM32_LTDC_BCCR_OFFSET)
#define STM32_LTDC_IER              (STM32_LTDC_BASE + STM32_LTDC_IER_OFFSET)
#define STM32_LTDC_ISR              (STM32_LTDC_BASE + STM32_LTDC_ISR_OFFSET)
#define STM32_LTDC_ICR              (STM32_LTDC_BASE + STM32_LTDC_ICR_OFFSET)
#define STM32_LTDC_LIPCR            (STM32_LTDC_BASE + STM32_LTDC_LIPCR_OFFSET)
#define STM32_LTDC_CPSR             (STM32_LTDC_BASE + STM32_LTDC_CPSR_OFFSET)
#define STM32_LTDC_CDSR             (STM32_LTDC_BASE + STM32_LTDC_CDSR_OFFSET)

#define STM32_LTDC_L1CR             (STM32_LTDC_BASE + STM32_LTDC_L1CR_OFFSET)
#define STM32_LTDC_L1WHPCR          (STM32_LTDC_BASE + STM32_LTDC_L1WHPCR_OFFSET)
#define STM32_LTDC_L1WVPCR          (STM32_LTDC_BASE + STM32_LTDC_L1WVPCR_OFFSET)
#define STM32_LTDC_L1CKCR           (STM32_LTDC_BASE + STM32_LTDC_L1CKCR_OFFSET)
#define STM32_LTDC_L1PFCR           (STM32_LTDC_BASE + STM32_LTDC_L1PFCR_OFFSET)
#define STM32_LTDC_L1CACR           (STM32_LTDC_BASE + STM32_LTDC_L1CACR_OFFSET)
#define STM32_LTDC_L1DCCR           (STM32_LTDC_BASE + STM32_LTDC_L1DCCR_OFFSET)
#define STM32_LTDC_L1BFCR           (STM32_LTDC_BASE + STM32_LTDC_L1BFCR_OFFSET)
#define STM32_LTDC_L1CFBAR          (STM32_LTDC_BASE + STM32_LTDC_L1CFBAR_OFFSET)
#define STM32_LTDC_L1CFBLR          (STM32_LTDC_BASE + STM32_LTDC_L1CFBLR_OFFSET)
#define STM32_LTDC_L1CFBLNR         (STM32_LTDC_BASE + STM32_LTDC_L1CFBLNR_OFFSET)
#define STM32_LTDC_L1CLUTWR         (STM32_LTDC_BASE + STM32_LTDC_L1CLUTWR_OFFSET)

#define STM32_LTDC_L2CR             (STM32_LTDC_BASE + STM32_LTDC_L2CR_OFFSET)
#define STM32_LTDC_L2WHPCR          (STM32_LTDC_BASE + STM32_LTDC_L2WHPCR_OFFSET)
#define STM32_LTDC_L2WVPCR          (STM32_LTDC_BASE + STM32_LTDC_L2WVPCR_OFFSET)
#define STM32_LTDC_L2CKCR           (STM32_LTDC_BASE + STM32_LTDC_L2CKCR_OFFSET)
#define STM32_LTDC_L2PFCR           (STM32_LTDC_BASE + STM32_LTDC_L2PFCR_OFFSET)
#define STM32_LTDC_L2CACR           (STM32_LTDC_BASE + STM32_LTDC_L2CACR_OFFSET)
#define STM32_LTDC_L2DCCR           (STM32_LTDC_BASE + STM32_LTDC_L2DCCR_OFFSET)
#define STM32_LTDC_L2BFCR           (STM32_LTDC_BASE + STM32_LTDC_L2BFCR_OFFSET)
#define STM32_LTDC_L2CFBAR          (STM32_LTDC_BASE + STM32_LTDC_L2CFBAR_OFFSET)
#define STM32_LTDC_L2CFBLR          (STM32_LTDC_BASE + STM32_LTDC_L2CFBLR_OFFSET)
#define STM32_LTDC_L2CFBLNR         (STM32_LTDC_BASE + STM32_LTDC_L2CFBLNR_OFFSET)
#define STM32_LTDC_L2CLUTWR         (STM32_LTDC_BASE + STM32_LTDC_L2CLUTWR_OFFSET)

/* LTDC Register Bit Definitions ********************************************/

/* LTDC Synchronization Size Configuration Register */

#define LTDC_SSCR_VSH_SHIFT         (0)       /* Bits 0-10: Vertical Sync Height (scan lines) */
#define LTDC_SSCR_VSH_MASK          (0x7ff << LTDC_SSCR_VSH_SHIFT)
#  define LTDC_SSCR_VSH(n)          ((uint32_t)(n) << LTDC_SSCR_VSH_SHIFT)
#define LTDC_SSCR_HSW_SHIFT         (16)      /* Bits 16-27: Horizontal Sync Width (pixel clocks) */
#define LTDC_SSCR_HSW_MASK          (0xfff << LTDC_SSCR_HSW_SHIFT)
#  define LTDC_SSCR_HSW(n)          ((uint32_t)(n) << LTDC_SSCR_HSW_SHIFT)

/* LTDC Back Porch Configuration Register */

#define LTDC_BPCR_AVBP_SHIFT        (0)       /* Bits 0-10: Accumulated Vertical back porch (scan lines) */
#define LTDC_BPCR_AVBP_MASK         (0x7ff << LTDC_BPCR_AVBP_SHIFT)
#  define LTDC_BPCR_AVBP(n)         ((uint32_t)(n) << LTDC_BPCR_AVBP_SHIFT)
#define LTDC_BPCR_AHBP_SHIFT        (16)      /* Bits 16-27: Accumulated Horizontal back porch (pixel clocks) */
#define LTDC_BPCR_AHBP_MASK         (0xfff << LTDC_BPCR_AVBP_SHIFT)
#  define LTDC_BPCR_AHBP(n)         ((uint32_t)(n) << LTDC_BPCR_AHBP_SHIFT)

/* LTDC Active Width Configuration Register */

#define LTDC_AWCR_AAH_SHIFT         (0)       /* Bits 0-10: Accumulated Active Height (scan lines) */
#define LTDC_AWCR_AAH_MASK          (0x7ff << LTDC_AWCR_AAH_SHIFT)
#  define LTDC_AWCR_AAH(n)          ((uint32_t)(n) << LTDC_AWCR_AAH_SHIFT)
#define LTDC_AWCR_AAW_SHIFT         (16)      /* Bits 16-27: Accumulated Active Width (pixel clocks) */
#define LTDC_AWCR_AAW_MASK          (0xfff << LTDC_AWCR_AAW_SHIFT)
#  define LTDC_AWCR_AAW(n)          ((uint32_t)(n) << LTDC_AWCR_AAW_SHIFT)

/* LTDC Total Width Configuration Register */

#define LTDC_TWCR_TOTALH_SHIFT      (0)       /* Bits 0-10: Total Height (scan lines) */
#define LTDC_TWCR_TOTALH_MASK       (0x7ff << LTDC_TWCR_TOTALH_SHIFT)
#  define LTDC_TWCR_TOTALH(n)       ((uint32_t)(n) << LTDC_TWCR_TOTALH_SHIFT)
#define LTDC_TWCR_TOTALW_SHIFT      (16)      /* Bits 16-27: Total Width (pixel clocks) */
#define LTDC_TWCR_TOTALW_MASK       (0xfff << LTDC_TWCR_TOTALW_SHIFT)
#  define LTDC_TWCR_TOTALW(n)       ((uint32_t)(n) << LTDC_TWCR_TOTALW_SHIFT)

/* LTDC Global Control Register */

#define LTDC_GCR_LTDCEN             (1 << 0)  /* Bit 0:  LCD-TFT Controller Enable Bit */
#define LTDC_GCR_DBW_SHIFT          (4)       /* Bits 4-6: Dither Blue Width */
#define LTDC_GCR_DBW_MASK           (0x7 << LTDC_GCR_DBW_SHIFT)
#  define LTDC_GCR_DBW(n)           ((uint32_t)(n) << LTDC_GCR_DBW_SHIFT)
#define LTDC_GCR_DGW_SHIFT          (8)       /* Bits 8-10: Dither Green Width */
#define LTDC_GCR_DGW_MASK           (0x7 << LTDC_GCR_DGW_SHIFT)
#  define LTDC_GCR_DGW(n)           ((uint32_t)(n) << LTDC_GCR_DGW_SHIFT)
#define LTDC_GCR_DRW_SHIFT          (12)      /* Bits 12-14: Dither Red Width */
#define LTDC_GCR_DRW_MASK           (0x7 << LTDC_GCR_DRW_SHIFT)
#  define LTDC_GCR_DRW(n)           ((uint32_t)(n) << LTDC_GCR_DRW_SHIFT)
#define LTDC_GCR_DEN                (1 << 16) /* Bit 16:  Dither Enable */
#define LTDC_GCR_PCPOL              (1 << 28) /* Bit 28:  Pixel Clock Polarity */
#define LTDC_GCR_DEPOL              (1 << 29) /* Bit 29:  Data Enable Polarity */
#define LTDC_GCR_VSPOL              (1 << 30) /* Bit 30:  Vertical Sync Polarity */
#define LTDC_GCR_HSPOL              (1 << 31) /* Bit 31:  Horizontal Sync Polarity */

/* LTDC Shadow Reload Configuration Register */

#define LTDC_SRCR_IMR               (1 << 0)  /* Bit 0:  Immediate Reload */
#define LTDC_SRCR_VBR               (1 << 1)  /* Bit 1:  Vertical Blanking Reload */

/* LTDC Background Color Configuration Register */

#define LTDC_BCCR_BCBLUE_SHIFT      (0)       /* Bits 0-7: Background Color Blue Value */
#define LTDC_BCCR_BCBLUE_MASK       (0xff << LTDC_BCCR_BCBLUE_SHIFT)
#  define LTDC_BCCR_BCBLUE(n)       ((uint32_t)(n) << LTDC_BCCR_BCBLUE_SHIFT)
#define LTDC_BCCR_BCGREEN_SHIFT     (8)       /* Bits 8-15: Background Color Green Value */
#define LTDC_BCCR_BCGREEN_MASK      (0xff << LTDC_BCCR_BCGREEN_SHIFT)
#  define LTDC_BCCR_BCGREEN(n)      ((uint32_t)(n) << LTDC_BCCR_BCGREEN_SHIFT)
#define LTDC_BCCR_BCRED_SHIFT       (16)       /* Bits 16-23: Background Color Red Value */
#define LTDC_BCCR_BCRED_MASK        (0xff << LTDC_BCCR_BCRED_SHIFT)
#  define LTDC_BCCR_BCRED(n)        ((uint32_t)(n) << LTDC_BCCR_BCRED_SHIFT)

/* LTDC Interrupt Enable Register */

#define LTDC_IER_LIE                (1 << 0)  /* Bit 0:  Line Interrupt Enable */
#define LTDC_IER_FUIE               (1 << 1)  /* Bit 1:  FIFO Underrun Interrupt Enable */
#define LTDC_IER_TERRIE             (1 << 2)  /* Bit 2:  Transfer Error Interrupt Enable */
#define LTDC_IER_RRIE               (1 << 3)  /* Bit 3:  Register Reload Interrupt Enable */

/* LTDC Interrupt Status Register */

#define LTDC_ISR_LIF                (1 << 0)  /* Bit 0:  Line Interrupt Flag */
#define LTDC_ISR_FUIF               (1 << 1)  /* Bit 1:  FIFO Underrun Interrupt Flag */
#define LTDC_IER_TERRIF             (1 << 2)  /* Bit 2:  Transfer Error Interrupt Flag */
#define LTDC_ISR_RRIF               (1 << 3)  /* Bit 3:  Register Reload Interrupt Flag */

/* LTDC Interrupt Clear Register */

#define LTDC_ICR_CLIF               (1 << 0)  /* Bit 0:  Clear Line Interrupt Flag */
#define LTDC_ICR_CFUIF              (1 << 1)  /* Bit 1:  Clear FIFO Underrun Interrupt Flag */
#define LTDC_ICR_CTERRIF            (1 << 2)  /* Bit 2:  Clear Transfer Error Interrupt Flag */
#define LTDC_ICR_CRRIF              (1 << 3)  /* Bit 3:  Clear Register Reload Interrupt Flag */

/* LTDC Line Interrupt Posittion Configuration Register */

#define LTDC_LIPCR_LIPOS_SHIFT      (0)       /* Bits 0-10: Line Interrupt Position */
#define LTDC_LIPCR_LIPOS_MASK       (0x7ff << LTDC_LIPCR_LIPOS_SHIFT)
#  define LTDC_LIPCR_LIPOS(n)       ((uint32_t)(n) << LTDC_LIPCR_LIPOS_SHIFT)

/* LTDC Current Position Status Register */

#define LTDC_CPSR_CYPOS_SHIFT       (0)       /* Bits 0-15: Current Y Position */
#define LTDC_CPSR_CYPOS_MASK        (0xffff << LTDC_CPSR_CYPOS_SHIFT)
#  define LTDC_CPSR_CYPOS(n)        ((uint32_t)(n) << LTDC_CPSR_CYPOS_SHIFT)
#define LTDC_CPSR_CXPOS_SHIFT       (16)      /* Bits 15-31: Current X Position */
#define LTDC_CPSR_CXPOS_MASK        (0xffff << LTDC_CPSR_CXPOS_SHIFT)
#  define LTDC_CPSR_CXPOS(n)        ((uint32_t)(n) << LTDC_CPSR_CXPOS_SHIFT)

/* LTDC Current Display Status Register */

#define LTDC_CDSR_VDES              (1 << 0)  /* Bit 0:  Vertical Data Enable display Status */
#define LTDC_CDSR_HDES              (1 << 1)  /* Bit 1:  Horizontal Data Enable display Status */
#define LTDC_CDSR_VSYNCS            (1 << 2)  /* Bit 2:  Vertical Sync display Status */
#define LTDC_CDSR_HSYNCS            (1 << 3)  /* Bit 3:  Horizontal Sync display Status */

/* LTDC Layer x Control Register */

#define LTDC_LXCR_LEN               (1 << 0)  /* Bit 0:  Layer Enable */
#define LTDC_LXCR_COLKEN            (1 << 1)  /* Bit 1:  Color Keying Enable */
#define LTDC_LXCR_CLUTEN            (1 << 4)  /* Bit 4:  Color Look-Up Table Enable */

/* LTDC Layer x Window Horizontal Position Configuration Register */

#define LTDC_LXWHPCR_WHSTPOS_SHIFT  (0)       /* Bits 0-11: Window Horizontal Start Position */
#define LTDC_LXWHPCR_WHSTPOS_MASK   (0xFFF << LTDC_LXWHPCR_WHSTPOS_SHIFT)
#  define LTDC_LXWHPCR_WHSTPOS(n)   ((uint32_t)(n) << LTDC_LXWHPCR_WHSTPOS_SHIFT)
#define LTDC_LXWHPCR_WHSPPOS_SHIFT  (16)      /* Bits 16-27: Window Horizontal Stop Position */
#define LTDC_LXWHPCR_WHSPPOS_MASK   (0xFFF << LTDC_LXWHPCR_WHSPPOS_SHIFT)
#  define LTDC_LXWHPCR_WHSPPOS(n)   ((uint32_t)(n) << LTDC_LXWHPCR_WHSPPOS_SHIFT)

/* LTDC Layer x Window Vertical Position Configuration Register */

#define LTDC_LXWVPCR_WVSTPOS_SHIFT  (0)       /* Bits 0-10: Window Vertical Start Position */
#define LTDC_LXWVPCR_WVSTPOS_MASK   (0x7ff << LTDC_LXWVPCR_WVSTPOS_SHIFT)
#  define LTDC_LXWVPCR_WVSTPOS(n)   ((uint32_t)(n) << LTDC_LXWVPCR_WVSTPOS_SHIFT)
#define LTDC_LXWVPCR_WVSPPOS_SHIFT  (16)      /* Bits 16-26: Window Vertical Stop Position */
#define LTDC_LXWVPCR_WVSPPOS_MASK   (0x7ff << LTDC_LXWVPCR_WVSPPOS_SHIFT)
#  define LTDC_LXWVPCR_WVSPPOS(n)   ((uint32_t)(n) << LTDC_LXWVPCR_WVSPPOS_SHIFT)

/* LTDC Layer x Color Keying Configuration Register */

#define LTDC_LXCKCR_CKBLUE_SHIFT    (0)       /* Bits 0-7: Color Key Blue Value */
#define LTDC_LXCKCR_CKBLUE_MASK     (0xff << LTDC_LXCKCR_CKBLUE_SHIFT)
#  define LTDC_LXCKCR_CKBLUE(n)     ((uint32_t)(n) << LTDC_LXCKCR_CKBLUE_SHIFT)
#define LTDC_LXCKCR_CKGREEN_SHIFT   (8)       /* Bits 8-15: Color Key Green Value */
#define LTDC_LXCKCR_CKGREEN_MASK    (0xff << LTDC_LXCKCR_CKGREEN_SHIFT)
#  define LTDC_LXCKCR_CKGREEN(n)    ((uint32_t)(n) << LTDC_LXCKCR_CKGREEN_SHIFT)
#define LTDC_LXCKCR_CKRED_SHIFT     (16)       /* Bits 16-23: Color Key Red Value */
#define LTDC_LXCKCR_CKRED_MASK      (0xff << LTDC_LXCKCR_CKRED_SHIFT)
#  define LTDC_LXCKCR_CKRED(n)      ((uint32_t)(n) << LTDC_LXCKCR_CKRED_SHIFT)

/* LTDC Layer x Pixel Format Configuration Register */

#define LTDC_LXPFCR_PF_SHIFT        (0)       /* Bits 0-2: Pixel Format */
#define LTDC_LXPFCR_PF_MASK         (0x7 << LTDC_LXPFCR_PF_SHIFT)
#  define LTDC_LXPFCR_PF(n)         ((uint32_t)(n) << LTDC_LXPFCR_PF_SHIFT)

#define LTDC_PF_ARGB8888            0
#define LTDC_PF_RGB888              1
#define LTDC_PF_RGB565              2
#define LTDC_PF_ARGB1555            3
#define LTDC_PF_ARGB4444            4
#define LTDC_PF_L8                  5     /* 8-bit Luninance (CLUT lookup) */
#define LTDC_PF_AL44                6     /* 4-bit Alpha, 4-bit Luminance */
#define LTDC_PF_AL88                7     /* 8-bit Alpha, 8-bit Luminance */

/* LTDC Layer x Constant Alpha Configuration Register */

#define LTDC_LXCACR_CONSTA_SHIFT    (0)       /* Bits 0-7: Constant Alpha */
#define LTDC_LXCACR_CONSTA_MASK     (0x7 << LTDC_LXCACR_CONSTA_SHIFT)
#  define LTDC_LXCACR_CONSTA(n)     ((uint32_t)(n) << LTDC_LXCACR_CONSTA_SHIFT)

/* LTDC Layer x Default Color Configuration Register */

#define LTDC_LXDCCR_DCBLUE_SHIFT    (0)       /* Bits 0-7: Default Color Blue Value */
#define LTDC_LXDCCR_DCBLUE_MASK     (0xff << LTDC_LXDCCR_DCBLUE_SHIFT)
#  define LTDC_LXDCCR_DCBLUE(n)     ((uint32_t)(n) << LTDC_LXDCCR_DCBLUE_SHIFT)
#define LTDC_LXDCCR_DCGREEN_SHIFT   (8)       /* Bits 8-15: Default Color Green Value */
#define LTDC_LXDCCR_DCGREEN_MASK    (0xff << LTDC_LXDCCR_DCGREEN_SHIFT)
#  define LTDC_LXDCCR_DCGREEN(n)    ((uint32_t)(n) << LTDC_LXDCCR_DCGREEN_SHIFT)
#define LTDC_LXDCCR_DCRED_SHIFT     (16)       /* Bits 16-23: Default Color Red Value */
#define LTDC_LXDCCR_DCRED_MASK      (0xff << LTDC_LXDCCR_DCRED_SHIFT)
#  define LTDC_LXDCCR_DCRED(n)      ((uint32_t)(n) << LTDC_LXDCCR_DCRED_SHIFT)
#define LTDC_LXDCCR_DCALPHA_SHIFT   (24)       /* Bits 24-31: Default Color Alpha Value */
#define LTDC_LXDCCR_DCALPHA_MASK    (0xff << LTDC_LXDCCR_DCALPHA_SHIFT)
#  define LTDC_LXDCCR_DCALPHA(n)    ((uint32_t)(n) << LTDC_LXDCCR_DCALPHA_SHIFT)

/* LTDC Layer x Blending Factors Configuration Register */

#define LTDC_LXBFCR_BF2_SHIFT       (0)       /* Bits 0-2: Blending Factor 2 */
#define LTDC_LXBFCR_BF2_MASK        (0x7 << LTDC_LXBFCR_BF2_SHIFT)
#  define LTDC_LXBFCR_BF2(n)        ((uint32_t)(n) << LTDC_LXBFCR_BF2_SHIFT)
#define LTDC_LXBFCR_BF1_SHIFT       (8)       /* Bits 8-10: Blending Factor 1 */
#define LTDC_LXBFCR_BF1_MASK        (0x7 << LTDC_LXBFCR_BF1_SHIFT)
#  define LTDC_LXBFCR_BF1(n)        ((uint32_t)(n) << LTDC_LXBFCR_BF1_SHIFT)

#define LTDC_BF1_CONST_ALPHA        0x04      /* Constant Alpha */
#define LTDC_BF1_PIXEL_ALPHA        0x06      /* Pixel Alpha x Constant Alpha */
#define LTDC_BF2_CONST_ALPHA        0x05      /* Constant Alpha */
#define LTDC_BF2_PIXEL_ALPHA        0x07      /* Pixel Alpha x Constant Alpha */

/* LTDC Layer x Color Frame Buffer Length Configuration Register */

#define LTDC_LXCFBLR_CFBLL_SHIFT    (0)       /* Bits 0-12: Color Frame Buffer Line Length */
#define LTDC_LXCFBLR_CFBLL_MASK     (0x1fff << LTDC_LXCFBLR_CFBLL_SHIFT)
#  define LTDC_LXCFBLR_CFBLL(n)     ((uint32_t)(n) << LTDC_LXCFBLR_CFBLL_SHIFT)
#define LTDC_LXCFBLR_CFBP_SHIFT     (16)       /* Bits 16-28: Color Frame Buffer Pitch */
#define LTDC_LXCFBLR_CFBP_MASK      (0x1fff << LTDC_LXCFBLR_CFBP_SHIFT)
#  define LTDC_LXCFBLR_CFBP(n)      ((uint32_t)(n) << LTDC_LXCFBLR_CFBP_SHIFT)

/* LTDC Layer x Color Frame Buffer Line Number Register */

#define LTDC_LXCFBLNR_LN_SHIFT      (0)       /* Bits 0-10: Color Frame Buffer Line Number */
#define LTDC_LXCFBLNR_LN_MASK       (0x7ff << LTDC_LXCFBLNR_LN_SHIFT)
#  define LTDC_LXCFBLNR_LN(n)       ((uint32_t)(n) << LTDC_LXCFBLNR_LN_SHIFT)

/* LTDC Layer x CLUT Write Register */

#define LTDC_LXCLUTWR_BLUE_SHIFT    (0)       /* Bits 0-7: Default Color Blue Value */
#define LTDC_LXCLUTWR_BLUE_MASK     (0xff << LTDC_LXCLUTWR_BLUE_SHIFT)
#  define LTDC_LXCLUTWR_BLUE(n)     ((uint32_t)(n) << LTDC_LXCLUTWR_BLUE_SHIFT)
#define LTDC_LXCLUTWR_GREEN_SHIFT   (8)       /* Bits 8-15: Default Color Green Value */
#define LTDC_LXCLUTWR_GREEN_MASK    (0xff << LTDC_LXCLUTWR_GREEN_SHIFT)
#  define LTDC_LXCLUTWR_GREEN(n)    ((uint32_t)(n) << LTDC_LXCLUTWR_GREEN_SHIFT)
#define LTDC_LXCLUTWR_RED_SHIFT     (16)       /* Bits 16-23: Default Color Red Value */
#define LTDC_LXCLUTWR_RED_MASK      (0xff << LTDC_LXCLUTWR_RED_SHIFT)
#  define LTDC_LXCLUTWR_RED(n)      ((uint32_t)(n) << LTDC_LXCLUTWR_RED_SHIFT)
#define LTDC_LXCLUTWR_CLUTADD_SHIFT (24)       /* Bits 24-31: CLUT Address */
#define LTDC_LXCLUTWR_CLUTADD_MASK  (0xff << LTDC_LXCLUTWR_CLUTADD_SHIFT)
#  define LTDC_LXCLUTWR_CLUTADD(n)  ((uint32_t)(n) << LTDC_LXCLUTWR_CLUTADD_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32_LTDC_H */
