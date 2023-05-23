/****************************************************************************
 * arch/arm/src/sama5/hardware/sam_lcdc.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_LCDC_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_LCDC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SAM_LCDC_NCLUT              256    /* Number of entries in the CLUTs */

/* LCDC Register Offsets ****************************************************/

#define SAM_LCDC_LCDCFG0_OFFSET     0x0000 /* LCD Controller Configuration Register 0 */
#define SAM_LCDC_LCDCFG1_OFFSET     0x0004 /* LCD Controller Configuration Register 1 */
#define SAM_LCDC_LCDCFG2_OFFSET     0x0008 /* LCD Controller Configuration Register 2 */
#define SAM_LCDC_LCDCFG3_OFFSET     0x000c /* LCD Controller Configuration Register 3 */
#define SAM_LCDC_LCDCFG4_OFFSET     0x0010 /* LCD Controller Configuration Register 4 */
#define SAM_LCDC_LCDCFG5_OFFSET     0x0014 /* LCD Controller Configuration Register 5 */
#define SAM_LCDC_LCDCFG6_OFFSET     0x0018 /* LCD Controller Configuration Register 6 */
                                           /* 0x001c Reserved */
#define SAM_LCDC_LCDEN_OFFSET       0x0020 /* LCD Controller Enable Register */
#define SAM_LCDC_LCDDIS_OFFSET      0x0024 /* LCD Controller Disable Register */
#define SAM_LCDC_LCDSR_OFFSET       0x0028 /* LCD Controller Status Register */
#define SAM_LCDC_LCDIER_OFFSET      0x002C /* LCD Controller Interrupt Enable Register */
#define SAM_LCDC_LCDIDR_OFFSET      0x0030 /* LCD Controller Interrupt Disable Register */
#define SAM_LCDC_LCDIMR_OFFSET      0x0034 /* LCD Controller Interrupt Mask Register */
#define SAM_LCDC_LCDISR_OFFSET      0x0038 /* LCD Controller Interrupt Status Register */

#if defined(ATSAMA5D4) || defined(ATSAMA5D2)
#  define SAM_LCDC_LCDATTR_OFFSET   0x003c /* LCD Controller Attribute Register */
#endif

#define SAM_LCDC_BASECHER_OFFSET    0x0040 /* Base Layer Channel Enable Register */
#define SAM_LCDC_BASECHDR_OFFSET    0x0044 /* Base Layer Channel Disable Register */
#define SAM_LCDC_BASECHSR_OFFSET    0x0048 /* Base Layer Channel Status Register */
#define SAM_LCDC_BASEIER_OFFSET     0x004c /* Base Layer Interrupt Enable Register */
#define SAM_LCDC_BASEIDR_OFFSET     0x0050 /* Base Layer Interrupt Disable Register */
#define SAM_LCDC_BASEIMR_OFFSET     0x0054 /* Base Layer Interrupt Mask Register */
#define SAM_LCDC_BASEISR_OFFSET     0x0058 /* Base Layer Interrupt Status Register */
#define SAM_LCDC_BASEHEAD_OFFSET    0x005c /* Base DMA Head Register */
#define SAM_LCDC_BASEADDR_OFFSET    0x0060 /* Base DMA Address Register */
#define SAM_LCDC_BASECTRL_OFFSET    0x0064 /* Base DMA Control Register */
#define SAM_LCDC_BASENEXT_OFFSET    0x0068 /* Base DMA Next Register */
#define SAM_LCDC_BASECFG0_OFFSET    0x006c /* Base Configuration register 0 */
#define SAM_LCDC_BASECFG1_OFFSET    0x0070 /* Base Configuration register 1 */
#define SAM_LCDC_BASECFG2_OFFSET    0x0074 /* Base Configuration register 2 */
#define SAM_LCDC_BASECFG3_OFFSET    0x0078 /* Base Configuration register 3 */
#define SAM_LCDC_BASECFG4_OFFSET    0x007c /* Base Configuration register 4 */
#define SAM_LCDC_BASECFG5_OFFSET    0x0080 /* Base Configuration register 5 */
#define SAM_LCDC_BASECFG6_OFFSET    0x0084 /* Base Configuration register 6 */
                                           /* 0x0088-0x013c Reserved */
#define SAM_LCDC_OVR1CHER_OFFSET    0x0140 /* Overlay 1 Channel Enable Register */
#define SAM_LCDC_OVR1CHDR_OFFSET    0x0144 /* Overlay 1 Channel Disable Register */
#define SAM_LCDC_OVR1CHSR_OFFSET    0x0148 /* Overlay 1 Channel Status Register */
#define SAM_LCDC_OVR1IER_OFFSET     0x014c /* Overlay 1 Interrupt Enable Register */
#define SAM_LCDC_OVR1IDR_OFFSET     0x0150 /* Overlay 1 Interrupt Disable Register */
#define SAM_LCDC_OVR1IMR_OFFSET     0x0154 /* Overlay 1 Interrupt Mask Register */
#define SAM_LCDC_OVR1ISR_OFFSET     0x0158 /* Overlay 1 Interrupt Status Register */
#define SAM_LCDC_OVR1HEAD_OFFSET    0x015c /* Overlay 1 DMA Head Register */
#define SAM_LCDC_OVR1ADDR_OFFSET    0x0160 /* Overlay 1 DMA Address Register */
#define SAM_LCDC_OVR1CTRL_OFFSET    0x0164 /* Overlay 1 DMA Control Register */
#define SAM_LCDC_OVR1NEXT_OFFSET    0x0168 /* Overlay 1 DMA Next Register */
#define SAM_LCDC_OVR1CFG0_OFFSET    0x016c /* Overlay 1 Configuration 0 Register */
#define SAM_LCDC_OVR1CFG1_OFFSET    0x0170 /* Overlay 1 Configuration 1 Register */
#define SAM_LCDC_OVR1CFG2_OFFSET    0x0174 /* Overlay 1 Configuration 2 Register */
#define SAM_LCDC_OVR1CFG3_OFFSET    0x0178 /* Overlay 1 Configuration 3 Register */
#define SAM_LCDC_OVR1CFG4_OFFSET    0x017c /* Overlay 1 Configuration 4 Register */
#define SAM_LCDC_OVR1CFG5_OFFSET    0x0180 /* Overlay 1 Configuration 5 Register */
#define SAM_LCDC_OVR1CFG6_OFFSET    0x0184 /* Overlay 1 Configuration 6 Register */
#define SAM_LCDC_OVR1CFG7_OFFSET    0x0188 /* Overlay 1 Configuration 7 Register */
#define SAM_LCDC_OVR1CFG8_OFFSET    0x018c /* Overlay 1 Configuration 8 Register */
#define SAM_LCDC_OVR1CFG9_OFFSET    0x0190 /* Overlay 1 Configuration 9 Register */
                                           /* 0x0194-0x023c Reserved */
#define SAM_LCDC_OVR2CHER_OFFSET    0x0240 /* Overlay 2 Channel Enable Register */
#define SAM_LCDC_OVR2CHDR_OFFSET    0x0244 /* Overlay 2 Channel Disable Register */
#define SAM_LCDC_OVR2CHSR_OFFSET    0x0248 /* Overlay 2 Channel Status Register */
#define SAM_LCDC_OVR2IER_OFFSET     0x024c /* Overlay 2 Interrupt Enable Register */
#define SAM_LCDC_OVR2IDR_OFFSET     0x0250 /* Overlay 2 Interrupt Disable Register */
#define SAM_LCDC_OVR2IMR_OFFSET     0x0254 /* Overlay 2 Interrupt Mask Register */
#define SAM_LCDC_OVR2ISR_OFFSET     0x0258 /* Overlay 2 Interrupt Status Register */
#define SAM_LCDC_OVR2HEAD_OFFSET    0x025c /* Overlay 2 DMA Head Register */
#define SAM_LCDC_OVR2ADDR_OFFSET    0x0260 /* Overlay 2 DMA Address Register */
#define SAM_LCDC_OVR2CTRL_OFFSET    0x0264 /* Overlay 2 DMA Control Register */
#define SAM_LCDC_OVR2NEXT_OFFSET    0x0268 /* Overlay 2 DMA Next Register */
#define SAM_LCDC_OVR2CFG0_OFFSET    0x026c /* Overlay 2 Configuration 0 Register */
#define SAM_LCDC_OVR2CFG1_OFFSET    0x0270 /* Overlay 2 Configuration 1 Register */
#define SAM_LCDC_OVR2CFG2_OFFSET    0x0274 /* Overlay 2 Configuration 2 Register */
#define SAM_LCDC_OVR2CFG3_OFFSET    0x0278 /* Overlay 2 Configuration 3 Register */
#define SAM_LCDC_OVR2CFG4_OFFSET    0x027c /* Overlay 2 Configuration 4 Register */
#define SAM_LCDC_OVR2CFG5_OFFSET    0x0280 /* Overlay 2 Configuration 5 Register */
#define SAM_LCDC_OVR2CFG6_OFFSET    0x0284 /* Overlay 2 Configuration 6 Register */
#define SAM_LCDC_OVR2CFG7_OFFSET    0x0288 /* Overlay 2 Configuration 7 Register */
#define SAM_LCDC_OVR2CFG8_OFFSET    0x028c /* Overlay 2 Configuration 8 Register */
#define SAM_LCDC_OVR2CFG9_OFFSET    0x0290 /* Overlay 2 Configuration 9 Register */
                                           /* 0x0294-0x033c Reserved */
#define SAM_LCDC_HEOCHER_OFFSET     0x0340 /* High-End Overlay Channel Enable Register */
#define SAM_LCDC_HEOCHDR_OFFSET     0x0344 /* High-End Overlay Channel Disable Register */
#define SAM_LCDC_HEOCHSR_OFFSET     0x0348 /* High-End Overlay Channel Status Register */
#define SAM_LCDC_HEOIER_OFFSET      0x034c /* High-End Overlay Interrupt Enable Register */
#define SAM_LCDC_HEOIDR_OFFSET      0x0350 /* High-End Overlay Interrupt Disable Register */
#define SAM_LCDC_HEOIMR_OFFSET      0x0354 /* High-End Overlay Interrupt Mask Register */
#define SAM_LCDC_HEOISR_OFFSET      0x0358 /* High-End Overlay Interrupt Status Register */
#define SAM_LCDC_HEOHEAD_OFFSET     0x035c /* High-End Overlay DMA Head Register */
#define SAM_LCDC_HEOADDR_OFFSET     0x0360 /* High-End Overlay DMA Address Register */
#define SAM_LCDC_HEOCTRL_OFFSET     0x0364 /* High-End Overlay DMA Control Register */
#define SAM_LCDC_HEONEXT_OFFSET     0x0368 /* High-End Overlay DMA Next Register */
#define SAM_LCDC_HEOUHEAD_OFFSET    0x036c /* High-End Overlay U DMA Head Register */
#define SAM_LCDC_HEOUADDR_OFFSET    0x0370 /* High-End Overlay U DMA Address Register */
#define SAM_LCDC_HEOUCTRL_OFFSET    0x0374 /* High-End Overlay U DMA Control Register */
#define SAM_LCDC_HEOUNEXT_OFFSET    0x0378 /* High-End Overlay U DMA Next Register */
#define SAM_LCDC_HEOVHEAD_OFFSET    0x037c /* High-End Overlay V DMA Head Register */
#define SAM_LCDC_HEOVADDR_OFFSET    0x0380 /* High-End Overlay V DMA Address Register */
#define SAM_LCDC_HEOVCTRL_OFFSET    0x0384 /* High-End Overlay V DMA Control Register */
#define SAM_LCDC_HEOVNEXT_OFFSET    0x0388 /* High-End Overlay V DMA Next Register */
#define SAM_LCDC_HEOCFG0_OFFSET     0x038c /* High-End Overlay Configuration Register 0 */
#define SAM_LCDC_HEOCFG1_OFFSET     0x0390 /* High-End Overlay Configuration Register 1 */
#define SAM_LCDC_HEOCFG2_OFFSET     0x0394 /* High-End Overlay Configuration Register 2 */
#define SAM_LCDC_HEOCFG3_OFFSET     0x0398 /* High-End Overlay Configuration Register 3 */
#define SAM_LCDC_HEOCFG4_OFFSET     0x039c /* High-End Overlay Configuration Register 4 */
#define SAM_LCDC_HEOCFG5_OFFSET     0x03a0 /* High-End Overlay Configuration Register 5 */
#define SAM_LCDC_HEOCFG6_OFFSET     0x03a4 /* High-End Overlay Configuration Register 6 */
#define SAM_LCDC_HEOCFG7_OFFSET     0x03a8 /* High-End Overlay Configuration Register 7 */
#define SAM_LCDC_HEOCFG8_OFFSET     0x03ac /* High-End Overlay Configuration Register 8 */
#define SAM_LCDC_HEOCFG9_OFFSET     0x03b0 /* High-End Overlay Configuration Register 9 */
#define SAM_LCDC_HEOCFG10_OFFSET    0x03b4 /* High-End Overlay Configuration Register 10 */
#define SAM_LCDC_HEOCFG11_OFFSET    0x03b8 /* High-End Overlay Configuration Register 11 */
#define SAM_LCDC_HEOCFG12_OFFSET    0x03bc /* High-End Overlay Configuration Register 12 */
#define SAM_LCDC_HEOCFG13_OFFSET    0x03c0 /* High-End Overlay Configuration Register 13 */
#define SAM_LCDC_HEOCFG14_OFFSET    0x03c4 /* High-End Overlay Configuration Register 14 */
#define SAM_LCDC_HEOCFG15_OFFSET    0x03c8 /* High-End Overlay Configuration Register 15 */
#define SAM_LCDC_HEOCFG16_OFFSET    0x03cc /* High-End Overlay Configuration Register 16 */
#define SAM_LCDC_HEOCFG17_OFFSET    0x03d0 /* High-End Overlay Configuration Register 17 */
#define SAM_LCDC_HEOCFG18_OFFSET    0x03d4 /* High-End Overlay Configuration Register 18 */
#define SAM_LCDC_HEOCFG19_OFFSET    0x03d8 /* High-End Overlay Configuration Register 19 */
#define SAM_LCDC_HEOCFG20_OFFSET    0x03dc /* High-End Overlay Configuration Register 20 */
#define SAM_LCDC_HEOCFG21_OFFSET    0x03e0 /* High-End Overlay Configuration Register 21 */
#define SAM_LCDC_HEOCFG22_OFFSET    0x03e4 /* High-End Overlay Configuration Register 22 */
#define SAM_LCDC_HEOCFG23_OFFSET    0x03e8 /* High-End Overlay Configuration Register 23 */
#define SAM_LCDC_HEOCFG24_OFFSET    0x03ec /* High-End Overlay Configuration Register 24 */
#define SAM_LCDC_HEOCFG25_OFFSET    0x03f0 /* High-End Overlay Configuration Register 25 */
#define SAM_LCDC_HEOCFG26_OFFSET    0x03f4 /* High-End Overlay Configuration Register 26 */
#define SAM_LCDC_HEOCFG27_OFFSET    0x03f8 /* High-End Overlay Configuration Register 27 */
#define SAM_LCDC_HEOCFG28_OFFSET    0x03fc /* High-End Overlay Configuration Register 28 */
#define SAM_LCDC_HEOCFG29_OFFSET    0x0400 /* High-End Overlay Configuration Register 29 */
#define SAM_LCDC_HEOCFG30_OFFSET    0x0404 /* High-End Overlay Configuration Register 30 */
#define SAM_LCDC_HEOCFG31_OFFSET    0x0408 /* High-End Overlay Configuration Register 31 */
#define SAM_LCDC_HEOCFG32_OFFSET    0x040c /* High-End Overlay Configuration Register 32 */
#define SAM_LCDC_HEOCFG33_OFFSET    0x0410 /* High-End Overlay Configuration Register 33 */
#define SAM_LCDC_HEOCFG34_OFFSET    0x0414 /* High-End Overlay Configuration Register 34 */
#define SAM_LCDC_HEOCFG35_OFFSET    0x0418 /* High-End Overlay Configuration Register 35 */
#define SAM_LCDC_HEOCFG36_OFFSET    0x041c /* High-End Overlay Configuration Register 36 */
#define SAM_LCDC_HEOCFG37_OFFSET    0x0420 /* High-End Overlay Configuration Register 37 */
#define SAM_LCDC_HEOCFG38_OFFSET    0x0424 /* High-End Overlay Configuration Register 38 */
#define SAM_LCDC_HEOCFG39_OFFSET    0x0428 /* High-End Overlay Configuration Register 39 */
#define SAM_LCDC_HEOCFG40_OFFSET    0x042c /* High-End Overlay Configuration Register 40 */
#define SAM_LCDC_HEOCFG41_OFFSET    0x0430 /* High-End Overlay Configuration Register 41 */
                                           /* 0x0434-0x043c Reserved */
#ifdef ATSAMA5D3
#  define SAMA5_HAVE_LCDC_HCRCH     1      /* Supports conditional compilation */
#  define SAM_LCDC_HCRCHER_OFFSET   0x0440 /* Hardware Cursor Channel Enable Register */
#  define SAM_LCDC_HCRCHDR_OFFSET   0x0444 /* Hardware Cursor Channel Disable Register */
#  define SAM_LCDC_HCRCHSR_OFFSET   0x0448 /* Hardware Cursor Channel Status Register */
#  define SAM_LCDC_HCRIER_OFFSET    0x044c /* Hardware Cursor Interrupt Enable Register */
#  define SAM_LCDC_HCRIDR_OFFSET    0x0450 /* Hardware Cursor Interrupt Disable Register */
#  define SAM_LCDC_HCRIMR_OFFSET    0x0454 /* Hardware Cursor Interrupt Mask Register */
#  define SAM_LCDC_HCRISR_OFFSET    0x0458 /* Hardware Cursor Interrupt Status Register */
#  define SAM_LCDC_HCRHEAD_OFFSET   0x045c /* Hardware Cursor DMA Head Register */
#  define SAM_LCDC_HCRADDR_OFFSET   0x0460 /* Hardware cursor DMA Address Register */
#  define SAM_LCDC_HCRCTRL_OFFSET   0x0464 /* Hardware Cursor DMA Control Register */
#  define SAM_LCDC_HCRNEXT_OFFSET   0x0468 /* Hardware Cursor DMA Next Register */
#  define SAM_LCDC_HCRCFG0_OFFSET   0x046c /* Hardware Cursor Configuration 0 Register */
#  define SAM_LCDC_HCRCFG1_OFFSET   0x0470 /* Hardware Cursor Configuration 1 Register */
#  define SAM_LCDC_HCRCFG2_OFFSET   0x0474 /* Hardware Cursor Configuration 2 Register */
#  define SAM_LCDC_HCRCFG3_OFFSET   0x0478 /* Hardware Cursor Configuration 3 Register */
#  define SAM_LCDC_HCRCFG4_OFFSET   0x047c /* Hardware Cursor Configuration 4 Register */
                                           /* 0x0480 Reserved */
#  define SAM_LCDC_HCRCFG6_OFFSET   0x0484 /* Hardware Cursor Configuration 6 Register */
#  define SAM_LCDC_HCRCFG7_OFFSET   0x0488 /* Hardware Cursor Configuration 7 Register */
#  define SAM_LCDC_HCRCFG8_OFFSET   0x048c /* Hardware Cursor Configuration 8 Register */
#  define SAM_LCDC_HCRCFG9_OFFSET   0x0490 /* Hardware Cursor Configuration 9 Register */
                                           /* 0x0494-0x053c Reserved */
#endif
#if defined(ATSAMA5D3) || defined(ATSAMA5D2)
#  define SAMA5_HAVE_LCDC_PPCH      1      /* Supports conditional compilation */
#  define SAM_LCDC_PPCHER_OFFSET    0x0540 /* Post Processing Channel Enable Register */
#  define SAM_LCDC_PPCHDR_OFFSET    0x0544 /* Post Processing Channel Disable Register */
#  define SAM_LCDC_PPCHSR_OFFSET    0x0548 /* Post Processing Channel Status Register */
#  define SAM_LCDC_PPIER_OFFSET     0x054c /* Post Processing Interrupt Enable Register */
#  define SAM_LCDC_PPIDR_OFFSET     0x0550 /* Post Processing Interrupt Disable Register */
#  define SAM_LCDC_PPIMR_OFFSET     0x0554 /* Post Processing Interrupt Mask Register */
#  define SAM_LCDC_PPISR_OFFSET     0x0558 /* Post Processing Interrupt Status Register */
#  define SAM_LCDC_PPHEAD_OFFSET    0x055c /* Post Processing Head Register */
#  define SAM_LCDC_PPADDR_OFFSET    0x0560 /* Post Processing Address Register */
#  define SAM_LCDC_PPCTRL_OFFSET    0x0564 /* Post Processing Control Register */
#  define SAM_LCDC_PPNEXT_OFFSET    0x0568 /* Post Processing Next Register */
#  define SAM_LCDC_PPCFG0_OFFSET    0x056c /* Post Processing Configuration Register 0 */
#  define SAM_LCDC_PPCFG1_OFFSET    0x0570 /* Post Processing Configuration Register 1 */
#  define SAM_LCDC_PPCFG2_OFFSET    0x0574 /* Post Processing Configuration Register 2 */
#  define SAM_LCDC_PPCFG3_OFFSET    0x0578 /* Post Processing Configuration Register 3 */
#  define SAM_LCDC_PPCFG4_OFFSET    0x057c /* Post Processing Configuration Register 4 */
#  define SAM_LCDC_PPCFG5_OFFSET    0x0580 /* Post Processing Configuration Register 5 */
#endif
                                           /* 0x0584-0x05fc Reserved */

/* 0x0600-0x08fc Base CLUT Registers 0-255 */

#define SAM_LCDC_BASECLUT_OFFSET(n) (0x0600 + ((n) << 2))

/* 0x0a00-0x0dfc Overlay 1 CLUT Registers 0-255 */

#define SAM_LCDC_OVR1CLUT_OFFSET(n) (0xa00 + ((n) << 2))

/* 0x0e00-0x11fc Overlay 2 CLUT Registers 0-255 */

#define SAM_LCDC_OVR2CLUT_OFFSET(n) (0x0e00 + ((n) << 2))

/* 0x1200-0x15fc High End Overlay CLUT Registers 0-255 */

#define SAM_LCDC_HEOCLUT_OFFSET(n)  (0x1200 + ((n) << 2))

/* 0x1600-0x19fc Hardware Cursor CLUT Registers 0-255 */

#ifdef ATSAMA5D3
#  define SAM_LCDC_HCRCLUT_OFFSET(n) (0x1600 + ((n) << 2))
#endif
                                           /* 0x1a00-0x1fe4 Reserved */

/* LCDC Register Addresses **************************************************/

#define SAM_LCDC_LCDCFG0            (SAM_LCDC_VBASE+SAM_LCDC_LCDCFG0_OFFSET)
#define SAM_LCDC_LCDCFG1            (SAM_LCDC_VBASE+SAM_LCDC_LCDCFG1_OFFSET)
#define SAM_LCDC_LCDCFG2            (SAM_LCDC_VBASE+SAM_LCDC_LCDCFG2_OFFSET)
#define SAM_LCDC_LCDCFG3            (SAM_LCDC_VBASE+SAM_LCDC_LCDCFG3_OFFSET)
#define SAM_LCDC_LCDCFG4            (SAM_LCDC_VBASE+SAM_LCDC_LCDCFG4_OFFSET)
#define SAM_LCDC_LCDCFG5            (SAM_LCDC_VBASE+SAM_LCDC_LCDCFG5_OFFSET)
#define SAM_LCDC_LCDCFG6            (SAM_LCDC_VBASE+SAM_LCDC_LCDCFG6_OFFSET)

#define SAM_LCDC_LCDEN              (SAM_LCDC_VBASE+SAM_LCDC_LCDEN_OFFSET)
#define SAM_LCDC_LCDDIS             (SAM_LCDC_VBASE+SAM_LCDC_LCDDIS_OFFSET)
#define SAM_LCDC_LCDSR              (SAM_LCDC_VBASE+SAM_LCDC_LCDSR_OFFSET)
#define SAM_LCDC_LCDIER             (SAM_LCDC_VBASE+SAM_LCDC_LCDIER_OFFSET)
#define SAM_LCDC_LCDIDR             (SAM_LCDC_VBASE+SAM_LCDC_LCDIDR_OFFSET)
#define SAM_LCDC_LCDIMR             (SAM_LCDC_VBASE+SAM_LCDC_LCDIMR_OFFSET)
#define SAM_LCDC_LCDISR             (SAM_LCDC_VBASE+SAM_LCDC_LCDISR_OFFSET)

#if defined(ATSAMA5D4) || defined(ATSAMA5D2)
#  define SAM_LCDC_LCDATTR          (SAM_LCDC_VBASE+SAM_LCDC_LCDATTR_OFFSET)
#endif

#define SAM_LCDC_BASECHER           (SAM_LCDC_VBASE+SAM_LCDC_BASECHER_OFFSET)
#define SAM_LCDC_BASECHDR           (SAM_LCDC_VBASE+SAM_LCDC_BASECHDR_OFFSET)
#define SAM_LCDC_BASECHSR           (SAM_LCDC_VBASE+SAM_LCDC_BASECHSR_OFFSET)
#define SAM_LCDC_BASEIER            (SAM_LCDC_VBASE+SAM_LCDC_BASEIER_OFFSET)
#define SAM_LCDC_BASEIDR            (SAM_LCDC_VBASE+SAM_LCDC_BASEIDR_OFFSET)
#define SAM_LCDC_BASEIMR            (SAM_LCDC_VBASE+SAM_LCDC_BASEIMR_OFFSET)
#define SAM_LCDC_BASEISR            (SAM_LCDC_VBASE+SAM_LCDC_BASEISR_OFFSET)
#define SAM_LCDC_BASEHEAD           (SAM_LCDC_VBASE+SAM_LCDC_BASEHEAD_OFFSET)
#define SAM_LCDC_BASEADDR           (SAM_LCDC_VBASE+SAM_LCDC_BASEADDR_OFFSET)
#define SAM_LCDC_BASECTRL           (SAM_LCDC_VBASE+SAM_LCDC_BASECTRL_OFFSET)
#define SAM_LCDC_BASENEXT           (SAM_LCDC_VBASE+SAM_LCDC_BASENEXT_OFFSET)
#define SAM_LCDC_BASECFG0           (SAM_LCDC_VBASE+SAM_LCDC_BASECFG0_OFFSET)
#define SAM_LCDC_BASECFG1           (SAM_LCDC_VBASE+SAM_LCDC_BASECFG1_OFFSET)
#define SAM_LCDC_BASECFG2           (SAM_LCDC_VBASE+SAM_LCDC_BASECFG2_OFFSET)
#define SAM_LCDC_BASECFG3           (SAM_LCDC_VBASE+SAM_LCDC_BASECFG3_OFFSET)
#define SAM_LCDC_BASECFG4           (SAM_LCDC_VBASE+SAM_LCDC_BASECFG4_OFFSET)
#define SAM_LCDC_BASECFG5           (SAM_LCDC_VBASE+SAM_LCDC_BASECFG5_OFFSET)
#define SAM_LCDC_BASECFG6           (SAM_LCDC_VBASE+SAM_LCDC_BASECFG6_OFFSET)

#define SAM_LCDC_OVR1CHER           (SAM_LCDC_VBASE+SAM_LCDC_OVR1CHER_OFFSET)
#define SAM_LCDC_OVR1CHDR           (SAM_LCDC_VBASE+SAM_LCDC_OVR1CHDR_OFFSET)
#define SAM_LCDC_OVR1CHSR           (SAM_LCDC_VBASE+SAM_LCDC_OVR1CHSR_OFFSET)
#define SAM_LCDC_OVR1IER            (SAM_LCDC_VBASE+SAM_LCDC_OVR1IER_OFFSET)
#define SAM_LCDC_OVR1IDR            (SAM_LCDC_VBASE+SAM_LCDC_OVR1IDR_OFFSET)
#define SAM_LCDC_OVR1IMR            (SAM_LCDC_VBASE+SAM_LCDC_OVR1IMR_OFFSET)
#define SAM_LCDC_OVR1ISR            (SAM_LCDC_VBASE+SAM_LCDC_OVR1ISR_OFFSET)
#define SAM_LCDC_OVR1HEAD           (SAM_LCDC_VBASE+SAM_LCDC_OVR1HEAD_OFFSET)
#define SAM_LCDC_OVR1ADDR           (SAM_LCDC_VBASE+SAM_LCDC_OVR1ADDR_OFFSET)
#define SAM_LCDC_OVR1CTRL           (SAM_LCDC_VBASE+SAM_LCDC_OVR1CTRL_OFFSET)
#define SAM_LCDC_OVR1NEXT           (SAM_LCDC_VBASE+SAM_LCDC_OVR1NEXT_OFFSET)
#define SAM_LCDC_OVR1CFG0           (SAM_LCDC_VBASE+SAM_LCDC_OVR1CFG0_OFFSET)
#define SAM_LCDC_OVR1CFG1           (SAM_LCDC_VBASE+SAM_LCDC_OVR1CFG1_OFFSET)
#define SAM_LCDC_OVR1CFG2           (SAM_LCDC_VBASE+SAM_LCDC_OVR1CFG2_OFFSET)
#define SAM_LCDC_OVR1CFG3           (SAM_LCDC_VBASE+SAM_LCDC_OVR1CFG3_OFFSET)
#define SAM_LCDC_OVR1CFG4           (SAM_LCDC_VBASE+SAM_LCDC_OVR1CFG4_OFFSET)
#define SAM_LCDC_OVR1CFG5           (SAM_LCDC_VBASE+SAM_LCDC_OVR1CFG5_OFFSET)
#define SAM_LCDC_OVR1CFG6           (SAM_LCDC_VBASE+SAM_LCDC_OVR1CFG6_OFFSET)
#define SAM_LCDC_OVR1CFG7           (SAM_LCDC_VBASE+SAM_LCDC_OVR1CFG7_OFFSET)
#define SAM_LCDC_OVR1CFG8           (SAM_LCDC_VBASE+SAM_LCDC_OVR1CFG8_OFFSET)
#define SAM_LCDC_OVR1CFG9           (SAM_LCDC_VBASE+SAM_LCDC_OVR1CFG9_OFFSET)

#define SAM_LCDC_OVR2CHER           (SAM_LCDC_VBASE+SAM_LCDC_OVR2CHER_OFFSET)
#define SAM_LCDC_OVR2CHDR           (SAM_LCDC_VBASE+SAM_LCDC_OVR2CHDR_OFFSET)
#define SAM_LCDC_OVR2CHSR           (SAM_LCDC_VBASE+SAM_LCDC_OVR2CHSR_OFFSET)
#define SAM_LCDC_OVR2IER            (SAM_LCDC_VBASE+SAM_LCDC_OVR2IER_OFFSET)
#define SAM_LCDC_OVR2IDR            (SAM_LCDC_VBASE+SAM_LCDC_OVR2IDR_OFFSET)
#define SAM_LCDC_OVR2IMR            (SAM_LCDC_VBASE+SAM_LCDC_OVR2IMR_OFFSET)
#define SAM_LCDC_OVR2ISR            (SAM_LCDC_VBASE+SAM_LCDC_OVR2ISR_OFFSET)
#define SAM_LCDC_OVR2HEAD           (SAM_LCDC_VBASE+SAM_LCDC_OVR2HEAD_OFFSET)
#define SAM_LCDC_OVR2ADDR           (SAM_LCDC_VBASE+SAM_LCDC_OVR2ADDR_OFFSET)
#define SAM_LCDC_OVR2CTRL           (SAM_LCDC_VBASE+SAM_LCDC_OVR2CTRL_OFFSET)
#define SAM_LCDC_OVR2NEXT           (SAM_LCDC_VBASE+SAM_LCDC_OVR2NEXT_OFFSET)
#define SAM_LCDC_OVR2CFG0           (SAM_LCDC_VBASE+SAM_LCDC_OVR2CFG0_OFFSET)
#define SAM_LCDC_OVR2CFG1           (SAM_LCDC_VBASE+SAM_LCDC_OVR2CFG1_OFFSET)
#define SAM_LCDC_OVR2CFG2           (SAM_LCDC_VBASE+SAM_LCDC_OVR2CFG2_OFFSET)
#define SAM_LCDC_OVR2CFG3           (SAM_LCDC_VBASE+SAM_LCDC_OVR2CFG3_OFFSET)
#define SAM_LCDC_OVR2CFG4           (SAM_LCDC_VBASE+SAM_LCDC_OVR2CFG4_OFFSET)
#define SAM_LCDC_OVR2CFG5           (SAM_LCDC_VBASE+SAM_LCDC_OVR2CFG5_OFFSET)
#define SAM_LCDC_OVR2CFG6           (SAM_LCDC_VBASE+SAM_LCDC_OVR2CFG6_OFFSET)
#define SAM_LCDC_OVR2CFG7           (SAM_LCDC_VBASE+SAM_LCDC_OVR2CFG7_OFFSET)
#define SAM_LCDC_OVR2CFG8           (SAM_LCDC_VBASE+SAM_LCDC_OVR2CFG8_OFFSET)
#define SAM_LCDC_OVR2CFG9           (SAM_LCDC_VBASE+SAM_LCDC_OVR2CFG9_OFFSET)

#define SAM_LCDC_HEOCHER            (SAM_LCDC_VBASE+SAM_LCDC_HEOCHER_OFFSET)
#define SAM_LCDC_HEOCHDR            (SAM_LCDC_VBASE+SAM_LCDC_HEOCHDR_OFFSET)
#define SAM_LCDC_HEOCHSR            (SAM_LCDC_VBASE+SAM_LCDC_HEOCHSR_OFFSET)
#define SAM_LCDC_HEOIER             (SAM_LCDC_VBASE+SAM_LCDC_HEOIER_OFFSET)
#define SAM_LCDC_HEOIDR             (SAM_LCDC_VBASE+SAM_LCDC_HEOIDR_OFFSET)
#define SAM_LCDC_HEOIMR             (SAM_LCDC_VBASE+SAM_LCDC_HEOIMR_OFFSET)
#define SAM_LCDC_HEOISR             (SAM_LCDC_VBASE+SAM_LCDC_HEOISR_OFFSET)
#define SAM_LCDC_HEOHEAD            (SAM_LCDC_VBASE+SAM_LCDC_HEOHEAD_OFFSET)
#define SAM_LCDC_HEOADDR            (SAM_LCDC_VBASE+SAM_LCDC_HEOADDR_OFFSET)
#define SAM_LCDC_HEOCTRL            (SAM_LCDC_VBASE+SAM_LCDC_HEOCTRL_OFFSET)
#define SAM_LCDC_HEONEXT            (SAM_LCDC_VBASE+SAM_LCDC_HEONEXT_OFFSET)
#define SAM_LCDC_HEOUHEAD           (SAM_LCDC_VBASE+SAM_LCDC_HEOUHEAD_OFFSET)
#define SAM_LCDC_HEOUADDR           (SAM_LCDC_VBASE+SAM_LCDC_HEOUADDR_OFFSET)
#define SAM_LCDC_HEOUCTRL           (SAM_LCDC_VBASE+SAM_LCDC_HEOUCTRL_OFFSET)
#define SAM_LCDC_HEOUNEXT           (SAM_LCDC_VBASE+SAM_LCDC_HEOUNEXT_OFFSET)
#define SAM_LCDC_HEOVHEAD           (SAM_LCDC_VBASE+SAM_LCDC_HEOVHEAD_OFFSET)
#define SAM_LCDC_HEOVADDR           (SAM_LCDC_VBASE+SAM_LCDC_HEOVADDR_OFFSET)
#define SAM_LCDC_HEOVCTRL           (SAM_LCDC_VBASE+SAM_LCDC_HEOVCTRL_OFFSET)
#define SAM_LCDC_HEOVNEXT           (SAM_LCDC_VBASE+SAM_LCDC_HEOVNEXT_OFFSET)
#define SAM_LCDC_HEOCFG0            (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG0_OFFSET)
#define SAM_LCDC_HEOCFG1            (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG1_OFFSET)
#define SAM_LCDC_HEOCFG2            (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG2_OFFSET)
#define SAM_LCDC_HEOCFG3            (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG3_OFFSET)
#define SAM_LCDC_HEOCFG4            (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG4_OFFSET)
#define SAM_LCDC_HEOCFG5            (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG5_OFFSET)
#define SAM_LCDC_HEOCFG6            (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG6_OFFSET)
#define SAM_LCDC_HEOCFG7            (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG7_OFFSET)
#define SAM_LCDC_HEOCFG8            (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG8_OFFSET)
#define SAM_LCDC_HEOCFG9            (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG9_OFFSET)
#define SAM_LCDC_HEOCFG10           (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG10_OFFSET)
#define SAM_LCDC_HEOCFG11           (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG11_OFFSET)
#define SAM_LCDC_HEOCFG12           (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG12_OFFSET)
#define SAM_LCDC_HEOCFG13           (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG13_OFFSET)
#define SAM_LCDC_HEOCFG14           (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG14_OFFSET)
#define SAM_LCDC_HEOCFG15           (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG15_OFFSET)
#define SAM_LCDC_HEOCFG16           (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG16_OFFSET)
#define SAM_LCDC_HEOCFG17           (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG17_OFFSET)
#define SAM_LCDC_HEOCFG18           (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG18_OFFSET)
#define SAM_LCDC_HEOCFG19           (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG19_OFFSET)
#define SAM_LCDC_HEOCFG20           (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG20_OFFSET)
#define SAM_LCDC_HEOCFG21           (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG21_OFFSET)
#define SAM_LCDC_HEOCFG22           (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG22_OFFSET)
#define SAM_LCDC_HEOCFG23           (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG23_OFFSET)
#define SAM_LCDC_HEOCFG24           (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG24_OFFSET)
#define SAM_LCDC_HEOCFG25           (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG25_OFFSET)
#define SAM_LCDC_HEOCFG26           (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG26_OFFSET)
#define SAM_LCDC_HEOCFG27           (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG27_OFFSET)
#define SAM_LCDC_HEOCFG28           (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG28_OFFSET)
#define SAM_LCDC_HEOCFG29           (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG29_OFFSET)
#define SAM_LCDC_HEOCFG30           (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG30_OFFSET)
#define SAM_LCDC_HEOCFG31           (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG31_OFFSET)
#define SAM_LCDC_HEOCFG32           (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG32_OFFSET)
#define SAM_LCDC_HEOCFG33           (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG33_OFFSET)
#define SAM_LCDC_HEOCFG34           (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG34_OFFSET)
#define SAM_LCDC_HEOCFG35           (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG35_OFFSET)
#define SAM_LCDC_HEOCFG36           (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG36_OFFSET)
#define SAM_LCDC_HEOCFG37           (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG37_OFFSET)
#define SAM_LCDC_HEOCFG38           (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG38_OFFSET)
#define SAM_LCDC_HEOCFG39           (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG39_OFFSET)
#define SAM_LCDC_HEOCFG40           (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG40_OFFSET)
#define SAM_LCDC_HEOCFG41           (SAM_LCDC_VBASE+SAM_LCDC_HEOCFG41_OFFSET)

#ifdef ATSAMA5D3
#  define SAM_LCDC_HCRCHER          (SAM_LCDC_VBASE+SAM_LCDC_HCRCHER_OFFSET)
#  define SAM_LCDC_HCRCHDR          (SAM_LCDC_VBASE+SAM_LCDC_HCRCHDR_OFFSET)
#  define SAM_LCDC_HCRCHSR          (SAM_LCDC_VBASE+SAM_LCDC_HCRCHSR_OFFSET)
#  define SAM_LCDC_HCRIER           (SAM_LCDC_VBASE+SAM_LCDC_HCRIER_OFFSET)
#  define SAM_LCDC_HCRIDR           (SAM_LCDC_VBASE+SAM_LCDC_HCRIDR_OFFSET)
#  define SAM_LCDC_HCRIMR           (SAM_LCDC_VBASE+SAM_LCDC_HCRIMR_OFFSET)
#  define SAM_LCDC_HCRISR           (SAM_LCDC_VBASE+SAM_LCDC_HCRISR_OFFSET)
#  define SAM_LCDC_HCRHEAD          (SAM_LCDC_VBASE+SAM_LCDC_HCRHEAD_OFFSET)
#  define SAM_LCDC_HCRADDR          (SAM_LCDC_VBASE+SAM_LCDC_HCRADDR_OFFSET)
#  define SAM_LCDC_HCRCTRL          (SAM_LCDC_VBASE+SAM_LCDC_HCRCTRL_OFFSET)
#  define SAM_LCDC_HCRNEXT          (SAM_LCDC_VBASE+SAM_LCDC_HCRNEXT_OFFSET)
#  define SAM_LCDC_HCRCFG0          (SAM_LCDC_VBASE+SAM_LCDC_HCRCFG0_OFFSET)
#  define SAM_LCDC_HCRCFG1          (SAM_LCDC_VBASE+SAM_LCDC_HCRCFG1_OFFSET)
#  define SAM_LCDC_HCRCFG2          (SAM_LCDC_VBASE+SAM_LCDC_HCRCFG2_OFFSET)
#  define SAM_LCDC_HCRCFG3          (SAM_LCDC_VBASE+SAM_LCDC_HCRCFG3_OFFSET)
#  define SAM_LCDC_HCRCFG4          (SAM_LCDC_VBASE+SAM_LCDC_HCRCFG4_OFFSET)
#  define SAM_LCDC_HCRCFG6          (SAM_LCDC_VBASE+SAM_LCDC_HCRCFG6_OFFSET)
#  define SAM_LCDC_HCRCFG7          (SAM_LCDC_VBASE+SAM_LCDC_HCRCFG7_OFFSET)
#  define SAM_LCDC_HCRCFG8          (SAM_LCDC_VBASE+SAM_LCDC_HCRCFG8_OFFSET)
#  define SAM_LCDC_HCRCFG9          (SAM_LCDC_VBASE+SAM_LCDC_HCRCFG9_OFFSET)
#endif

#if defined(ATSAMA5D2) || defined(ATSAMA5D3)
#  define SAM_LCDC_PPCHER           (SAM_LCDC_VBASE+SAM_LCDC_PPCHER_OFFSET)
#  define SAM_LCDC_PPCHDR           (SAM_LCDC_VBASE+SAM_LCDC_PPCHDR_OFFSET)
#  define SAM_LCDC_PPCHSR           (SAM_LCDC_VBASE+SAM_LCDC_PPCHSR_OFFSET)
#  define SAM_LCDC_PPIER            (SAM_LCDC_VBASE+SAM_LCDC_PPIER_OFFSET)
#  define SAM_LCDC_PPIDR            (SAM_LCDC_VBASE+SAM_LCDC_PPIDR_OFFSET)
#  define SAM_LCDC_PPIMR            (SAM_LCDC_VBASE+SAM_LCDC_PPIMR_OFFSET)
#  define SAM_LCDC_PPISR            (SAM_LCDC_VBASE+SAM_LCDC_PPISR_OFFSET)
#  define SAM_LCDC_PPHEAD           (SAM_LCDC_VBASE+SAM_LCDC_PPHEAD_OFFSET)
#  define SAM_LCDC_PPADDR           (SAM_LCDC_VBASE+SAM_LCDC_PPADDR_OFFSET)
#  define SAM_LCDC_PPCTRL           (SAM_LCDC_VBASE+SAM_LCDC_PPCTRL_OFFSET)
#  define SAM_LCDC_PPNEXT           (SAM_LCDC_VBASE+SAM_LCDC_PPNEXT_OFFSET)
#  define SAM_LCDC_PPCFG0           (SAM_LCDC_VBASE+SAM_LCDC_PPCFG0_OFFSET)
#  define SAM_LCDC_PPCFG1           (SAM_LCDC_VBASE+SAM_LCDC_PPCFG1_OFFSET)
#  define SAM_LCDC_PPCFG2           (SAM_LCDC_VBASE+SAM_LCDC_PPCFG2_OFFSET)
#  define SAM_LCDC_PPCFG3           (SAM_LCDC_VBASE+SAM_LCDC_PPCFG3_OFFSET)
#  define SAM_LCDC_PPCFG4           (SAM_LCDC_VBASE+SAM_LCDC_PPCFG4_OFFSET)
#  define SAM_LCDC_PPCFG5           (SAM_LCDC_VBASE+SAM_LCDC_PPCFG5_OFFSET)
#endif

/* 0x0600-0x08fc Base CLUT Registers 0-255 */

#define SAM_LCDC_BASECLUT(n)        (SAM_LCDC_VBASE+SAM_LCDC_BASECLUT_OFFSET(n))

/* 0x0a00-0x0dfc Overlay 1 CLUT Registers 0-255 */

#define SAM_LCDC_OVR1CLUT(n)        (SAM_LCDC_VBASE+SAM_LCDC_OVR1CLUT_OFFSET(n))

/* 0x0e00-0x11fc Overlay 2 CLUT Registers 0-255 */

#define SAM_LCDC_OVR2CLUT(n)        (SAM_LCDC_VBASE+SAM_LCDC_OVR2CLUT_OFFSET(n))

/* 0x1200-0x15fc High End Overlay CLUT Registers 0-255 */

#define SAM_LCDC_HEOCLUT(n)         (SAM_LCDC_VBASE+SAM_LCDC_HEOCLUT_OFFSET(n))

/* 0x1600-0x19fc Hardware Cursor CLUT Registers 0-255 */

#ifdef ATSAMA5D3
#  define SAM_LCDC_HCRCLUT(n)       (SAM_LCDC_VBASE+SAM_LCDC_HCRCLUT_OFFSET(n))
#endif

/* LCDC Register Bit Definitions ********************************************/

/* LCD Controller Configuration Register 0 */

#define LCDC_LCDCFG0_CLKPOL         (1 << 0)  /* Bit 0:  LCD Controller Clock Polarity */
#define LCDC_LCDCFG0_CLKSEL         (1 << 2)  /* Bit 2:  LCD Controller Clock Source */
#define LCDC_LCDCFG0_CLKPWMSEL      (1 << 3)  /* Bit 3:  LCD Controller PWM Clock Source */
#define LCDC_LCDCFG0_CGDISBASE      (1 << 8)  /* Bit 8:  Clock Gating Disable Control Base */
#define LCDC_LCDCFG0_CGDISOVR1      (1 << 9)  /* Bit 9:  Clock Gating Disable Control Overlay 1 */
#define LCDC_LCDCFG0_CGDISOVR2      (1 << 10) /* Bit 10: Clock Gating Disable Control Overlay 2 */
#define LCDC_LCDCFG0_CGDISHEO       (1 << 11) /* Bit 11: Clock Gating Disable Control HE Overlay */

#ifdef ATSAMA5D3
#  define LCDC_LCDCFG0_CGDISHCR     (1 << 12) /* Bit 12: Clock Gating Disable Control NW Cursor */
#endif

#if defined(ATSAMA5D2) || defined(ATSAMA5D3)
#  define LCDC_LCDCFG0_CGDISPP      (1 << 13) /* Bit 13: Clock Gating Disable Control PP */
#endif

#define LCDC_LCDCFG0_CLKDIV_SHIFT   (16)      /* Bits 16-23: LCD Controller Clock Divider */
#define LCDC_LCDCFG0_CLKDIV_MASK    (0xff << LCDC_LCDCFG0_CLKDIV_SHIFT)
#  define LCDC_LCDCFG0_CLKDIV(n)    ((uint32_t)(n) << LCDC_LCDCFG0_CLKDIV_SHIFT)

/* LCD Controller Configuration Register 1 */

#if defined(ATSAMA5D3) || defined(ATSAMA5D2)
#  define LCDC_LCDCFG1_HSPW_SHIFT   (0)       /* Bits 0-5: Horizontal Sync Pulse Width */
#  define LCDC_LCDCFG1_HSPW_MASK    (0x3f << LCDC_LCDCFG1_HSPW_SHIFT)
#    define LCDC_LCDCFG1_HSPW(n)    ((uint32_t)(n) << LCDC_LCDCFG1_HSPW_SHIFT)
#  define LCDC_LCDCFG1_VSPW_SHIFT   (16)      /* Bits 16-21: Vertical Sync Pulse Width */
#  define LCDC_LCDCFG1_VSPW_MASK    (0x3f << LCDC_LCDCFG1_VSPW_SHIFT)
#    define LCDC_LCDCFG1_VSPW(n)    ((uint32_t)(n) << LCDC_LCDCFG1_VSPW_SHIFT)
#elif defined(ATSAMA5D4)
#  define LCDC_LCDCFG1_HSPW_SHIFT   (0)       /* Bits 0-7: Horizontal Sync Pulse Width */
#  define LCDC_LCDCFG1_HSPW_MASK    (0xff << LCDC_LCDCFG1_HSPW_SHIFT)
#    define LCDC_LCDCFG1_HSPW(n)    ((uint32_t)(n) << LCDC_LCDCFG1_HSPW_SHIFT)
#  define LCDC_LCDCFG1_VSPW_SHIFT   (16)      /* Bits 16-23: Vertical Sync Pulse Width */
#  define LCDC_LCDCFG1_VSPW_MASK    (0xff << LCDC_LCDCFG1_VSPW_SHIFT)
#    define LCDC_LCDCFG1_VSPW(n)    ((uint32_t)(n) << LCDC_LCDCFG1_VSPW_SHIFT)
#endif

/* LCD Controller Configuration Register 2 */

#if defined(ATSAMA5D3) || defined(ATSAMA5D2)
#  define LCDC_LCDCFG2_VFPW_SHIFT   (0)       /* Bits 0-5: Vertical Front Porch Width */
#  define LCDC_LCDCFG2_VFPW_MASK    (0x3f << LCDC_LCDCFG2_VFPW_SHIFT)
#    define LCDC_LCDCFG2_VFPW(n)    ((uint32_t)(n) << LCDC_LCDCFG2_VFPW_SHIFT)
#  define LCDC_LCDCFG2_VBPW_SHIFT   (16)      /* Bits 16-21: Vertical Back Porch Width */
#  define LCDC_LCDCFG2_VBPW_MASK    (0x3f << LCDC_LCDCFG2_VBPW_SHIFT)
#    define LCDC_LCDCFG2_VBPW(n)    ((uint32_t)(n) << LCDC_LCDCFG2_VBPW_SHIFT)
#elif defined(ATSAMA5D4)
#  define LCDC_LCDCFG2_VFPW_SHIFT   (0)       /* Bits 0-7: Vertical Front Porch Width */
#  define LCDC_LCDCFG2_VFPW_MASK    (0xff << LCDC_LCDCFG2_VFPW_SHIFT)
#    define LCDC_LCDCFG2_VFPW(n)    ((uint32_t)(n) << LCDC_LCDCFG2_VFPW_SHIFT)
#  define LCDC_LCDCFG2_VBPW_SHIFT   (16)      /* Bits 16-23: Vertical Back Porch Width */
#  define LCDC_LCDCFG2_VBPW_MASK    (0xff << LCDC_LCDCFG2_VBPW_SHIFT)
#    define LCDC_LCDCFG2_VBPW(n)    ((uint32_t)(n) << LCDC_LCDCFG2_VBPW_SHIFT)
#endif

/* LCD Controller Configuration Register 3 */

#if defined(ATSAMA5D2)
#  define LCDC_LCDCFG3_HFPW_SHIFT   (0)       /* Bits 0-8: Horizontal Front Porch Width */
#  define LCDC_LCDCFG3_HFPW_MASK    (0x3ff << LCDC_LCDCFG3_HFPW_SHIFT)
#    define LCDC_LCDCFG3_HFPW(n)    ((uint32_t)(n) << LCDC_LCDCFG3_HFPW_SHIFT)
#  define LCDC_LCDCFG3_HBPW_SHIFT   (16)      /* Bits 16-24: Horizontal Back Porch Width */
#  define LCDC_LCDCFG3_HBPW_MASK    (0x3ff << LCDC_LCDCFG3_HBPW_SHIFT)
#    define LCDC_LCDCFG3_HBPW(n)    ((uint32_t)(n) << LCDC_LCDCFG3_HBPW_SHIFT)
#elif defined(ATSAMA5D3)
#  define LCDC_LCDCFG3_HFPW_SHIFT   (0)       /* Bits 0-8: Horizontal Front Porch Width */
#  define LCDC_LCDCFG3_HFPW_MASK    (0x1ff << LCDC_LCDCFG3_HFPW_SHIFT)
#    define LCDC_LCDCFG3_HFPW(n)    ((uint32_t)(n) << LCDC_LCDCFG3_HFPW_SHIFT)
#  define LCDC_LCDCFG3_HBPW_SHIFT   (16)      /* Bits 16-24: Horizontal Back Porch Width */
#  define LCDC_LCDCFG3_HBPW_MASK    (0x1ff << LCDC_LCDCFG3_HBPW_SHIFT)
#    define LCDC_LCDCFG3_HBPW(n)    ((uint32_t)(n) << LCDC_LCDCFG3_HBPW_SHIFT)
#elif defined(ATSAMA5D4)
#  define LCDC_LCDCFG3_HFPW_SHIFT   (0)       /* Bits 0-9: Horizontal Front Porch Width */
#  define LCDC_LCDCFG3_HFPW_MASK    (0x3ff << LCDC_LCDCFG3_HFPW_SHIFT)
#    define LCDC_LCDCFG3_HFPW(n)    ((uint32_t)(n) << LCDC_LCDCFG3_HFPW_SHIFT)
#  define LCDC_LCDCFG3_HBPW_SHIFT   (16)      /* Bits 16-25: Horizontal Back Porch Width */
#  define LCDC_LCDCFG3_HBPW_MASK    (0x3ff << LCDC_LCDCFG3_HBPW_SHIFT)
#    define LCDC_LCDCFG3_HBPW(n)    ((uint32_t)(n) << LCDC_LCDCFG3_HBPW_SHIFT)
#endif

/* LCD Controller Configuration Register 4 */

#define LCDC_LCDCFG4_PPL_SHIFT      (0)       /* Bits 0-10: Number of Pixels Per Line */
#define LCDC_LCDCFG4_PPL_MASK       (0x7ff << LCDC_LCDCFG4_PPL_SHIFT)
#  define LCDC_LCDCFG4_PPL(n)       ((uint32_t)(n) << LCDC_LCDCFG4_PPL_SHIFT)
#define LCDC_LCDCFG4_RPF_SHIFT      (16)      /* Bits 16-26: Number of Active Row Per Frame */
#define LCDC_LCDCFG4_RPF_MASK       (0x7ff << LCDC_LCDCFG4_RPF_SHIFT)
#  define LCDC_LCDCFG4_RPF(n)       ((uint32_t)(n) << LCDC_LCDCFG4_RPF_SHIFT)

/* LCD Controller Configuration Register 5 */

#define LCDC_LCDCFG5_HSPOL          (1 << 0)  /* Bit 0:  Horizontal Synchronization Pulse Polarity */
#define LCDC_LCDCFG5_VSPOL          (1 << 1)  /* Bit 1:  VSync Pulse Polarity */
#define LCDC_LCDCFG5_VSPDLYS        (1 << 2)  /* Bit 2:  VSync Pulse Start */
#define LCDC_LCDCFG5_VSPDLYE        (1 << 3)  /* Bit 3:  VSync Pulse End */
#define LCDC_LCDCFG5_DISPPOL        (1 << 4)  /* Bit 4:  Display Signal Polarity */
#define LCDC_LCDCFG5_DITHER         (1 << 6)  /* Bit 6:  LCDC Dithering */
#define LCDC_LCDCFG5_DISPDLY        (1 << 7)  /* Bit 7:  LCDC Power Signal Sync */
#define LCDC_LCDCFG5_MODE_SHIFT     (8)       /* Bits 8-9: LCDC Output Mode */
#define LCDC_LCDCFG5_MODE_MASK      (3 << LCDC_LCDCFG5_MODE_SHIFT)
#  define LCDC_LCDCFG5_MODE_12BPP   (0 << LCDC_LCDCFG5_MODE_SHIFT) /* Output mode 12 bits per pixel */
#  define LCDC_LCDCFG5_MODE_16BPP   (1 << LCDC_LCDCFG5_MODE_SHIFT) /* Output mode 16 bits per pixel */
#  define LCDC_LCDCFG5_MODE_18BPP   (2 << LCDC_LCDCFG5_MODE_SHIFT) /* Output mode 18 bits per pixel */
#  define LCDC_LCDCFG5_MODE_24BPP   (3 << LCDC_LCDCFG5_MODE_SHIFT) /* Output mode 24 bits per pixel */

#if defined(ATSAMA5D2) || defined(ATSAMA5D3)
#  define LCDC_LCDCFG5_PP           (1 << 10) /* Bit 10: Post Processing Enable */
#endif

#define LCDC_LCDCFG5_VSPSU          (1 << 12) /* Bit 12: LCDC VSync Pulse Setup Configuration */
#define LCDC_LCDCFG5_VSPHO          (1 << 13) /* Bit 13: LCDC VSync Pulse Hold Configuration */

#if defined(ATSAMA5D2)
#  define LCDC_LCDCFG5_GUARDTIME_SHIFT (16)     /* Bits 16-20: LCD DISPLAY Guard Time */
#  define LCDC_LCDCFG5_GUARDTIME_MASK  (0x0f << LCDC_LCDCFG5_GUARDTIME_SHIFT)
#    define LCDC_LCDCFG5_GUARDTIME(n)  ((uint32_t)(n) << LCDC_LCDCFG5_GUARDTIME_SHIFT)
#elif defined(ATSAMA5D3)
#  define LCDC_LCDCFG5_GUARDTIME_SHIFT (16)     /* Bits 16-20: LCD DISPLAY Guard Time */
#  define LCDC_LCDCFG5_GUARDTIME_MASK  (0x1f << LCDC_LCDCFG5_GUARDTIME_SHIFT)
#    define LCDC_LCDCFG5_GUARDTIME(n)  ((uint32_t)(n) << LCDC_LCDCFG5_GUARDTIME_SHIFT)
#elif defined(ATSAMA5D4)
#  define LCDC_LCDCFG5_GUARDTIME_SHIFT (16)     /* Bits 16-23: LCD DISPLAY Guard Time */
#  define LCDC_LCDCFG5_GUARDTIME_MASK  (0xff << LCDC_LCDCFG5_GUARDTIME_SHIFT)
#    define LCDC_LCDCFG5_GUARDTIME(n)  ((uint32_t)(n) << LCDC_LCDCFG5_GUARDTIME_SHIFT)
#endif

/* LCD Controller Configuration Register 6 */

#define LCDC_LCDCFG6_PWMPS_SHIFT    (0)       /* Bits 0-2: PWM Clock Prescaler */
#define LCDC_LCDCFG6_PWMPS_MASK     (7 << LCDC_LCDCFG6_PWMPS_SHIFT)
#  define LCDC_LCDCFG6_PWMPS_DIV1   (0 << LCDC_LCDCFG6_PWMPS_SHIFT) /* Fcounter = Fpwm_selected_clock */
#  define LCDC_LCDCFG6_PWMPS_DIV2   (1 << LCDC_LCDCFG6_PWMPS_SHIFT) /* Fcounter = Fpwm_selected_clock/2 */
#  define LCDC_LCDCFG6_PWMPS_DIV4   (2 << LCDC_LCDCFG6_PWMPS_SHIFT) /* Fcounter = Fpwm_selected_clock/4 */
#  define LCDC_LCDCFG6_PWMPS_DIV8   (3 << LCDC_LCDCFG6_PWMPS_SHIFT) /* Fcounter = Fpwm_selected_clock/8 */
#  define LCDC_LCDCFG6_PWMPS_DIV16  (4 << LCDC_LCDCFG6_PWMPS_SHIFT) /* Fcounter = Fpwm_selected_clock/16 */
#  define LCDC_LCDCFG6_PWMPS_DIV32  (5 << LCDC_LCDCFG6_PWMPS_SHIFT) /* Fcounter = Fpwm_selected_clock/32 */
#  define LCDC_LCDCFG6_PWMPS_DIV64  (6 << LCDC_LCDCFG6_PWMPS_SHIFT) /* Fcounter = Fpwm_selected_clock/64 */

#define LCDC_LCDCFG6_PWMPOL         (1 << 4)  /* Bit 4: LCD Controller PWM Signal Polarity */
#define LCDC_LCDCFG6_PWMCVAL_SHIFT  (8)       /* Bits 8-15: LCD Controller PWM Compare Value */
#define LCDC_LCDCFG6_PWMCVAL_MASK   (0xff << LCDC_LCDCFG6_PWMCVAL_SHIFT)
#  define LCDC_LCDCFG6_PWMCVAL(n)   ((uint32_t)(n) << LCDC_LCDCFG6_PWMCVAL_SHIFT)

/* LCD Controller Enable Register */

#define LCDC_LCDEN_CLK              (1 << 0)  /* Bit 0:  LCDC Pixel Clock Enable */
#define LCDC_LCDEN_SYNC             (1 << 1)  /* Bit 1:  LCDC H/V Sync Enable */
#define LCDC_LCDEN_DISP             (1 << 2)  /* Bit 2:  LCDC DISP Signal Enable */
#define LCDC_LCDEN_PWM              (1 << 3)  /* Bit 3:  LCDC PWM Enable */

/* LCD Controller Disable Register */

#define LCDC_LCDDIS_CLK             (1 << 0)  /* Bit 0:  LCDC Pixel Clock Disable */
#define LCDC_LCDDIS_SYNC            (1 << 1)  /* Bit 1:  LCDC H/V Sync Disable */
#define LCDC_LCDDIS_DISP            (1 << 2)  /* Bit 2:  LCDC DISP Signal Disable */
#define LCDC_LCDDIS_PWM             (1 << 3)  /* Bit 3:  LCDC PWM Disable */
#define LCDC_LCDDIS_CLKRST          (1 << 8)  /* Bit 8:  LCDC Clock Reset */
#define LCDC_LCDDIS_SYNCRST         (1 << 9)  /* Bit 9:  LCDC H/V Sync Reset */
#define LCDC_LCDDIS_DISPRST         (1 << 10) /* Bit 10: LCDC DISP Signal Reset */
#define LCDC_LCDDIS_PWMRST          (1 << 11) /* Bit 11: LCDC PWM Reset */

/* LCD Controller Status Register */

#define LCDC_LCDSR_CLK              (1 << 0)  /* Bit 0:  Clock Status */
#define LCDC_LCDSR_LCD              (1 << 1)  /* Bit 1:  LCDC Sync Status */
#define LCDC_LCDSR_DISP             (1 << 2)  /* Bit 2:  LCDC DISP Signal Status */
#define LCDC_LCDSR_PWM              (1 << 3)  /* Bit 3:  LCDC PWM Signal Status */
#define LCDC_LCDSR_SIP              (1 << 4)  /* Bit 4:  Synchronization In Progress */

/* LCD Controller Interrupt Enable Register,
 * LCD Controller Interrupt Disable Register,
 * LCD Controller Interrupt Mask Register,
 * and LCD Controller Interrupt Status Register
 */

#define LCDC_LCDINT_SOF             (1 << 0)  /* Bit 0:  Start of Frame Interrupt */
#define LCDC_LCDINT_DIS             (1 << 1)  /* Bit 1:  LCD Disable Interrupt */
#define LCDC_LCDINT_DISP            (1 << 2)  /* Bit 2:  Power-up/down Sequence Terminated */
#define LCDC_LCDINT_FIFOERR         (1 << 4)  /* Bit 4:  Output FIFO Error */
#define LCDC_LCDINT_BASE            (1 << 8)  /* Bit 8:  Base Layer Raw Interrupt */
#define LCDC_LCDINT_OVR1            (1 << 9)  /* Bit 9:  Overlay 1 Raw Interrupt */
#define LCDC_LCDINT_OVR2            (1 << 10) /* Bit 10: Overlay 2 Raw Interrupt */
#define LCDC_LCDINT_HEO             (1 << 11) /* Bit 11: High End Overlay Raw Interrupt */

#if defined(ATSAMA5D2)
#  define LCDC_LCDINT_PP            (1 << 13) /* Bit 13: Post Processing Raw Interrupt */
#  define LCDC_LCDINT_ALL           (0x00002f17)
#elif defined(ATSAMA5D3)
#  define LCDC_LCDINT_HCR           (1 << 12) /* Bit 12: Hardware Cursor Raw Interrupt */
#  define LCDC_LCDINT_PP            (1 << 13) /* Bit 13: Post Processing Raw Interrupt */
#  define LCDC_LCDINT_ALL           (0x00003f17)
#elif defined(ATSAMA5D4)
#  define LCDC_LCDINT_ALL           (0x00000f17)
#endif

#if defined(ATSAMA5D2) || defined(ATSAMA5D4)
/* LCD Controller Attribute Register */

#  define LCDC_LCDATTR_BASE         (1 << 0)  /* Bit 0:  Base Layer Update Attribute Register */
#  define LCDC_LCDATTR_OVR1         (1 << 1)  /* Bit 1:  Overlay 1 Update Attribute Register */
#  define LCDC_LCDATTR_OVR2         (1 << 2)  /* Bit 2:  Overlay 2 Update Attribute Register */
#  define LCDC_LCDATTR_HEO          (1 << 3)  /* Bit 3:  High-End Overlay Update Attribute Register */
#  define LCDC_LCDATTR_BASEA2Q      (1 << 8)  /* Bit 8:  Base Layer Update Attribute Register */
#  define LCDC_LCDATTR_OVR1A2Q      (1 << 9)  /* Bit 9:  Overlay 1 Update Attribute Register */
#  define LCDC_LCDATTR_OVR2A2Q      (1 << 10) /* Bit 10: Overlay 2 Update Attribute Register */
#  define LCDC_LCDATTR_HEOA2Q       (1 << 11) /* Bit 11: High-End Overlay Update Attribute Register */
#ifdef ATSAMA5D2
#  define LCDC_LCDATTR_PPA2Q        (1 << 13) /* Bit 13: Post Processing Update Add To Queue Register */
#endif
#endif

/* Base Layer Channel Enable Register */

#define LCDC_BASECHER_CH            (1 << 0)  /* Bit 0:  Channel Enable */
#define LCDC_BASECHER_UPDATE        (1 << 1)  /* Bit 1:  Update Overlay Attributes Enable */
#define LCDC_BASECHER_A2Q           (1 << 2)  /* Bit 2:  Add Head Pointer Enable */

/* Base Layer Channel Disable Register */

#define LCDC_BASECHDR_CH            (1 << 0)  /* Bit 0:  Channel Disable */
#define LCDC_BASECHDR_CHRST         (1 << 8)  /* Bit 8:  Channel Reset */

/* Base Layer Channel Status Register */

#define LCDC_BASECHSR_CH            (1 << 0)  /* Bit 0:  Channel Status */
#define LCDC_BASECHSR_UPDATE        (1 << 1)  /* Bit 1:  Update Overlay Attributes In */
#define LCDC_BASECHSR_A2Q           (1 << 2)  /* Bit 2:  Add To Queue Pending */

/* Base Layer Interrupt Enable Register,
 * Base Layer Interrupt Disable Register,
 * Base Layer Interrupt Mask Register,
 * and Base Layer Interrupt Status Register.
 */

#define LCDC_BASEINT_DMA            (1 << 2)  /* Bit 2:  End of DMA Transfer */
#define LCDC_BASEINT_DSCR           (1 << 3)  /* Bit 3:  DMA Descriptor Loaded */
#define LCDC_BASEINT_ADD            (1 << 4)  /* Bit 4:  Head Descriptor Loaded */
#define LCDC_BASEINT_DONE           (1 << 5)  /* Bit 5:  End of List Detected */
#define LCDC_BASEINT_OVR            (1 << 6)  /* Bit 6:  Overflow Detected */

/* Base DMA Head Register */

#define LCDC_BASEHEAD_MASK          (0xfffffffc) /* Bits 2-31: DMA Head Pointer */

/* Base DMA Address Register (32-bit address) */

/* Base DMA Control Register */

#define LCDC_BASECTRL_DFETCH        (1 << 0)  /* Bit 0:  Transfer Descriptor Fetch Enable */
#define LCDC_BASECTRL_LFETCH        (1 << 1)  /* Bit 1:  Lookup Table Fetch Enable */
#define LCDC_BASECTRL_DMAIEN        (1 << 2)  /* Bit 2:  End of DMA Transfer Interrupt Enable */
#define LCDC_BASECTRL_DSCRIEN       (1 << 3)  /* Bit 3:  Descriptor Loaded Interrupt Enable */
#define LCDC_BASECTRL_ADDIEN        (1 << 4)  /* Bit 4:  Add Head Descriptor to Queue Interrupt Enable */
#define LCDC_BASECTRL_DONEIEN       (1 << 5)  /* Bit 5:  End of List Interrupt Enable */

/* Base DMA Next Register (32-bit address) */

/* Base Configuration register 0 */

#define LCDC_BASECFG0_SIF           (1 << 0)  /* Bit 0:  Source Interface */
#define LCDC_BASECFG0_BLEN_SHIFT    (4)       /* Bits 4-5: AHB Burst Length */
#define LCDC_BASECFG0_BLEN_MASK     (3 << LCDC_BASECFG0_BLEN_SHIFT)
#  define LCDC_BASECFG0_BLEN_SINGLE (0 << LCDC_BASECFG0_BLEN_SHIFT)
#  define LCDC_BASECFG0_BLEN_INCR4  (1 << LCDC_BASECFG0_BLEN_SHIFT)
#  define LCDC_BASECFG0_BLEN_INCR8  (2 << LCDC_BASECFG0_BLEN_SHIFT)
#  define LCDC_BASECFG0_BLEN_INCR16 (3 << LCDC_BASECFG0_BLEN_SHIFT)
#define LCDC_BASECFG0_DLBO          (1 << 8)  /* Bit 8: Defined Length Burst Only */

/* Base Configuration register 1 */

#define LCDC_BASECFG1_CLUTEN           (1 << 0)  /* Bit 0:  Color Lookup Table Enable */
#define LCDC_BASECFG1_RGBMODE_SHIFT    (4)       /* Bits 4-7: RGB Input Mode Selection */
#define LCDC_BASECFG1_RGBMODE_MASK     (15 << LCDC_BASECFG1_RGBMODE_SHIFT)
#  define LCDC_BASECFG1_12BPP_RGB444   (0 << LCDC_BASECFG1_RGBMODE_SHIFT)  /* 12 bpp RGB 444 */
#  define LCDC_BASECFG1_16BPP_ARGB4444 (1 << LCDC_BASECFG1_RGBMODE_SHIFT)  /* 16 bpp ARGB 4444 */
#  define LCDC_BASECFG1_16BPP_RGBA4444 (2 << LCDC_BASECFG1_RGBMODE_SHIFT)  /* 16 bpp RGBA 4444 */
#  define LCDC_BASECFG1_16BPP_RGB565   (3 << LCDC_BASECFG1_RGBMODE_SHIFT)  /* 16 bpp RGB 565 */
#  define LCDC_BASECFG1_16BPP_TRGB1555 (4 << LCDC_BASECFG1_RGBMODE_SHIFT)  /* 16 bpp TRGB 1555 */
#  define LCDC_BASECFG1_18BPP_RGB666   (5 << LCDC_BASECFG1_RGBMODE_SHIFT)  /* 18 bpp RGB 666 */
#  define LCDC_BASECFG1_18BPP_RGB666P  (6 << LCDC_BASECFG1_RGBMODE_SHIFT)  /* 18 bpp RGB 666 PACKED */
#  define LCDC_BASECFG1_19BPP_TRGB1666 (7 << LCDC_BASECFG1_RGBMODE_SHIFT)  /* 19 bpp TRGB 1666 */
#  define LCDC_BASECFG1_19BPP_TRGBP    (8 << LCDC_BASECFG1_RGBMODE_SHIFT)  /* 19 bpp TRGB 1666 PACKED */
#  define LCDC_BASECFG1_24BPP_RGB888   (9 << LCDC_BASECFG1_RGBMODE_SHIFT)  /* 24 bpp RGB 888 */
#  define LCDC_BASECFG1_24BPP_RGB888P  (10 << LCDC_BASECFG1_RGBMODE_SHIFT) /* 24 bpp RGB 888 PACKED */
#  define LCDC_BASECFG1_25BPP_TRGB1888 (11 << LCDC_BASECFG1_RGBMODE_SHIFT) /* 25 bpp TRGB 1888 */
#  define LCDC_BASECFG1_32BPP_ARGB8888 (12 << LCDC_BASECFG1_RGBMODE_SHIFT) /* 32 bpp ARGB 8888 */
#  define LCDC_BASECFG1_32BPP_RGBA8888 (13 << LCDC_BASECFG1_RGBMODE_SHIFT) /* 32 bpp RGBA 8888 */

#define LCDC_BASECFG1_CLUTMODE_SHIFT   (8)       /* Bits 8-9: CLUT Input Mode Selection */
#define LCDC_BASECFG1_CLUTMODE_MASK    (3 << LCDC_BASECFG1_CLUTMODE_SHIFT)
#  define LCDC_BASECFG1_CLUTMODE_1BPP  (0 << LCDC_BASECFG1_CLUTMODE_SHIFT) /* CLUT input 1 bit per pixel */
#  define LCDC_BASECFG1_CLUTMODE_2BPP  (1 << LCDC_BASECFG1_CLUTMODE_SHIFT) /* CLUT input 2 bits per pixel */
#  define LCDC_BASECFG1_CLUTMODE_4BPP  (2 << LCDC_BASECFG1_CLUTMODE_SHIFT) /* CLUT input 4 bits per pixel */
#  define LCDC_BASECFG1_CLUTMODE_8BPP  (3 << LCDC_BASECFG1_CLUTMODE_SHIFT) /* CLUT input 8 bits per pixel */

/* Base Configuration register 2 (32-bit value) */

/* Base Configuration register 3 */

#define LCDC_BASECFG3_BDEF_SHIFT    (0)       /* Bits 0-7: B Default */
#define LCDC_BASECFG3_BDEF_MASK     (0xff << LCDC_BASECFG3_BDEF_SHIFT)
#  define LCDC_BASECFG3_BDEF(n)     ((uint32_t)(n) << LCDC_BASECFG3_BDEF_SHIFT)
#define LCDC_BASECFG3_GDEF_SHIFT    (8)       /* Bits 8-15: G Default */
#define LCDC_BASECFG3_GDEF_MASK     (0xff << LCDC_BASECFG3_GDEF_SHIFT)
#  define LCDC_BASECFG3_GDEF(n)     ((uint32_t)(n) << LCDC_BASECFG3_GDEF_SHIFT)
#define LCDC_BASECFG3_RDEF_SHIFT    (16)      /* Bits 16-23: R Default */
#define LCDC_BASECFG3_RDEF_MASK     (0xff << LCDC_BASECFG3_RDEF_SHIFT)
#  define LCDC_BASECFG3_RDEF(n)     ((uint32_t)(n) << LCDC_BASECFG3_RDEF_SHIFT)

/* Base Configuration register 4 */

#define LCDC_BASECFG4_DMA           (1 << 8)  /* Bit 8:  Use DMA Data Path */
#define LCDC_BASECFG4_REP           (1 << 9)  /* Bit 9:  Use Replication logic to expand RGB */
#define LCDC_BASECFG4_DISCEN        (1 << 11) /* Bit 11: Discard Area Enable */

/* Base Configuration register 5 */

#define LCDC_BASECFG5_DISCXPOS_SHIFT (0)      /* Bits 0-10: Discard Area H coordinate */
#define LCDC_BASECFG5_DISCXPOS_MASK  (0x7ff << LCDC_BASECFG5_DISCXPOS_SHIFT)
#  define LCDC_BASECFG5_DISCXPOS(n)  ((uint32_t)(n) << LCDC_BASECFG5_DISCXPOS_SHIFT)
#define LCDC_BASECFG5_DISCYPOS_SHIFT (16)     /* Bits 16-26: Discard Area V coordinate */
#define LCDC_BASECFG5_DISCYPOS_MASK  (0x7ff << LCDC_BASECFG5_DISCYPOS_SHIFT)
#  define LCDC_BASECFG5_DISCYPOS(n)  ((uint32_t)(n) << LCDC_BASECFG5_DISCYPOS_SHIFT)

/* Base Configuration register 6 */

#define LCDC_BASECFG6_DISCXSIZE_SHIFT (0)     /* Bits 0-10: Discard Area H Size */
#define LCDC_BASECFG6_DISCXSIZE_MASK  (0x7ff << LCDC_BASECFG6_DISCXSIZE_SHIFT)
#  define LCDC_BASECFG6_DISCXSIZE(n)  ((uint32_t)(n) << LCDC_BASECFG6_DISCXSIZE_SHIFT)
#define LCDC_BASECFG6_DISCYSIZE_SHIFT (16)    /* Bits 16-26: Discard Area V Size */
#define LCDC_BASECFG6_DISCYSIZE_MASK  (0x7ff << LCDC_BASECFG6_DISCYSIZE_SHIFT)
#  define LCDC_BASECFG6_DISCYSIZE(n)  ((uint32_t)(n) << LCDC_BASECFG6_DISCYSIZE_SHIFT)

/* Overlay 1 Channel Enable Register */

#define LCDC_OVR1CHER_CH            (1 << 0)  /* Bit 0:  Channel Enable */
#define LCDC_OVR1CHER_UPDATE        (1 << 1)  /* Bit 1:  Update Overlay Attributes Enable */
#define LCDC_OVR1CHER_A2Q           (1 << 2)  /* Bit 2:  Add Head Pointer Enable */

/* Overlay 1 Channel Disable Register */

#define LCDC_OVR1CHDR_CH            (1 << 0)  /* Bit 0:  Channel Disable */
#define LCDC_OVR1CHDR_CHRST         (1 << 8)  /* Bit 8:  Channel Reset */

/* Overlay 1 Channel Status Register */

#define LCDC_OVR1CHSR_CH            (1 << 0)  /* Bit 0:  Channel Status */
#define LCDC_OVR1CHSR_UPDATE        (1 << 1)  /* Bit 1:  Update Overlay Attributes In */
#define LCDC_OVR1CHSR_A2Q           (1 << 2)  /* Bit 2:  Add To Queue Pending */

/* Overlay 1 Interrupt Enable Register, Overlay 1 Interrupt Disable Register,
 * Overlay 1 Interrupt Mask Register, and Overlay 1 Interrupt Status Register
 */

#define LCDC_OVR1INT_DMA            (1 << 2)  /* Bit 2:  End of DMA Transfer */
#define LCDC_OVR1INT_DSCR           (1 << 3)  /* Bit 3:  DMA Descriptor Loaded */
#define LCDC_OVR1INT_ADD            (1 << 4)  /* Bit 4:  Head Descriptor Loaded */
#define LCDC_OVR1INT_DONE           (1 << 5)  /* Bit 5:  End of List Detected */
#define LCDC_OVR1INT_OVR            (1 << 6)  /* Bit 6:  Overflow Detected */

/* Overlay 1 DMA Head Register */

#define LCDC_OVR1HEAD_MASK          (0xfffffffc) /* Bits 2-31: DMA Head Pointer */

/* Overlay 1 DMA Address Register (32-bit address) */

/* Overlay 1 DMA Control Register */

#define LCDC_OVR1CTRL_DFETCH        (1 << 0)  /* Bit 0:  Transfer Descriptor Fetch Enable */
#define LCDC_OVR1CTRL_LFETCH        (1 << 1)  /* Bit 1:  Lookup Table Fetch Enable */
#define LCDC_OVR1CTRL_DMAIEN        (1 << 2)  /* Bit 2:  End of DMA Transfer Interrupt Enable */
#define LCDC_OVR1CTRL_DSCRIEN       (1 << 3)  /* Bit 3:  Descriptor Loaded Interrupt Enable */
#define LCDC_OVR1CTRL_ADDIEN        (1 << 4)  /* Bit 4:  Add Head Descriptor to Queue Interrupt Enable */
#define LCDC_OVR1CTRL_DONEIEN       (1 << 5)  /* Bit 5:  End of List Interrupt Enable */

/* Overlay 1 DMA Next Register (32-bit address) */

/* Overlay 1 Configuration 0 Register */

#define LCDC_OVR1CFG0_SIF           (1 << 0)  /* Bit 0:  Source Interface */
#define LCDC_OVR1CFG0_BLEN_SHIFT    (4)       /* Bits 4-5: AHB Burst Length */
#define LCDC_OVR1CFG0_BLEN_MASK     (3 << LCDC_OVR1CFG0_BLEN_SHIFT)
#  define LCDC_OVR1CFG0_BLEN_SINGLE (0 << LCDC_OVR1CFG0_BLEN_SHIFT)
#  define LCDC_OVR1CFG0_BLEN_INCR4  (1 << LCDC_OVR1CFG0_BLEN_SHIFT)
#  define LCDC_OVR1CFG0_BLEN_INCR8  (2 << LCDC_OVR1CFG0_BLEN_SHIFT)
#  define LCDC_OVR1CFG0_BLEN_INCR16 (3 << LCDC_OVR1CFG0_BLEN_SHIFT)
#define LCDC_OVR1CFG0_DLBO          (1 << 8)  /* Bit 8:  Defined Length Burst Only */
#define LCDC_OVR1CFG0_ROTDIS        (1 << 12) /* Bit 12: Hardware Rotation Optimization Disable */
#define LCDC_OVR1CFG0_LOCKDIS       (1 << 13) /* Bit 13: Hardware Rotation Lock Disable */

/* Overlay 1 Configuration 1 Register */

#define LCDC_OVR1CFG1_CLUTEN           (1 << 0)  /* Bit 0:  Color Lookup Table Enable */
#define LCDC_OVR1CFG1_RGBMODE_SHIFT    (4)       /* Bits 4-7: RGB Input Mode Selection */
#define LCDC_OVR1CFG1_RGBMODE_MASK     (15 << LCDC_OVR1CFG1_RGBMODE_SHIFT)
#  define LCDC_OVR1CFG1_12BPP_RGB444   (0 << LCDC_OVR1CFG1_RGBMODE_SHIFT)  /* 12 bpp RGB 444 */
#  define LCDC_OVR1CFG1_16BPP_ARGB4444 (1 << LCDC_OVR1CFG1_RGBMODE_SHIFT)  /* 16 bpp ARGB 4444 */
#  define LCDC_OVR1CFG1_16BPP_RGBA4444 (2 << LCDC_OVR1CFG1_RGBMODE_SHIFT)  /* 16 bpp RGBA 4444 */
#  define LCDC_OVR1CFG1_16BPP_RGB565   (3 << LCDC_OVR1CFG1_RGBMODE_SHIFT)  /* 16 bpp RGB 565 */
#  define LCDC_OVR1CFG1_16BPP_TRGB1555 (4 << LCDC_OVR1CFG1_RGBMODE_SHIFT)  /* 16 bpp TRGB 1555 */
#  define LCDC_OVR1CFG1_18BPP_RGB666   (5 << LCDC_OVR1CFG1_RGBMODE_SHIFT)  /* 18 bpp RGB 666 */
#  define LCDC_OVR1CFG1_18BPP_RGB666P  (6 << LCDC_OVR1CFG1_RGBMODE_SHIFT)  /* 18 bpp RGB 666 PACKED */
#  define LCDC_OVR1CFG1_19BPP_TRGB1666 (7 << LCDC_OVR1CFG1_RGBMODE_SHIFT)  /* 19 bpp TRGB 1666 */
#  define LCDC_OVR1CFG1_19BPP_TRGBP    (8 << LCDC_OVR1CFG1_RGBMODE_SHIFT)  /* 19 bpp TRGB 1666 PACKED */
#  define LCDC_OVR1CFG1_24BPP_RGB888   (9 << LCDC_OVR1CFG1_RGBMODE_SHIFT)  /* 24 bpp RGB 888 */
#  define LCDC_OVR1CFG1_24BPP_RGB888P  (10 << LCDC_OVR1CFG1_RGBMODE_SHIFT) /* 24 bpp RGB 888 PACKED */
#  define LCDC_OVR1CFG1_25BPP_TRGB1888 (11 << LCDC_OVR1CFG1_RGBMODE_SHIFT) /* 25 bpp TRGB 1888 */
#  define LCDC_OVR1CFG1_32BPP_ARGB8888 (12 << LCDC_OVR1CFG1_RGBMODE_SHIFT) /* 32 bpp ARGB 8888 */
#  define LCDC_OVR1CFG1_32BPP_RGBA8888 (13 << LCDC_OVR1CFG1_RGBMODE_SHIFT) /* 32 bpp RGBA 8888 */

#define LCDC_OVR1CFG1_CLUTMODE_SHIFT   (8)       /* Bits 8-9: CLUT Input Mode Selection */
#define LCDC_OVR1CFG1_CLUTMODE_MASK    (3 << LCDC_OVR1CFG1_CLUTMODE_SHIFT)
#  define LCDC_OVR1CFG1_CLUTMODE_1BPP  (0 << LCDC_OVR1CFG1_CLUTMODE_SHIFT) /* CLUT input 1 bit per pixel */
#  define LCDC_OVR1CFG1_CLUTMODE_2BPP  (1 << LCDC_OVR1CFG1_CLUTMODE_SHIFT) /* CLUT input 2 bits per pixel */
#  define LCDC_OVR1CFG1_CLUTMODE_4BPP  (2 << LCDC_OVR1CFG1_CLUTMODE_SHIFT) /* CLUT input 4 bits per pixel */
#  define LCDC_OVR1CFG1_CLUTMODE_8BPP  (3 << LCDC_OVR1CFG1_CLUTMODE_SHIFT) /* CLUT input 8 bits per pixel */

/* Overlay 1 Configuration 2 Register */

#define LCDC_OVR1CFG2_XPOS_SHIFT    (0)     /* Bits 0-10: Horizontal Window Position */
#define LCDC_OVR1CFG2_XPOS_MASK     (0x7ff << LCDC_OVR1CFG2_XPOS_SHIFT)
#  define LCDC_OVR1CFG2_XPOS(n)     ((uint32_t)(n) << LCDC_OVR1CFG2_XPOS_SHIFT)
#define LCDC_OVR1CFG2_YPOS_SHIFT    (16)    /* Bits 16-26: Vertical Window Position */
#define LCDC_OVR1CFG2_YPOS_MASK     (0x7ff << LCDC_OVR1CFG2_YPOS_SHIFT)
#  define LCDC_OVR1CFG2_YPOS(n)     ((uint32_t)(n) << LCDC_OVR1CFG2_YPOS_SHIFT)

/* Overlay 1 Configuration 3 Register */

#define LCDC_OVR1CFG3_XSIZE_SHIFT   (0)     /* Bits 0-10: Horizontal Window Size */
#define LCDC_OVR1CFG3_XSIZE_MASK    (0x7ff << LCDC_OVR1CFG3_XSIZE_SHIFT)
#  define LCDC_OVR1CFG3_XSIZE(n)    ((uint32_t)(n) << LCDC_OVR1CFG3_XSIZE_SHIFT)
#define LCDC_OVR1CFG3_YSIZE_SHIFT   (16)    /* Bits 16-26: Vertical Window Size */
#define LCDC_OVR1CFG3_YSIZE_MASK    (0x7ff << LCDC_OVR1CFG3_YSIZE_SHIFT)
#  define LCDC_OVR1CFG3_YSIZE(n)    ((uint32_t)(n) << LCDC_OVR1CFG3_YSIZE_SHIFT)

/* Overlay 1 Configuration 4 Register (32-bit horizontal stride value) */

/* Overlay 1 Configuration 5 Register (32-bit pixel stride value) */

/* Overlay 1 Configuration 6 Register */

#define LCDC_OVR1CFG6_BDEF_SHIFT    (0)       /* Bits 0-7: B Default */
#define LCDC_OVR1CFG6_BDEF_MASK     (0xff << LCDC_OVR1CFG6_BDEF_SHIFT)
#  define LCDC_OVR1CFG6_BDEF(n)     ((uint32_t)(n) << LCDC_OVR1CFG6_BDEF_SHIFT)
#define LCDC_OVR1CFG6_GDEF_SHIFT    (8)       /* Bits 8-15: G Default */
#define LCDC_OVR1CFG6_GDEF_MASK     (0xff << LCDC_OVR1CFG6_GDEF_SHIFT)
#  define LCDC_OVR1CFG6_GDEF(n)     ((uint32_t)(n) << LCDC_OVR1CFG6_GDEF_SHIFT)
#define LCDC_OVR1CFG6_RDEF_SHIFT    (16)      /* Bits 16-23: R Default */
#define LCDC_OVR1CFG6_RDEF_MASK     (0xff << LCDC_OVR1CFG6_RDEF_SHIFT)
#  define LCDC_OVR1CFG6_RDEF(n)     ((uint32_t)(n) << LCDC_OVR1CFG6_RDEF_SHIFT)

/* Overlay 1 Configuration 7 Register */

#define LCDC_OVR1CFG7_BKEY_SHIFT    (0)       /* Bits 0-7: B Color Component Chroma Key */
#define LCDC_OVR1CFG7_BKEY_MASK     (0xff << LCDC_OVR1CFG7_BKEY_SHIFT)
#  define LCDC_OVR1CFG7_BKEY(n)     ((uint32_t)(n) << LCDC_OVR1CFG7_BKEY_SHIFT)
#define LCDC_OVR1CFG7_GKEY_SHIFT    (8)       /* Bits 8-15: G Color Component Chroma Key */
#define LCDC_OVR1CFG7_GKEY_MASK     (0xff << LCDC_OVR1CFG7_GKEY_SHIFT)
#  define LCDC_OVR1CFG7_GKEY(n)     ((uint32_t)(n) << LCDC_OVR1CFG7_GKEY_SHIFT)
#define LCDC_OVR1CFG7_RKEY_SHIFT    (16)      /* Bits 16-23: R Color Component Chroma Key */
#define LCDC_OVR1CFG7_RKEY_MASK     (0xff << LCDC_OVR1CFG7_RKEY_SHIFT)
#  define LCDC_OVR1CFG7_RKEY(n)     ((uint32_t)(n) << LCDC_OVR1CFG7_RKEY_SHIFT)

/* Overlay 1 Configuration 8 Register */

#define LCDC_OVR1CFG8_BMASK_SHIFT   (0)       /* Bits 0-7: B Color Component Chroma Key Mask */
#define LCDC_OVR1CFG8_BMASK_MASK    (0xff << LCDC_OVR1CFG8_BMASK_SHIFT)
#  define LCDC_OVR1CFG8_BMASK(n)    ((uint32_t)(n) << LCDC_OVR1CFG8_BMASK_SHIFT)
#define LCDC_OVR1CFG8_GMASK_SHIFT   (8)       /* Bits 8-15: G Color Component Chroma Key Mask */
#define LCDC_OVR1CFG8_GMASK_MASK    (0xff << LCDC_OVR1CFG8_GMASK_SHIFT)
#  define LCDC_OVR1CFG8_GMASK(n)    ((uint32_t)(n) << LCDC_OVR1CFG8_GMASK_SHIFT)
#define LCDC_OVR1CFG8_RMASK_SHIFT   (16)      /* Bits 16-23: R Color Component Chroma Key Mask */
#define LCDC_OVR1CFG8_RMASK_MASK    (0xff << LCDC_OVR1CFG8_RMASK_SHIFT)
#  define LCDC_OVR1CFG8_RMASK(n)    ((uint32_t)(n) << LCDC_OVR1CFG8_RMASK_SHIFT)

/* Overlay 1 Configuration 9 Register */

#define LCDC_OVR1CFG9_CRKEY         (1 << 0)  /* Bit 0:  Blender Chroma Key Enable */
#define LCDC_OVR1CFG9_INV           (1 << 1)  /* Bit 1:  Blender Inverted Blender Output Enable */
#define LCDC_OVR1CFG9_ITER2BL       (1 << 2)  /* Bit 2:  Blender Iterated Color Enable */
#define LCDC_OVR1CFG9_ITER          (1 << 3)  /* Bit 3:  Blender Use Iterated Color */
#define LCDC_OVR1CFG9_REVALPHA      (1 << 4)  /* Bit 4:  Blender Reverse Alpha */
#define LCDC_OVR1CFG9_GAEN          (1 << 5)  /* Bit 5:  Blender Global Alpha Enable */
#define LCDC_OVR1CFG9_LAEN          (1 << 6)  /* Bit 6:  Blender Local Alpha Enable */
#define LCDC_OVR1CFG9_OVR           (1 << 7)  /* Bit 7:  Blender Overlay Layer Enable */
#define LCDC_OVR1CFG9_DMA           (1 << 8)  /* Bit 8:  Blender DMA Layer Enable */
#define LCDC_OVR1CFG9_REP           (1 << 9)  /* Bit 9:  Use Replication logic to expand RGB color */
#define LCDC_OVR1CFG9_DSTKEY        (1 << 10) /* Bit 10: Destination Chroma Keying */
#define LCDC_OVR1CFG9_GA_SHIFT      (16)      /* Bits 16-23: Blender Global Alpha */
#define LCDC_OVR1CFG9_GA_MASK       (0xff << LCDC_OVR1CFG9_GA_SHIFT)
#  define LCDC_OVR1CFG9_GA(n)       ((uint32_t)(n) << LCDC_OVR1CFG9_GA_SHIFT)

/* Overlay 2 Channel Enable Register */

#define LCDC_OVR2CHER_CH            (1 << 0)  /* Bit 0:  Channel Enable */
#define LCDC_OVR2CHER_UPDATE        (1 << 1)  /* Bit 1:  Update Overlay Attributes Enable */
#define LCDC_OVR2CHER_A2Q           (1 << 2)  /* Bit 2:  Add Head Pointer Enable */

/* Overlay 2 Channel Disable Register */

#define LCDC_OVR2CHDR_CH            (1 << 0)  /* Bit 0:  Channel Disable */
#define LCDC_OVR2CHDR_CHRST         (1 << 8)  /* Bit 8:  Channel Reset */

/* Overlay 2 Channel Status Register */

#define LCDC_OVR2CHSR_CH            (1 << 0)  /* Bit 0:  Channel Status */
#define LCDC_OVR2CHSR_UPDATE        (1 << 1)  /* Bit 1:  Update Overlay Attributes In */
#define LCDC_OVR2CHSR_A2Q           (1 << 2)  /* Bit 2:  Add To Queue Pending */

/* Overlay 2 Interrupt Enable Register, Overlay 2 Interrupt Disable Register,
 * Overlay 2 Interrupt Mask Register, and Overlay 2 Interrupt Status Register
 */

#define LCDC_OVR2INT_DMA            (1 << 2)  /* Bit 2:  End of DMA Transfer */
#define LCDC_OVR2INT_DSCR           (1 << 3)  /* Bit 3:  DMA Descriptor Loaded */
#define LCDC_OVR2INT_ADD            (1 << 4)  /* Bit 4:  Head Descriptor Loaded */
#define LCDC_OVR2INT_DONE           (1 << 5)  /* Bit 5:  End of List Detected */
#define LCDC_OVR2INT_OVR            (1 << 6)  /* Bit 6:  Overflow Detected */

/* Overlay 2 DMA Head Register */

#define LCDC_OVR2HEAD_MASK          (0xfffffffc) /* Bits 2-31: DMA Head Pointer */

/* Overlay 2 DMA Address Register (32-bit address) */

/* Overlay 2 DMA Control Register */

#define LCDC_OVR2CTRL_DFETCH        (1 << 0)  /* Bit 0:  Transfer Descriptor Fetch Enable */
#define LCDC_OVR2CTRL_LFETCH        (1 << 1)  /* Bit 1:  Lookup Table Fetch Enable */
#define LCDC_OVR2CTRL_DMAIEN        (1 << 2)  /* Bit 2:  End of DMA Transfer Interrupt Enable */
#define LCDC_OVR2CTRL_DSCRIEN       (1 << 3)  /* Bit 3:  Descriptor Loaded Interrupt Enable */
#define LCDC_OVR2CTRL_ADDIEN        (1 << 4)  /* Bit 4:  Add Head Descriptor to Queue Interrupt Enable */
#define LCDC_OVR2CTRL_DONEIEN       (1 << 5)  /* Bit 5:  End of List Interrupt Enable */

/* Overlay 2 DMA Next Register (32-bit address) */

/* Overlay 2 Configuration 0 Register */

#define LCDC_OVR2CFG0_BLEN_SHIFT    (4)       /* Bits 4-5: AHB Burst Length */
#define LCDC_OVR2CFG0_BLEN_MASK     (3 << LCDC_OVR2CFG0_BLEN_SHIFT)
#  define LCDC_OVR2CFG0_BLEN_SINGLE (0 << LCDC_OVR2CFG0_BLEN_SHIFT)
#  define LCDC_OVR2CFG0_BLEN_INCR4  (1 << LCDC_OVR2CFG0_BLEN_SHIFT)
#  define LCDC_OVR2CFG0_BLEN_INCR8  (2 << LCDC_OVR2CFG0_BLEN_SHIFT)
#  define LCDC_OVR2CFG0_BLEN_INCR16 (3 << LCDC_OVR2CFG0_BLEN_SHIFT)
#define LCDC_OVR2CFG0_DLBO          (1 << 8)  /* Bit 8:  Defined Length Burst Only */
#define LCDC_OVR2CFG0_ROTDIS        (1 << 12) /* Bit 12: Hardware Rotation Optimization Disable */
#define LCDC_OVR2CFG0_LOCKDIS       (1 << 13) /* Bit 13: Hardware Rotation Lock Disable */

/* Overlay 2 Configuration 1 Register */

#define LCDC_OVR2CFG1_CLUTEN           (1 << 0)  /* Bit 0:  Color Lookup Table Enable */
#define LCDC_OVR2CFG1_RGBMODE_SHIFT    (4)       /* Bits 4-7: RGB Input Mode Selection */
#define LCDC_OVR2CFG1_RGBMODE_MASK     (15 << LCDC_OVR2CFG1_RGBMODE_SHIFT)
#  define LCDC_OVR2CFG1_12BPP_RGB444   (0 << LCDC_OVR2CFG1_RGBMODE_SHIFT)  /* 12 bpp RGB 444 */
#  define LCDC_OVR2CFG1_16BPP_ARGB4444 (1 << LCDC_OVR2CFG1_RGBMODE_SHIFT)  /* 16 bpp ARGB 4444 */
#  define LCDC_OVR2CFG1_16BPP_RGBA4444 (2 << LCDC_OVR2CFG1_RGBMODE_SHIFT)  /* 16 bpp RGBA 4444 */
#  define LCDC_OVR2CFG1_16BPP_RGB565   (3 << LCDC_OVR2CFG1_RGBMODE_SHIFT)  /* 16 bpp RGB 565 */
#  define LCDC_OVR2CFG1_16BPP_TRGB1555 (4 << LCDC_OVR2CFG1_RGBMODE_SHIFT)  /* 16 bpp TRGB 1555 */
#  define LCDC_OVR2CFG1_18BPP_RGB666   (5 << LCDC_OVR2CFG1_RGBMODE_SHIFT)  /* 18 bpp RGB 666 */
#  define LCDC_OVR2CFG1_18BPP_RGB666P  (6 << LCDC_OVR2CFG1_RGBMODE_SHIFT)  /* 18 bpp RGB 666 PACKED */
#  define LCDC_OVR2CFG1_19BPP_TRGB1666 (7 << LCDC_OVR2CFG1_RGBMODE_SHIFT)  /* 19 bpp TRGB 1666 */
#  define LCDC_OVR2CFG1_19BPP_TRGBP    (8 << LCDC_OVR2CFG1_RGBMODE_SHIFT)  /* 19 bpp TRGB 1666 PACKED */
#  define LCDC_OVR2CFG1_24BPP_RGB888   (9 << LCDC_OVR2CFG1_RGBMODE_SHIFT)  /* 24 bpp RGB 888 */
#  define LCDC_OVR2CFG1_24BPP_RGB888P  (10 << LCDC_OVR2CFG1_RGBMODE_SHIFT) /* 24 bpp RGB 888 PACKED */
#  define LCDC_OVR2CFG1_25BPP_TRGB1888 (11 << LCDC_OVR2CFG1_RGBMODE_SHIFT) /* 25 bpp TRGB 1888 */
#  define LCDC_OVR2CFG1_32BPP_ARGB8888 (12 << LCDC_OVR2CFG1_RGBMODE_SHIFT) /* 32 bpp ARGB 8888 */
#  define LCDC_OVR2CFG1_32BPP_RGBA8888 (13 << LCDC_OVR2CFG1_RGBMODE_SHIFT) /* 32 bpp RGBA 8888 */

#define LCDC_OVR2CFG1_CLUTMODE_SHIFT   (8)       /* Bits 8-9: CLUT Input Mode Selection */
#define LCDC_OVR2CFG1_CLUTMODE_MASK    (3 << LCDC_OVR2CFG1_CLUTMODE_SHIFT)
#  define LCDC_OVR2CFG1_CLUTMODE_1BPP  (0 << LCDC_OVR2CFG1_CLUTMODE_SHIFT) /* CLUT input 1 bit per pixel */
#  define LCDC_OVR2CFG1_CLUTMODE_2BPP  (1 << LCDC_OVR2CFG1_CLUTMODE_SHIFT) /* CLUT input 2 bits per pixel */
#  define LCDC_OVR2CFG1_CLUTMODE_4BPP  (2 << LCDC_OVR2CFG1_CLUTMODE_SHIFT) /* CLUT input 4 bits per pixel */
#  define LCDC_OVR2CFG1_CLUTMODE_8BPP  (3 << LCDC_OVR2CFG1_CLUTMODE_SHIFT) /* CLUT input 8 bits per pixel */

/* Overlay 2 Configuration 2 Register */

#define LCDC_OVR2CFG2_XPOS_SHIFT    (0)     /* Bits 0-10: Horizontal Window Position */
#define LCDC_OVR2CFG2_XPOS_MASK     (0x7ff << LCDC_OVR2CFG2_XPOS_SHIFT)
#  define LCDC_OVR2CFG2_XPOS(n)     ((uint32_t)(n) << LCDC_OVR2CFG2_XPOS_SHIFT)
#define LCDC_OVR2CFG2_YPOS_SHIFT    (16)    /* Bits 16-26: Vertical Window Position */
#define LCDC_OVR2CFG2_YPOS_MASK     (0x7ff << LCDC_OVR2CFG2_YPOS_SHIFT)
#  define LCDC_OVR2CFG2_YPOS(n)     ((uint32_t)(n) << LCDC_OVR2CFG2_YPOS_SHIFT)

/* Overlay 2 Configuration 3 Register */

#define LCDC_OVR2CFG3_XSIZE_SHIFT   (0)     /* Bits 0-10: Horizontal Window Size */
#define LCDC_OVR2CFG3_XSIZE_MASK    (0x7ff << LCDC_OVR2CFG3_XSIZE_SHIFT)
#  define LCDC_OVR2CFG3_XSIZE(n)    ((uint32_t)(n) << LCDC_OVR2CFG3_XSIZE_SHIFT)
#define LCDC_OVR2CFG3_YSIZE_SHIFT   (16)    /* Bits 16-26: Vertical Window Size */
#define LCDC_OVR2CFG3_YSIZE_MASK    (0x7ff << LCDC_OVR2CFG3_YSIZE_SHIFT)
#  define LCDC_OVR2CFG3_YSIZE(n)    ((uint32_t)(n) << LCDC_OVR2CFG3_YSIZE_SHIFT)

/* Overlay 2 Configuration 4 Register (32-bit horizontal stride value) */

/* Overlay 2 Configuration 5 Register (32-bit pixel stride value) */

/* Overlay 2 Configuration 6 Register */

#define LCDC_OVR2CFG6_BDEF_SHIFT    (0)       /* Bits 0-7: B Default */
#define LCDC_OVR2CFG6_BDEF_MASK     (0xff << LCDC_OVR2CFG6_BDEF_SHIFT)
#  define LCDC_OVR2CFG6_BDEF(n)     ((uint32_t)(n) << LCDC_OVR2CFG6_BDEF_SHIFT)
#define LCDC_OVR2CFG6_GDEF_SHIFT    (8)       /* Bits 8-15: G Default */
#define LCDC_OVR2CFG6_GDEF_MASK     (0xff << LCDC_OVR2CFG6_GDEF_SHIFT)
#  define LCDC_OVR2CFG6_GDEF(n)     ((uint32_t)(n) << LCDC_OVR2CFG6_GDEF_SHIFT)
#define LCDC_OVR2CFG6_RDEF_SHIFT    (16)      /* Bits 16-23: R Default */
#define LCDC_OVR2CFG6_RDEF_MASK     (0xff << LCDC_OVR2CFG6_RDEF_SHIFT)
#  define LCDC_OVR2CFG6_RDEF(n)     ((uint32_t)(n) << LCDC_OVR2CFG6_RDEF_SHIFT)

/* Overlay 2 Configuration 7 Register */

#define LCDC_OVR2CFG7_BKEY_SHIFT    (0)       /* Bits 0-7: B Color Component Chroma Key */
#define LCDC_OVR2CFG7_BKEY_MASK     (0xff << LCDC_OVR2CFG7_BKEY_SHIFT)
#  define LCDC_OVR2CFG7_BKEY(n)     ((uint32_t)(n) << LCDC_OVR2CFG7_BKEY_SHIFT)
#define LCDC_OVR2CFG7_GKEY_SHIFT    (8)       /* Bits 8-15: G Color Component Chroma Key */
#define LCDC_OVR2CFG7_GKEY_MASK     (0xff << LCDC_OVR2CFG7_GKEY_SHIFT)
#  define LCDC_OVR2CFG7_GKEY(n)     ((uint32_t)(n) << LCDC_OVR2CFG7_GKEY_SHIFT)
#define LCDC_OVR2CFG7_RKEY_SHIFT    (16)      /* Bits 16-23: R Color Component Chroma Key */
#define LCDC_OVR2CFG7_RKEY_MASK     (0xff << LCDC_OVR2CFG7_RKEY_SHIFT)
#  define LCDC_OVR2CFG7_RKEY(n)     ((uint32_t)(n) << LCDC_OVR2CFG7_RKEY_SHIFT)

/* Overlay 2 Configuration 8 Register */

#define LCDC_OVR2CFG8_BMASK_SHIFT   (0)       /* Bits 0-7: B Color Component Chroma Key Mask */
#define LCDC_OVR2CFG8_BMASK_MASK    (0xff << LCDC_OVR2CFG8_BMASK_SHIFT)
#  define LCDC_OVR2CFG8_BMASK(n)    ((uint32_t)(n) << LCDC_OVR2CFG8_BMASK_SHIFT)
#define LCDC_OVR2CFG8_GMASK_SHIFT   (8)       /* Bits 8-15: G Color Component Chroma Key Mask */
#define LCDC_OVR2CFG8_GMASK_MASK    (0xff << LCDC_OVR2CFG8_GMASK_SHIFT)
#  define LCDC_OVR2CFG8_GMASK(n)    ((uint32_t)(n) << LCDC_OVR2CFG8_GMASK_SHIFT)
#define LCDC_OVR2CFG8_RMASK_SHIFT   (16)      /* Bits 16-23: R Color Component Chroma Key Mask */
#define LCDC_OVR2CFG8_RMASK_MASK    (0xff << LCDC_OVR2CFG8_RMASK_SHIFT)
#  define LCDC_OVR2CFG8_RMASK(n)    ((uint32_t)(n) << LCDC_OVR2CFG8_RMASK_SHIFT)

/* Overlay 2 Configuration 9 Register */

#define LCDC_OVR2CFG9_CRKEY         (1 << 0)  /* Bit 0:  Blender Chroma Key Enable */
#define LCDC_OVR2CFG9_INV           (1 << 1)  /* Bit 1:  Blender Inverted Blender Output Enable */
#define LCDC_OVR2CFG9_ITER2BL       (1 << 2)  /* Bit 2:  Blender Iterated Color Enable */
#define LCDC_OVR2CFG9_ITER          (1 << 3)  /* Bit 3:  Blender Use Iterated Color */
#define LCDC_OVR2CFG9_REVALPHA      (1 << 4)  /* Bit 4:  Blender Reverse Alpha */
#define LCDC_OVR2CFG9_GAEN          (1 << 5)  /* Bit 5:  Blender Global Alpha Enable */
#define LCDC_OVR2CFG9_LAEN          (1 << 6)  /* Bit 6:  Blender Local Alpha Enable */
#define LCDC_OVR2CFG9_OVR           (1 << 7)  /* Bit 7:  Blender Overlay Layer Enable */
#define LCDC_OVR2CFG9_DMA           (1 << 8)  /* Bit 8:  Blender DMA Layer Enable */
#define LCDC_OVR2CFG9_REP           (1 << 9)  /* Bit 9:  Use Replication logic to expand RGB color */
#define LCDC_OVR2CFG9_DSTKEY        (1 << 10) /* Bit 10: Destination Chroma Keying */
#define LCDC_OVR2CFG9_GA_SHIFT      (16)      /* Bits 16-23: Blender Global Alpha */
#define LCDC_OVR2CFG9_GA_MASK       (0xff << LCDC_OVR2CFG9_GA_SHIFT)
#  define LCDC_OVR2CFG9_GA(n)       ((uint32_t)(n) << LCDC_OVR2CFG9_GA_SHIFT)

/* High-End Overlay Channel Enable Register */

#define LCDC_HEOCHER_CH             (1 << 0)  /* Bit 0:  Channel Enable */
#define LCDC_HEOCHER_UPDATE         (1 << 1)  /* Bit 1:  Update Overlay Attributes Enable */
#define LCDC_HEOCHER_A2Q            (1 << 2)  /* Bit 2:  Add Head Pointer Enable */

/* High-End Overlay Channel Disable Register */

#define LCDC_HEOCHDR_CH             (1 << 0)  /* Bit 0:  Channel Disable */
#define LCDC_HEOCHDR_CHRST          (1 << 8)  /* Bit 8:  Channel Reset */

/* High-End Overlay Channel Status Register */

#define LCDC_HEOCHSR_CH             (1 << 0)  /* Bit 0:  Channel Status */
#define LCDC_HEOCHSR_UPDATE         (1 << 1)  /* Bit 1:  Update Overlay Attributes In */
#define LCDC_HEOCHSR_A2Q            (1 << 2)  /* Bit 2:  Add To Queue Pending */

/* High-End Overlay Interrupt Enable Register,
 * High-End Overlay Interrupt Disable Register,
 * High-End Overlay Interrupt Mask Register,
 * and High-End Overlay Interrupt Status Register
 */

#define LCDC_HEOINT_DMA             (1 << 2)  /* Bit 2:  End of DMA Transfer */
#define LCDC_HEOINT_DSCR            (1 << 3)  /* Bit 3:  DMA Descriptor Loaded */
#define LCDC_HEOINT_ADD             (1 << 4)  /* Bit 4:  Head Descriptor Loaded */
#define LCDC_HEOINT_DONE            (1 << 5)  /* Bit 5:  End of List Detected */
#define LCDC_HEOINT_OVR             (1 << 6)  /* Bit 6:  Overflow Detected */
#define LCDC_HEOINT_UDMA            (1 << 10) /* Bit 10: End of DMA Transfer (U or UV Chrominance) */
#define LCDC_HEOINT_UDSCR           (1 << 11) /* Bit 11: DMA Descriptor Loaded (U or UV Chrominance) */
#define LCDC_HEOINT_UADD            (1 << 12) /* Bit 12: Head Descriptor Loaded (U or UV Chrominance) */
#define LCDC_HEOINT_UDONE           (1 << 13) /* Bit 13: End of List Detected (U or UV Chrominance) */
#define LCDC_HEOINT_UOVR            (1 << 14) /* Bit 14: Overflow Detected (U or UV Chrominance) */
#define LCDC_HEOINT_VDMA            (1 << 18) /* Bit 18: End of DMA Transfer (V Chrominance) */
#define LCDC_HEOINT_VDSCR           (1 << 19) /* Bit 19: DMA Descriptor Loaded (V Chrominance) */
#define LCDC_HEOINT_VADD            (1 << 20) /* Bit 20: Head Descriptor Loaded (V Chrominance) */
#define LCDC_HEOINT_VDONE           (1 << 21) /* Bit 21: End of List Detected (V Chrominance) */
#define LCDC_HEOINT_VOVR            (1 << 22) /* Bit 22: Overflow Detected (V Chrominance) */

/* High-End Overlay DMA Head Register */

#define LCDC_HEOHEAD_MASK           (0xfffffffc) /* Bits 2-31: DMA Head Pointer */

/* High-End Overlay DMA Address Register (32-bit address) */

/* High-End Overlay DMA Control Register */

#define LCDC_HEOCTRL_DFETCH         (1 << 0)  /* Bit 0:  Transfer Descriptor Fetch Enable */
#define LCDC_HEOCTRL_LFETCH         (1 << 1)  /* Bit 1:  Lookup Table Fetch Enable */
#define LCDC_HEOCTRL_DMAIEN         (1 << 2)  /* Bit 2:  End of DMA Transfer Interrupt Enable */
#define LCDC_HEOCTRL_DSCRIEN        (1 << 3)  /* Bit 3:  Descriptor Loaded Interrupt Enable */
#define LCDC_HEOCTRL_ADDIEN         (1 << 4)  /* Bit 4:  Add Head Descriptor to Queue Interrupt Enable */
#define LCDC_HEOCTRL_DONEIEN        (1 << 5)  /* Bit 5:  End of List Interrupt Enable */

/* High-End Overlay DMA Next Register (32-bit address) */

/* High-End Overlay U-UV DMA Head Register (32-bit address) */

/* High-End Overlay U-UV DMA Address Register (32-bit address) */

/* High-End Overlay U-UV DMA Control Register */

#define LCDC_HEOUCTRL_DFETCH        (1 << 0)  /* Bit 0:  Transfer Descriptor Fetch Enable */
#define LCDC_HEOUCTRL_DMAIEN        (1 << 2)  /* Bit 2:  End of DMA Transfer Interrupt Enable */
#define LCDC_HEOUCTRL_DSCRIEN       (1 << 3)  /* Bit 3:  Descriptor Loaded Interrupt Enable */
#define LCDC_HEOUCTRL_ADDIEN        (1 << 4)  /* Bit 4:  Add Head Descriptor to Queue Interrupt Enable */
#define LCDC_HEOUCTRL_DONEIEN       (1 << 5)  /* Bit 5:  End of List Interrupt Enable */

/* High-End Overlay U-UV DMA Next Register (32-bit address) */

/* High-End Overlay V DMA Head Register (32-bit address) */

/* High-End Overlay V DMA Address Register )32-bit address) */

/* High-End Overlay V DMA Control Register */

#define LCDC_HEOVCTRL_DFETCH        (1 << 0)  /* Bit 0:  Transfer Descriptor Fetch Enable */
#define LCDC_HEOVCTRL_DMAIEN        (1 << 2)  /* Bit 2:  End of DMA Transfer Interrupt Enable */
#define LCDC_HEOVCTRL_DSCRIEN       (1 << 3)  /* Bit 3:  Descriptor Loaded Interrupt Enable */
#define LCDC_HEOVCTRL_ADDIEN        (1 << 4)  /* Bit 4:  Add Head Descriptor to Queue Interrupt Enable */
#define LCDC_HEOVCTRL_DONEIEN       (1 << 5)  /* Bit 5:  End of List Interrupt Enable */

/* High-End Overlay VDMA Next Register (32-bit address) */

/* High-End Overlay Configuration Register 0 */

#define LCDC_HEOCFG0_SIF             (1 << 0)  /* Bit 0:  Source Interface */
#define LCDC_HEOCFG0_BLEN_SHIFT      (4)       /* Bits 4-5: AHB Burst Length */
#define LCDC_HEOCFG0_BLEN_MASK       (3 << LCDC_HEOCFG0_BLEN_SHIFT)
#  define LCDC_HEOCFG0_BLEN_SINGLE   (0 << LCDC_HEOCFG0_BLEN_SHIFT)
#  define LCDC_HEOCFG0_BLEN_INCR4    (1 << LCDC_HEOCFG0_BLEN_SHIFT)
#  define LCDC_HEOCFG0_BLEN_INCR8    (2 << LCDC_HEOCFG0_BLEN_SHIFT)
#  define LCDC_HEOCFG0_BLEN_INCR16   (3 << LCDC_HEOCFG0_BLEN_SHIFT)
#define LCDC_HEOCFG0_BLENUV_SHIFT    (6)       /* Bits 6-7: AHB Burst Length for U-V channel */
#define LCDC_HEOCFG0_BLENUV_MASK     (3 << LCDC_HEOCFG0_BLENUV_SHIFT)
#  define LCDC_HEOCFG0_BLENUV_SINGLE (0 << LCDC_HEOCFG0_BLENUV_SHIFT)
#  define LCDC_HEOCFG0_BLENUV_INCR4  (1 << LCDC_HEOCFG0_BLENUV_SHIFT)
#  define LCDC_HEOCFG0_BLENUV_INCR8  (2 << LCDC_HEOCFG0_BLENUV_SHIFT)
#  define LCDC_HEOCFG0_BLENUV_INCR16 (3 << LCDC_HEOCFG0_BLENUV_SHIFT)
#define LCDC_HEOCFG0_DLBO            (1 << 8)  /* Bit 8:  Defined Length Burst Only */
#define LCDC_HEOCFG0_ROTDIS          (1 << 12) /* Bit 12: Hardware Rotation Optimization Disable */
#define LCDC_HEOCFG0_LOCKDIS         (1 << 13) /* Bit 13: Hardware Rotation Lock Disable */

/* High-End Overlay Configuration Register 1 */

#define LCDC_HEOCFG1_CLUTEN           (1 << 0)  /* Bit 0:  Color Lookup Table Enable */
#define LCDC_HEOCFG1_YUVEN            (1 << 1)  /* Bit 1:  YUV Color Space Enable */
#define LCDC_HEOCFG1_RGBMODE_SHIFT    (4)       /* Bits 4-7: RGB Input Mode Selection */
#define LCDC_HEOCFG1_RGBMODE_MASK     (15 << LCDC_HEOCFG1_RGBMODE_SHIFT)
#  define LCDC_HEOCFG1_12BPP_RGB444   (0 << LCDC_HEOCFG1_RGBMODE_SHIFT)  /* 12 bpp RGB 444 */
#  define LCDC_HEOCFG1_16BPP_ARGB4444 (1 << LCDC_HEOCFG1_RGBMODE_SHIFT)  /* 16 bpp ARGB 4444 */
#  define LCDC_HEOCFG1_16BPP_RGBA4444 (2 << LCDC_HEOCFG1_RGBMODE_SHIFT)  /* 16 bpp RGBA 4444 */
#  define LCDC_HEOCFG1_16BPP_RGB565   (3 << LCDC_HEOCFG1_RGBMODE_SHIFT)  /* 16 bpp RGB 565 */
#  define LCDC_HEOCFG1_16BPP_TRGB1555 (4 << LCDC_HEOCFG1_RGBMODE_SHIFT)  /* 16 bpp TRGB 1555 */
#  define LCDC_HEOCFG1_18BPP_RGB666   (5 << LCDC_HEOCFG1_RGBMODE_SHIFT)  /* 18 bpp RGB 666 */
#  define LCDC_HEOCFG1_18BPP_RGB666P  (6 << LCDC_HEOCFG1_RGBMODE_SHIFT)  /* 18 bpp RGB 666 PACKED */
#  define LCDC_HEOCFG1_19BPP_TRGB1666 (7 << LCDC_HEOCFG1_RGBMODE_SHIFT)  /* 19 bpp TRGB 1666 */
#  define LCDC_HEOCFG1_19BPP_TRGBP    (8 << LCDC_HEOCFG1_RGBMODE_SHIFT)  /* 19 bpp TRGB 1666 PACKED */
#  define LCDC_HEOCFG1_24BPP_RGB888   (9 << LCDC_HEOCFG1_RGBMODE_SHIFT)  /* 24 bpp RGB 888 */
#  define LCDC_HEOCFG1_24BPP_RGB888P  (10 << LCDC_HEOCFG1_RGBMODE_SHIFT) /* 24 bpp RGB 888 PACKED */
#  define LCDC_HEOCFG1_25BPP_TRGB1888 (11 << LCDC_HEOCFG1_RGBMODE_SHIFT) /* 25 bpp TRGB 1888 */
#  define LCDC_HEOCFG1_32BPP_ARGB8888 (12 << LCDC_HEOCFG1_RGBMODE_SHIFT) /* 32 bpp ARGB 8888 */
#  define LCDC_HEOCFG1_32BPP_RGBA8888 (13 << LCDC_HEOCFG1_RGBMODE_SHIFT) /* 32 bpp RGBA 8888 */

#define LCDC_HEOCFG1_CLUTMODE_SHIFT   (8)       /* Bits 8-9: CLUT Input Mode Selection */
#define LCDC_HEOCFG1_CLUTMODE_MASK    (3 << LCDC_HEOCFG1_CLUTMODE_SHIFT)
#  define LCDC_HEOCFG1_CLUTMODE_1BPP  (0 << LCDC_HEOCFG1_CLUTMODE_SHIFT) /* CLUT input 1 bit per pixel */
#  define LCDC_HEOCFG1_CLUTMODE_2BPP  (1 << LCDC_HEOCFG1_CLUTMODE_SHIFT) /* CLUT input 2 bits per pixel */
#  define LCDC_HEOCFG1_CLUTMODE_4BPP  (2 << LCDC_HEOCFG1_CLUTMODE_SHIFT) /* CLUT input 4 bits per pixel */
#  define LCDC_HEOCFG1_CLUTMODE_8BPP  (3 << LCDC_HEOCFG1_CLUTMODE_SHIFT) /* CLUT input 8 bits per pixel */

#define LCDC_HEOCFG1_YUVMODE_SHIFT    (12)      /* Bits 12-15: YUV Mode Input Selection */
#define LCDC_HEOCFG1_YUVMODE_MASK     (15 << LCDC_HEOCFG1_YUVMODE_SHIFT)
#  define LCDC_HEOCFG1_32BPP_AYCBCR           (0 << LCDC_HEOCFG1_YUVMODE_SHIFT) /* 32 bpp AYCbCr 444 */
#  define LCDC_HEOCFG1_16BPP_YCBCR_MODE0      (1 << LCDC_HEOCFG1_YUVMODE_SHIFT) /* 16 bpp Cr(n)Y(n+1)Cb(n)Y(n) 422 */
#  define LCDC_HEOCFG1_16BPP_YCBCR_MODE1      (2 << LCDC_HEOCFG1_YUVMODE_SHIFT) /* 16 bpp Y(n+1)Cr(n)Y(n)Cb(n) 422 */
#  define LCDC_HEOCFG1_16BPP_YCBCR_MODE2      (3 << LCDC_HEOCFG1_YUVMODE_SHIFT) /* 16 bpp Cb(n)Y(+1)Cr(n)Y(n) 422 */
#  define LCDC_HEOCFG1_16BPP_YCBCR_MODE3      (4 << LCDC_HEOCFG1_YUVMODE_SHIFT) /* 16 bpp Y(n+1)Cb(n)Y(n)Cr(n) 422 */
#  define LCDC_HEOCFG1_16BPP_YCBCR_SEMIPLANAR (5 << LCDC_HEOCFG1_YUVMODE_SHIFT) /* 16 bpp Semiplanar 422 YCbCr */
#  define LCDC_HEOCFG1_16BPP_YCBCR_PLANAR     (6 << LCDC_HEOCFG1_YUVMODE_SHIFT) /* 16 bpp Planar 422 YCbCr */
#  define LCDC_HEOCFG1_12BPP_YCBCR_SEMIPLANAR (7 << LCDC_HEOCFG1_YUVMODE_SHIFT) /* 12 bpp Semiplanar 420 YCbCr */
#  define LCDC_HEOCFG1_12BPP_YCBCR_PLANAR     (8 << LCDC_HEOCFG1_YUVMODE_SHIFT) /* 12 bpp Planar 420 YCbCr */

#define LCDC_HEOCFG1_YUV422ROT        (1 << 16) /* Bit 16: YUV 4:2:2 Rotation */
#define LCDC_HEOCFG1_YUV422SWP        (1 << 17) /* Bit 17: YUV 4:2:2 SWAP */
#define LCDC_HEOCFG1_DSCALEOPT        (1 << 20) /* Bit 20: Down Scaling Bandwidth Optimization */

/* High-End Overlay Configuration Register 2 */

#define LCDC_HEOCFG2_XPOS_SHIFT     (0)     /* Bits 0-10: Horizontal Window Position */
#define LCDC_HEOCFG2_XPOS_MASK      (0x7ff << LCDC_HEOCFG2_XPOS_SHIFT)
#  define LCDC_HEOCFG2_XPOS(n)      ((uint32_t)(n) << LCDC_HEOCFG2_XPOS_SHIFT)
#define LCDC_HEOCFG2_YPOS_SHIFT     (16)    /* Bits 16-26: Vertical Window Position */
#define LCDC_HEOCFG2_YPOS_MASK      (0x7ff << LCDC_HEOCFG2_YPOS_SHIFT)
#  define LCDC_HEOCFG2_YPOS(n)      ((uint32_t)(n) << LCDC_HEOCFG2_YPOS_SHIFT)

/* High-End Overlay Configuration Register 3 */

#define LCDC_HEOCFG3_XSIZE_SHIFT    (0)     /* Bits 0-10: Horizontal Window Size */
#define LCDC_HEOCFG3_XSIZE_MASK     (0x7ff << LCDC_HEOCFG3_XSIZE_SHIFT)
#  define LCDC_HEOCFG3_XSIZE(n)     ((uint32_t)(n) << LCDC_HEOCFG3_XSIZE_SHIFT)
#define LCDC_HEOCFG3_YSIZE_SHIFT    (16)    /* Bits 16-26: Vertical Window Size */
#define LCDC_HEOCFG3_YSIZE_MASK     (0x7ff << LCDC_HEOCFG3_YSIZE_SHIFT)
#  define LCDC_HEOCFG3_YSIZE(n)     ((uint32_t)(n) << LCDC_HEOCFG3_YSIZE_SHIFT)

/* High-End Overlay Configuration Register 4 */

#define LCDC_HEOCFG4_XMEMSIZE_SHIFT (0)     /* Bits 0-10: Horizontal image Size in Memory */
#define LCDC_HEOCFG4_XMEMSIZE_MASK  (0x7ff << LCDC_HEOCFG4_XMEMSIZE_SHIFT)
#  define LCDC_HEOCFG4_XMEMSIZE(n)  ((uint32_t)(n) << LCDC_HEOCFG4_XMEMSIZE_SHIFT)
#define LCDC_HEOCFG4_YMEMSIZE_SHIFT (16)    /* Bits 16-26: Vertical image Size in Memory */
#define LCDC_HEOCFG4_YMEMSIZE_MASK  (0x7ff << LCDC_HEOCFG4_YMEMSIZE_SHIFT)
#  define LCDC_HEOCFG4_YMEMSIZE(n)  ((uint32_t)(n) << LCDC_HEOCFG4_YMEMSIZE_SHIFT)

/* High-End Overlay Configuration Register 5
 * (32-bit horizontal stride value)
 */

/* High-End Overlay Configuration Register 6
 * (32-bit pixel stride value)
 */

/* High-End Overlay Configuration Register 7
 * (32-bit horizontal stride value)
 */

/* High-End Overlay Configuration Register 8
 * (32-bit pixel stride value)
 */

/* High-End Overlay Configuration Register 9 */

#define LCDC_HEOCFG9_BDEF_SHIFT     (0)       /* Bits 0-7: B Default */
#define LCDC_HEOCFG9_BDEF_MASK      (0xff << LCDC_HEOCFG9_BDEF_SHIFT)
#  define LCDC_HEOCFG9_BDEF(n)      ((uint32_t)(n) << LCDC_HEOCFG9_BDEF_SHIFT)
#define LCDC_HEOCFG9_GDEF_SHIFT     (8)       /* Bits 8-15: G Default */
#define LCDC_HEOCFG9_GDEF_MASK      (0xff << LCDC_HEOCFG9_GDEF_SHIFT)
#  define LCDC_HEOCFG9_GDEF(n)      ((uint32_t)(n) << LCDC_HEOCFG9_GDEF_SHIFT)
#define LCDC_HEOCFG9_RDEF_SHIFT     (16)      /* Bits 16-23: R Default */
#define LCDC_HEOCFG9_RDEF_MASK      (0xff << LCDC_HEOCFG9_RDEF_SHIFT)
#  define LCDC_HEOCFG9_RDEF(n)      ((uint32_t)(n) << LCDC_HEOCFG9_RDEF_SHIFT)

/* High-End Overlay Configuration Register 10 */

#define LCDC_HEOCFG10_BKEY_SHIFT    (0)       /* Bits 0-7: B Color Component Chroma Key */
#define LCDC_HEOCFG10_BKEY_MASK     (0xff << LCDC_HEOCFG10_BKEY_SHIFT)
#  define LCDC_HEOCFG10_BKEY(n)     ((uint32_t)(n) << LCDC_HEOCFG10_BKEY_SHIFT)
#define LCDC_HEOCFG10_GKEY_SHIFT    (8)       /* Bits 8-15: G Color Component Chroma Key */
#define LCDC_HEOCFG10_GKEY_MASK     (0xff << LCDC_HEOCFG10_GKEY_SHIFT)
#  define LCDC_HEOCFG10_GKEY(n)     ((uint32_t)(n) << LCDC_HEOCFG10_GKEY_SHIFT)
#define LCDC_HEOCFG10_RKEY_SHIFT    (16)      /* Bits 16-23: R Color Component Chroma Key */
#define LCDC_HEOCFG10_RKEY_MASK     (0xff << LCDC_HEOCFG10_RKEY_SHIFT)
#  define LCDC_HEOCFG10_RKEY(n)     ((uint32_t)(n) << LCDC_HEOCFG10_RKEY_SHIFT)

/* High-End Overlay Configuration Register 11 */

#define LCDC_HEOCFG11_BMASK_SHIFT   (0)       /* Bits 0-7: B Color Component Chroma Key Mask */
#define LCDC_HEOCFG11_BMASK_MASK    (0xff << LCDC_HEOCFG11_BMASK_SHIFT)
#  define LCDC_HEOCFG11_BMASK(n)    ((uint32_t)(n) << LCDC_HEOCFG11_BMASK_SHIFT)
#define LCDC_HEOCFG11_GMASK_SHIFT   (8)       /* Bits 8-15: G Color Component Chroma Key Mask */
#define LCDC_HEOCFG11_GMASK_MASK    (0xff << LCDC_HEOCFG11_GMASK_SHIFT)
#  define LCDC_HEOCFG11_GMASK(n)    ((uint32_t)(n) << LCDC_HEOCFG11_GMASK_SHIFT)
#define LCDC_HEOCFG11_RMASK_SHIFT   (16)      /* Bits 16-23: R Color Component Chroma Key Mask */
#define LCDC_HEOCFG11_RMASK_MASK    (0xff << LCDC_HEOCFG11_RMASK_SHIFT)
#  define LCDC_HEOCFG11_RMASK(n)    ((uint32_t)(n) << LCDC_HEOCFG11_RMASK_SHIFT)

/* High-End Overlay Configuration Register 12 */

#define LCDC_HEOCFG12_CRKEY         (1 << 0)  /* Bit 0:  Blender Chroma Key Enable */
#define LCDC_HEOCFG12_INV           (1 << 1)  /* Bit 1:  Blender Inverted Blender Output Enable */
#define LCDC_HEOCFG12_ITER2BL       (1 << 2)  /* Bit 2:  Blender Iterated Color Enable */
#define LCDC_HEOCFG12_ITER          (1 << 3)  /* Bit 3:  Blender Use Iterated Color */
#define LCDC_HEOCFG12_REVALPHA      (1 << 4)  /* Bit 4:  Blender Reverse Alpha */
#define LCDC_HEOCFG12_GAEN          (1 << 5)  /* Bit 5:  Blender Global Alpha Enable */
#define LCDC_HEOCFG12_LAEN          (1 << 6)  /* Bit 6:  Blender Local Alpha Enable */
#define LCDC_HEOCFG12_OVR           (1 << 7)  /* Bit 7:  Blender Overlay Layer Enable */
#define LCDC_HEOCFG12_DMA           (1 << 8)  /* Bit 8:  Blender DMA Layer Enable */
#define LCDC_HEOCFG12_REP           (1 << 9)  /* Bit 9:  Use Replication logic to expand RGB color */
#define LCDC_HEOCFG12_DSTKEY        (1 << 10) /* Bit 10: Destination Chroma Keying */
#define LCDC_HEOCFG12_VIDPRI        (1 << 12) /* Bit 12: Video Priority */
#define LCDC_HEOCFG12_GA_SHIFT      (16)      /* Bits 16-23: Blender Global Alpha */
#define LCDC_HEOCFG12_GA_MASK       (0xff << LCDC_HEOCFG12_GA_SHIFT)
#  define LCDC_HEOCFG12_GA(n)       ((uint32_t)(n) << LCDC_HEOCFG12_GA_SHIFT)

/* High-End Overlay Configuration Register 13 */

#define LCDC_HEOCFG13_XFACTOR_SHIFT (0)       /* Bits 0-13: Horizontal Scaling Factor */
#define LCDC_HEOCFG13_XFACTOR_MASK  (0x3fff << LCDC_HEOCFG13_XFACTOR_SHIFT)
#  define LCDC_HEOCFG13_XFACTOR(n)  ((uint32_t)(n) << LCDC_HEOCFG13_XFACTOR_SHIFT)
#define LCDC_HEOCFG13_YFACTOR_SHIFT (16)       /* Bits 16-29: Vertical Scaling Factor */
#define LCDC_HEOCFG13_YFACTOR_MASK  (0x3fff << LCDC_HEOCFG13_YFACTOR_SHIFT)
#  define LCDC_HEOCFG13_YFACTOR(n)  ((uint32_t)(n) << LCDC_HEOCFG13_YFACTOR_SHIFT)
#define LCDC_HEOCFG13_SCALEN        (1 << 31) /* Bit 31: Hardware Scaler Enable */

/* High-End Overlay Configuration Register 14 */

#define LCDC_HEOCFG14_CSCRY_SHIFT   (0)       /* Bits 0-9: Color Space Conversion Y coeff for R */
#define LCDC_HEOCFG14_CSCRY_MASK    (0x3ff << LCDC_HEOCFG14_CSCRY_SHIFT)
#  define LCDC_HEOCFG14_CSCRY(n)    ((uint32_t)(n) << LCDC_HEOCFG14_CSCRY_SHIFT)
#define LCDC_HEOCFG14_CSCRU_SHIFT   (10)       /* Bits 10-19: Color Space Conversion U coeff for R */
#define LCDC_HEOCFG14_CSCRU_MASK    (0x3ff << LCDC_HEOCFG14_CSCRU_SHIFT)
#  define LCDC_HEOCFG14_CSCRU(n)    ((uint32_t)(n) << LCDC_HEOCFG14_CSCRU_SHIFT)
#define LCDC_HEOCFG14_CSCRV_SHIFT   (20)       /* Bits 20-29: Color Space Conversion V coeff for R */
#define LCDC_HEOCFG14_CSCRV_MASK    (0x3ff << LCDC_HEOCFG14_CSCRV_SHIFT)
#  define LCDC_HEOCFG14_CSCRV(n)    ((uint32_t)(n) << LCDC_HEOCFG14_CSCRV_SHIFT)
#define LCDC_HEOCFG14_CSCYOFF       (1 << 30)  /* Bit 30: Color Space Conversion Offset */

/* High-End Overlay Configuration Register 15 */

#define LCDC_HEOCFG15_CSCGY_SHIFT   (0)       /* Bits 0-9: Color Space Conversion Y coeff for G */
#define LCDC_HEOCFG15_CSCGY_MASK    (0x3ff << LCDC_HEOCFG15_CSCGY_SHIFT)
#  define LCDC_HEOCFG15_CSCGY(n)    ((uint32_t)(n) << LCDC_HEOCFG15_CSCGY_SHIFT)
#define LCDC_HEOCFG15_CSCGU_SHIFT   (10)       /* Bits 10-19: Color Space Conversion U coeff for G */
#define LCDC_HEOCFG15_CSCGU_MASK    (0x3ff << LCDC_HEOCFG15_CSCGU_SHIFT)
#  define LCDC_HEOCFG15_CSCGU(n)    ((uint32_t)(n) << LCDC_HEOCFG15_CSCGU_SHIFT)
#define LCDC_HEOCFG15_CSCGV_SHIFT   (20)       /* Bits 20-29: Color Space Conversion V coeff for G */
#define LCDC_HEOCFG15_CSCGV_MASK    (0x3ff << LCDC_HEOCFG15_CSCGV_SHIFT)
#  define LCDC_HEOCFG15_CSCGV(n)    ((uint32_t)(n) << LCDC_HEOCFG15_CSCGV_SHIFT)
#define LCDC_HEOCFG15_CSCUOFF       (1 << 30)  /* Bit 30: Color Space Conversion Offset */

/* High-End Overlay Configuration Register 16 */

#define LCDC_HEOCFG16_CSCBY_SHIFT   (0)       /* Bits 0-9: Color Space Conversion Y coeff for B */
#define LCDC_HEOCFG16_CSCBY_MASK    (0x3ff << LCDC_HEOCFG16_CSCBY_SHIFT)
#  define LCDC_HEOCFG16_CSCBY(n)    ((uint32_t)(n) << LCDC_HEOCFG16_CSCBY_SHIFT)
#define LCDC_HEOCFG16_CSCBU_SHIFT   (10)       /* Bits 10-19: Color Space Conversion U coeff for B */
#define LCDC_HEOCFG16_CSCBU_MASK    (0x3ff << LCDC_HEOCFG16_CSCBU_SHIFT)
#  define LCDC_HEOCFG16_CSCBU(n)    ((uint32_t)(n) << LCDC_HEOCFG16_CSCBU_SHIFT)
#define LCDC_HEOCFG16_CSCBV_SHIFT   (20)       /* Bits 20-29: Color Space Conversion V coeff for B */
#define LCDC_HEOCFG16_CSCBV_MASK    (0x3ff << LCDC_HEOCFG16_CSCBV_SHIFT)
#  define LCDC_HEOCFG16_CSCBV(n)    ((uint32_t)(n) << LCDC_HEOCFG16_CSCBV_SHIFT)
#define LCDC_HEOCFG16_CSCVOFF       (1 << 30)  /* Bit 30: Color Space Conversion Offset */

/* High-End Overlay Configuration Register 17 */

#define LCDC_HEOCFG17_XPHI0COEFF0_SHIFT (0)    /* Bits 0-7: Horizontal Coefficient for phase 0 tap 0 */
#define LCDC_HEOCFG17_XPHI0COEFF0_MASK  (0xff << LCDC_HEOCFG17_XPHI0COEFF0_SHIFT)
#  define LCDC_HEOCFG17_XPHI0COEFF0(n)  ((uint32_t)(n) << LCDC_HEOCFG17_XPHI0COEFF0_SHIFT)
#define LCDC_HEOCFG17_XPHI0COEFF1_SHIFT (8)    /* Bits 8-15: Horizontal Coefficient for phase 0 tap 1 */
#define LCDC_HEOCFG17_XPHI0COEFF1_MASK  (0xff << LCDC_HEOCFG17_XPHI0COEFF1_SHIFT)
#  define LCDC_HEOCFG17_XPHI0COEFF1(n)  ((uint32_t)(n) << LCDC_HEOCFG17_XPHI0COEFF1_SHIFT)
#define LCDC_HEOCFG17_XPHI0COEFF2_SHIFT (16)   /* Bits 16-23: Horizontal Coefficient for phase 0 tap 2 */
#define LCDC_HEOCFG17_XPHI0COEFF2_MASK  (0xff << LCDC_HEOCFG17_XPHI0COEFF2_SHIFT)
#  define LCDC_HEOCFG17_XPHI0COEFF2(n)  ((uint32_t)(n) << LCDC_HEOCFG17_XPHI0COEFF2_SHIFT)
#define LCDC_HEOCFG17_XPHI0COEFF3_SHIFT (24)   /* Bits 24-31: Horizontal Coefficient for phase 0 tap 3 */
#define LCDC_HEOCFG17_XPHI0COEFF3_MASK  (0xff << LCDC_HEOCFG17_XPHI0COEFF3_SHIFT)
#  define LCDC_HEOCFG17_XPHI0COEFF3(n)  ((uint32_t)(n) << LCDC_HEOCFG17_XPHI0COEFF3_SHIFT)

/* High-End Overlay Configuration Register 18 */

#define LCDC_HEOCFG18_XPHI0COEFF4_SHIFT (0)    /* Bits 0-7: Horizontal Coefficient for phase 0 tap 4 */
#define LCDC_HEOCFG18_XPHI0COEFF4_MASK  (0xff << LCDC_HEOCFG18_XPHI0COEFF4_SHIFT)
#  define LCDC_HEOCFG18_XPHI0COEFF4(n)  ((uint32_t)(n) << LCDC_HEOCFG18_XPHI0COEFF4_SHIFT)

/* High-End Overlay Configuration Register 19 */

#define LCDC_HEOCFG19_XPHI1COEFF0_SHIFT (0)    /* Bits 0-7: Horizontal Coefficient for phase 1 tap 0 */
#define LCDC_HEOCFG19_XPHI1COEFF0_MASK  (0xff << LCDC_HEOCFG19_XPHI1COEFF0_SHIFT)
#  define LCDC_HEOCFG19_XPHI1COEFF0(n)  ((uint32_t)(n) << LCDC_HEOCFG19_XPHI1COEFF0_SHIFT)
#define LCDC_HEOCFG19_XPHI1COEFF1_SHIFT (8)    /* Bits 8-15: Horizontal Coefficient for phase 1 tap 1 */
#define LCDC_HEOCFG19_XPHI1COEFF1_MASK  (0xff << LCDC_HEOCFG19_XPHI1COEFF1_SHIFT)
#  define LCDC_HEOCFG19_XPHI1COEFF1(n)  ((uint32_t)(n) << LCDC_HEOCFG19_XPHI1COEFF1_SHIFT)
#define LCDC_HEOCFG19_XPHI1COEFF2_SHIFT (16)   /* Bits 16-23: Horizontal Coefficient for phase 1 tap 2 */
#define LCDC_HEOCFG19_XPHI1COEFF2_MASK  (0xff << LCDC_HEOCFG19_XPHI1COEFF2_SHIFT)
#  define LCDC_HEOCFG19_XPHI1COEFF2(n)  ((uint32_t)(n) << LCDC_HEOCFG19_XPHI1COEFF2_SHIFT)
#define LCDC_HEOCFG19_XPHI1COEFF3_SHIFT (24)   /* Bits 24-31: Horizontal Coefficient for phase 1 tap 3 */
#define LCDC_HEOCFG19_XPHI1COEFF3_MASK  (0xff << LCDC_HEOCFG19_XPHI1COEFF3_SHIFT)
#  define LCDC_HEOCFG19_XPHI1COEFF3(n)  ((uint32_t)(n) << LCDC_HEOCFG19_XPHI1COEFF3_SHIFT)

/* High-End Overlay Configuration Register 20 */

#define LCDC_HEOCFG20_XPHI1COEFF4_SHIFT (0)    /* Bits 0-7: Horizontal Coefficient for phase 1 tap 4 */
#define LCDC_HEOCFG20_XPHI1COEFF4_MASK  (0xff << LCDC_HEOCFG20_XPHI1COEFF4_SHIFT)
#  define LCDC_HEOCFG20_XPHI1COEFF4(n)  ((uint32_t)(n) << LCDC_HEOCFG20_XPHI1COEFF4_SHIFT)

/* High-End Overlay Configuration Register 21 */

#define LCDC_HEOCFG21_XPHI2COEFF0_SHIFT (0)    /* Bits 0-7: Horizontal Coefficient for phase 2 tap 0 */
#define LCDC_HEOCFG21_XPHI2COEFF0_MASK  (0xff << LCDC_HEOCFG21_XPHI2COEFF0_SHIFT)
#  define LCDC_HEOCFG21_XPHI2COEFF0(n)  ((uint32_t)(n) << LCDC_HEOCFG21_XPHI2COEFF0_SHIFT)
#define LCDC_HEOCFG21_XPHI2COEFF1_SHIFT (8)    /* Bits 8-15: Horizontal Coefficient for phase 2 tap 1 */
#define LCDC_HEOCFG21_XPHI2COEFF1_MASK  (0xff << LCDC_HEOCFG21_XPHI2COEFF1_SHIFT)
#  define LCDC_HEOCFG21_XPHI2COEFF1(n)  ((uint32_t)(n) << LCDC_HEOCFG21_XPHI2COEFF1_SHIFT)
#define LCDC_HEOCFG21_XPHI2COEFF2_SHIFT (16)   /* Bits 16-23: Horizontal Coefficient for phase 2 tap 2 */
#define LCDC_HEOCFG21_XPHI2COEFF2_MASK  (0xff << LCDC_HEOCFG21_XPHI2COEFF2_SHIFT)
#  define LCDC_HEOCFG21_XPHI2COEFF2(n)  ((uint32_t)(n) << LCDC_HEOCFG21_XPHI2COEFF2_SHIFT)
#define LCDC_HEOCFG21_XPHI2COEFF3_SHIFT (24)   /* Bits 24-31: Horizontal Coefficient for phase 2 tap 3 */
#define LCDC_HEOCFG21_XPHI2COEFF3_MASK  (0xff << LCDC_HEOCFG21_XPHI2COEFF3_SHIFT)
#  define LCDC_HEOCFG21_XPHI2COEFF3(n)  ((uint32_t)(n) << LCDC_HEOCFG21_XPHI2COEFF3_SHIFT)

/* High-End Overlay Configuration Register 22 */

#define LCDC_HEOCFG22_XPHI2COEFF4_SHIFT (0)    /* Bits 0-7: Horizontal Coefficient for phase 2 tap 4 */
#define LCDC_HEOCFG22_XPHI2COEFF4_MASK  (0xff << LCDC_HEOCFG22_XPHI2COEFF4_SHIFT)
#  define LCDC_HEOCFG22_XPHI2COEFF4(n)  ((uint32_t)(n) << LCDC_HEOCFG22_XPHI2COEFF4_SHIFT)

/* High-End Overlay Configuration Register 23 */

#define LCDC_HEOCFG23_XPHI3COEFF0_SHIFT (0)    /* Bits 0-7: Horizontal Coefficient for phase 3 tap 0 */
#define LCDC_HEOCFG23_XPHI3COEFF0_MASK  (0xff << LCDC_HEOCFG23_XPHI3COEFF0_SHIFT)
#  define LCDC_HEOCFG23_XPHI3COEFF0(n)  ((uint32_t)(n) << LCDC_HEOCFG23_XPHI3COEFF0_SHIFT)
#define LCDC_HEOCFG23_XPHI3COEFF1_SHIFT (8)    /* Bits 8-15: Horizontal Coefficient for phase 3 tap 1 */
#define LCDC_HEOCFG23_XPHI3COEFF1_MASK  (0xff << LCDC_HEOCFG23_XPHI3COEFF1_SHIFT)
#  define LCDC_HEOCFG23_XPHI3COEFF1(n)  ((uint32_t)(n) << LCDC_HEOCFG23_XPHI3COEFF1_SHIFT)
#define LCDC_HEOCFG23_XPHI3COEFF2_SHIFT (16)   /* Bits 16-23: Horizontal Coefficient for phase 3 tap 2 */
#define LCDC_HEOCFG23_XPHI3COEFF2_MASK  (0xff << LCDC_HEOCFG23_XPHI3COEFF2_SHIFT)
#  define LCDC_HEOCFG23_XPHI3COEFF2(n)  ((uint32_t)(n) << LCDC_HEOCFG23_XPHI3COEFF2_SHIFT)
#define LCDC_HEOCFG23_XPHI3COEFF3_SHIFT (24)   /* Bits 24-31: Horizontal Coefficient for phase 3 tap 3 */
#define LCDC_HEOCFG23_XPHI3COEFF3_MASK  (0xff << LCDC_HEOCFG23_XPHI3COEFF3_SHIFT)
#  define LCDC_HEOCFG23_XPHI3COEFF3(n)  ((uint32_t)(n) << LCDC_HEOCFG23_XPHI3COEFF3_SHIFT)

/* High-End Overlay Configuration Register 24 */

#define LCDC_HEOCFG24_XPHI3COEFF4_SHIFT (0)    /* Bits 0-7: Horizontal Coefficient for phase 3 tap 4 */
#define LCDC_HEOCFG24_XPHI3COEFF4_MASK  (0xff << LCDC_HEOCFG24_XPHI3COEFF4_SHIFT)
#  define LCDC_HEOCFG24_XPHI3COEFF4(n)  ((uint32_t)(n) << LCDC_HEOCFG24_XPHI3COEFF4_SHIFT)

/* High-End Overlay Configuration Register 25 */

#define LCDC_HEOCFG25_XPHI4COEFF0_SHIFT (0)    /* Bits 0-7: Horizontal Coefficient for phase 4 tap 0 */
#define LCDC_HEOCFG25_XPHI4COEFF0_MASK  (0xff << LCDC_HEOCFG25_XPHI4COEFF0_SHIFT)
#  define LCDC_HEOCFG25_XPHI4COEFF0(n)  ((uint32_t)(n) << LCDC_HEOCFG25_XPHI4COEFF0_SHIFT)
#define LCDC_HEOCFG25_XPHI4COEFF1_SHIFT (8)    /* Bits 8-15: Horizontal Coefficient for phase 4 tap 1 */
#define LCDC_HEOCFG25_XPHI4COEFF1_MASK  (0xff << LCDC_HEOCFG25_XPHI4COEFF1_SHIFT)
#  define LCDC_HEOCFG25_XPHI4COEFF1(n)  ((uint32_t)(n) << LCDC_HEOCFG25_XPHI4COEFF1_SHIFT)
#define LCDC_HEOCFG25_XPHI4COEFF2_SHIFT (16)   /* Bits 16-25: Horizontal Coefficient for phase 4 tap 2 */
#define LCDC_HEOCFG25_XPHI4COEFF2_MASK  (0xff << LCDC_HEOCFG25_XPHI4COEFF2_SHIFT)
#  define LCDC_HEOCFG25_XPHI4COEFF2(n)  ((uint32_t)(n) << LCDC_HEOCFG25_XPHI4COEFF2_SHIFT)
#define LCDC_HEOCFG25_XPHI4COEFF3_SHIFT (24)   /* Bits 24-31: Horizontal Coefficient for phase 4 tap 3 */
#define LCDC_HEOCFG25_XPHI4COEFF3_MASK  (0xff << LCDC_HEOCFG25_XPHI4COEFF3_SHIFT)
#  define LCDC_HEOCFG25_XPHI4COEFF3(n)  ((uint32_t)(n) << LCDC_HEOCFG25_XPHI4COEFF3_SHIFT)

/* High-End Overlay Configuration Register 26 */

#define LCDC_HEOCFG26_XPHI4COEFF4_SHIFT (0)    /* Bits 0-7: Horizontal Coefficient for phase 4 tap 4 */
#define LCDC_HEOCFG26_XPHI4COEFF4_MASK  (0xff << LCDC_HEOCFG26_XPHI4COEFF4_SHIFT)
#  define LCDC_HEOCFG26_XPHI4COEFF4(n)  ((uint32_t)(n) << LCDC_HEOCFG26_XPHI4COEFF4_SHIFT)

/* High-End Overlay Configuration Register 27 */

#define LCDC_HEOCFG27_XPHI5COEFF0_SHIFT (0)    /* Bits 0-7: Horizontal Coefficient for phase 5 tap 0 */
#define LCDC_HEOCFG27_XPHI5COEFF0_MASK  (0xff << LCDC_HEOCFG27_XPHI5COEFF0_SHIFT)
#  define LCDC_HEOCFG27_XPHI5COEFF0(n)  ((uint32_t)(n) << LCDC_HEOCFG27_XPHI5COEFF0_SHIFT)
#define LCDC_HEOCFG27_XPHI5COEFF1_SHIFT (8)    /* Bits 8-15: Horizontal Coefficient for phase 5 tap 1 */
#define LCDC_HEOCFG27_XPHI5COEFF1_MASK  (0xff << LCDC_HEOCFG27_XPHI5COEFF1_SHIFT)
#  define LCDC_HEOCFG27_XPHI5COEFF1(n)  ((uint32_t)(n) << LCDC_HEOCFG27_XPHI5COEFF1_SHIFT)
#define LCDC_HEOCFG27_XPHI5COEFF2_SHIFT (16)   /* Bits 16-25: Horizontal Coefficient for phase 5 tap 2 */
#define LCDC_HEOCFG27_XPHI5COEFF2_MASK  (0xff << LCDC_HEOCFG27_XPHI5COEFF2_SHIFT)
#  define LCDC_HEOCFG27_XPHI5COEFF2(n)  ((uint32_t)(n) << LCDC_HEOCFG27_XPHI5COEFF2_SHIFT)
#define LCDC_HEOCFG27_XPHI5COEFF3_SHIFT (24)   /* Bits 24-31: Horizontal Coefficient for phase 5 tap 3 */
#define LCDC_HEOCFG27_XPHI5COEFF3_MASK  (0xff << LCDC_HEOCFG27_XPHI5COEFF3_SHIFT)
#  define LCDC_HEOCFG27_XPHI5COEFF3(n)  ((uint32_t)(n) << LCDC_HEOCFG27_XPHI5COEFF3_SHIFT)

/* High-End Overlay Configuration Register 28 */

#define LCDC_HEOCFG28_XPHI5COEFF4_SHIFT (0)    /* Bits 0-7: Horizontal Coefficient for phase 5 tap 4 */
#define LCDC_HEOCFG28_XPHI5COEFF4_MASK  (0xff << LCDC_HEOCFG28_XPHI5COEFF4_SHIFT)
#  define LCDC_HEOCFG28_XPHI5COEFF4(n)  ((uint32_t)(n) << LCDC_HEOCFG28_XPHI5COEFF4_SHIFT)

/* High-End Overlay Configuration Register 29 */

#define LCDC_HEOCFG29_XPHI6COEFF0_SHIFT (0)    /* Bits 0-7: Horizontal Coefficient for phase 6 tap 0 */
#define LCDC_HEOCFG29_XPHI6COEFF0_MASK  (0xff << LCDC_HEOCFG29_XPHI6COEFF0_SHIFT)
#  define LCDC_HEOCFG29_XPHI6COEFF0(n)  ((uint32_t)(n) << LCDC_HEOCFG29_XPHI6COEFF0_SHIFT)
#define LCDC_HEOCFG29_XPHI6COEFF1_SHIFT (8)    /* Bits 8-15: Horizontal Coefficient for phase 6 tap 1 */
#define LCDC_HEOCFG29_XPHI6COEFF1_MASK  (0xff << LCDC_HEOCFG29_XPHI6COEFF1_SHIFT)
#  define LCDC_HEOCFG29_XPHI6COEFF1(n)  ((uint32_t)(n) << LCDC_HEOCFG29_XPHI6COEFF1_SHIFT)
#define LCDC_HEOCFG29_XPHI6COEFF2_SHIFT (16)   /* Bits 16-25: Horizontal Coefficient for phase 6 tap 2 */
#define LCDC_HEOCFG29_XPHI6COEFF2_MASK  (0xff << LCDC_HEOCFG29_XPHI6COEFF2_SHIFT)
#  define LCDC_HEOCFG29_XPHI6COEFF2(n)  ((uint32_t)(n) << LCDC_HEOCFG29_XPHI6COEFF2_SHIFT)
#define LCDC_HEOCFG29_XPHI6COEFF3_SHIFT (24)   /* Bits 24-31: Horizontal Coefficient for phase 6 tap 3 */
#define LCDC_HEOCFG29_XPHI6COEFF3_MASK  (0xff << LCDC_HEOCFG29_XPHI6COEFF3_SHIFT)
#  define LCDC_HEOCFG29_XPHI6COEFF3(n)  ((uint32_t)(n) << LCDC_HEOCFG29_XPHI6COEFF3_SHIFT)

/* High-End Overlay Configuration Register 30 */

#define LCDC_HEOCFG30_XPHI6COEFF4_SHIFT (0)    /* Bits 0-7: Horizontal Coefficient for phase 6 tap 4 */
#define LCDC_HEOCFG30_XPHI6COEFF4_MASK  (0xff << LCDC_HEOCFG30_XPHI6COEFF4_SHIFT)
#  define LCDC_HEOCFG30_XPHI6COEFF4(n)  ((uint32_t)(n) << LCDC_HEOCFG30_XPHI6COEFF4_SHIFT)

/* High-End Overlay Configuration Register 31 */

#define LCDC_HEOCFG31_XPHI7COEFF0_SHIFT (0)    /* Bits 0-7: Horizontal Coefficient for phase 7 tap 0 */
#define LCDC_HEOCFG31_XPHI7COEFF0_MASK  (0xff << LCDC_HEOCFG31_XPHI7COEFF0_SHIFT)
#  define LCDC_HEOCFG31_XPHI7COEFF0(n)  ((uint32_t)(n) << LCDC_HEOCFG31_XPHI7COEFF0_SHIFT)
#define LCDC_HEOCFG31_XPHI7COEFF1_SHIFT (8)    /* Bits 8-15: Horizontal Coefficient for phase 7 tap 1 */
#define LCDC_HEOCFG31_XPHI7COEFF1_MASK  (0xff << LCDC_HEOCFG31_XPHI7COEFF1_SHIFT)
#  define LCDC_HEOCFG31_XPHI7COEFF1(n)  ((uint32_t)(n) << LCDC_HEOCFG31_XPHI7COEFF1_SHIFT)
#define LCDC_HEOCFG31_XPHI7COEFF2_SHIFT (16)   /* Bits 16-25: Horizontal Coefficient for phase 7 tap 2 */
#define LCDC_HEOCFG31_XPHI7COEFF2_MASK  (0xff << LCDC_HEOCFG31_XPHI7COEFF2_SHIFT)
#  define LCDC_HEOCFG31_XPHI7COEFF2(n)  ((uint32_t)(n) << LCDC_HEOCFG31_XPHI7COEFF2_SHIFT)
#define LCDC_HEOCFG31_XPHI7COEFF3_SHIFT (24)   /* Bits 24-31: Horizontal Coefficient for phase 7 tap 3 */
#define LCDC_HEOCFG31_XPHI7COEFF3_MASK  (0xff << LCDC_HEOCFG31_XPHI7COEFF3_SHIFT)
#  define LCDC_HEOCFG31_XPHI7COEFF3(n)  ((uint32_t)(n) << LCDC_HEOCFG31_XPHI7COEFF3_SHIFT)

/* High-End Overlay Configuration Register 32 */

#define LCDC_HEOCFG30_XPHI7COEFF4_SHIFT (0)    /* Bits 0-7: Horizontal Coefficient for phase 7 tap 4 */
#define LCDC_HEOCFG30_XPHI7COEFF4_MASK  (0xff << LCDC_HEOCFG30_XPHI7COEFF4_SHIFT)
#  define LCDC_HEOCFG30_XPHI7COEFF4(n)  ((uint32_t)(n) << LCDC_HEOCFG30_XPHI7COEFF4_SHIFT)

/* High-End Overlay Configuration Register 33 */

#define LCDC_HEOCFG33_YPHI0COEFF0_SHIFT (0)    /* Bits 0-7: Vertical Coefficient for phase 0 tap 0 */
#define LCDC_HEOCFG33_YPHI0COEFF0_MASK  (0xff << LCDC_HEOCFG33_YPHI0COEFF0_SHIFT)
#  define LCDC_HEOCFG33_YPHI0COEFF0(n)  ((uint32_t)(n) << LCDC_HEOCFG33_YPHI0COEFF0_SHIFT)
#define LCDC_HEOCFG33_YPHI0COEFF1_SHIFT (8)    /* Bits 8-15: Vertical Coefficient for phase 0 tap 1 */
#define LCDC_HEOCFG33_YPHI0COEFF1_MASK  (0xff << LCDC_HEOCFG33_YPHI0COEFF1_SHIFT)
#  define LCDC_HEOCFG33_YPHI0COEFF1(n)  ((uint32_t)(n) << LCDC_HEOCFG33_YPHI0COEFF1_SHIFT)
#define LCDC_HEOCFG33_YPHI0COEFF2_SHIFT (16)   /* Bits 16-23: Vertical Coefficient for phase 0 tap 2 */
#define LCDC_HEOCFG33_YPHI0COEFF2_MASK  (0xff << LCDC_HEOCFG33_YPHI0COEFF2_SHIFT)
#  define LCDC_HEOCFG33_YPHI0COEFF2(n)  ((uint32_t)(n) << LCDC_HEOCFG33_YPHI0COEFF2_SHIFT)

/* High-End Overlay Configuration Register 34 */

#define LCDC_HEOCFG34_YPHI1COEFF0_SHIFT (0)    /* Bits 0-7: Vertical Coefficient for phase 1 tap 0 */
#define LCDC_HEOCFG34_YPHI1COEFF0_MASK  (0xff << LCDC_HEOCFG34_YPHI1COEFF0_SHIFT)
#  define LCDC_HEOCFG34_YPHI1COEFF0(n)  ((uint32_t)(n) << LCDC_HEOCFG34_YPHI1COEFF0_SHIFT)
#define LCDC_HEOCFG34_YPHI1COEFF1_SHIFT (8)    /* Bits 8-15: Vertical Coefficient for phase 1 tap 1 */
#define LCDC_HEOCFG34_YPHI1COEFF1_MASK  (0xff << LCDC_HEOCFG34_YPHI1COEFF1_SHIFT)
#  define LCDC_HEOCFG34_YPHI1COEFF1(n)  ((uint32_t)(n) << LCDC_HEOCFG34_YPHI1COEFF1_SHIFT)
#define LCDC_HEOCFG34_YPHI1COEFF2_SHIFT (16)   /* Bits 16-23: Vertical Coefficient for phase 1 tap 2 */
#define LCDC_HEOCFG34_YPHI1COEFF2_MASK  (0xff << LCDC_HEOCFG34_YPHI1COEFF2_SHIFT)
#  define LCDC_HEOCFG34_YPHI1COEFF2(n)  ((uint32_t)(n) << LCDC_HEOCFG34_YPHI1COEFF2_SHIFT)

/* High-End Overlay Configuration Register 35 */

#define LCDC_HEOCFG35_YPHI2COEFF0_SHIFT (0)    /* Bits 0-7: Vertical Coefficient for phase 2 tap 0 */
#define LCDC_HEOCFG35_YPHI2COEFF0_MASK  (0xff << LCDC_HEOCFG35_YPHI2COEFF0_SHIFT)
#  define LCDC_HEOCFG35_YPHI2COEFF0(n)  ((uint32_t)(n) << LCDC_HEOCFG35_YPHI2COEFF0_SHIFT)
#define LCDC_HEOCFG35_YPHI2COEFF1_SHIFT (8)    /* Bits 8-15: Vertical Coefficient for phase 2 tap 1 */
#define LCDC_HEOCFG35_YPHI2COEFF1_MASK  (0xff << LCDC_HEOCFG35_YPHI2COEFF1_SHIFT)
#  define LCDC_HEOCFG35_YPHI2COEFF1(n)  ((uint32_t)(n) << LCDC_HEOCFG35_YPHI2COEFF1_SHIFT)
#define LCDC_HEOCFG35_YPHI2COEFF2_SHIFT (16)   /* Bits 16-23: Vertical Coefficient for phase 2 tap 2 */
#define LCDC_HEOCFG35_YPHI2COEFF2_MASK  (0xff << LCDC_HEOCFG35_YPHI2COEFF2_SHIFT)
#  define LCDC_HEOCFG35_YPHI2COEFF2(n)  ((uint32_t)(n) << LCDC_HEOCFG35_YPHI2COEFF2_SHIFT)

/* High-End Overlay Configuration Register 36 */

#define LCDC_HEOCFG36_YPHI3COEFF0_SHIFT (0)    /* Bits 0-7: Vertical Coefficient for phase 3 tap 0 */
#define LCDC_HEOCFG36_YPHI3COEFF0_MASK  (0xff << LCDC_HEOCFG36_YPHI3COEFF0_SHIFT)
#  define LCDC_HEOCFG36_YPHI3COEFF0(n)  ((uint32_t)(n) << LCDC_HEOCFG36_YPHI3COEFF0_SHIFT)
#define LCDC_HEOCFG36_YPHI3COEFF1_SHIFT (8)    /* Bits 8-15: Vertical Coefficient for phase 3 tap 1 */
#define LCDC_HEOCFG36_YPHI3COEFF1_MASK  (0xff << LCDC_HEOCFG36_YPHI3COEFF1_SHIFT)
#  define LCDC_HEOCFG36_YPHI3COEFF1(n)  ((uint32_t)(n) << LCDC_HEOCFG36_YPHI3COEFF1_SHIFT)
#define LCDC_HEOCFG36_YPHI3COEFF2_SHIFT (16)   /* Bits 16-23: Vertical Coefficient for phase 3 tap 2 */
#define LCDC_HEOCFG36_YPHI3COEFF2_MASK  (0xff << LCDC_HEOCFG36_YPHI3COEFF2_SHIFT)
#  define LCDC_HEOCFG36_YPHI3COEFF2(n)  ((uint32_t)(n) << LCDC_HEOCFG36_YPHI3COEFF2_SHIFT)

/* High-End Overlay Configuration Register 37 */

#define LCDC_HEOCFG37_YPHI4COEFF0_SHIFT (0)    /* Bits 0-7: Vertical Coefficient for phase 4 tap 0 */
#define LCDC_HEOCFG37_YPHI4COEFF0_MASK  (0xff << LCDC_HEOCFG37_YPHI4COEFF0_SHIFT)
#  define LCDC_HEOCFG37_YPHI4COEFF0(n)  ((uint32_t)(n) << LCDC_HEOCFG37_YPHI4COEFF0_SHIFT)
#define LCDC_HEOCFG37_YPHI4COEFF1_SHIFT (8)    /* Bits 8-15: Vertical Coefficient for phase 4 tap 1 */
#define LCDC_HEOCFG37_YPHI4COEFF1_MASK  (0xff << LCDC_HEOCFG37_YPHI4COEFF1_SHIFT)
#  define LCDC_HEOCFG37_YPHI4COEFF1(n)  ((uint32_t)(n) << LCDC_HEOCFG37_YPHI4COEFF1_SHIFT)
#define LCDC_HEOCFG37_YPHI4COEFF2_SHIFT (16)   /* Bits 16-23: Vertical Coefficient for phase 4 tap 2 */
#define LCDC_HEOCFG37_YPHI4COEFF2_MASK  (0xff << LCDC_HEOCFG37_YPHI4COEFF2_SHIFT)
#  define LCDC_HEOCFG37_YPHI4COEFF2(n)  ((uint32_t)(n) << LCDC_HEOCFG37_YPHI4COEFF2_SHIFT)

/* High-End Overlay Configuration Register 38 */

#define LCDC_HEOCFG38_YPHI5COEFF0_SHIFT (0)    /* Bits 0-7: Vertical Coefficient for phase 5 tap 0 */
#define LCDC_HEOCFG38_YPHI5COEFF0_MASK  (0xff << LCDC_HEOCFG38_YPHI5COEFF0_SHIFT)
#  define LCDC_HEOCFG38_YPHI5COEFF0(n)  ((uint32_t)(n) << LCDC_HEOCFG38_YPHI5COEFF0_SHIFT)
#define LCDC_HEOCFG38_YPHI5COEFF1_SHIFT (8)    /* Bits 8-15: Vertical Coefficient for phase 5 tap 1 */
#define LCDC_HEOCFG38_YPHI5COEFF1_MASK  (0xff << LCDC_HEOCFG38_YPHI5COEFF1_SHIFT)
#  define LCDC_HEOCFG38_YPHI5COEFF1(n)  ((uint32_t)(n) << LCDC_HEOCFG38_YPHI5COEFF1_SHIFT)
#define LCDC_HEOCFG38_YPHI5COEFF2_SHIFT (16)   /* Bits 16-23: Vertical Coefficient for phase 5 tap 2 */
#define LCDC_HEOCFG38_YPHI5COEFF2_MASK  (0xff << LCDC_HEOCFG38_YPHI5COEFF2_SHIFT)
#  define LCDC_HEOCFG38_YPHI5COEFF2(n)  ((uint32_t)(n) << LCDC_HEOCFG38_YPHI5COEFF2_SHIFT)

/* High-End Overlay Configuration Register 39 */

#define LCDC_HEOCFG39_YPHI6COEFF0_SHIFT (0)    /* Bits 0-7: Vertical Coefficient for phase 6 tap 0 */
#define LCDC_HEOCFG39_YPHI6COEFF0_MASK  (0xff << LCDC_HEOCFG39_YPHI6COEFF0_SHIFT)
#  define LCDC_HEOCFG39_YPHI6COEFF0(n)  ((uint32_t)(n) << LCDC_HEOCFG39_YPHI6COEFF0_SHIFT)
#define LCDC_HEOCFG39_YPHI6COEFF1_SHIFT (8)    /* Bits 8-15: Vertical Coefficient for phase 6 tap 1 */
#define LCDC_HEOCFG39_YPHI6COEFF1_MASK  (0xff << LCDC_HEOCFG39_YPHI6COEFF1_SHIFT)
#  define LCDC_HEOCFG39_YPHI6COEFF1(n)  ((uint32_t)(n) << LCDC_HEOCFG39_YPHI6COEFF1_SHIFT)
#define LCDC_HEOCFG39_YPHI6COEFF2_SHIFT (16)   /* Bits 16-23: Vertical Coefficient for phase 6 tap 2 */
#define LCDC_HEOCFG39_YPHI6COEFF2_MASK  (0xff << LCDC_HEOCFG39_YPHI6COEFF2_SHIFT)
#  define LCDC_HEOCFG39_YPHI6COEFF2(n)  ((uint32_t)(n) << LCDC_HEOCFG39_YPHI6COEFF2_SHIFT)

/* High-End Overlay Configuration Register 40 */

#define LCDC_HEOCFG40_YPHI7COEFF0_SHIFT (0)    /* Bits 0-7: Vertical Coefficient for phase 7 tap 0 */
#define LCDC_HEOCFG40_YPHI7COEFF0_MASK  (0xff << LCDC_HEOCFG40_YPHI7COEFF0_SHIFT)
#  define LCDC_HEOCFG40_YPHI7COEFF0(n)  ((uint32_t)(n) << LCDC_HEOCFG40_YPHI7COEFF0_SHIFT)
#define LCDC_HEOCFG40_YPHI7COEFF1_SHIFT (8)    /* Bits 8-15: Vertical Coefficient for phase 7 tap 1 */
#define LCDC_HEOCFG40_YPHI7COEFF1_MASK  (0xff << LCDC_HEOCFG40_YPHI7COEFF1_SHIFT)
#  define LCDC_HEOCFG40_YPHI7COEFF1(n)  ((uint32_t)(n) << LCDC_HEOCFG40_YPHI7COEFF1_SHIFT)
#define LCDC_HEOCFG40_YPHI7COEFF2_SHIFT (16)   /* Bits 16-23: Vertical Coefficient for phase 7 tap 2 */
#define LCDC_HEOCFG40_YPHI7COEFF2_MASK  (0xff << LCDC_HEOCFG40_YPHI7COEFF2_SHIFT)
#  define LCDC_HEOCFG40_YPHI7COEFF2(n)  ((uint32_t)(n) << LCDC_HEOCFG40_YPHI7COEFF2_SHIFT)

/* High-End Overlay Configuration Register 41 */

#define LCDC_HEOCFG41_XPHIDEF_SHIFT (0)        /* Bits 0-2: Horizontal Filter Phase Offset */
#define LCDC_HEOCFG41_XPHIDEF_MASK  (7 << LCDC_HEOCFG41_XPHIDEF_SHIFT)
#  define LCDC_HEOCFG41_XPHIDEF(n)  ((uint32_t)(n) << LCDC_HEOCFG41_XPHIDEF_SHIFT)
#define LCDC_HEOCFG41_YPHIDEF_SHIFT (16)       /* Bits 16-18: Vertical Filter Phase Offset */
#define LCDC_HEOCFG41_YPHIDEF_MASK  (7 << LCDC_HEOCFG41_YPHIDEF_SHIFT)
#  define LCDC_HEOCFG41_YPHIDEF(n)  ((uint32_t)(n) << LCDC_HEOCFG41_YPHIDEF_SHIFT)

#ifdef ATSAMA5D3
/* Hardware Cursor Channel Enable Register */

#  define LCDC_HCRCHER_CH           (1 << 0)  /* Bit 0:  Channel Enable */
#  define LCDC_HCRCHER_UPDATE       (1 << 1)  /* Bit 1:  Update Overlay Attributes Enable */
#  define LCDC_HCRCHER_A2Q          (1 << 2)  /* Bit 2:  Add Head Pointer Enable */

/* Hardware Cursor Channel Disable Register */

#  define LCDC_HCRCHDR_CH           (1 << 0)  /* Bit 0:  Channel Disable */
#  define LCDC_HCRCHDR_CHRST        (1 << 8)  /* Bit 8:  Channel Reset */

/* Hardware Cursor Channel Status Register */

#  define LCDC_HCRCHSR_CH           (1 << 0)  /* Bit 0:  Channel Status */
#  define LCDC_HCRCHSR_UPDATE       (1 << 1)  /* Bit 1:  Update Overlay Attributes In */
#  define LCDC_HCRCHSR_A2Q          (1 << 2)  /* Bit 2:  Add To Queue Pending */

/* Hardware Cursor Interrupt Enable Register,
 * Hardware Cursor Interrupt Disable Register,
 * Hardware Cursor Interrupt Mask Register,
 * and Hardware Cursor Interrupt Status Register
 */

#  define LCDC_HCRINT_DMA           (1 << 2)  /* Bit 2:  End of DMA Transfer */
#  define LCDC_HCRINT_DSCR          (1 << 3)  /* Bit 3:  DMA Descriptor Loaded */
#  define LCDC_HCRINT_ADD           (1 << 4)  /* Bit 4:  Head Descriptor Loaded */
#  define LCDC_HCRINT_DONE          (1 << 5)  /* Bit 5:  End of List Detected */
#  define LCDC_HCRINT_OVR           (1 << 6)  /* Bit 6:  Overflow Detected */

/* Hardware Cursor DMA Head Register */

#  define LCDC_HCRHEAD_MASK         (0xfffffffc) /* Bits 2-31: DMA Head Pointer */

/* Hardware cursor DMA Address Register (32-bit address) */

/* Hardware Cursor DMA Control Register */

#  define LCDC_HCRCTRL_DFETCH       (1 << 0)  /* Bit 0:  Transfer Descriptor Fetch Enable */
#  define LCDC_HCRCTRL_LFETCH       (1 << 1)  /* Bit 1:  Lookup Table Fetch Enable */
#  define LCDC_HCRCTRL_DMAIEN       (1 << 2)  /* Bit 2:  End of DMA Transfer Interrupt Enable */
#  define LCDC_HCRCTRL_DSCRIEN      (1 << 3)  /* Bit 3:  Descriptor Loaded Interrupt Enable */
#  define LCDC_HCRCTRL_ADDIEN       (1 << 4)  /* Bit 4:  Add Head Descriptor to Queue Interrupt Enable */
#  define LCDC_HCRCTRL_DONEIEN      (1 << 5)  /* Bit 5:  End of List Interrupt Enable */

/* Hardware Cursor DMA Next Register (32-bit address) */

/* Hardware Cursor Configuration 0 Register */

#  define LCDC_HCRCFG0_SIF           (1 << 0)  /* Bit 0:  Source Interface */
#  define LCDC_HCRCFG0_BLEN_SHIFT    (4)       /* Bits 4-5: AHB Burst Length */
#  define LCDC_HCRCFG0_BLEN_MASK     (3 << LCDC_HCRCFG0_BLEN_SHIFT)
#    define LCDC_HCRCFG0_BLEN_SINGLE (0 << LCDC_HCRCFG0_BLEN_SHIFT)
#    define LCDC_HCRCFG0_BLEN_INCR4  (1 << LCDC_HCRCFG0_BLEN_SHIFT)
#    define LCDC_HCRCFG0_BLEN_INCR8  (2 << LCDC_HCRCFG0_BLEN_SHIFT)
#    define LCDC_HCRCFG0_BLEN_INCR16 (3 << LCDC_HCRCFG0_BLEN_SHIFT)
#  define LCDC_HCRCFG0_DLBO          (1 << 8)  /* Bit 8: Defined Length Burst Only */

/* Hardware Cursor Configuration 1 Register */

#  define LCDC_HCRCFG1_CLUTEN           (1 << 0)  /* Bit 0:  Color Lookup Table Enable */
#  define LCDC_HCRCFG1_RGBMODE_SHIFT    (4)       /* Bits 4-7: RGB Input Mode Selection */
#  define LCDC_HCRCFG1_RGBMODE_MASK     (15 << LCDC_HCRCFG1_RGBMODE_SHIFT)
#    define LCDC_HCRCFG1_12BPP_RGB444   (0 << LCDC_HCRCFG1_RGBMODE_SHIFT)  /* 12 bpp RGB 444 */
#    define LCDC_HCRCFG1_16BPP_ARGB4444 (1 << LCDC_HCRCFG1_RGBMODE_SHIFT)  /* 16 bpp ARGB 4444 */
#    define LCDC_HCRCFG1_16BPP_RGBA4444 (2 << LCDC_HCRCFG1_RGBMODE_SHIFT)  /* 16 bpp RGBA 4444 */
#    define LCDC_HCRCFG1_16BPP_RGB565   (3 << LCDC_HCRCFG1_RGBMODE_SHIFT)  /* 16 bpp RGB 565 */
#    define LCDC_HCRCFG1_16BPP_TRGB1555 (4 << LCDC_HCRCFG1_RGBMODE_SHIFT)  /* 16 bpp TRGB 1555 */
#    define LCDC_HCRCFG1_18BPP_RGB666   (5 << LCDC_HCRCFG1_RGBMODE_SHIFT)  /* 18 bpp RGB 666 */
#    define LCDC_HCRCFG1_18BPP_RGB666P  (6 << LCDC_HCRCFG1_RGBMODE_SHIFT)  /* 18 bpp RGB 666 PACKED */
#    define LCDC_HCRCFG1_19BPP_TRGB1666 (7 << LCDC_HCRCFG1_RGBMODE_SHIFT)  /* 19 bpp TRGB 1666 */
#    define LCDC_HCRCFG1_19BPP_TRGBP    (8 << LCDC_HCRCFG1_RGBMODE_SHIFT)  /* 19 bpp TRGB 1666 PACKED */
#    define LCDC_HCRCFG1_24BPP_RGB888   (9 << LCDC_HCRCFG1_RGBMODE_SHIFT)  /* 24 bpp RGB 888 */
#    define LCDC_HCRCFG1_24BPP_RGB888P  (10 << LCDC_HCRCFG1_RGBMODE_SHIFT) /* 24 bpp RGB 888 PACKED */
#    define LCDC_HCRCFG1_25BPP_TRGB1888 (11 << LCDC_HCRCFG1_RGBMODE_SHIFT) /* 25 bpp TRGB 1888 */
#    define LCDC_HCRCFG1_32BPP_ARGB8888 (12 << LCDC_HCRCFG1_RGBMODE_SHIFT) /* 32 bpp ARGB 8888 */
#    define LCDC_HCRCFG1_32BPP_RGBA8888 (13 << LCDC_HCRCFG1_RGBMODE_SHIFT) /* 32 bpp RGBA 8888 */

#  define LCDC_HCRCFG1_CLUTMODE_SHIFT   (8)       /* Bits 8-9: CLUT Input Mode Selection */
#  define LCDC_HCRCFG1_CLUTMODE_MASK    (3 << LCDC_HCRCFG1_CLUTMODE_SHIFT)
#    define LCDC_HCRCFG1_CLUTMODE_1BPP  (0 << LCDC_HCRCFG1_CLUTMODE_SHIFT) /* CLUT input 1 bit per pixel */
#    define LCDC_HCRCFG1_CLUTMODE_2BPP  (1 << LCDC_HCRCFG1_CLUTMODE_SHIFT) /* CLUT input 2 bits per pixel */
#    define LCDC_HCRCFG1_CLUTMODE_4BPP  (2 << LCDC_HCRCFG1_CLUTMODE_SHIFT) /* CLUT input 4 bits per pixel */
#    define LCDC_HCRCFG1_CLUTMODE_8BPP  (3 << LCDC_HCRCFG1_CLUTMODE_SHIFT) /* CLUT input 8 bits per pixel */

/* Hardware Cursor Configuration 2 Register */

#  define LCDC_HCRCFG2_XPOS_SHIFT   (0)     /* Bits 0-10: Horizontal Window Position */
#  define LCDC_HCRCFG2_XPOS_MASK    (0x7ff << LCDC_HCRCFG2_XPOS_SHIFT)
#    define LCDC_HCRCFG2_XPOS(n)    ((uint32_t)(n) << LCDC_HCRCFG2_XPOS_SHIFT)
#  define LCDC_HCRCFG2_YPOS_SHIFT   (16)    /* Bits 16-26: Vertical Window Position */
#  define LCDC_HCRCFG2_YPOS_MASK    (0x7ff << LCDC_HCRCFG2_YPOS_SHIFT)
#    define LCDC_HCRCFG2_YPOS(n)    ((uint32_t)(n) << LCDC_HCRCFG2_YPOS_SHIFT)

/* Hardware Cursor Configuration 3 Register */

#  define LCDC_HCRCFG3_XSIZE_SHIFT  (0)     /* Bits 0-10: Horizontal Window Size */
#  define LCDC_HCRCFG3_XSIZE_MASK   (0x7ff << LCDC_HCRCFG3_XSIZE_SHIFT)
#    define LCDC_HCRCFG3_XSIZE(n)   ((uint32_t)(n) << LCDC_HCRCFG3_XSIZE_SHIFT)
#  define LCDC_HCRCFG3_YSIZE_SHIFT  (16)    /* Bits 16-26: Vertical Window Size */
#  define LCDC_HCRCFG3_YSIZE_MASK   (0x7ff << LCDC_HCRCFG3_YSIZE_SHIFT)
#    define LCDC_HCRCFG3_YSIZE(n)   ((uint32_t)(n) << LCDC_HCRCFG3_YSIZE_SHIFT)

/* Hardware Cursor Configuration 4 Register
 * (32-bit horizontal stride value)
 */

/* Hardware Cursor Configuration 6 Register */

#  define LCDC_HCRCFG6_BDEF_SHIFT   (0)       /* Bits 0-7: B Default */
#  define LCDC_HCRCFG6_BDEF_MASK    (0xff << LCDC_HCRCFG6_BDEF_SHIFT)
#    define LCDC_HCRCFG6_BDEF(n)    ((uint32_t)(n) << LCDC_HCRCFG6_BDEF_SHIFT)
#  define LCDC_HCRCFG6_GDEF_SHIFT   (8)       /* Bits 8-15: G Default */
#  define LCDC_HCRCFG6_GDEF_MASK    (0xff << LCDC_HCRCFG6_GDEF_SHIFT)
#    define LCDC_HCRCFG6_GDEF(n)    ((uint32_t)(n) << LCDC_HCRCFG6_GDEF_SHIFT)
#  define LCDC_HCRCFG6_RDEF_SHIFT   (16)      /* Bits 16-23: R Default */
#  define LCDC_HCRCFG6_RDEF_MASK    (0xff << LCDC_HCRCFG6_RDEF_SHIFT)
#    define LCDC_HCRCFG6_RDEF(n)    ((uint32_t)(n) << LCDC_HCRCFG6_RDEF_SHIFT)

/* Hardware Cursor Configuration 7 Register */

#  define LCDC_HCRCFG7_BKEY_SHIFT   (0)       /* Bits 0-7: B Color Component Chroma Key */
#  define LCDC_HCRCFG7_BKEY_MASK    (0xff << LCDC_HCRCFG7_BKEY_SHIFT)
#    define LCDC_HCRCFG7_BKEY(n)    ((uint32_t)(n) << LCDC_HCRCFG7_BKEY_SHIFT)
#  define LCDC_HCRCFG7_GKEY_SHIFT   (8)       /* Bits 8-15: G Color Component Chroma Key */
#  define LCDC_HCRCFG7_GKEY_MASK    (0xff << LCDC_HCRCFG7_GKEY_SHIFT)
#    define LCDC_HCRCFG7_GKEY(n)    ((uint32_t)(n) << LCDC_HCRCFG7_GKEY_SHIFT)
#  define LCDC_HCRCFG7_RKEY_SHIFT   (16)      /* Bits 16-23: R Color Component Chroma Key */
#  define LCDC_HCRCFG7_RKEY_MASK    (0xff << LCDC_HCRCFG7_RKEY_SHIFT)
#    define LCDC_HCRCFG7_RKEY(n)    ((uint32_t)(n) << LCDC_HCRCFG7_RKEY_SHIFT)

/* Hardware Cursor Configuration 8 Register */

#  define LCDC_HCRCFG8_BMASK_SHIFT  (0)       /* Bits 0-7: B Color Component Chroma Key Mask */
#  define LCDC_HCRCFG8_BMASK_MASK   (0xff << LCDC_HCRCFG8_BMASK_SHIFT)
#    define LCDC_HCRCFG8_BMASK(n)   ((uint32_t)(n) << LCDC_HCRCFG8_BMASK_SHIFT)
#  define LCDC_HCRCFG8_GMASK_SHIFT  (8)       /* Bits 8-15: G Color Component Chroma Key Mask */
#  define LCDC_HCRCFG8_GMASK_MASK   (0xff << LCDC_HCRCFG8_GMASK_SHIFT)
#    define LCDC_HCRCFG8_GMASK(n)   ((uint32_t)(n) << LCDC_HCRCFG8_GMASK_SHIFT)
#  define LCDC_HCRCFG8_RMASK_SHIFT  (16)      /* Bits 16-23: R Color Component Chroma Key Mask */
#  define LCDC_HCRCFG8_RMASK_MASK   (0xff << LCDC_HCRCFG8_RMASK_SHIFT)
#    define LCDC_HCRCFG8_RMASK(n)   ((uint32_t)(n) << LCDC_HCRCFG8_RMASK_SHIFT)

/* Hardware Cursor Configuration 9 Register */

#  define LCDC_HCRCFG9_CRKEY        (1 << 0)  /* Bit 0:  Blender Chroma Key Enable */
#  define LCDC_HCRCFG9_INV          (1 << 1)  /* Bit 1:  Blender Inverted Blender Output Enable */
#  define LCDC_HCRCFG9_ITER2BL      (1 << 2)  /* Bit 2:  Blender Iterated Color Enable */
#  define LCDC_HCRCFG9_ITER         (1 << 3)  /* Bit 3:  Blender Use Iterated Color */
#  define LCDC_HCRCFG9_REVALPHA     (1 << 4)  /* Bit 4:  Blender Reverse Alpha */
#  define LCDC_HCRCFG9_GAEN         (1 << 5)  /* Bit 5:  Blender Global Alpha Enable */
#  define LCDC_HCRCFG9_LAEN         (1 << 6)  /* Bit 6:  Blender Local Alpha Enable */
#  define LCDC_HCRCFG9_OVR          (1 << 7)  /* Bit 7:  Blender Overlay Layer Enable */
#  define LCDC_HCRCFG9_DMA          (1 << 8)  /* Bit 8:  Blender DMA Layer Enable */
#  define LCDC_HCRCFG9_REP          (1 << 9)  /* Bit 9:  Use Replication logic to expand RGB color */
#  define LCDC_HCRCFG9_DSTKEY       (1 << 10) /* Bit 10: Destination Chroma Keying */
#  define LCDC_HCRCFG9_GA_SHIFT     (16)      /* Bits 16-23: Blender Global Alpha */
#  define LCDC_HCRCFG9_GA_MASK      (0xff << LCDC_HCRCFG9_GA_SHIFT)
#    define LCDC_HCRCFG9_GA(n)      ((uint32_t)(n) << LCDC_HCRCFG9_GA_SHIFT)
#endif
#if defined(ATSAMA5D3) || defined(ATSAMA5D2)
/* Post Processing Channel Enable Register */

#  define LCDC_PPCHER_CH            (1 << 0)  /* Bit 0:  Channel Enable */
#  define LCDC_PPCHER_UPDATE        (1 << 1)  /* Bit 1:  Update Overlay Attributes Enable */
#  define LCDC_PPCHER_A2Q           (1 << 2)  /* Bit 2:  Add Head Pointer Enable */

/* Post Processing Channel Disable Register */

#  define LCDC_PPCHDR_CH            (1 << 0)  /* Bit 0:  Channel Disable */
#  define LCDC_PPCHDR_CHRST         (1 << 8)  /* Bit 8:  Channel Reset */

/* Post Processing Channel Status Register */

#  define LCDC_PPCHSR_CH            (1 << 0)  /* Bit 0:  Channel Status */
#  define LCDC_PPCHSR_UPDATE        (1 << 1)  /* Bit 1:  Update Overlay Attributes In */
#  define LCDC_PPCHSR_A2Q           (1 << 2)  /* Bit 2:  Add To Queue Pending */

/* Post Processing Interrupt Enable Register,
 * Post Processing Interrupt Disable Register,
 * Post Processing Interrupt Mask Register,
 * and Post Processing Interrupt Status Register
 */

#  define LCDC_PPINT_DMA            (1 << 2)  /* Bit 2:  End of DMA Transfer */
#  define LCDC_PPINT_DSCR           (1 << 3)  /* Bit 3:  DMA Descriptor Loaded */
#  define LCDC_PPINT_ADD            (1 << 4)  /* Bit 4:  Head Descriptor Loaded */
#  define LCDC_PPINT_DONE           (1 << 5)  /* Bit 5:  End of List Detected */

/* Post Processing Head Register */

#  define LCDC_PPHEAD__MASK         (0xfffffffc) /* Bits 2-31: DMA Head Pointer */

/* Post Processing Address Register (32-bit address) */

/* Post Processing Control Register */

#  define LCDC_PPCTRL_DFETCH        (1 << 0)  /* Bit 0:  Transfer Descriptor Fetch Enable */
#  define LCDC_PPCTRL_DMAIEN        (1 << 2)  /* Bit 2:  End of DMA Transfer Interrupt Enable */
#  define LCDC_PPCTRL_DSCRIEN       (1 << 3)  /* Bit 3:  Descriptor Loaded Interrupt Enable */
#  define LCDC_PPCTRL_ADDIEN        (1 << 4)  /* Bit 4:  Add Head Descriptor to Queue Interrupt Enable */
#  define LCDC_PPCTRL_DONEIEN       (1 << 5)  /* Bit 5:  End of List Interrupt Enable */

/* Post Processing Next Register (32-bit address) */

/* Post Processing Configuration Register 0 */

#  define LCDC_PPCFG0_SIF           (1 << 0)  /* Bit 0:  Source Interface */
#  define LCDC_PPCFG0_BLEN_SHIFT    (4)       /* Bits 4-5: AHB Burst Length */
#  define LCDC_PPCFG0_BLEN_MASK     (3 << LCDC_PPCFG0_BLEN_SHIFT)
#    define LCDC_PPCFG0_BLEN_SINGLE (0 << LCDC_PPCFG0_BLEN_SHIFT)
#    define LCDC_PPCFG0_BLEN_INCR4  (1 << LCDC_PPCFG0_BLEN_SHIFT)
#    define LCDC_PPCFG0_BLEN_INCR8  (2 << LCDC_PPCFG0_BLEN_SHIFT)
#    define LCDC_PPCFG0_BLEN_INCR16 (3 << LCDC_PPCFG0_BLEN_SHIFT)
#  define LCDC_PPCFG0_DLBO          (1 << 8)  /* Bit 8: Defined Length Burst Only */

/* Post Processing Configuration Register 1 */

#  define LCDC_PPCFG1_PPMODE_SHIFT             (0)       /* Bits 0-2: Post Processing Output Format selection */
#  define LCDC_PPCFG1_PPMODE_MASK              (7 << LCDC_PPCFG1_PPMODE_SHIFT)
#    define LCDC_PPCFG1_PPMODE_RGB_16BPP       (0 << LCDC_PPCFG1_PPMODE_SHIFT) /* RGB 16 bpp */
#    define LCDC_PPCFG1_PPMODE_RGB_24BPP_P     (1 << LCDC_PPCFG1_PPMODE_SHIFT) /* RGB 24 bpp PACKED */
#    define LCDC_PPCFG1_PPMODE_RGB_24BPP_UNP   (2 << LCDC_PPCFG1_PPMODE_SHIFT) /* RGB 24 bpp UNPACKED */
#    define LCDC_PPCFG1_PPMODE_YCBCR_422_MODE0 (3 << LCDC_PPCFG1_PPMODE_SHIFT) /* YCbCr 422 16 bpp (Mode 0) */
#    define LCDC_PPCFG1_PPMODE_YCBCR_422_MODE1 (4 << LCDC_PPCFG1_PPMODE_SHIFT) /* YCbCr 422 16 bpp (Mode 1) */
#    define LCDC_PPCFG1_PPMODE_YCBCR_422_MODE2 (5 << LCDC_PPCFG1_PPMODE_SHIFT) /* YCbCr 422 16 bpp (Mode 2) */
#    define LCDC_PPCFG1_PPMODE_YCBCR_422_MODE3 (6 << LCDC_PPCFG1_PPMODE_SHIFT) /* YCbCr 422 16 bpp (Mode 3) */

#  define LCDC_PPCFG1_ITUBT601                 (1 << 4)  /* Bit 4:  Color Space Conversion U */

/* Post Processing Configuration Register 2 (32-bit horizontal stride) */

/* Post Processing Configuration Register 3 */

#  define LCDC_PPCFG3_CSCYR_SHIFT   (0)       /* Bits 0-9: Color Space Conversion R coeff for Y */
#  define LCDC_PPCFG3_CSCYR_MASK    (0x3ff << LCDC_PPCFG3_CSCYR_SHIFT)
#    define LCDC_PPCFG3_CSCYR(n)    ((uint32_t)(n) << LCDC_PPCFG3_CSCYR_SHIFT)
#  define LCDC_PPCFG3_CSCYG_SHIFT   (10)       /* Bits 10-19: Color Space Conversion G coeff for Y */
#  define LCDC_PPCFG3_CSCYG_MASK    (0x3ff << LCDC_PPCFG3_CSCYG_SHIFT)
#    define LCDC_PPCFG3_CSCYG(n)    ((uint32_t)(n) << LCDC_PPCFG3_CSCYG_SHIFT)
#  define LCDC_PPCFG3_CSCYB_SHIFT   (20)       /* Bits 20-29: Color Space Conversion B coeff for Y */
#  define LCDC_PPCFG3_CSCYB_MASK    (0x3ff << LCDC_PPCFG3_CSCYB_SHIFT)
#    define LCDC_PPCFG3_CSCYB(n)    ((uint32_t)(n) << LCDC_PPCFG3_CSCYB_SHIFT)
#  define LCDC_PPCFG3_CSCYOFF       (1 << 30)  /* Bit 30: Color Space Conversion Y Offset */

/* Post Processing Configuration Register 4 */

#  define LCDC_PPCFG4_CSCUR_SHIFT   (0)       /* Bits 0-9: Color Space Conversion R coeff for u */
#  define LCDC_PPCFG4_CSCUR_MASK    (0x3ff << LCDC_PPCFG4_CSCUR_SHIFT)
#    define LCDC_PPCFG4_CSCUR(n)    ((uint32_t)(n) << LCDC_PPCFG4_CSCUR_SHIFT)
#  define LCDC_PPCFG4_CSCUG_SHIFT   (10)       /* Bits 10-19: Color Space Conversion G coeff for u */
#  define LCDC_PPCFG4_CSCUG_MASK    (0x3ff << LCDC_PPCFG4_CSCUG_SHIFT)
#    define LCDC_PPCFG4_CSCUG(n)    ((uint32_t)(n) << LCDC_PPCFG4_CSCUG_SHIFT)
#  define LCDC_PPCFG4_CSCUB_SHIFT   (20)       /* Bits 20-29: Color Space Conversion B coeff for u */
#  define LCDC_PPCFG4_CSCUB_MASK    (0x3ff << LCDC_PPCFG4_CSCUB_SHIFT)
#    define LCDC_PPCFG4_CSCUB(n)    ((uint32_t)(n) << LCDC_PPCFG4_CSCUB_SHIFT)
#  define LCDC_PPCFG4_CSCUOFF       (1 << 30)  /* Bit 30: Color Space Conversion u Offset */

/* Post Processing Configuration Register 5 */

#  define LCDC_PPCFG5_CSCVR_SHIFT   (0)       /* Bits 0-9: Color Space Conversion R coeff for v */
#  define LCDC_PPCFG5_CSCVR_MASK    (0x3ff << LCDC_PPCFG5_CSCVR_SHIFT)
#    define LCDC_PPCFG5_CSCVR(n)    ((uint32_t)(n) << LCDC_PPCFG5_CSCVR_SHIFT)
#  define LCDC_PPCFG5_CSCVG_SHIFT   (10)       /* Bits 10-19: Color Space Conversion G coeff for v */
#  define LCDC_PPCFG5_CSCVG_MASK    (0x3ff << LCDC_PPCFG5_CSCVG_SHIFT)
#    define LCDC_PPCFG5_CSCVG(n)    ((uint32_t)(n) << LCDC_PPCFG5_CSCVG_SHIFT)
#  define LCDC_PPCFG5_CSCVB_SHIFT   (20)       /* Bits 20-29: Color Space Conversion B coeff for v */
#  define LCDC_PPCFG5_CSCVB_MASK    (0x3ff << LCDC_PPCFG5_CSCVB_SHIFT)
#    define LCDC_PPCFG5_CSCVB(n)    ((uint32_t)(n) << LCDC_PPCFG5_CSCVB_SHIFT)
#  define LCDC_PPCFG5_CSCVOFF       (1 << 30)  /* Bit 30: Color Space Conversion v Offset */
#endif

/* Base CLUT Registers 0-255 */

#define LCDC_BASECLUT_BCLUT_SHIFT   (0)       /* Bits 0-7: B color entry */
#define LCDC_BASECLUT_BCLUT_MASK    (0xff << LCDC_BASECLUT_BCLUT_SHIFT)
#  define LCDC_BASECLUT_BCLUT(n)    ((uint32_t)(n) << LCDC_BASECLUT_BCLUT_SHIFT)
#define LCDC_BASECLUT_GCLUT_SHIFT   (8)       /* Bits 8-15: G color entry */
#define LCDC_BASECLUT_GCLUT_MASK    (0xff << LCDC_BASECLUT_GCLUT_SHIFT)
#  define LCDC_BASECLUT_GCLUT(n)    ((uint32_t)(n) << LCDC_BASECLUT_GCLUT_SHIFT)
#define LCDC_BASECLUT_RCLUT_SHIFT   (16)      /* Bits 16-23: R color entry */
#define LCDC_BASECLUT_RCLUT_MASK    (0xff << LCDC_BASECLUT_RCLUT_SHIFT)
#  define LCDC_BASECLUT_RCLUT(n)    ((uint32_t)(n) << LCDC_BASECLUT_RCLUT_SHIFT)

/* Overlay 1 CLUT Registers 0-255 */

#define LCDC_OVR1CLUT_BCLUT_SHIFT   (0)       /* Bits 0-7: B color entry */
#define LCDC_OVR1CLUT_BCLUT_MASK    (0xff << LCDC_OVR1CLUT_BCLUT_SHIFT)
#  define LCDC_OVR1CLUT_BCLUT(n)    ((uint32_t)(n) << LCDC_OVR1CLUT_BCLUT_SHIFT)
#define LCDC_OVR1CLUT_GCLUT_SHIFT   (8)       /* Bits 8-15: G color entry */
#define LCDC_OVR1CLUT_GCLUT_MASK    (0xff << LCDC_OVR1CLUT_GCLUT_SHIFT)
#  define LCDC_OVR1CLUT_GCLUT(n)    ((uint32_t)(n) << LCDC_OVR1CLUT_GCLUT_SHIFT)
#define LCDC_OVR1CLUT_RCLUT_SHIFT   (16)      /* Bits 16-23: R color entry */
#define LCDC_OVR1CLUT_RCLUT_MASK    (0xff << LCDC_OVR1CLUT_RCLUT_SHIFT)
#  define LCDC_OVR1CLUT_RCLUT(n)    ((uint32_t)(n) << LCDC_OVR1CLUT_RCLUT_SHIFT)
#define LCDC_OVR1CLUT_ACLUT_SHIFT   (24)      /* Bits 24-31: Alpha color entry */
#define LCDC_OVR1CLUT_ACLUT_MASK    (0xff << LCDC_OVR1CLUT_ACLUT_SHIFT)
#  define LCDC_OVR1CLUT_ACLUT(n)    ((uint32_t)(n) << LCDC_OVR1CLUT_ACLUT_SHIFT)

/* Overlay 2 CLUT Registers 0-255 */

#define LCDC_OVR2CLUT_BCLUT_SHIFT   (0)       /* Bits 0-7: B color entry */
#define LCDC_OVR2CLUT_BCLUT_MASK    (0xff << LCDC_OVR2CLUT_BCLUT_SHIFT)
#  define LCDC_OVR2CLUT_BCLUT(n)    ((uint32_t)(n) << LCDC_OVR2CLUT_BCLUT_SHIFT)
#define LCDC_OVR2CLUT_GCLUT_SHIFT   (8)       /* Bits 8-15: G color entry */
#define LCDC_OVR2CLUT_GCLUT_MASK    (0xff << LCDC_OVR2CLUT_GCLUT_SHIFT)
#  define LCDC_OVR2CLUT_GCLUT(n)    ((uint32_t)(n) << LCDC_OVR2CLUT_GCLUT_SHIFT)
#define LCDC_OVR2CLUT_RCLUT_SHIFT   (16)      /* Bits 16-23: R color entry */
#define LCDC_OVR2CLUT_RCLUT_MASK    (0xff << LCDC_OVR2CLUT_RCLUT_SHIFT)
#  define LCDC_OVR2CLUT_RCLUT(n)    ((uint32_t)(n) << LCDC_OVR2CLUT_RCLUT_SHIFT)
#define LCDC_OVR2CLUT_ACLUT_SHIFT   (24)      /* Bits 24-31: Alpha color entry */
#define LCDC_OVR2CLUT_ACLUT_MASK    (0xff << LCDC_OVR2CLUT_ACLUT_SHIFT)
#  define LCDC_OVR2CLUT_ACLUT(n)    ((uint32_t)(n) << LCDC_OVR2CLUT_ACLUT_SHIFT)

/* High End Overlay CLUT Registers 0-255 */

#define LCDC_HEOCLUT_BCLUT_SHIFT    (0)       /* Bits 0-7: B color entry */
#define LCDC_HEOCLUT_BCLUT_MASK     (0xff << LCDC_HEOCLUT_BCLUT_SHIFT)
#  define LCDC_HEOCLUT_BCLUT(n)     ((uint32_t)(n) << LCDC_HEOCLUT_BCLUT_SHIFT)
#define LCDC_HEOCLUT_GCLUT_SHIFT    (8)       /* Bits 8-15: G color entry */
#define LCDC_HEOCLUT_GCLUT_MASK     (0xff << LCDC_HEOCLUT_GCLUT_SHIFT)
#  define LCDC_HEOCLUT_GCLUT(n)     ((uint32_t)(n) << LCDC_HEOCLUT_GCLUT_SHIFT)
#define LCDC_HEOCLUT_RCLUT_SHIFT    (16)      /* Bits 16-23: R color entry */
#define LCDC_HEOCLUT_RCLUT_MASK     (0xff << LCDC_HEOCLUT_RCLUT_SHIFT)
#  define LCDC_HEOCLUT_RCLUT(n)     ((uint32_t)(n) << LCDC_HEOCLUT_RCLUT_SHIFT)
#define LCDC_HEOCLUT_ACLUT_SHIFT    (24)      /* Bits 24-31: Alpha color entry */
#define LCDC_HEOCLUT_ACLUT_MASK     (0xff << LCDC_HEOCLUT_ACLUT_SHIFT)
#  define LCDC_HEOCLUT_ACLUT(n)     ((uint32_t)(n) << LCDC_HEOCLUT_ACLUT_SHIFT)

/* Hardware Cursor CLUT Registers 0-255 */

#ifdef ATSAMA5D3
#  define LCDC_HCRCLUT_BCLUT_SHIFT  (0)       /* Bits 0-7: B color entry */
#  define LCDC_HCRCLUT_BCLUT_MASK   (0xff << LCDC_HCRCLUT_BCLUT_SHIFT)
#    define LCDC_HCRCLUT_BCLUT(n)   ((uint32_t)(n) << LCDC_HCRCLUT_BCLUT_SHIFT)
#  define LCDC_HCRCLUT_GCLUT_SHIFT  (8)       /* Bits 8-15: G color entry */
#  define LCDC_HCRCLUT_GCLUT_MASK   (0xff << LCDC_HCRCLUT_GCLUT_SHIFT)
#    define LCDC_HCRCLUT_GCLUT(n)   ((uint32_t)(n) << LCDC_HCRCLUT_GCLUT_SHIFT)
#  define LCDC_HCRCLUT_RCLUT_SHIFT  (16)      /* Bits 16-23: R color entry */
#  define LCDC_HCRCLUT_RCLUT_MASK   (0xff << LCDC_HCRCLUT_RCLUT_SHIFT)
#    define LCDC_HCRCLUT_RCLUT(n)   ((uint32_t)(n) << LCDC_HCRCLUT_RCLUT_SHIFT)
#  define LCDC_HCRCLUT_ACLUT_SHIFT  (24)      /* Bits 24-31: Alpha color entry */
#  define LCDC_HCRCLUT_ACLUT_MASK   (0xff << LCDC_HCRCLUT_ACLUT_SHIFT)
#    define LCDC_HCRCLUT_ACLUT(n)   ((uint32_t)(n) << LCDC_HCRCLUT_ACLUT_SHIFT)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* DMA channel descriptor.
 *  This descriptor must be aligned on a 64-bit boundary.
 */

struct sam_dscr_s
{
  uint32_t addr;  /* Frame buffer base address register */
  uint32_t ctrl;  /* Transfer Control register */
  uint32_t next;  /* Next descriptor address register */
  uint32_t pad;   /* Padding to assure 64-bit aligned when used as an array */
};
#define SIZEOF_SAM_DSCR_S 16

#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_LCDC_H */
