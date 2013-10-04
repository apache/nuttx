/************************************************************************************
 * arch/arm/src/sama5/chip/sam_lcdc.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMA5_CHIP_SAM_LCDC_H
#define __ARCH_ARM_SRC_SAMA5_CHIP_SAM_LCDC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip/sam_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* LCDC Register Offsets ************************************************************/

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
                                           /* 0x003c Reserved */
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
#define SAM_LCDC_HEOVNEXT_OFFSET    0x0388 /* High-End Overlay VDMA Next Register */
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
#define SAM_LCDC_HCRCHER_OFFSET     0x0440 /* Hardware Cursor Channel Enable Register */
#define SAM_LCDC_HCRCHDR_OFFSET     0x0444 /* Hardware Cursor Channel Disable Register */
#define SAM_LCDC_HCRCHSR_OFFSET     0x0448 /* Hardware Cursor Channel Status Register */
#define SAM_LCDC_HCRIER_OFFSET      0x044c /* Hardware Cursor Interrupt Enable Register */
#define SAM_LCDC_HCRIDR_OFFSET      0x0450 /* Hardware Cursor Interrupt Disable Register */
#define SAM_LCDC_HCRIMR_OFFSET      0x0454 /* Hardware Cursor Interrupt Mask Register */
#define SAM_LCDC_HCRISR_OFFSET      0x0458 /* Hardware Cursor Interrupt Status Register */
#define SAM_LCDC_HCRHEAD_OFFSET     0x045c /* Hardware Cursor DMA Head Register */
#define SAM_LCDC_HCRADDR_OFFSET     0x0460 /* Hardware cursor DMA Address Register */
#define SAM_LCDC_HCRCTRL_OFFSET     0x0464 /* Hardware Cursor DMA Control Register */
#define SAM_LCDC_HCRNEXT_OFFSET     0x0468 /* Hardware Cursor DMA Next Register */
#define SAM_LCDC_HCRCFG0_OFFSET     0x046c /* Hardware Cursor Configuration 0 Register */
#define SAM_LCDC_HCRCFG1_OFFSET     0x0470 /* Hardware Cursor Configuration 1 Register */
#define SAM_LCDC_HCRCFG2_OFFSET     0x0474 /* Hardware Cursor Configuration 2 Register */
#define SAM_LCDC_HCRCFG3_OFFSET     0x0478 /* Hardware Cursor Configuration 3 Register */
#define SAM_LCDC_HCRCFG4_OFFSET     0x047c /* Hardware Cursor Configuration 4 Register */
                                           /* 0x0480 Reserved */
#define SAM_LCDC_HCRCFG6_OFFSET     0x0484 /* Hardware Cursor Configuration 6 Register */
#define SAM_LCDC_HCRCFG7_OFFSET     0x0488 /* Hardware Cursor Configuration 7 Register */
#define SAM_LCDC_HCRCFG8_OFFSET     0x048c /* Hardware Cursor Configuration 8 Register */
#define SAM_LCDC_HCRCFG9_OFFSET     0x0490 /* Hardware Cursor Configuration 9 Register */
                                           /* 0x0494-0x053c Reserved */
#define SAM_LCDC_PPCHER_OFFSET      0x0540 /* Post Processing Channel Enable Register */
#define SAM_LCDC_PPCHDR_OFFSET      0x0544 /* Post Processing Channel Disable Register */
#define SAM_LCDC_PPCHSR_OFFSET      0x0548 /* Post Processing Channel Status Register */
#define SAM_LCDC_PPIER_OFFSET       0x054c /* Post Processing Interrupt Enable Register */
#define SAM_LCDC_PPIDR_OFFSET       0x0550 /* Post Processing Interrupt Disable Register */
#define SAM_LCDC_PPIMR_OFFSET       0x0554 /* Post Processing Interrupt Mask Register */
#define SAM_LCDC_PPISR_OFFSET       0x0558 /* Post Processing Interrupt Status Register */
#define SAM_LCDC_PPHEAD_OFFSET      0x055c /* Post Processing Head Register */
#define SAM_LCDC_PPADDR_OFFSET      0x0560 /* Post Processing Address Register */
#define SAM_LCDC_PPCTRL_OFFSET      0x0564 /* Post Processing Control Register */
#define SAM_LCDC_PPNEXT_OFFSET      0x0568 /* Post Processing Next Register */
#define SAM_LCDC_PPCFG0_OFFSET      0x056c /* Post Processing Configuration Register 0 */
#define SAM_LCDC_PPCFG1_OFFSET      0x0570 /* Post Processing Configuration Register 1 */
#define SAM_LCDC_PPCFG2_OFFSET      0x0574 /* Post Processing Configuration Register 2 */
#define SAM_LCDC_PPCFG3_OFFSET      0x0578 /* Post Processing Configuration Register 3 */
#define SAM_LCDC_PPCFG4_OFFSET      0x057c /* Post Processing Configuration Register 4 */
#define SAM_LCDC_PPCFG5_OFFSET      0x0580 /* Post Processing Configuration Register 5 */
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

#define SAM_LCDC_HCRCLUT_OFFSET(n)  (0x1600 + ((n) << 2))
                                           /* 0x1a00-0x1fe4 Reserved */

/* LCDC Register Addresses *********************************************************/

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

#define SAM_LCDC_HCRCHER            (SAM_LCDC_VBASE+SAM_LCDC_HCRCHER_OFFSET)
#define SAM_LCDC_HCRCHDR            (SAM_LCDC_VBASE+SAM_LCDC_HCRCHDR_OFFSET)
#define SAM_LCDC_HCRCHSR            (SAM_LCDC_VBASE+SAM_LCDC_HCRCHSR_OFFSET)
#define SAM_LCDC_HCRIER             (SAM_LCDC_VBASE+SAM_LCDC_HCRIER_OFFSET)
#define SAM_LCDC_HCRIDR             (SAM_LCDC_VBASE+SAM_LCDC_HCRIDR_OFFSET)
#define SAM_LCDC_HCRIMR             (SAM_LCDC_VBASE+SAM_LCDC_HCRIMR_OFFSET)
#define SAM_LCDC_HCRISR             (SAM_LCDC_VBASE+SAM_LCDC_HCRISR_OFFSET)
#define SAM_LCDC_HCRHEAD            (SAM_LCDC_VBASE+SAM_LCDC_HCRHEAD_OFFSET)
#define SAM_LCDC_HCRADDR            (SAM_LCDC_VBASE+SAM_LCDC_HCRADDR_OFFSET)
#define SAM_LCDC_HCRCTRL            (SAM_LCDC_VBASE+SAM_LCDC_HCRCTRL_OFFSET)
#define SAM_LCDC_HCRNEXT            (SAM_LCDC_VBASE+SAM_LCDC_HCRNEXT_OFFSET)
#define SAM_LCDC_HCRCFG0            (SAM_LCDC_VBASE+SAM_LCDC_HCRCFG0_OFFSET)
#define SAM_LCDC_HCRCFG1            (SAM_LCDC_VBASE+SAM_LCDC_HCRCFG1_OFFSET)
#define SAM_LCDC_HCRCFG2            (SAM_LCDC_VBASE+SAM_LCDC_HCRCFG2_OFFSET)
#define SAM_LCDC_HCRCFG3            (SAM_LCDC_VBASE+SAM_LCDC_HCRCFG3_OFFSET)
#define SAM_LCDC_HCRCFG4            (SAM_LCDC_VBASE+SAM_LCDC_HCRCFG4_OFFSET)
#define SAM_LCDC_HCRCFG6            (SAM_LCDC_VBASE+SAM_LCDC_HCRCFG6_OFFSET)
#define SAM_LCDC_HCRCFG7            (SAM_LCDC_VBASE+SAM_LCDC_HCRCFG7_OFFSET)
#define SAM_LCDC_HCRCFG8            (SAM_LCDC_VBASE+SAM_LCDC_HCRCFG8_OFFSET)
#define SAM_LCDC_HCRCFG9            (SAM_LCDC_VBASE+SAM_LCDC_HCRCFG9_OFFSET)

#define SAM_LCDC_PPCHER             (SAM_LCDC_VBASE+SAM_LCDC_PPCHER_OFFSET)
#define SAM_LCDC_PPCHDR             (SAM_LCDC_VBASE+SAM_LCDC_PPCHDR_OFFSET)
#define SAM_LCDC_PPCHSR             (SAM_LCDC_VBASE+SAM_LCDC_PPCHSR_OFFSET)
#define SAM_LCDC_PPIER              (SAM_LCDC_VBASE+SAM_LCDC_PPIER_OFFSET)
#define SAM_LCDC_PPIDR              (SAM_LCDC_VBASE+SAM_LCDC_PPIDR_OFFSET)
#define SAM_LCDC_PPIMR              (SAM_LCDC_VBASE+SAM_LCDC_PPIMR_OFFSET)
#define SAM_LCDC_PPISR              (SAM_LCDC_VBASE+SAM_LCDC_PPISR_OFFSET)
#define SAM_LCDC_PPHEAD             (SAM_LCDC_VBASE+SAM_LCDC_PPHEAD_OFFSET)
#define SAM_LCDC_PPADDR             (SAM_LCDC_VBASE+SAM_LCDC_PPADDR_OFFSET)
#define SAM_LCDC_PPCTRL             (SAM_LCDC_VBASE+SAM_LCDC_PPCTRL_OFFSET)
#define SAM_LCDC_PPNEXT             (SAM_LCDC_VBASE+SAM_LCDC_PPNEXT_OFFSET)
#define SAM_LCDC_PPCFG0             (SAM_LCDC_VBASE+SAM_LCDC_PPCFG0_OFFSET)
#define SAM_LCDC_PPCFG1             (SAM_LCDC_VBASE+SAM_LCDC_PPCFG1_OFFSET)
#define SAM_LCDC_PPCFG2             (SAM_LCDC_VBASE+SAM_LCDC_PPCFG2_OFFSET)
#define SAM_LCDC_PPCFG3             (SAM_LCDC_VBASE+SAM_LCDC_PPCFG3_OFFSET)
#define SAM_LCDC_PPCFG4             (SAM_LCDC_VBASE+SAM_LCDC_PPCFG4_OFFSET)
#define SAM_LCDC_PPCFG5             (SAM_LCDC_VBASE+SAM_LCDC_PPCFG5_OFFSET)

/* 0x0600-0x08fc Base CLUT Registers 0-255 */

#define SAM_LCDC_BASECLUT(n)        (SAM_LCDC_VBASE+SAM_LCDC_BASECLUT_OFFSET(n))

/* 0x0a00-0x0dfc Overlay 1 CLUT Registers 0-255 */

#define SAM_LCDC_OVR1CLUT(n)        (SAM_LCDC_VBASE+SAM_LCDC_OVR1CLUT_OFFSET(n))

/* 0x0e00-0x11fc Overlay 2 CLUT Registers 0-255 */

#define SAM_LCDC_OVR2CLUT(n)        (SAM_LCDC_VBASE+SAM_LCDC_OVR2CLUT_OFFSET(n))

/* 0x1200-0x15fc High End Overlay CLUT Registers 0-255 */

#define SAM_LCDC_HEOCLUT(n)         (SAM_LCDC_VBASE+SAM_LCDC_HEOCLUT_OFFSET(n))

/* 0x1600-0x19fc Hardware Cursor CLUT Registers 0-255 */

#define SAM_LCDC_HCRCLUT(n)         (SAM_LCDC_VBASE+SAM_LCDC_HCRCLUT_OFFSET(n))

/* LCDC Register Bit Definitions ***************************************************/

/* LCD Controller Configuration Register 0 */
#define LCDC_LCDCFG0_
/* LCD Controller Configuration Register 1 */
#define LCDC_LCDCFG1_
/* LCD Controller Configuration Register 2 */
#define LCDC_LCDCFG2_
/* LCD Controller Configuration Register 3 */
#define LCDC_LCDCFG3_
/* LCD Controller Configuration Register 4 */
#define LCDC_LCDCFG4_
/* LCD Controller Configuration Register 5 */
#define LCDC_LCDCFG5_
/* LCD Controller Configuration Register 6 */
#define LCDC_LCDCFG6_

/* LCD Controller Enable Register */
#define LCDC_LCDEN_
/* LCD Controller Disable Register */
#define LCDC_LCDDIS_
/* LCD Controller Status Register */
#define LCDC_LCDSR_
/* LCD Controller Interrupt Enable Register */
#define LCDC_LCDIER_
/* LCD Controller Interrupt Disable Register */
#define LCDC_LCDIDR_
/* LCD Controller Interrupt Mask Register */
#define LCDC_LCDIMR_
/* LCD Controller Interrupt Status Register */
#define LCDC_LCDISR_

/* Base Layer Channel Enable Register */
#define LCDC_BASECHER_
/* Base Layer Channel Disable Register */
#define LCDC_BASECHDR_
/* Base Layer Channel Status Register */
#define LCDC_BASECHSR_
/* Base Layer Interrupt Enable Register */
#define LCDC_BASEIER_
/* Base Layer Interrupt Disable Register */
#define LCDC_BASEIDR_
/* Base Layer Interrupt Mask Register */
#define LCDC_BASEIMR_
/* Base Layer Interrupt Status Register */
#define LCDC_BASEISR_
/* Base DMA Head Register */
#define LCDC_BASEHEAD_
/* Base DMA Address Register */
#define LCDC_BASEADDR_
/* Base DMA Control Register */
#define LCDC_BASECTRL_
/* Base DMA Next Register */
#define LCDC_BASENEXT_

/* Base Configuration register 0 */
#define LCDC_BASECFG0_
/* Base Configuration register 1 */
#define LCDC_BASECFG1_
/* Base Configuration register 2 */
#define LCDC_BASECFG2_
/* Base Configuration register 3 */
#define LCDC_BASECFG3_
/* Base Configuration register 4 */
#define LCDC_BASECFG4_
/* Base Configuration register 5 */
#define LCDC_BASECFG5_
/* Base Configuration register 6 */
#define LCDC_BASECFG6_

/* Overlay 1 Channel Enable Register */
#define LCDC_OVR1CHER_
/* Overlay 1 Channel Disable Register */
#define LCDC_OVR1CHDR_
/* Overlay 1 Channel Status Register */
#define LCDC_OVR1CHSR_
/* Overlay 1 Interrupt Enable Register */
#define LCDC_OVR1IER_
/* Overlay 1 Interrupt Disable Register */
#define LCDC_OVR1IDR_
/* Overlay 1 Interrupt Mask Register */
#define LCDC_OVR1IMR_
/* Overlay 1 Interrupt Status Register */
#define LCDC_OVR1ISR_
/* Overlay 1 DMA Head Register */
#define LCDC_OVR1HEAD_
/* Overlay 1 DMA Address Register */
#define LCDC_OVR1ADDR_
/* Overlay 1 DMA Control Register */
#define LCDC_OVR1CTRL_
/* Overlay 1 DMA Next Register */
#define LCDC_OVR1NEXT_

/* Overlay 1 Configuration 0 Register */
#define LCDC_OVR1CFG0_
/* Overlay 1 Configuration 1 Register */
#define LCDC_OVR1CFG1_
/* Overlay 1 Configuration 2 Register */
#define LCDC_OVR1CFG2_
/* Overlay 1 Configuration 3 Register */
#define LCDC_OVR1CFG3_
/* Overlay 1 Configuration 4 Register */
#define LCDC_OVR1CFG4_
/* Overlay 1 Configuration 5 Register */
#define LCDC_OVR1CFG5_
/* Overlay 1 Configuration 6 Register */
#define LCDC_OVR1CFG6_
/* Overlay 1 Configuration 7 Register */
#define LCDC_OVR1CFG7_
/* Overlay 1 Configuration 8 Register */
#define LCDC_OVR1CFG8_
/* Overlay 1 Configuration 9 Register */
#define LCDC_OVR1CFG9_

/* Overlay 2 Channel Enable Register */
#define LCDC_OVR2CHER_
/* Overlay 2 Channel Disable Register */
#define LCDC_OVR2CHDR_
/* Overlay 2 Channel Status Register */
#define LCDC_OVR2CHSR_
/* Overlay 2 Interrupt Enable Register */
#define LCDC_OVR2IER_
/* Overlay 2 Interrupt Disable Register */
#define LCDC_OVR2IDR_
/* Overlay 2 Interrupt Mask Register */
#define LCDC_OVR2IMR_
/* Overlay 2 Interrupt Status Register */
#define LCDC_OVR2ISR_
/* Overlay 2 DMA Head Register */
#define LCDC_OVR2HEAD_
/* Overlay 2 DMA Address Register */
#define LCDC_OVR2ADDR_
/* Overlay 2 DMA Control Register */
#define LCDC_OVR2CTRL_
/* Overlay 2 DMA Next Register */
#define LCDC_OVR2NEXT_

/* Overlay 2 Configuration 0 Register */
#define LCDC_OVR2CFG0_
/* Overlay 2 Configuration 1 Register */
#define LCDC_OVR2CFG1_
/* Overlay 2 Configuration 2 Register */
#define LCDC_OVR2CFG2_
/* Overlay 2 Configuration 3 Register */
#define LCDC_OVR2CFG3_
/* Overlay 2 Configuration 4 Register */
#define LCDC_OVR2CFG4_
/* Overlay 2 Configuration 5 Register */
#define LCDC_OVR2CFG5_
/* Overlay 2 Configuration 6 Register */
#define LCDC_OVR2CFG6_
/* Overlay 2 Configuration 7 Register */
#define LCDC_OVR2CFG7_
/* Overlay 2 Configuration 8 Register */
#define LCDC_OVR2CFG8_
/* Overlay 2 Configuration 9 Register */
#define LCDC_OVR2CFG9_

/* High-End Overlay Channel Enable Register */
#define LCDC_HEOCHER_
/* High-End Overlay Channel Disable Register */
#define LCDC_HEOCHDR_
/* High-End Overlay Channel Status Register */
#define LCDC_HEOCHSR_
/* High-End Overlay Interrupt Enable Register */
#define LCDC_HEOIER_
/* High-End Overlay Interrupt Disable Register */
#define LCDC_HEOIDR_
/* High-End Overlay Interrupt Mask Register */
#define LCDC_HEOIMR_
/* High-End Overlay Interrupt Status Register */
#define LCDC_HEOISR_
/* High-End Overlay DMA Head Register */
#define LCDC_HEOHEAD_
/* High-End Overlay DMA Address Register */
#define LCDC_HEOADDR_
/* High-End Overlay DMA Control Register */
#define LCDC_HEOCTRL_
/* High-End Overlay DMA Next Register */
#define LCDC_HEONEXT_
/* High-End Overlay U DMA Head Register */
#define LCDC_HEOUHEAD_
/* High-End Overlay U DMA Address Register */
#define LCDC_HEOUADDR_
/* High-End Overlay U DMA Control Register */
#define LCDC_HEOUCTRL_
/* High-End Overlay U DMA Next Register */
#define LCDC_HEOUNEXT_
/* High-End Overlay V DMA Head Register */
#define LCDC_HEOVHEAD_
/* High-End Overlay V DMA Address Register */
#define LCDC_HEOVADDR_
/* High-End Overlay V DMA Control Register */
#define LCDC_HEOVCTRL_
/* High-End Overlay VDMA Next Register */
#define LCDC_HEOVNEXT_

/* High-End Overlay Configuration Register 0 */
#define LCDC_HEOCFG0_
/* High-End Overlay Configuration Register 1 */
#define LCDC_HEOCFG1_
/* High-End Overlay Configuration Register 2 */
#define LCDC_HEOCFG2_
/* High-End Overlay Configuration Register 3 */
#define LCDC_HEOCFG3_
/* High-End Overlay Configuration Register 4 */
#define LCDC_HEOCFG4_
/* High-End Overlay Configuration Register 5 */
#define LCDC_HEOCFG5_
/* High-End Overlay Configuration Register 6 */
#define LCDC_HEOCFG6_
/* High-End Overlay Configuration Register 7 */
#define LCDC_HEOCFG7_
/* High-End Overlay Configuration Register 8 */
#define LCDC_HEOCFG8_
/* High-End Overlay Configuration Register 9 */
#define LCDC_HEOCFG9_
/* High-End Overlay Configuration Register 10 */
#define LCDC_HEOCFG10_
/* High-End Overlay Configuration Register 11 */
#define LCDC_HEOCFG11_
/* High-End Overlay Configuration Register 12 */
#define LCDC_HEOCFG12_
/* High-End Overlay Configuration Register 13 */
#define LCDC_HEOCFG13_
/* High-End Overlay Configuration Register 14 */
#define LCDC_HEOCFG14_
/* High-End Overlay Configuration Register 15 */
#define LCDC_HEOCFG15_
/* High-End Overlay Configuration Register 16 */
#define LCDC_HEOCFG16_
/* High-End Overlay Configuration Register 17 */
#define LCDC_HEOCFG17_
/* High-End Overlay Configuration Register 18 */
#define LCDC_HEOCFG18_
/* High-End Overlay Configuration Register 19 */
#define LCDC_HEOCFG19_
/* High-End Overlay Configuration Register 20 */
#define LCDC_HEOCFG20_
/* High-End Overlay Configuration Register 21 */
#define LCDC_HEOCFG21_
/* High-End Overlay Configuration Register 22 */
#define LCDC_HEOCFG22_
/* High-End Overlay Configuration Register 23 */
#define LCDC_HEOCFG23_
/* High-End Overlay Configuration Register 24 */
#define LCDC_HEOCFG24_
/* High-End Overlay Configuration Register 25 */
#define LCDC_HEOCFG25_
/* High-End Overlay Configuration Register 26 */
#define LCDC_HEOCFG26_
/* High-End Overlay Configuration Register 27 */
#define LCDC_HEOCFG27_
/* High-End Overlay Configuration Register 28 */
#define LCDC_HEOCFG28_
/* High-End Overlay Configuration Register 29 */
#define LCDC_HEOCFG29_
/* High-End Overlay Configuration Register 30 */
#define LCDC_HEOCFG30_
/* High-End Overlay Configuration Register 31 */
#define LCDC_HEOCFG31_
/* High-End Overlay Configuration Register 32 */
#define LCDC_HEOCFG32_
/* High-End Overlay Configuration Register 33 */
#define LCDC_HEOCFG33_
/* High-End Overlay Configuration Register 34 */
#define LCDC_HEOCFG34_
/* High-End Overlay Configuration Register 35 */
#define LCDC_HEOCFG35_
/* High-End Overlay Configuration Register 36 */
#define LCDC_HEOCFG36_
/* High-End Overlay Configuration Register 37 */
#define LCDC_HEOCFG37_
/* High-End Overlay Configuration Register 38 */
#define LCDC_HEOCFG38_
/* High-End Overlay Configuration Register 39 */
#define LCDC_HEOCFG39_
/* High-End Overlay Configuration Register 40 */
#define LCDC_HEOCFG40_
/* High-End Overlay Configuration Register 41 */
#define LCDC_HEOCFG41_

/* Hardware Cursor Channel Enable Register */
#define LCDC_HCRCHER_OFFSET     0x0440 /* Hardware Cursor Channel Enable Register */
/* Hardware Cursor Channel Disable Register */
#define LCDC_HCRCHDR_OFFSET     0x0444 /* Hardware Cursor Channel Disable Register */
/* Hardware Cursor Channel Status Register */
#define LCDC_HCRCHSR_OFFSET     0x0448 /* Hardware Cursor Channel Status Register */
/* Hardware Cursor Interrupt Enable Register */
#define LCDC_HCRIER_OFFSET      0x044c /* Hardware Cursor Interrupt Enable Register */
/* Hardware Cursor Interrupt Disable Register */
#define LCDC_HCRIDR_OFFSET      0x0450 /* Hardware Cursor Interrupt Disable Register */
/* Hardware Cursor Interrupt Mask Register */
#define LCDC_HCRIMR_OFFSET      0x0454 /* Hardware Cursor Interrupt Mask Register */
/* Hardware Cursor Interrupt Status Register */
#define LCDC_HCRISR_OFFSET      0x0458 /* Hardware Cursor Interrupt Status Register */
/* Hardware Cursor DMA Head Register */
#define LCDC_HCRHEAD_OFFSET     0x045c /* Hardware Cursor DMA Head Register */
/* Hardware cursor DMA Address Register */
#define LCDC_HCRADDR_OFFSET     0x0460 /* Hardware cursor DMA Address Register */
/* Hardware Cursor DMA Control Register */
#define LCDC_HCRCTRL_OFFSET     0x0464 /* Hardware Cursor DMA Control Register */
/* Hardware Cursor DMA Next Register */
#define LCDC_HCRNEXT_OFFSET     0x0468 /* Hardware Cursor DMA Next Register */

/* Hardware Cursor Configuration 0 Register */
#define LCDC_HCRCFG0_
/* Hardware Cursor Configuration 1 Register */
#define LCDC_HCRCFG1_
/* Hardware Cursor Configuration 2 Register */
#define LCDC_HCRCFG2_
/* Hardware Cursor Configuration 3 Register */
#define LCDC_HCRCFG3_
/* Hardware Cursor Configuration 4 Register */
#define LCDC_HCRCFG4_
/* Hardware Cursor Configuration 6 Register */
#define LCDC_HCRCFG6_
/* Hardware Cursor Configuration 7 Register */
#define LCDC_HCRCFG7_
/* Hardware Cursor Configuration 8 Register */
#define LCDC_HCRCFG8_
/* Hardware Cursor Configuration 9 Register */
#define LCDC_HCRCFG9_

/* Post Processing Channel Enable Register */
#define LCDC_PPCHER_
/* Post Processing Channel Disable Register */
#define LCDC_PPCHDR_
/* Post Processing Channel Status Register */
#define LCDC_PPCHSR_
/* Post Processing Interrupt Enable Register */
#define LCDC_PPIER_
/* Post Processing Interrupt Disable Register */
#define LCDC_PPIDR_
/* Post Processing Interrupt Mask Register */
#define LCDC_PPIMR_
/* Post Processing Interrupt Status Register */
#define LCDC_PPISR_
/* Post Processing Head Register */
#define LCDC_PPHEAD_
/* Post Processing Address Register */
#define LCDC_PPADDR_
/* Post Processing Control Register */
#define LCDC_PPCTRL_
/* Post Processing Next Register */
#define LCDC_PPNEXT_

/* Post Processing Configuration Register 0 */
#define LCDC_PPCFG0_
/* Post Processing Configuration Register 1 */
#define LCDC_PPCFG1_
/* Post Processing Configuration Register 2 */
#define LCDC_PPCFG2_
/* Post Processing Configuration Register 3 */
#define LCDC_PPCFG3_
/* Post Processing Configuration Register 4 */
#define LCDC_PPCFG4_
/* Post Processing Configuration Register 5 */
#define LCDC_PPCFG5_

/* Base CLUT Registers 0-255 */
#define LCDC_BASECLUT_
/* Overlay 1 CLUT Registers 0-255 */
#define LCDC_OVR1CLUT_
/* Overlay 2 CLUT Registers 0-255 */
#define LCDC_OVR2CLUT_
/* High End Overlay CLUT Registers 0-255 */
#define LCDC_HEOCLUT_
/* Hardware Cursor CLUT Registers 0-255 */
#define LCDC_HCRCLUT_


        (1 << nn)  /* Bit nn:
_MASK   (nn)       /* Bits nn-nn:
_SHIFT  (xx << yy)

#endif /* __ARCH_ARM_SRC_SAMA5_CHIP_SAM_LCDC_H */
