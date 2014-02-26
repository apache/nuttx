/****************************************************************************************
 * arch/arm/src/sam34/chip/sam4e_pio.h
 * Parallel Input/Output (PIO) Controller definitions for the SAM4E
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAM34_CHIP_SAM4E_PIO_H
#define __ARCH_ARM_SRC_SAM34_CHIP_SAM4E_PIO_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/sam_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* PIO register offsets *****************************************************************/

#define SAM_PIO_PER_OFFSET         0x0000 /* PIO Enable Register */
#define SAM_PIO_PDR_OFFSET         0x0004 /* PIO Disable Register */
#define SAM_PIO_PSR_OFFSET         0x0008 /* PIO Status Register */
                                          /* 0x000c: Reserved */
#define SAM_PIO_OER_OFFSET         0x0010 /* Output Enable Register */
#define SAM_PIO_ODR_OFFSET         0x0014 /* Output Disable Register */
#define SAM_PIO_OSR_OFFSET         0x0018 /* Output Status Register */
                                          /* 0x001c: Reserved */
#define SAM_PIO_IFER_OFFSET        0x0020 /* Glitch Input Filter Enable Register */
#define SAM_PIO_IFDR_OFFSET        0x0024 /* Glitch Input Filter Disable Register */
#define SAM_PIO_IFSR_OFFSET        0x0028 /* Glitch Input Filter Status Register */
                                          /* 0x002c: Reserved */
#define SAM_PIO_SODR_OFFSET        0x0030 /* Set Output Data Register */
#define SAM_PIO_CODR_OFFSET        0x0034 /* Clear Output Data Register */
#define SAM_PIO_ODSR_OFFSET        0x0038 /* Output Data Status Register */
#define SAM_PIO_PDSR_OFFSET        0x003c /* Pin Data Status Register */
#define SAM_PIO_IER_OFFSET         0x0040 /* Interrupt Enable Register */
#define SAM_PIO_IDR_OFFSET         0x0044 /* Interrupt Disable Register */
#define SAM_PIO_IMR_OFFSET         0x0048 /* Interrupt Mask Register */
#define SAM_PIO_ISR_OFFSET         0x004c /* Interrupt Status Register */
#define SAM_PIO_MDER_OFFSET        0x0050 /* Multi-driver Enable Register */
#define SAM_PIO_MDDR_OFFSET        0x0054 /* Multi-driver Disable Register */
#define SAM_PIO_MDSR_OFFSET        0x0058 /* Multi-driver Status Register */
                                          /* 0x005c: Reserved */
#define SAM_PIO_PUDR_OFFSET        0x0060 /* Pull-up Disable Register */
#define SAM_PIO_PUER_OFFSET        0x0064 /* Pull-up Enable Register */
#define SAM_PIO_PUSR_OFFSET        0x0068 /* Pad Pull-up Status Register */
                                          /* 0x006c: Reserved */
#define SAM_PIO_ABCDSR1_OFFSET     0x0070 /* Peripheral Select Register 1 */
#define SAM_PIO_ABCDSR2_OFFSET     0x0074 /* Peripheral Select Register 2 */
                                          /* 0x0078-0x007c: Reserved */
#define SAM_PIO_IFSCDR_OFFSET      0x0080 /* Input Filter Slow Clock Disable Register */
#define SAM_PIO_IFSCER_OFFSET      0x0084 /* Input Filter Slow Clock Enable Register */
#define SAM_PIO_IFSCSR_OFFSET      0x0088 /* Input Filter Slow Clock Status Register */
#define SAM_PIO_SCDR_OFFSET        0x008c /* Slow Clock Divider Debouncing Register */
#define SAM_PIO_PPDDR_OFFSET       0x0090 /* Pad Pull Down Disable Register */
#define SAM_PIO_PPDER_OFFSET       0x0094 /* PIO Pad Pull Down Enable Register */
#define SAM_PIO_PPDSR_OFFSET       0x0098 /* PIO Pad Pull Down Status Register */
                                          /* 0x009c: Reserved */
#define SAM_PIO_OWER_OFFSET        0x00a0 /* Output Write Enable */
#define SAM_PIO_OWDR_OFFSET        0x00a4 /* Output Write Disable */
#define SAM_PIO_OWSR_OFFSET        0x00a8 /* Output Write Status Register */
                                          /* 0x00ac: Reserved */
#define SAM_PIO_AIMER_OFFSET       0x00b0 /* Additional Interrupt Modes Enable Register */
#define SAM_PIO_AIMDR_OFFSET       0x00b4 /* Additional Interrupt Modes Disables Register */
#define SAM_PIO_AIMMR_OFFSET       0x00b8 /* Additional Interrupt Modes Mask Register */
                                          /* 0x00bc: Reserved */
#define SAM_PIO_ESR_OFFSET         0x00c0 /* Edge Select Register */
#define SAM_PIO_LSR_OFFSET         0x00c4 /* Level Select Register */
#define SAM_PIO_ELSR_OFFSET        0x00c8 /* Edge/Level Status Register */
                                          /* 0x00cc: Reserved */
#define SAM_PIO_FELLSR_OFFSET      0x00d0 /* Falling Edge/Low Level Select Register */
#define SAM_PIO_REHLSR_OFFSET      0x00d4 /* Rising Edge/ High Level Select Register */
#define SAM_PIO_FRLHSR_OFFSET      0x00d8 /* Fall/Rise - Low/High Status Register */
                                          /* 0x00dc: Reserved */
#define SAM_PIO_LOCKSR_OFFSET      0x00e0 /* Lock Status */
#define SAM_PIO_WPMR_OFFSET        0x00e4 /* Write Protect Mode Register */
#define SAM_PIO_WPSR_OFFSET        0x00e8 /* Write Protect Status Register */
                                          /* 0x00ec-0x00f8: Reserved */
#define SAM_PIO_SCHMITT_OFFSET     0x0100 /* Schmitt Trigger Register */
                                          /* 0x0104-0x10c: Reserved */
#define SAM_PIO_DELAYR_OFFSET      0x0100 /* IO Delay Register */
                                          /* 0x0114-0x14c: Reserved */
#define SAM_PIO_PCMR_OFFSET        0x0150 /* Parallel Capture Mode Register */
#define SAM_PIO_PCIER_OFFSET       0x0154 /* Parallel Capture Interrupt Enable Register */
#define SAM_PIO_PCIDR_OFFSET       0x0158 /* Parallel Capture Interrupt Disable Register */
#define SAM_PIO_PCIMR_OFFSET       0x015c /* Parallel Capture Interrupt Mask Register */
#define SAM_PIO_PCISR_OFFSET       0x0160 /* Parallel Capture Interrupt Status Register */
#define SAM_PIO_PCRHR_OFFSET       0x0164 /* Parallel Capture Reception Holding Register */
                                          /* 0x0168-0x018c: Reserved for PDC registers */

/* PIO register addresses ***************************************************************/

#define PIOA                       (0)
#define PIOB                       (1)
#define PIOC                       (2)
#define NPIO                       (3)

#define SAM_PIO_PER(n)             (SAM_PIO_BASE(n)+SAM_PIO_PER_OFFSET)
#define SAM_PIO_PDR(n)             (SAM_PIO_BASE(n)+SAM_PIO_PDR_OFFSET)
#define SAM_PIO_PSR(n)             (SAM_PIO_BASE(n)+SAM_PIO_PSR_OFFSET)
#define SAM_PIO_OER(n)             (SAM_PIO_BASE(n)+SAM_PIO_OER_OFFSET)
#define SAM_PIO_ODR(n)             (SAM_PIO_BASE(n)+SAM_PIO_ODR_OFFSET)
#define SAM_PIO_OSR(n)             (SAM_PIO_BASE(n)+SAM_PIO_OSR_OFFSET)
#define SAM_PIO_IFER(n)            (SAM_PIO_BASE(n)+SAM_PIO_IFER_OFFSET)
#define SAM_PIO_IFDR(n)            (SAM_PIO_BASE(n)+SAM_PIO_IFDR_OFFSET)
#define SAM_PIO_IFSR(n)            (SAM_PIO_BASE(n)+SAM_PIO_IFSR_OFFSET)
#define SAM_PIO_SODR(n)            (SAM_PIO_BASE(n)+SAM_PIO_SODR_OFFSET)
#define SAM_PIO_CODR(n)            (SAM_PIO_BASE(n)+SAM_PIO_CODR_OFFSET)
#define SAM_PIO_ODSR(n)            (SAM_PIO_BASE(n)+SAM_PIO_ODSR_OFFSET)
#define SAM_PIO_PDSR(n)            (SAM_PIO_BASE(n)+SAM_PIO_PDSR_OFFSET)
#define SAM_PIO_IER(n)             (SAM_PIO_BASE(n)+SAM_PIO_IER_OFFSET)
#define SAM_PIO_IDR(n)             (SAM_PIO_BASE(n)+SAM_PIO_IDR_OFFSET)
#define SAM_PIO_IMR(n)             (SAM_PIO_BASE(n)+SAM_PIO_IMR_OFFSET)
#define SAM_PIO_ISR(n)             (SAM_PIO_BASE(n)+SAM_PIO_ISR_OFFSET)
#define SAM_PIO_MDER(n)            (SAM_PIO_BASE(n)+SAM_PIO_MDER_OFFSET)
#define SAM_PIO_MDDR(n)            (SAM_PIO_BASE(n)+SAM_PIO_MDDR_OFFSET)
#define SAM_PIO_MDSR(n)            (SAM_PIO_BASE(n)+SAM_PIO_MDSR_OFFSET)
#define SAM_PIO_PUDR(n)            (SAM_PIO_BASE(n)+SAM_PIO_PUDR_OFFSET)
#define SAM_PIO_PUER(n)            (SAM_PIO_BASE(n)+SAM_PIO_PUER_OFFSET)
#define SAM_PIO_PUSR(n)            (SAM_PIO_BASE(n)+SAM_PIO_PUSR_OFFSET)
#define SAM_PIO_ABCDSR1(n)         (SAM_PIO_BASE(n)+SAM_PIO_ABCDSR1_OFFSET)
#define SAM_PIO_ABCDSR2(n)         (SAM_PIO_BASE(n)+SAM_PIO_ABCDSR2_OFFSET)
#define SAM_PIO_IFSCDR(n)          (SAM_PIO_BASE(n)+SAM_PIO_IFSCDR_OFFSET)
#define SAM_PIO_IFSCER(n)          (SAM_PIO_BASE(n)+SAM_PIO_IFSCER_OFFSET)
#define SAM_PIO_IFSCSR(n)          (SAM_PIO_BASE(n)+SAM_PIO_IFSCSR_OFFSET)
#define SAM_PIO_SCDR(n)            (SAM_PIO_BASE(n)+SAM_PIO_SCDR_OFFSET)
#define SAM_PIO_PPDDR(n)           (SAM_PIO_BASE(n)+SAM_PIO_PPDDR_OFFSET)
#define SAM_PIO_PPDER(n)           (SAM_PIO_BASE(n)+SAM_PIO_PPDER_OFFSET)
#define SAM_PIO_PPDSR(n)           (SAM_PIO_BASE(n)+SAM_PIO_PPDSR_OFFSET)
#define SAM_PIO_OWER(n)            (SAM_PIO_BASE(n)+SAM_PIO_OWER_OFFSET)
#define SAM_PIO_OWDR(n)            (SAM_PIO_BASE(n)+SAM_PIO_OWDR_OFFSET)
#define SAM_PIO_OWSR(n)            (SAM_PIO_BASE(n)+SAM_PIO_OWSR_OFFSET)
#define SAM_PIO_AIMER(n)           (SAM_PIO_BASE(n)+SAM_PIO_AIMER_OFFSET)
#define SAM_PIO_AIMDR(n)           (SAM_PIO_BASE(n)+SAM_PIO_AIMDR_OFFSET)
#define SAM_PIO_AIMMR(n)           (SAM_PIO_BASE(n)+SAM_PIO_AIMMR_OFFSET)
#define SAM_PIO_ESR(n)             (SAM_PIO_BASE(n)+SAM_PIO_ESR_OFFSET)
#define SAM_PIO_LSR(n)             (SAM_PIO_BASE(n)+SAM_PIO_LSR_OFFSET)
#define SAM_PIO_ELSR(n)            (SAM_PIO_BASE(n)+SAM_PIO_ELSR_OFFSET)
#define SAM_PIO_FELLSR(n)          (SAM_PIO_BASE(n)+SAM_PIO_FELLSR_OFFSET)
#define SAM_PIO_REHLSR(n)          (SAM_PIO_BASE(n)+SAM_PIO_REHLSR_OFFSET)
#define SAM_PIO_FRLHSR(n)          (SAM_PIO_BASE(n)+SAM_PIO_FRLHSR_OFFSET)
#define SAM_PIO_LOCKSR(n)          (SAM_PIO_BASE(n)+SAM_PIO_LOCKSR_OFFSET)
#define SAM_PIO_WPMR(n)            (SAM_PIO_BASE(n)+SAM_PIO_WPMR_OFFSET)
#define SAM_PIO_WPSR(n)            (SAM_PIO_BASE(n)+SAM_PIO_WPSR_OFFSET)
#define SAM_PIO_SCHMITT(n)         (SAM_PIO_BASE(n)+SAM_PIO_SCHMITT_OFFSET)
#define SAM_PIO_DELAYR(n)          (SAM_PIO_BASE(n)+SAM_PIO_DELAYR_OFFSET)
#define SAM_PIO_PCMR(n)            (SAM_PIO_BASE(n)+SAM_PIO_PCMR_OFFSET)
#define SAM_PIO_PCIER(n)           (SAM_PIO_BASE(n)+SAM_PIO_PCIER_OFFSET)
#define SAM_PIO_PCIDR(n)           (SAM_PIO_BASE(n)+SAM_PIO_PCIDR_OFFSET)
#define SAM_PIO_PCIMR(n)           (SAM_PIO_BASE(n)+SAM_PIO_PCIMR_OFFSET)
#define SAM_PIO_PCISR(n)           (SAM_PIO_BASE(n)+SAM_PIO_PCISR_OFFSET)
#define SAM_PIO_PCRHR(n)           (SAM_PIO_BASE(n)+SAM_PIO_PCRHR_OFFSET

#define SAM_PIOA_PER               (SAM_PIOA_BASE+SAM_PIO_PER_OFFSET)
#define SAM_PIOA_PDR               (SAM_PIOA_BASE+SAM_PIO_PDR_OFFSET)
#define SAM_PIOA_PSR               (SAM_PIOA_BASE+SAM_PIO_PSR_OFFSET)
#define SAM_PIOA_OER               (SAM_PIOA_BASE+SAM_PIO_OER_OFFSET)
#define SAM_PIOA_ODR               (SAM_PIOA_BASE+SAM_PIO_ODR_OFFSET)
#define SAM_PIOA_OSR               (SAM_PIOA_BASE+SAM_PIO_OSR_OFFSET)
#define SAM_PIOA_IFER              (SAM_PIOA_BASE+SAM_PIO_IFER_OFFSET)
#define SAM_PIOA_IFDR              (SAM_PIOA_BASE+SAM_PIO_IFDR_OFFSET)
#define SAM_PIOA_IFSR              (SAM_PIOA_BASE+SAM_PIO_IFSR_OFFSET)
#define SAM_PIOA_SODR              (SAM_PIOA_BASE+SAM_PIO_SODR_OFFSET)
#define SAM_PIOA_CODR              (SAM_PIOA_BASE+SAM_PIO_CODR_OFFSET)
#define SAM_PIOA_ODSR              (SAM_PIOA_BASE+SAM_PIO_ODSR_OFFSET)
#define SAM_PIOA_PDSR              (SAM_PIOA_BASE+SAM_PIO_PDSR_OFFSET)
#define SAM_PIOA_IER               (SAM_PIOA_BASE+SAM_PIO_IER_OFFSET)
#define SAM_PIOA_IDR               (SAM_PIOA_BASE+SAM_PIO_IDR_OFFSET)
#define SAM_PIOA_IMR               (SAM_PIOA_BASE+SAM_PIO_IMR_OFFSET)
#define SAM_PIOA_ISR               (SAM_PIOA_BASE+SAM_PIO_ISR_OFFSET)
#define SAM_PIOA_MDER              (SAM_PIOA_BASE+SAM_PIO_MDER_OFFSET)
#define SAM_PIOA_MDDR              (SAM_PIOA_BASE+SAM_PIO_MDDR_OFFSET)
#define SAM_PIOA_MDSR              (SAM_PIOA_BASE+SAM_PIO_MDSR_OFFSET)
#define SAM_PIOA_PUDR              (SAM_PIOA_BASE+SAM_PIO_PUDR_OFFSET)
#define SAM_PIOA_PUER              (SAM_PIOA_BASE+SAM_PIO_PUER_OFFSET)
#define SAM_PIOA_PUSR              (SAM_PIOA_BASE+SAM_PIO_PUSR_OFFSET)
#define SAM_PIOA_ABCDSR1           (SAM_PIOA_BASE+SAM_PIO_ABCDSR1_OFFSET)
#define SAM_PIOA_ABCDSR2           (SAM_PIOA_BASE+SAM_PIO_ABCDSR2_OFFSET)
#define SAM_PIOA_IFSCDR            (SAM_PIOA_BASE+SAM_PIO_IFSCDR_OFFSET)
#define SAM_PIOA_IFSCER            (SAM_PIOA_BASE+SAM_PIO_IFSCER_OFFSET)
#define SAM_PIOA_IFSCSR            (SAM_PIOA_BASE+SAM_PIO_IFSCSR_OFFSET)
#define SAM_PIOA_SCDR              (SAM_PIOA_BASE+SAM_PIO_SCDR_OFFSET)
#define SAM_PIOA_PPDDR             (SAM_PIOA_BASE+SAM_PIO_PPDDR_OFFSET)
#define SAM_PIOA_PPDER             (SAM_PIOA_BASE+SAM_PIO_PPDER_OFFSET)
#define SAM_PIOA_PPDSR             (SAM_PIOA_BASE+SAM_PIO_PPDSR_OFFSET)
#define SAM_PIOA_OWER              (SAM_PIOA_BASE+SAM_PIO_OWER_OFFSET)
#define SAM_PIOA_OWDR              (SAM_PIOA_BASE+SAM_PIO_OWDR_OFFSET)
#define SAM_PIOA_OWSR              (SAM_PIOA_BASE+SAM_PIO_OWSR_OFFSET)
#define SAM_PIOA_AIMER             (SAM_PIOA_BASE+SAM_PIO_AIMER_OFFSET)
#define SAM_PIOA_AIMDR             (SAM_PIOA_BASE+SAM_PIO_AIMDR_OFFSET)
#define SAM_PIOA_AIMMR             (SAM_PIOA_BASE+SAM_PIO_AIMMR_OFFSET)
#define SAM_PIOA_ESR               (SAM_PIOA_BASE+SAM_PIO_ESR_OFFSET)
#define SAM_PIOA_LSR               (SAM_PIOA_BASE+SAM_PIO_LSR_OFFSET)
#define SAM_PIOA_ELSR              (SAM_PIOA_BASE+SAM_PIO_ELSR_OFFSET)
#define SAM_PIOA_FELLSR            (SAM_PIOA_BASE+SAM_PIO_FELLSR_OFFSET)
#define SAM_PIOA_REHLSR            (SAM_PIOA_BASE+SAM_PIO_REHLSR_OFFSET)
#define SAM_PIOA_FRLHSR            (SAM_PIOA_BASE+SAM_PIO_FRLHSR_OFFSET)
#define SAM_PIOA_LOCKSR            (SAM_PIOA_BASE+SAM_PIO_LOCKSR_OFFSET)
#define SAM_PIOA_WPMR              (SAM_PIOA_BASE+SAM_PIO_WPMR_OFFSET)
#define SAM_PIOA_WPSR              (SAM_PIOA_BASE+SAM_PIO_WPSR_OFFSET)
#define SAM_PIOA_SCHMITT           (SAM_PIOA_BASE+SAM_PIO_SCHMITT_OFFSET)
#define SAM_PIOA_DELAYR            (SAM_PIOA_BASE+SAM_PIO_DELAYR_OFFSET)
#define SAM_PIOA_PCMR              (SAM_PIOA_BASE+SAM_PIO_PCMR_OFFSET)
#define SAM_PIOA_PCIER             (SAM_PIOA_BASE+SAM_PIO_PCIER_OFFSET)
#define SAM_PIOA_PCIDR             (SAM_PIOA_BASE+SAM_PIO_PCIDR_OFFSET)
#define SAM_PIOA_PCIMR             (SAM_PIOA_BASE+SAM_PIO_PCIMR_OFFSET)
#define SAM_PIOA_PCISR             (SAM_PIOA_BASE+SAM_PIO_PCISR_OFFSET)
#define SAM_PIOA_PCRHR             (SAM_PIOA_BASE+SAM_PIO_PCRHR_OFFSET

#define SAM_PIOB_PER               (SAM_PIOB_BASE+SAM_PIO_PER_OFFSET)
#define SAM_PIOB_PDR               (SAM_PIOB_BASE+SAM_PIO_PDR_OFFSET)
#define SAM_PIOB_PSR               (SAM_PIOB_BASE+SAM_PIO_PSR_OFFSET)
#define SAM_PIOB_OER               (SAM_PIOB_BASE+SAM_PIO_OER_OFFSET)
#define SAM_PIOB_ODR               (SAM_PIOB_BASE+SAM_PIO_ODR_OFFSET)
#define SAM_PIOB_OSR               (SAM_PIOB_BASE+SAM_PIO_OSR_OFFSET)
#define SAM_PIOB_IFER              (SAM_PIOB_BASE+SAM_PIO_IFER_OFFSET)
#define SAM_PIOB_IFDR              (SAM_PIOB_BASE+SAM_PIO_IFDR_OFFSET)
#define SAM_PIOB_IFSR              (SAM_PIOB_BASE+SAM_PIO_IFSR_OFFSET)
#define SAM_PIOB_SODR              (SAM_PIOB_BASE+SAM_PIO_SODR_OFFSET)
#define SAM_PIOB_CODR              (SAM_PIOB_BASE+SAM_PIO_CODR_OFFSET)
#define SAM_PIOB_ODSR              (SAM_PIOB_BASE+SAM_PIO_ODSR_OFFSET)
#define SAM_PIOB_PDSR              (SAM_PIOB_BASE+SAM_PIO_PDSR_OFFSET)
#define SAM_PIOB_IER               (SAM_PIOB_BASE+SAM_PIO_IER_OFFSET)
#define SAM_PIOB_IDR               (SAM_PIOB_BASE+SAM_PIO_IDR_OFFSET)
#define SAM_PIOB_IMR               (SAM_PIOB_BASE+SAM_PIO_IMR_OFFSET)
#define SAM_PIOB_ISR               (SAM_PIOB_BASE+SAM_PIO_ISR_OFFSET)
#define SAM_PIOB_MDER              (SAM_PIOB_BASE+SAM_PIO_MDER_OFFSET)
#define SAM_PIOB_MDDR              (SAM_PIOB_BASE+SAM_PIO_MDDR_OFFSET)
#define SAM_PIOB_MDSR              (SAM_PIOB_BASE+SAM_PIO_MDSR_OFFSET)
#define SAM_PIOB_PUDR              (SAM_PIOB_BASE+SAM_PIO_PUDR_OFFSET)
#define SAM_PIOB_PUER              (SAM_PIOB_BASE+SAM_PIO_PUER_OFFSET)
#define SAM_PIOB_PUSR              (SAM_PIOB_BASE+SAM_PIO_PUSR_OFFSET)
#define SAM_PIOB_ABCDSR1           (SAM_PIOB_BASE+SAM_PIO_ABCDSR1_OFFSET)
#define SAM_PIOB_ABCDSR2           (SAM_PIOB_BASE+SAM_PIO_ABCDSR2_OFFSET)
#define SAM_PIOB_IFSCDR            (SAM_PIOB_BASE+SAM_PIO_IFSCDR_OFFSET)
#define SAM_PIOB_IFSCER            (SAM_PIOB_BASE+SAM_PIO_IFSCER_OFFSET)
#define SAM_PIOB_IFSCSR            (SAM_PIOB_BASE+SAM_PIO_IFSCSR_OFFSET)
#define SAM_PIOB_SCDR              (SAM_PIOB_BASE+SAM_PIO_SCDR_OFFSET)
#define SAM_PIOB_PPDDR             (SAM_PIOB_BASE+SAM_PIO_PPDDR_OFFSET)
#define SAM_PIOB_PPDER             (SAM_PIOB_BASE+SAM_PIO_PPDER_OFFSET)
#define SAM_PIOB_PPDSR             (SAM_PIOB_BASE+SAM_PIO_PPDSR_OFFSET)
#define SAM_PIOB_OWER              (SAM_PIOB_BASE+SAM_PIO_OWER_OFFSET)
#define SAM_PIOB_OWDR              (SAM_PIOB_BASE+SAM_PIO_OWDR_OFFSET)
#define SAM_PIOB_OWSR              (SAM_PIOB_BASE+SAM_PIO_OWSR_OFFSET)
#define SAM_PIOB_AIMER             (SAM_PIOB_BASE+SAM_PIO_AIMER_OFFSET)
#define SAM_PIOB_AIMDR             (SAM_PIOB_BASE+SAM_PIO_AIMDR_OFFSET)
#define SAM_PIOB_AIMMR             (SAM_PIOB_BASE+SAM_PIO_AIMMR_OFFSET)
#define SAM_PIOB_ESR               (SAM_PIOB_BASE+SAM_PIO_ESR_OFFSET)
#define SAM_PIOB_LSR               (SAM_PIOB_BASE+SAM_PIO_LSR_OFFSET)
#define SAM_PIOB_ELSR              (SAM_PIOB_BASE+SAM_PIO_ELSR_OFFSET)
#define SAM_PIOB_FELLSR            (SAM_PIOB_BASE+SAM_PIO_FELLSR_OFFSET)
#define SAM_PIOB_REHLSR            (SAM_PIOB_BASE+SAM_PIO_REHLSR_OFFSET)
#define SAM_PIOB_FRLHSR            (SAM_PIOB_BASE+SAM_PIO_FRLHSR_OFFSET)
#define SAM_PIOB_LOCKSR            (SAM_PIOB_BASE+SAM_PIO_LOCKSR_OFFSET)
#define SAM_PIOB_WPMR              (SAM_PIOB_BASE+SAM_PIO_WPMR_OFFSET)
#define SAM_PIOB_WPSR              (SAM_PIOB_BASE+SAM_PIO_WPSR_OFFSET)
#define SAM_PIOB_SCHMITT           (SAM_PIOB_BASE+SAM_PIO_SCHMITT_OFFSET)
#define SAM_PIOB_DELAYR            (SAM_PIOB_BASE+SAM_PIO_DELAYR_OFFSET)
#define SAM_PIOB_PCMR              (SAM_PIOB_BASE+SAM_PIO_PCMR_OFFSET)
#define SAM_PIOB_PCIER             (SAM_PIOB_BASE+SAM_PIO_PCIER_OFFSET)
#define SAM_PIOB_PCIDR             (SAM_PIOB_BASE+SAM_PIO_PCIDR_OFFSET)
#define SAM_PIOB_PCIMR             (SAM_PIOB_BASE+SAM_PIO_PCIMR_OFFSET)
#define SAM_PIOB_PCISR             (SAM_PIOB_BASE+SAM_PIO_PCISR_OFFSET)
#define SAM_PIOB_PCRHR             (SAM_PIOB_BASE+SAM_PIO_PCRHR_OFFSET

#define SAM_PIOC_PER               (SAM_PIOC_BASE+SAM_PIO_PER_OFFSET)
#define SAM_PIOC_PDR               (SAM_PIOC_BASE+SAM_PIO_PDR_OFFSET)
#define SAM_PIOC_PSR               (SAM_PIOC_BASE+SAM_PIO_PSR_OFFSET)
#define SAM_PIOC_OER               (SAM_PIOC_BASE+SAM_PIO_OER_OFFSET)
#define SAM_PIOC_ODR               (SAM_PIOC_BASE+SAM_PIO_ODR_OFFSET)
#define SAM_PIOC_OSR               (SAM_PIOC_BASE+SAM_PIO_OSR_OFFSET)
#define SAM_PIOC_IFER              (SAM_PIOC_BASE+SAM_PIO_IFER_OFFSET)
#define SAM_PIOC_IFDR              (SAM_PIOC_BASE+SAM_PIO_IFDR_OFFSET)
#define SAM_PIOC_IFSR              (SAM_PIOC_BASE+SAM_PIO_IFSR_OFFSET)
#define SAM_PIOC_SODR              (SAM_PIOC_BASE+SAM_PIO_SODR_OFFSET)
#define SAM_PIOC_CODR              (SAM_PIOC_BASE+SAM_PIO_CODR_OFFSET)
#define SAM_PIOC_ODSR              (SAM_PIOC_BASE+SAM_PIO_ODSR_OFFSET)
#define SAM_PIOC_PDSR              (SAM_PIOC_BASE+SAM_PIO_PDSR_OFFSET)
#define SAM_PIOC_IER               (SAM_PIOC_BASE+SAM_PIO_IER_OFFSET)
#define SAM_PIOC_IDR               (SAM_PIOC_BASE+SAM_PIO_IDR_OFFSET)
#define SAM_PIOC_IMR               (SAM_PIOC_BASE+SAM_PIO_IMR_OFFSET)
#define SAM_PIOC_ISR               (SAM_PIOC_BASE+SAM_PIO_ISR_OFFSET)
#define SAM_PIOC_MDER              (SAM_PIOC_BASE+SAM_PIO_MDER_OFFSET)
#define SAM_PIOC_MDDR              (SAM_PIOC_BASE+SAM_PIO_MDDR_OFFSET)
#define SAM_PIOC_MDSR              (SAM_PIOC_BASE+SAM_PIO_MDSR_OFFSET)
#define SAM_PIOC_PUDR              (SAM_PIOC_BASE+SAM_PIO_PUDR_OFFSET)
#define SAM_PIOC_PUER              (SAM_PIOC_BASE+SAM_PIO_PUER_OFFSET)
#define SAM_PIOC_PUSR              (SAM_PIOC_BASE+SAM_PIO_PUSR_OFFSET)
#define SAM_PIOC_ABCDSR1           (SAM_PIOC_BASE+SAM_PIO_ABCDSR1_OFFSET)
#define SAM_PIOC_ABCDSR2           (SAM_PIOC_BASE+SAM_PIO_ABCDSR2_OFFSET)
#define SAM_PIOC_IFSCDR            (SAM_PIOC_BASE+SAM_PIO_IFSCDR_OFFSET)
#define SAM_PIOC_IFSCER            (SAM_PIOC_BASE+SAM_PIO_IFSCER_OFFSET)
#define SAM_PIOC_IFSCSR            (SAM_PIOC_BASE+SAM_PIO_IFSCSR_OFFSET)
#define SAM_PIOC_SCDR              (SAM_PIOC_BASE+SAM_PIO_SCDR_OFFSET)
#define SAM_PIOC_PPDDR             (SAM_PIOC_BASE+SAM_PIO_PPDDR_OFFSET)
#define SAM_PIOC_PPDER             (SAM_PIOC_BASE+SAM_PIO_PPDER_OFFSET)
#define SAM_PIOC_PPDSR             (SAM_PIOC_BASE+SAM_PIO_PPDSR_OFFSET)
#define SAM_PIOC_OWER              (SAM_PIOC_BASE+SAM_PIO_OWER_OFFSET)
#define SAM_PIOC_OWDR              (SAM_PIOC_BASE+SAM_PIO_OWDR_OFFSET)
#define SAM_PIOC_OWSR              (SAM_PIOC_BASE+SAM_PIO_OWSR_OFFSET)
#define SAM_PIOC_AIMER             (SAM_PIOC_BASE+SAM_PIO_AIMER_OFFSET)
#define SAM_PIOC_AIMDR             (SAM_PIOC_BASE+SAM_PIO_AIMDR_OFFSET)
#define SAM_PIOC_AIMMR             (SAM_PIOC_BASE+SAM_PIO_AIMMR_OFFSET)
#define SAM_PIOC_ESR               (SAM_PIOC_BASE+SAM_PIO_ESR_OFFSET)
#define SAM_PIOC_LSR               (SAM_PIOC_BASE+SAM_PIO_LSR_OFFSET)
#define SAM_PIOC_ELSR              (SAM_PIOC_BASE+SAM_PIO_ELSR_OFFSET)
#define SAM_PIOC_FELLSR            (SAM_PIOC_BASE+SAM_PIO_FELLSR_OFFSET)
#define SAM_PIOC_REHLSR            (SAM_PIOC_BASE+SAM_PIO_REHLSR_OFFSET)
#define SAM_PIOC_FRLHSR            (SAM_PIOC_BASE+SAM_PIO_FRLHSR_OFFSET)
#define SAM_PIOC_LOCKSR            (SAM_PIOC_BASE+SAM_PIO_LOCKSR_OFFSET)
#define SAM_PIOC_WPMR              (SAM_PIOC_BASE+SAM_PIO_WPMR_OFFSET)
#define SAM_PIOC_WPSR              (SAM_PIOC_BASE+SAM_PIO_WPSR_OFFSET)
#define SAM_PIOC_SCHMITT           (SAM_PIOC_BASE+SAM_PIO_SCHMITT_OFFSET)
#define SAM_PIOC_DELAYR            (SAM_PIOC_BASE+SAM_PIO_DELAYR_OFFSET)
#define SAM_PIOC_PCMR              (SAM_PIOC_BASE+SAM_PIO_PCMR_OFFSET)
#define SAM_PIOC_PCIER             (SAM_PIOC_BASE+SAM_PIO_PCIER_OFFSET)
#define SAM_PIOC_PCIDR             (SAM_PIOC_BASE+SAM_PIO_PCIDR_OFFSET)
#define SAM_PIOC_PCIMR             (SAM_PIOC_BASE+SAM_PIO_PCIMR_OFFSET)
#define SAM_PIOC_PCISR             (SAM_PIOC_BASE+SAM_PIO_PCISR_OFFSET)
#define SAM_PIOC_PCRHR             (SAM_PIOC_BASE+SAM_PIO_PCRHR_OFFSET

#define SAM_PIOD_PER               (SAM_PIOD_BASE+SAM_PIO_PER_OFFSET)
#define SAM_PIOD_PDR               (SAM_PIOD_BASE+SAM_PIO_PDR_OFFSET)
#define SAM_PIOD_PSR               (SAM_PIOD_BASE+SAM_PIO_PSR_OFFSET)
#define SAM_PIOD_OER               (SAM_PIOD_BASE+SAM_PIO_OER_OFFSET)
#define SAM_PIOD_ODR               (SAM_PIOD_BASE+SAM_PIO_ODR_OFFSET)
#define SAM_PIOD_OSR               (SAM_PIOD_BASE+SAM_PIO_OSR_OFFSET)
#define SAM_PIOD_IFER              (SAM_PIOD_BASE+SAM_PIO_IFER_OFFSET)
#define SAM_PIOD_IFDR              (SAM_PIOD_BASE+SAM_PIO_IFDR_OFFSET)
#define SAM_PIOD_IFSR              (SAM_PIOD_BASE+SAM_PIO_IFSR_OFFSET)
#define SAM_PIOD_SODR              (SAM_PIOD_BASE+SAM_PIO_SODR_OFFSET)
#define SAM_PIOD_CODR              (SAM_PIOD_BASE+SAM_PIO_CODR_OFFSET)
#define SAM_PIOD_ODSR              (SAM_PIOD_BASE+SAM_PIO_ODSR_OFFSET)
#define SAM_PIOD_PDSR              (SAM_PIOD_BASE+SAM_PIO_PDSR_OFFSET)
#define SAM_PIOD_IER               (SAM_PIOD_BASE+SAM_PIO_IER_OFFSET)
#define SAM_PIOD_IDR               (SAM_PIOD_BASE+SAM_PIO_IDR_OFFSET)
#define SAM_PIOD_IMR               (SAM_PIOD_BASE+SAM_PIO_IMR_OFFSET)
#define SAM_PIOD_ISR               (SAM_PIOD_BASE+SAM_PIO_ISR_OFFSET)
#define SAM_PIOD_MDER              (SAM_PIOD_BASE+SAM_PIO_MDER_OFFSET)
#define SAM_PIOD_MDDR              (SAM_PIOD_BASE+SAM_PIO_MDDR_OFFSET)
#define SAM_PIOD_MDSR              (SAM_PIOD_BASE+SAM_PIO_MDSR_OFFSET)
#define SAM_PIOD_PUDR              (SAM_PIOD_BASE+SAM_PIO_PUDR_OFFSET)
#define SAM_PIOD_PUER              (SAM_PIOD_BASE+SAM_PIO_PUER_OFFSET)
#define SAM_PIOD_PUSR              (SAM_PIOD_BASE+SAM_PIO_PUSR_OFFSET)
#define SAM_PIOD_ABCDSR1           (SAM_PIOD_BASE+SAM_PIO_ABCDSR1_OFFSET)
#define SAM_PIOD_ABCDSR2           (SAM_PIOD_BASE+SAM_PIO_ABCDSR2_OFFSET)
#define SAM_PIOD_IFSCDR            (SAM_PIOD_BASE+SAM_PIO_IFSCDR_OFFSET)
#define SAM_PIOD_IFSCER            (SAM_PIOD_BASE+SAM_PIO_IFSCER_OFFSET)
#define SAM_PIOD_IFSCSR            (SAM_PIOD_BASE+SAM_PIO_IFSCSR_OFFSET)
#define SAM_PIOD_SCDR              (SAM_PIOD_BASE+SAM_PIO_SCDR_OFFSET)
#define SAM_PIOD_PPDDR             (SAM_PIOD_BASE+SAM_PIO_PPDDR_OFFSET)
#define SAM_PIOD_PPDER             (SAM_PIOD_BASE+SAM_PIO_PPDER_OFFSET)
#define SAM_PIOD_PPDSR             (SAM_PIOD_BASE+SAM_PIO_PPDSR_OFFSET)
#define SAM_PIOD_OWER              (SAM_PIOD_BASE+SAM_PIO_OWER_OFFSET)
#define SAM_PIOD_OWDR              (SAM_PIOD_BASE+SAM_PIO_OWDR_OFFSET)
#define SAM_PIOD_OWSR              (SAM_PIOD_BASE+SAM_PIO_OWSR_OFFSET)
#define SAM_PIOD_AIMER             (SAM_PIOD_BASE+SAM_PIO_AIMER_OFFSET)
#define SAM_PIOD_AIMDR             (SAM_PIOD_BASE+SAM_PIO_AIMDR_OFFSET)
#define SAM_PIOD_AIMMR             (SAM_PIOD_BASE+SAM_PIO_AIMMR_OFFSET)
#define SAM_PIOD_ESR               (SAM_PIOD_BASE+SAM_PIO_ESR_OFFSET)
#define SAM_PIOD_LSR               (SAM_PIOD_BASE+SAM_PIO_LSR_OFFSET)
#define SAM_PIOD_ELSR              (SAM_PIOD_BASE+SAM_PIO_ELSR_OFFSET)
#define SAM_PIOD_FELLSR            (SAM_PIOD_BASE+SAM_PIO_FELLSR_OFFSET)
#define SAM_PIOD_REHLSR            (SAM_PIOD_BASE+SAM_PIO_REHLSR_OFFSET)
#define SAM_PIOD_FRLHSR            (SAM_PIOD_BASE+SAM_PIO_FRLHSR_OFFSET)
#define SAM_PIOD_LOCKSR            (SAM_PIOD_BASE+SAM_PIO_LOCKSR_OFFSET)
#define SAM_PIOD_WPMR              (SAM_PIOD_BASE+SAM_PIO_WPMR_OFFSET)
#define SAM_PIOD_WPSR              (SAM_PIOD_BASE+SAM_PIO_WPSR_OFFSET)
#define SAM_PIOD_SCHMITT           (SAM_PIOD_BASE+SAM_PIO_SCHMITT_OFFSET)
#define SAM_PIOD_DELAYR            (SAM_PIOD_BASE+SAM_PIO_DELAYR_OFFSET)
#define SAM_PIOD_PCMR              (SAM_PIOD_BASE+SAM_PIO_PCMR_OFFSET)
#define SAM_PIOD_PCIER             (SAM_PIOD_BASE+SAM_PIO_PCIER_OFFSET)
#define SAM_PIOD_PCIDR             (SAM_PIOD_BASE+SAM_PIO_PCIDR_OFFSET)
#define SAM_PIOD_PCIMR             (SAM_PIOD_BASE+SAM_PIO_PCIMR_OFFSET)
#define SAM_PIOD_PCISR             (SAM_PIOD_BASE+SAM_PIO_PCISR_OFFSET)
#define SAM_PIOD_PCRHR             (SAM_PIOD_BASE+SAM_PIO_PCRHR_OFFSET

#define SAM_PIOE_PER               (SAM_PIOE_BASE+SAM_PIO_PER_OFFSET)
#define SAM_PIOE_PDR               (SAM_PIOE_BASE+SAM_PIO_PDR_OFFSET)
#define SAM_PIOE_PSR               (SAM_PIOE_BASE+SAM_PIO_PSR_OFFSET)
#define SAM_PIOE_OER               (SAM_PIOE_BASE+SAM_PIO_OER_OFFSET)
#define SAM_PIOE_ODR               (SAM_PIOE_BASE+SAM_PIO_ODR_OFFSET)
#define SAM_PIOE_OSR               (SAM_PIOE_BASE+SAM_PIO_OSR_OFFSET)
#define SAM_PIOE_IFER              (SAM_PIOE_BASE+SAM_PIO_IFER_OFFSET)
#define SAM_PIOE_IFDR              (SAM_PIOE_BASE+SAM_PIO_IFDR_OFFSET)
#define SAM_PIOE_IFSR              (SAM_PIOE_BASE+SAM_PIO_IFSR_OFFSET)
#define SAM_PIOE_SODR              (SAM_PIOE_BASE+SAM_PIO_SODR_OFFSET)
#define SAM_PIOE_CODR              (SAM_PIOE_BASE+SAM_PIO_CODR_OFFSET)
#define SAM_PIOE_ODSR              (SAM_PIOE_BASE+SAM_PIO_ODSR_OFFSET)
#define SAM_PIOE_PDSR              (SAM_PIOE_BASE+SAM_PIO_PDSR_OFFSET)
#define SAM_PIOE_IER               (SAM_PIOE_BASE+SAM_PIO_IER_OFFSET)
#define SAM_PIOE_IDR               (SAM_PIOE_BASE+SAM_PIO_IDR_OFFSET)
#define SAM_PIOE_IMR               (SAM_PIOE_BASE+SAM_PIO_IMR_OFFSET)
#define SAM_PIOE_ISR               (SAM_PIOE_BASE+SAM_PIO_ISR_OFFSET)
#define SAM_PIOE_MDER              (SAM_PIOE_BASE+SAM_PIO_MDER_OFFSET)
#define SAM_PIOE_MDDR              (SAM_PIOE_BASE+SAM_PIO_MDDR_OFFSET)
#define SAM_PIOE_MDSR              (SAM_PIOE_BASE+SAM_PIO_MDSR_OFFSET)
#define SAM_PIOE_PUDR              (SAM_PIOE_BASE+SAM_PIO_PUDR_OFFSET)
#define SAM_PIOE_PUER              (SAM_PIOE_BASE+SAM_PIO_PUER_OFFSET)
#define SAM_PIOE_PUSR              (SAM_PIOE_BASE+SAM_PIO_PUSR_OFFSET)
#define SAM_PIOE_ABCDSR1           (SAM_PIOE_BASE+SAM_PIO_ABCDSR1_OFFSET)
#define SAM_PIOE_ABCDSR2           (SAM_PIOE_BASE+SAM_PIO_ABCDSR2_OFFSET)
#define SAM_PIOE_IFSCDR            (SAM_PIOE_BASE+SAM_PIO_IFSCDR_OFFSET)
#define SAM_PIOE_IFSCER            (SAM_PIOE_BASE+SAM_PIO_IFSCER_OFFSET)
#define SAM_PIOE_IFSCSR            (SAM_PIOE_BASE+SAM_PIO_IFSCSR_OFFSET)
#define SAM_PIOE_SCDR              (SAM_PIOE_BASE+SAM_PIO_SCDR_OFFSET)
#define SAM_PIOE_PPDDR             (SAM_PIOE_BASE+SAM_PIO_PPDDR_OFFSET)
#define SAM_PIOE_PPDER             (SAM_PIOE_BASE+SAM_PIO_PPDER_OFFSET)
#define SAM_PIOE_PPDSR             (SAM_PIOE_BASE+SAM_PIO_PPDSR_OFFSET)
#define SAM_PIOE_OWER              (SAM_PIOE_BASE+SAM_PIO_OWER_OFFSET)
#define SAM_PIOE_OWDR              (SAM_PIOE_BASE+SAM_PIO_OWDR_OFFSET)
#define SAM_PIOE_OWSR              (SAM_PIOE_BASE+SAM_PIO_OWSR_OFFSET)
#define SAM_PIOE_AIMER             (SAM_PIOE_BASE+SAM_PIO_AIMER_OFFSET)
#define SAM_PIOE_AIMDR             (SAM_PIOE_BASE+SAM_PIO_AIMDR_OFFSET)
#define SAM_PIOE_AIMMR             (SAM_PIOE_BASE+SAM_PIO_AIMMR_OFFSET)
#define SAM_PIOE_ESR               (SAM_PIOE_BASE+SAM_PIO_ESR_OFFSET)
#define SAM_PIOE_LSR               (SAM_PIOE_BASE+SAM_PIO_LSR_OFFSET)
#define SAM_PIOE_ELSR              (SAM_PIOE_BASE+SAM_PIO_ELSR_OFFSET)
#define SAM_PIOE_FELLSR            (SAM_PIOE_BASE+SAM_PIO_FELLSR_OFFSET)
#define SAM_PIOE_REHLSR            (SAM_PIOE_BASE+SAM_PIO_REHLSR_OFFSET)
#define SAM_PIOE_FRLHSR            (SAM_PIOE_BASE+SAM_PIO_FRLHSR_OFFSET)
#define SAM_PIOE_LOCKSR            (SAM_PIOE_BASE+SAM_PIO_LOCKSR_OFFSET)
#define SAM_PIOE_WPMR              (SAM_PIOE_BASE+SAM_PIO_WPMR_OFFSET)
#define SAM_PIOE_WPSR              (SAM_PIOE_BASE+SAM_PIO_WPSR_OFFSET)
#define SAM_PIOE_SCHMITT           (SAM_PIOE_BASE+SAM_PIO_SCHMITT_OFFSET)
#define SAM_PIOE_DELAYR            (SAM_PIOE_BASE+SAM_PIO_DELAYR_OFFSET)
#define SAM_PIOE_PCMR              (SAM_PIOE_BASE+SAM_PIO_PCMR_OFFSET)
#define SAM_PIOE_PCIER             (SAM_PIOE_BASE+SAM_PIO_PCIER_OFFSET)
#define SAM_PIOE_PCIDR             (SAM_PIOE_BASE+SAM_PIO_PCIDR_OFFSET)
#define SAM_PIOE_PCIMR             (SAM_PIOE_BASE+SAM_PIO_PCIMR_OFFSET)
#define SAM_PIOE_PCISR             (SAM_PIOE_BASE+SAM_PIO_PCISR_OFFSET)
#define SAM_PIOE_PCRHR             (SAM_PIOE_BASE+SAM_PIO_PCRHR_OFFSET

/* PIO register bit definitions *********************************************************/

/* Common bit definitions for ALMOST all IO registers (exceptions follow) */

#define PIO(n)                     (1 << (n)) /* Bit n: PIO n, n=0-31 */

/* PIO Slow Clock Divider Debouncing Register */

#define PIO_SCDR_MASK              (0x3fff) /* Bits 0-13: Slow Clock Divider */

/* PIO Write Protect Mode Register */

#define PIO_WPMR_WPEN              (1 << 0)  /* Bit 0:  Write Protect Enable */
#define PIO_WPMR_WPKEY_SHIFT       (8)       /* Bits 8-31: Write Protect KEY */
#define PIO_WPMR_WPKEY_MASK        (0xffffff << PIO_WPMR_WPKEY_SHIFT)
#  define PIO_WPMR_WPKEY           (0x50494f << PIO_WPMR_WPKEY_SHIFT)

/* PIO Write Protect Status Register */

#define PIO_WPSR_WPVS              (1 << 0)  /* Bit 0:  Write Protect Violation Status */
#define PIO_WPSR_WPVSRC_SHIFT      (8)       /* Bits 8-23: Write Protect Violation Source */
#define PIO_WPSR_WPVSRC_MASK       (0xffff << PIO_WPSR_WPVSRC_SHIFT)

/* IO Delay Register */

#define PIO_DELAYR_DELAY_SHIFT(d)  ((d) << 2)
#define PIO_DELAYR_DELAY_MASK(d)   (15 << PIO_DELAYR_SHIFT(d))
#  define PIO_DELAYR_DELAY(d,n)    ((uint32_t)(n) << PIO_DELAYR_SHIFT(d))

#define PIO_DELAYR_DELAY0_SHIFT    (0)       /* Bits 0-3: Delay 0 */
#define PIO_DELAYR_DELAY0_MASK     (15 << PIO_DELAYR_DELAY0_SHIFT)
#  define PIO_DELAYR_DELAY0(n)     ((uint32_t)(n) << PIO_DELAYR_DELAY0_SHIFT)
#define PIO_DELAYR_DELAY1_SHIFT    (4)       /* Bits 4-7: Delay 0 */
#define PIO_DELAYR_DELAY1_MASK     (15 << PIO_DELAYR_DELAY1_SHIFT)
#  define PIO_DELAYR_DELAY1(n)     ((uint32_t)(n) << PIO_DELAYR_DELAY1_SHIFT)
#define PIO_DELAYR_DELAY2_SHIFT    (8)       /* Bits 8-11: Delay 0 */
#define PIO_DELAYR_DELAY2_MASK     (15 << PIO_DELAYR_DELAY2_SHIFT)
#  define PIO_DELAYR_DELAY2(n)     ((uint32_t)(n) << PIO_DELAYR_DELAY2_SHIFT)
#define PIO_DELAYR_DELAY3_SHIFT    (12)      /* Bits 12-15: Delay 0 */
#define PIO_DELAYR_DELAY3_MASK     (15 << PIO_DELAYR_DELAY3_SHIFT)
#  define PIO_DELAYR_DELAY3(n)     ((uint32_t)(n) << PIO_DELAYR_DELAY3_SHIFT)
#define PIO_DELAYR_DELAY4_SHIFT    (16)      /* Bits 16-19: Delay 0 */
#define PIO_DELAYR_DELAY4_MASK     (15 << PIO_DELAYR_DELAY4_SHIFT)
#  define PIO_DELAYR_DELAY4(n)     ((uint32_t)(n) << PIO_DELAYR_DELAY4_SHIFT)
#define PIO_DELAYR_DELAY5_SHIFT    (20)      /* Bits 20-23: Delay 0 */
#define PIO_DELAYR_DELAY5_MASK     (15 << PIO_DELAYR_DELAY5_SHIFT)
#  define PIO_DELAYR_DELAY5(n)     ((uint32_t)(n) << PIO_DELAYR_DELAY5_SHIFT)
#define PIO_DELAYR_DELAY6_SHIFT    (24)      /* Bits 24-27: Delay 0 */
#define PIO_DELAYR_DELAY6_MASK     (15 << PIO_DELAYR_DELAY6_SHIFT)
#  define PIO_DELAYR_DELAY6(n)     ((uint32_t)(n) << PIO_DELAYR_DELAY6_SHIFT)
#define PIO_DELAYR_DELAY7_SHIFT    (28)      /* Bits 28-31: Delay 0 */
#define PIO_DELAYR_DELAY7_MASK     (15 << PIO_DELAYR_DELAY7_SHIFT)
#  define PIO_DELAYR_DELAY7(n)     ((uint32_t)(n) << PIO_DELAYR_DELAY7_SHIFT)

/* PIO Parallel Capture Mode Register */

#define PIO_PCMR_PCEN              (1 << 0)  /* Bit 0:  Parallel Capture Mode Enable */
#define PIO_PCMR_DSIZE_SHIFT       (4)       /* Bits 4-5: Parallel Capture Mode Data Size */
#define PIO_PCMR_DSIZE_MASK        (3 << PIO_PCMR_DSIZE_SHIFT)
#  define PIO_PCMR_DSIZE_BYTE      (0 << PIO_PCMR_DSIZE_SHIFT) /* 8-bit data in PIO_PCRHR */
#  define PIO_PCMR_DSIZE_HWORD     (1 << PIO_PCMR_DSIZE_SHIFT) /* 16-bit data in PIO_PCRHR */
#  define PIO_PCMR_DSIZE_WORD      (2 << PIO_PCMR_DSIZE_SHIFT) /* 32-bit data in PIO_PCRHR */
#define PIO_PCMR_ALWYS             (1 << 9)  /* Bit 9:  Parallel Capture Mode Always Sampling */
#define PIO_PCMR_HALFS             (1 << 10) /* Bit 10: Parallel Capture Mode Half Sampling */
#define PIO_PCMR_FRSTS             (1 << 11) /* Bit 11: Parallel Capture Mode First Sample */

/* PIO Parallel Capture Interrupt Enable, Disable, Mask, and Status Registers */

#define PIOC_PCINT_DRDY            (1 << 0)  /* Bit 0:  Parallel Capture Mode Data Ready Interrupt Enable */
#define PIOC_PCINT_OVRE            (1 << 1)  /* Bit 1:  Parallel Capture Mode Overrun Error Interrupt Enable */
#define PIOC_PCINT_ENDRX           (1 << 2)  /* Bit 2:  End of Reception Transfer Interrupt Enable */
#define PIOC_PCINT_RXBUFF          (1 << 3)  /* Bit 3:  Reception Buffer Full Interrupt Enable */

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_CHIP_SAM4E_PIO_H */
