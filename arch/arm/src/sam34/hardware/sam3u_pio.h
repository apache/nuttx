/****************************************************************************************
 * arch/arm/src/sam34/hardware/sam3u_pio.h
 * Parallel Input/Output (PIO) Controller definitions for the SAM3U, SAM3X, and SAM3A.
 *
 *   Copyright (C) 2009, 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAM34_HARDWARE_SAM3U_PIO_H
#define __ARCH_ARM_SRC_SAM34_HARDWARE_SAM3U_PIO_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

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
#define SAM_PIO_ABSR_OFFSET        0x0070 /* Peripheral AB Select Register */
                                          /* 0x0074-0x007c: Reserved */
#define SAM_PIO_SCIFSR_OFFSET      0x0080 /* System Clock Glitch Input Filter Select Register */
#define SAM_PIO_DIFSR_OFFSET       0x0084 /* Debouncing Input Filter Select Register */
#define SAM_PIO_IFDGSR_OFFSET      0x0088 /* Glitch or Debouncing Input Filter Clock Selection Status Register */
#define SAM_PIO_SCDR_OFFSET        0x008c /* Slow Clock Divider Debouncing Register */
                                          /* 0x0090-0x009c: Reserved */
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
                                          /* 0x0100-0x0144: Reserved */

/* PIO register addresses ***************************************************************/

#define PIOA                       (0)
#define PIOB                       (1)
#define PIOC                       (2)
#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A)
#  define PIOD                     (3)
#  define PIOE                     (4)
#  define PIOF                     (5)
#  define NPIO                     (6)
#else
#  define NPIO                     (3)
#endif

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
#define SAM_PIO_ABSR(n)            (SAM_PIO_BASE(n)+SAM_PIO_ABSR_OFFSET)
#define SAM_PIO_SCIFSR(n)          (SAM_PIO_BASE(n)+SAM_PIO_SCIFSR_OFFSET)
#define SAM_PIO_DIFSR(n)           (SAM_PIO_BASE(n)+SAM_PIO_DIFSR_OFFSET)
#define SAM_PIO_IFDGSR(n)          (SAM_PIO_BASE(n)+SAM_PIO_IFDGSR_OFFSET)
#define SAM_PIO_SCDR(n)            (SAM_PIO_BASE(n)+SAM_PIO_SCDR_OFFSET)
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
#define SAM_PIOA_ABSR              (SAM_PIOA_BASE+SAM_PIO_ABSR_OFFSET)
#define SAM_PIOA_SCIFSR            (SAM_PIOA_BASE+SAM_PIO_SCIFSR_OFFSET)
#define SAM_PIOA_DIFSR             (SAM_PIOA_BASE+SAM_PIO_DIFSR_OFFSET)
#define SAM_PIOA_IFDGSR            (SAM_PIOA_BASE+SAM_PIO_IFDGSR_OFFSET)
#define SAM_PIOA_SCDR              (SAM_PIOA_BASE+SAM_PIO_SCDR_OFFSET)
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
#define SAM_PIOB_ABSR              (SAM_PIOB_BASE+SAM_PIO_ABSR_OFFSET)
#define SAM_PIOB_SCIFSR            (SAM_PIOB_BASE+SAM_PIO_SCIFSR_OFFSET)
#define SAM_PIOB_DIFSR             (SAM_PIOB_BASE+SAM_PIO_DIFSR_OFFSET)
#define SAM_PIOB_IFDGSR            (SAM_PIOB_BASE+SAM_PIO_IFDGSR_OFFSET)
#define SAM_PIOB_SCDR              (SAM_PIOB_BASE+SAM_PIO_SCDR_OFFSET)
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
#define SAM_PIOC_ABSR              (SAM_PIOC_BASE+SAM_PIO_ABSR_OFFSET)
#define SAM_PIOC_SCIFSR            (SAM_PIOC_BASE+SAM_PIO_SCIFSR_OFFSET)
#define SAM_PIOC_DIFSR             (SAM_PIOC_BASE+SAM_PIO_DIFSR_OFFSET)
#define SAM_PIOC_IFDGSR            (SAM_PIOC_BASE+SAM_PIO_IFDGSR_OFFSET)
#define SAM_PIOC_SCDR              (SAM_PIOC_BASE+SAM_PIO_SCDR_OFFSET)
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

#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SAM_PIOD_PER             (SAM_PIOD_BASE+SAM_PIO_PER_OFFSET)
#  define SAM_PIOD_PDR             (SAM_PIOD_BASE+SAM_PIO_PDR_OFFSET)
#  define SAM_PIOD_PSR             (SAM_PIOD_BASE+SAM_PIO_PSR_OFFSET)
#  define SAM_PIOD_OER             (SAM_PIOD_BASE+SAM_PIO_OER_OFFSET)
#  define SAM_PIOD_ODR             (SAM_PIOD_BASE+SAM_PIO_ODR_OFFSET)
#  define SAM_PIOD_OSR             (SAM_PIOD_BASE+SAM_PIO_OSR_OFFSET)
#  define SAM_PIOD_IFER            (SAM_PIOD_BASE+SAM_PIO_IFER_OFFSET)
#  define SAM_PIOD_IFDR            (SAM_PIOD_BASE+SAM_PIO_IFDR_OFFSET)
#  define SAM_PIOD_IFSR            (SAM_PIOD_BASE+SAM_PIO_IFSR_OFFSET)
#  define SAM_PIOD_SODR            (SAM_PIOD_BASE+SAM_PIO_SODR_OFFSET)
#  define SAM_PIOD_CODR            (SAM_PIOD_BASE+SAM_PIO_CODR_OFFSET)
#  define SAM_PIOD_ODSR            (SAM_PIOD_BASE+SAM_PIO_ODSR_OFFSET)
#  define SAM_PIOD_PDSR            (SAM_PIOD_BASE+SAM_PIO_PDSR_OFFSET)
#  define SAM_PIOD_IER             (SAM_PIOD_BASE+SAM_PIO_IER_OFFSET)
#  define SAM_PIOD_IDR             (SAM_PIOD_BASE+SAM_PIO_IDR_OFFSET)
#  define SAM_PIOD_IMR             (SAM_PIOD_BASE+SAM_PIO_IMR_OFFSET)
#  define SAM_PIOD_ISR             (SAM_PIOD_BASE+SAM_PIO_ISR_OFFSET)
#  define SAM_PIOD_MDER            (SAM_PIOD_BASE+SAM_PIO_MDER_OFFSET)
#  define SAM_PIOD_MDDR            (SAM_PIOD_BASE+SAM_PIO_MDDR_OFFSET)
#  define SAM_PIOD_MDSR            (SAM_PIOD_BASE+SAM_PIO_MDSR_OFFSET)
#  define SAM_PIOD_PUDR            (SAM_PIOD_BASE+SAM_PIO_PUDR_OFFSET)
#  define SAM_PIOD_PUER            (SAM_PIOD_BASE+SAM_PIO_PUER_OFFSET)
#  define SAM_PIOD_PUSR            (SAM_PIOD_BASE+SAM_PIO_PUSR_OFFSET)
#  define SAM_PIOD_ABSR            (SAM_PIOD_BASE+SAM_PIO_ABSR_OFFSET)
#  define SAM_PIOD_SCIFSR          (SAM_PIOD_BASE+SAM_PIO_SCIFSR_OFFSET)
#  define SAM_PIOD_DIFSR           (SAM_PIOD_BASE+SAM_PIO_DIFSR_OFFSET)
#  define SAM_PIOD_IFDGSR          (SAM_PIOD_BASE+SAM_PIO_IFDGSR_OFFSET)
#  define SAM_PIOD_SCDR            (SAM_PIOD_BASE+SAM_PIO_SCDR_OFFSET)
#  define SAM_PIOD_OWER            (SAM_PIOD_BASE+SAM_PIO_OWER_OFFSET)
#  define SAM_PIOD_OWDR            (SAM_PIOD_BASE+SAM_PIO_OWDR_OFFSET)
#  define SAM_PIOD_OWSR            (SAM_PIOD_BASE+SAM_PIO_OWSR_OFFSET)
#  define SAM_PIOD_AIMER           (SAM_PIOD_BASE+SAM_PIO_AIMER_OFFSET)
#  define SAM_PIOD_AIMDR           (SAM_PIOD_BASE+SAM_PIO_AIMDR_OFFSET)
#  define SAM_PIOD_AIMMR           (SAM_PIOD_BASE+SAM_PIO_AIMMR_OFFSET)
#  define SAM_PIOD_ESR             (SAM_PIOD_BASE+SAM_PIO_ESR_OFFSET)
#  define SAM_PIOD_LSR             (SAM_PIOD_BASE+SAM_PIO_LSR_OFFSET)
#  define SAM_PIOD_ELSR            (SAM_PIOD_BASE+SAM_PIO_ELSR_OFFSET)
#  define SAM_PIOD_FELLSR          (SAM_PIOD_BASE+SAM_PIO_FELLSR_OFFSET)
#  define SAM_PIOD_REHLSR          (SAM_PIOD_BASE+SAM_PIO_REHLSR_OFFSET)
#  define SAM_PIOD_FRLHSR          (SAM_PIOD_BASE+SAM_PIO_FRLHSR_OFFSET)
#  define SAM_PIOD_LOCKSR          (SAM_PIOD_BASE+SAM_PIO_LOCKSR_OFFSET)
#  define SAM_PIOD_WPMR            (SAM_PIOD_BASE+SAM_PIO_WPMR_OFFSET)
#  define SAM_PIOD_WPSR            (SAM_PIOD_BASE+SAM_PIO_WPSR_OFFSET)

#  define SAM_PIOE_PER             (SAM_PIOE_BASE+SAM_PIO_PER_OFFSET)
#  define SAM_PIOE_PDR             (SAM_PIOE_BASE+SAM_PIO_PDR_OFFSET)
#  define SAM_PIOE_PSR             (SAM_PIOE_BASE+SAM_PIO_PSR_OFFSET)
#  define SAM_PIOE_OER             (SAM_PIOE_BASE+SAM_PIO_OER_OFFSET)
#  define SAM_PIOE_ODR             (SAM_PIOE_BASE+SAM_PIO_ODR_OFFSET)
#  define SAM_PIOE_OSR             (SAM_PIOE_BASE+SAM_PIO_OSR_OFFSET)
#  define SAM_PIOE_IFER            (SAM_PIOE_BASE+SAM_PIO_IFER_OFFSET)
#  define SAM_PIOE_IFDR            (SAM_PIOE_BASE+SAM_PIO_IFDR_OFFSET)
#  define SAM_PIOE_IFSR            (SAM_PIOE_BASE+SAM_PIO_IFSR_OFFSET)
#  define SAM_PIOE_SODR            (SAM_PIOE_BASE+SAM_PIO_SODR_OFFSET)
#  define SAM_PIOE_CODR            (SAM_PIOE_BASE+SAM_PIO_CODR_OFFSET)
#  define SAM_PIOE_ODSR            (SAM_PIOE_BASE+SAM_PIO_ODSR_OFFSET)
#  define SAM_PIOE_PDSR            (SAM_PIOE_BASE+SAM_PIO_PDSR_OFFSET)
#  define SAM_PIOE_IER             (SAM_PIOE_BASE+SAM_PIO_IER_OFFSET)
#  define SAM_PIOE_IDR             (SAM_PIOE_BASE+SAM_PIO_IDR_OFFSET)
#  define SAM_PIOE_IMR             (SAM_PIOE_BASE+SAM_PIO_IMR_OFFSET)
#  define SAM_PIOE_ISR             (SAM_PIOE_BASE+SAM_PIO_ISR_OFFSET)
#  define SAM_PIOE_MDER            (SAM_PIOE_BASE+SAM_PIO_MDER_OFFSET)
#  define SAM_PIOE_MDDR            (SAM_PIOE_BASE+SAM_PIO_MDDR_OFFSET)
#  define SAM_PIOE_MDSR            (SAM_PIOE_BASE+SAM_PIO_MDSR_OFFSET)
#  define SAM_PIOE_PUDR            (SAM_PIOE_BASE+SAM_PIO_PUDR_OFFSET)
#  define SAM_PIOE_PUER            (SAM_PIOE_BASE+SAM_PIO_PUER_OFFSET)
#  define SAM_PIOE_PUSR            (SAM_PIOE_BASE+SAM_PIO_PUSR_OFFSET)
#  define SAM_PIOE_ABSR            (SAM_PIOE_BASE+SAM_PIO_ABSR_OFFSET)
#  define SAM_PIOE_SCIFSR          (SAM_PIOE_BASE+SAM_PIO_SCIFSR_OFFSET)
#  define SAM_PIOE_DIFSR           (SAM_PIOE_BASE+SAM_PIO_DIFSR_OFFSET)
#  define SAM_PIOE_IFDGSR          (SAM_PIOE_BASE+SAM_PIO_IFDGSR_OFFSET)
#  define SAM_PIOE_SCDR            (SAM_PIOE_BASE+SAM_PIO_SCDR_OFFSET)
#  define SAM_PIOE_OWER            (SAM_PIOE_BASE+SAM_PIO_OWER_OFFSET)
#  define SAM_PIOE_OWDR            (SAM_PIOE_BASE+SAM_PIO_OWDR_OFFSET)
#  define SAM_PIOE_OWSR            (SAM_PIOE_BASE+SAM_PIO_OWSR_OFFSET)
#  define SAM_PIOE_AIMER           (SAM_PIOE_BASE+SAM_PIO_AIMER_OFFSET)
#  define SAM_PIOE_AIMDR           (SAM_PIOE_BASE+SAM_PIO_AIMDR_OFFSET)
#  define SAM_PIOE_AIMMR           (SAM_PIOE_BASE+SAM_PIO_AIMMR_OFFSET)
#  define SAM_PIOE_ESR             (SAM_PIOE_BASE+SAM_PIO_ESR_OFFSET)
#  define SAM_PIOE_LSR             (SAM_PIOE_BASE+SAM_PIO_LSR_OFFSET)
#  define SAM_PIOE_ELSR            (SAM_PIOE_BASE+SAM_PIO_ELSR_OFFSET)
#  define SAM_PIOE_FELLSR          (SAM_PIOE_BASE+SAM_PIO_FELLSR_OFFSET)
#  define SAM_PIOE_REHLSR          (SAM_PIOE_BASE+SAM_PIO_REHLSR_OFFSET)
#  define SAM_PIOE_FRLHSR          (SAM_PIOE_BASE+SAM_PIO_FRLHSR_OFFSET)
#  define SAM_PIOE_LOCKSR          (SAM_PIOE_BASE+SAM_PIO_LOCKSR_OFFSET)
#  define SAM_PIOE_WPMR            (SAM_PIOE_BASE+SAM_PIO_WPMR_OFFSET)
#  define SAM_PIOE_WPSR            (SAM_PIOE_BASE+SAM_PIO_WPSR_OFFSET)

#  define SAM_PIOF_PER             (SAM_PIOF_BASE+SAM_PIO_PER_OFFSET)
#  define SAM_PIOF_PDR             (SAM_PIOF_BASE+SAM_PIO_PDR_OFFSET)
#  define SAM_PIOF_PSR             (SAM_PIOF_BASE+SAM_PIO_PSR_OFFSET)
#  define SAM_PIOF_OER             (SAM_PIOF_BASE+SAM_PIO_OER_OFFSET)
#  define SAM_PIOF_ODR             (SAM_PIOF_BASE+SAM_PIO_ODR_OFFSET)
#  define SAM_PIOF_OSR             (SAM_PIOF_BASE+SAM_PIO_OSR_OFFSET)
#  define SAM_PIOF_IFER            (SAM_PIOF_BASE+SAM_PIO_IFER_OFFSET)
#  define SAM_PIOF_IFDR            (SAM_PIOF_BASE+SAM_PIO_IFDR_OFFSET)
#  define SAM_PIOF_IFSR            (SAM_PIOF_BASE+SAM_PIO_IFSR_OFFSET)
#  define SAM_PIOF_SODR            (SAM_PIOF_BASE+SAM_PIO_SODR_OFFSET)
#  define SAM_PIOF_CODR            (SAM_PIOF_BASE+SAM_PIO_CODR_OFFSET)
#  define SAM_PIOF_ODSR            (SAM_PIOF_BASE+SAM_PIO_ODSR_OFFSET)
#  define SAM_PIOF_PDSR            (SAM_PIOF_BASE+SAM_PIO_PDSR_OFFSET)
#  define SAM_PIOF_IER             (SAM_PIOF_BASE+SAM_PIO_IER_OFFSET)
#  define SAM_PIOF_IDR             (SAM_PIOF_BASE+SAM_PIO_IDR_OFFSET)
#  define SAM_PIOF_IMR             (SAM_PIOF_BASE+SAM_PIO_IMR_OFFSET)
#  define SAM_PIOF_ISR             (SAM_PIOF_BASE+SAM_PIO_ISR_OFFSET)
#  define SAM_PIOF_MDER            (SAM_PIOF_BASE+SAM_PIO_MDER_OFFSET)
#  define SAM_PIOF_MDDR            (SAM_PIOF_BASE+SAM_PIO_MDDR_OFFSET)
#  define SAM_PIOF_MDSR            (SAM_PIOF_BASE+SAM_PIO_MDSR_OFFSET)
#  define SAM_PIOF_PUDR            (SAM_PIOF_BASE+SAM_PIO_PUDR_OFFSET)
#  define SAM_PIOF_PUER            (SAM_PIOF_BASE+SAM_PIO_PUER_OFFSET)
#  define SAM_PIOF_PUSR            (SAM_PIOF_BASE+SAM_PIO_PUSR_OFFSET)
#  define SAM_PIOF_ABSR            (SAM_PIOF_BASE+SAM_PIO_ABSR_OFFSET)
#  define SAM_PIOF_SCIFSR          (SAM_PIOF_BASE+SAM_PIO_SCIFSR_OFFSET)
#  define SAM_PIOF_DIFSR           (SAM_PIOF_BASE+SAM_PIO_DIFSR_OFFSET)
#  define SAM_PIOF_IFDGSR          (SAM_PIOF_BASE+SAM_PIO_IFDGSR_OFFSET)
#  define SAM_PIOF_SCDR            (SAM_PIOF_BASE+SAM_PIO_SCDR_OFFSET)
#  define SAM_PIOF_OWER            (SAM_PIOF_BASE+SAM_PIO_OWER_OFFSET)
#  define SAM_PIOF_OWDR            (SAM_PIOF_BASE+SAM_PIO_OWDR_OFFSET)
#  define SAM_PIOF_OWSR            (SAM_PIOF_BASE+SAM_PIO_OWSR_OFFSET)
#  define SAM_PIOF_AIMER           (SAM_PIOF_BASE+SAM_PIO_AIMER_OFFSET)
#  define SAM_PIOF_AIMDR           (SAM_PIOF_BASE+SAM_PIO_AIMDR_OFFSET)
#  define SAM_PIOF_AIMMR           (SAM_PIOF_BASE+SAM_PIO_AIMMR_OFFSET)
#  define SAM_PIOF_ESR             (SAM_PIOF_BASE+SAM_PIO_ESR_OFFSET)
#  define SAM_PIOF_LSR             (SAM_PIOF_BASE+SAM_PIO_LSR_OFFSET)
#  define SAM_PIOF_ELSR            (SAM_PIOF_BASE+SAM_PIO_ELSR_OFFSET)
#  define SAM_PIOF_FELLSR          (SAM_PIOF_BASE+SAM_PIO_FELLSR_OFFSET)
#  define SAM_PIOF_REHLSR          (SAM_PIOF_BASE+SAM_PIO_REHLSR_OFFSET)
#  define SAM_PIOF_FRLHSR          (SAM_PIOF_BASE+SAM_PIO_FRLHSR_OFFSET)
#  define SAM_PIOF_LOCKSR          (SAM_PIOF_BASE+SAM_PIO_LOCKSR_OFFSET)
#  define SAM_PIOF_WPMR            (SAM_PIOF_BASE+SAM_PIO_WPMR_OFFSET)
#  define SAM_PIOF_WPSR            (SAM_PIOF_BASE+SAM_PIO_WPSR_OFFSET)
#endif

/* PIO register bit definitions *********************************************************/

/* Common bit definitions for ALMOST all IO registers (exceptions follow) */

#define PIO(n)                     (1<<(n)) /* Bit n: PIO n */

/* PIO Write Protect Mode Register */

#define PIO_WPMR_WPEN              (1 << 0)  /* Bit 0:  Write Protect Enable */
#define PIO_WPMR_WPKEY_SHIFT       (8)       /* Bits 8-31: Write Protect KEY */
#define PIO_WPMR_WPKEY_MASK        (0xffffff << PIO_WPMR_WPKEY_SHIFT)
#  define PIO_WPMR_WPKEY           (0x50494f << PIO_WPMR_WPKEY_SHIFT)

/* PIO Write Protect Status Register */

#define PIO_WPSR_WPVS              (1 << 0)  /* Bit 0:  Write Protect Violation Status */
#define PIO_WPSR_WPVSRC_SHIFT      (8)       /* Bits 8-23: Write Protect Violation Source */
#define PIO_WPSR_WPVSRC_MASK       (0xffff << PIO_WPSR_WPVSRC_SHIFT)

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_HARDWARE_SAM3U_PIO_H */
