/********************************************************************************************
 * arch/arm/src/kinetis/hardware/kinetis_pmc.h
 *
 *   Copyright (C) 2011, 2016, 2018 Gregory Nutt. All rights reserved.
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_PMC_H
#define __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_PMC_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* Register Offsets *************************************************************************/

#define KINETIS_PMC_LVDSC1_OFFSET     0x0000 /* Low Voltage Detect Status and Control 1 Register */
#define KINETIS_PMC_LVDSC2_OFFSET     0x0001 /* Low Voltage Detect Status and Control 2 Register */
#define KINETIS_PMC_REGSC_OFFSET      0x0002 /* Regulator Status and Control Register */
#if defined(KINETIS_PMC_HAS_HVDSC1)
#  define KINETIS_PMC_HVDSC1_OFFSET   0x000b /* High Voltage Detect Status And Control 1 register */
#endif
#if defined(KINETIS_PMC_HAS_SRAMCTL)
#  define KINETIS_PMC_SRAMCTL_OFFSET  0x000c /* SRAM VLLS2 Control register */
#endif

/* Register Addresses ***********************************************************************/

#define KINETIS_PMC_LVDSC1            (KINETIS_PMC_BASE + KINETIS_PMC_LVDSC1_OFFSET)
#define KINETIS_PMC_LVDSC2            (KINETIS_PMC_BASE + KINETIS_PMC_LVDSC2_OFFSET)
#define KINETIS_PMC_REGSC             (KINETIS_PMC_BASE + KINETIS_PMC_REGSC_OFFSET)
#if defined(KINETIS_PMC_HAS_HVDSC1)
#  define KINETIS_PMC_HVDSC1          (KINETIS_PMC_BASE + KINETIS_PMC_HVDSC1_OFFSET)
#endif
#if defined(KINETIS_PMC_HAS_SRAMCTL)
#  define KINETIS_PMC_SRAMCTL         (KINETIS_PMC_BASE + KINETIS_PMC_SRAMCTL_OFFSET)
#endif

/* Register Bit Definitions *****************************************************************/

/* Low Voltage Detect Status and Control 1 Register */

#define PMC_LVDSC1_LVDV_SHIFT         (0)       /* Bits 0-1: Low-Voltage Detect Voltage Select */
#define PMC_LVDSC1_LVDV_MASK          (3 << PMC_LVDSC1_LVDV_SHIFT)
#  define PMC_LVDSC1_LVDV_LOW         (0 << PMC_LVDSC1_LVDV_SHIFT) /* Low trip point selected (VLVD = VLVDL) */
#  define PMC_LVDSC1_LVDV_HIGH        (1 << PMC_LVDSC1_LVDV_SHIFT) /* High trip point selected (VLVD = VLVDH) */
                                                /* Bits 2-3: Reserved */
#define PMC_LVDSC1_LVDRE              (1 << 4)  /* Bit 4:  Low-Voltage Detect Reset Enable */
#define PMC_LVDSC1_LVDIE              (1 << 5)  /* Bit 5:  Low-Voltage Detect Interrupt Enable */
#define PMC_LVDSC1_LVDACK             (1 << 6)  /* Bit 6:  Low-Voltage Detect Acknowledge */
#define PMC_LVDSC1_LVDF               (1 << 7)  /* Bit 7:  Low-Voltage Detect Flag */

/* Low Voltage Detect Status and Control 2 Register */

#define PMC_LVDSC2_LVWV_SHIFT         (0)       /* Bits 0-1: Low-Voltage Warning Voltage Select */
#define PMC_LVDSC2_LVWV_MASK          (3 << PMC_LVDSC2_LVWV_SHIFT)
#  define PMC_LVDSC2_LVWV_LOW         (0 << PMC_LVDSC2_LVWV_SHIFT) /* Low trip point selected (VLVW = VLVW1H/L) */
#  define PMC_LVDSC2_LVWV_MID1        (1 << PMC_LVDSC2_LVWV_SHIFT) /* Mid 1 trip point selected (VLVW = VLVW2H/L) */
#  define PMC_LVDSC2_LVWV_MID2        (2 << PMC_LVDSC2_LVWV_SHIFT) /* Mid 2 trip point selected (VLVW = VLVW3H/L) */
#  define PMC_LVDSC2_LVWV_HIGH        (3 << PMC_LVDSC2_LVWV_SHIFT) /* High trip point selected (VLVW = VLVW4H/L) */
                                              /* Bits 2-4: Reserved */
#define PMC_LVDSC2_LVWIE              (1 << 5)  /* Bit 5:  Low-Voltage Warning Interrupt Enable */
#define PMC_LVDSC2_LVWACK             (1 << 6)  /* Bit 6:  Low-Voltage Warning Acknowledge */
#define PMC_LVDSC2_LVWF               (1 << 7)  /* Bit 7:  Low-Voltage Warning Flag */

/* Regulator Status and Control Register */

#define PMC_REGSC_BGBE                (1 << 0)  /* Bit 0:  Bandgap Buffer Enable */
                                                /* Bit 1: Reserved */
#if defined(KINETIS_PMC_HAS_REGSC_REGONS)
#  define PMC_REGSC_REGONS            (1 << 2)  /* Bit 2:  Regulator in Run Regulation Status */
#endif
#if defined(KINETIS_PMC_HAS_REGSC_ACKISO)
#  define PMC_REGSC_ACKISO            (1 << 3)  /* Bit 3:  Acknowledge Isolation */
#endif
#if defined(KINETIS_PMC_HAS_REGSC_VLPRS)
#  define PMC_REGSC_VLPRS             (1 << 3)  /* Bit 3:  Very Low Power Run Status */
#endif
#if defined(KINETIS_PMC_HAS_REGSC_BGEN)
#  define PMC_REGSC_BGEN              (1 << 4)  /* Bit 4:  Bandgap Enable In VLPx Operation */
#endif
#if defined(KINETIS_PMC_HAS_REGSC_TRAMPO)
#  define PMC_REGSC_TRAMPO            (1 << 4)  /* Bit 4:  For devices with FlexNVM: Traditional RAM Power Option */
#endif
                                                /* Bits 5-7: Reserved */

/* High Voltage Detect Status And Control 1 register */

#if defined(KINETIS_PMC_HAS_HVDSC1)
#  define PMC_HVDSC1_HVDV             (1 << 0)  /* Bit 0:  High-Voltage Detect Voltage Select */
                                                /* Bits 1-3: Reserved */
#  define PMC_HVDSC1_HVDRE            (1 << 4)  /* Bit 4:  High-Voltage Detect Reset Enable */
#  define PMC_HVDSC1_HVDIE            (1 << 5)  /* Bit 5:  High-Voltage Detect Interrupt Enable */
#  define PMC_HVDSC1_HVDACK           (1 << 6)  /* Bit 6:  High-Voltage Detect Acknowledge */
#  define PMC_HVDSC1_HVDF             (1 << 7)  /* Bit 7:  High-Voltage Detect Flag */
#endif

/* SRAM VLLS2 Control register */

#if defined(KINETIS_PMC_HAS_SRAMCTL)
#  define PMC_SRAMCTL_MASK            0xff
#  define PMC_SRAMCTL_VLLS2PD         (1 << (n)) /* Bits 0-7: SRAM VLLS2 Powerdown */
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_PMC_H */
