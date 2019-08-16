/*********************************************************************************************
 * arch/arm/src/s32k1xx/chip/s32k1xx_pmc.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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
 *********************************************************************************************/

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_PMC_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_PMC_H

/*********************************************************************************************
 * Included Files
 *********************************************************************************************/

#include <nuttx/config.h>
#include <hardware/s32k1xx_memorymap.h>

/*********************************************************************************************
 * Pre-processor Definitions
 *********************************************************************************************/

/* PMC Register Offsets **********************************************************************/

#define S32K1XX_PMC_LVDSC1_OFFSET   0x0000  /* Low Voltage Detect Status and Control 1 Register */
#define S32K1XX_PMC_LVDSC2_OFFSET   0x0001  /* Low Voltage Detect Status and Control 2 Register */
#define S32K1XX_PMC_REGSC_OFFSET    0x0002  /* Regulator Status and Control Register */
#define S32K1XX_PMC_LPOTRIM_OFFSET  0x0004  /* Low Power Oscillator Trim Register */

/* PMC Register Addresses ********************************************************************/

#define S32K1XX_PMC_LVDSC1          (S32K1XX_PMC_BASE + S32K1XX_PMC_LVDSC1_OFFSET)
#define S32K1XX_PMC_LVDSC2          (S32K1XX_PMC_BASE + S32K1XX_PMC_LVDSC2_OFFSET)
#define S32K1XX_PMC_REGSC           (S32K1XX_PMC_BASE + S32K1XX_PMC_REGSC_OFFSET)
#define S32K1XX_PMC_LPOTRIM         (S32K1XX_PMC_BASE + S32K1XX_PMC_LPOTRIM_OFFSET)

/* PMC Register Bitfield Definitions *********************************************************/

/* Low Voltage Detect Status and Control 1 Register */

#define PMC_LVDSC1_LVDRE            (1 << 4)  /* Bit 4:  Low Voltage Detect Reset Enable */
#define PMC_LVDSC1_LVDIE            (1 << 5)  /* Bit 5:  Low Voltage Detect Interrupt Enable */
#define PMC_LVDSC1_LVDACK           (1 << 6)  /* Bit 6:  Low Voltage Detect Acknowledge */
#define PMC_LVDSC1_LVDF             (1 << 7)  /* Bit 7:  Low Voltage Detect Flag */

/* Low Voltage Detect Status and Control 2 Register */

#define PMC_LVDSC2_LVWIE            (1 << 5)  /* Bit 5:  Low-Voltage Warning Interrupt Enable */
#define PMC_LVDSC2_LVWACK           (1 << 6)  /* Bit 6:  Low-Voltage Warning Acknowledge */
#define PMC_LVDSC2_LVWF             (1 << 7)  /* Bit 7:  Low-Voltage Warning Flag */

/* Regulator Status and Control Register */

#define PMC_REGSC_CLKBIASDIS        (1 << 1)  /* Bit 1:  Clock Bias Disable Bit */
#define PMC_REGSC_REGFPM            (1 << 2)  /* Bit 2:  Regulator in Full Performance Mode Status Bit */
#define PMC_REGSC_LPOSTAT           (1 << 6)  /* Bit 6:  LPO Status Bit */
#define PMC_REGSC_LPODIS            (1 << 7)  /* Bit 7:  LPO Disable Bit */

/* Low Power Oscillator Trim Register */

#define PMC_LPOTRIM_MASK            0x0f      /* Bits 0-3:  LPOCLK trim value */
#  define PMC_LPOTRIM_LOWEST        0x80      /*            Lowest value -16 */
#  define PMC_LPOTRIM_TYPICAL       0x00      /*            Typical 0 (128 kHz) */
#  define PMC_LPOTRIM_HIGEST        0x7f      /*            Highest value 15 */

#endif /* __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_PMC_H */
