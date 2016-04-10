/************************************************************************************
 * arch/arm/src/lpc11xx/chip/lpc11_pmu.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC11XX_CHIP_LPC11_PMU_H
#define __ARCH_ARM_SRC_LPC11XX_CHIP_LPC11_PMU_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/lpc11_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define LPC11_PMU_PCON_OFFSET   0x0000  /* Power control register */
#define LPC11_PMU_GPREG0        0x0004  /* General purpose register 0 */
#define LPC11_PMU_GPREG1        0x0008  /* General purpose register 1 */
#define LPC11_PMU_GPREG2        0x000C  /* General purpose register 2 */
#define LPC11_PMU_GPREG3        0x0010  /* General purpose register 3 */
#define LPC11_PMU_GPREG4        0x0014  /* General purpose register 0 */

/* Register addresses ***************************************************************/

#define LPC11_PMU_PCON          (LPC11_PMU_BASE+LPC11_PMU_PCON_OFFSET)
#define LPC11_PMU_GPREG0        (LPC11_PMU_BASE+LPC11_PMU_GPREG0)
#define LPC11_PMU_GPREG1        (LPC11_PMU_BASE+LPC11_PMU_GPREG1)
#define LPC11_PMU_GPREG2        (LPC11_PMU_BASE+LPC11_PMU_GPREG2)
#define LPC11_PMU_GPREG3        (LPC11_PMU_BASE+LPC11_PMU_GPREG3)
#define LPC11_PMU_GPREG4        (LPC11_PMU_BASE+LPC11_PMU_GPREG4)

/* Register bit definitions *********************************************************/

/* Power control register */
                                          /* Bit 0: Reserved. Do not write 1 to this bit */
#define PMU_PCON_DPDEN          (1 << 1)  /* Deep power-down mode enable */
                                          /* Bits 2-7: Reserved. Do not write ones to this bit */
#define PMU_PCON_SLEEPFLAG      (1 << 8)  /* Sleep mode flag */
                                          /* Bits 9-10: Reserved. Do not write ones to this bit */
#define PMU_PCON_DPDFLAG        (1 << 11) /* Deep power-down flag. */
                                          /* Bits 12-31: Reserved. Do not write ones to this bit */


/* General Purpose REG */

#define PMU_GPREG03_GPDATA_MASK (0xffffffff)  /* Bits 0-31: Data retained during Deep power-down mode */


/* General Purpose REG4 Register */

                                            /* Bits 0-9: Reserved. Do not write ones to this bit */
#define PMU_GPREG4_WAKEUPHYS    (1 << 10) /* WAKEUP pin hysteresis enable */
#define PMU_GPREG4_GPDATA_SHIFT 11        /* Data retained during Deep power-down mode. */
#define PMU_GPREG4_GPDATA_MASK  (0x1fffff << PMU_GPREG4_GPDATA_SHIFT)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC11XX_CHIP_LPC11_PMU_H */
