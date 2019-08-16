/********************************************************************************************
 * arch/arm/src/s32k1xx/chip/s32k1xx_smc.h
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_SMC_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_SMC_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>
#include <hardware/s32k1xx_memorymap.h>

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* SMC Register Offsets *********************************************************************/

#define S32K1XX_SMC_VERID_OFFSET    0x0000  /* SMC Version ID Register */
#define S32K1XX_SMC_PARAM_OFFSET    0x0004  /* SMC Parameter Register */
#define S32K1XX_SMC_PMPROT_OFFSET   0x0008  /* SMC Power Mode Protection register */
#define S32K1XX_SMC_PMCTRL_OFFSET   0x000c  /* SMC Power Mode Control register */
#define S32K1XX_SMC_STOPCTRL_OFFSET 0x0010  /* SMC Stop Control Register */
#define S32K1XX_SMC_PMSTAT_OFFSET   0x0014  /* SMC Power Mode Status register */

/* SMC Register Addresses *******************************************************************/

#define S32K1XX_SMC_VERID           (S32K1XX_SMC_BASE + S32K1XX_SMC_VERID_OFFSET)
#define S32K1XX_SMC_PARAM           (S32K1XX_SMC_BASE + S32K1XX_SMC_PARAM_OFFSET)
#define S32K1XX_SMC_PMPROT          (S32K1XX_SMC_BASE + S32K1XX_SMC_PMPROT_OFFSET)
#define S32K1XX_SMC_PMCTRL          (S32K1XX_SMC_BASE + S32K1XX_SMC_PMCTRL_OFFSET)
#define S32K1XX_SMC_STOPCTRL        (S32K1XX_SMC_BASE + S32K1XX_SMC_STOPCTRL_OFFSET)
#define S32K1XX_SMC_PMSTAT          (S32K1XX_SMC_BASE + S32K1XX_SMC_PMSTAT_OFFSET)

/* SMC Register Bitfield Definitions ********************************************************/

/* SMC Version ID Register */

#define SMC_VERID_FEATURE_SHIFT     (0)        /* Bits 0-15: Feature Identification Number */
#define SMC_VERID_FEATURE_MASK      (0xffff << SMC_VERID_FEATURE_SHIFT)
#  define SMC_VERID_FEATURE_STD     (1 << SMC_VERID_FEATURE_SHIFT) /* Standard feature set */
#define SMC_VERID_MINOR_SHIFT       (16)       /* Bits 16-23: Minor Version Number */
#define SMC_VERID_MINOR_MASK        (0xff << SMC_VERID_MINOR_SHIFT)
#define SMC_VERID_MAJOR_SHIFT       (24)       /* Bits 24-31: Major Version Number */
#define SMC_VERID_MAJOR_MASK        (0xff << SMC_VERID_MAJOR_SHIFT)


/* SMC Parameter Register */

#define SMC_PARAM_EHSRUN            (1 << 0)  /* Bit 0:  Existence of HSRUN feature */
#define SMC_PARAM_ELLS              (1 << 3)  /* Bit 3:  Existence of LLS feature */
#define SMC_PARAM_ELLS2             (1 << 5)  /* Bit 5:  Existence of LLS2 feature */
#define SMC_PARAM_EVLLS0            (1 << 6)  /* Bit 6:  Existence of VLLS0 feature */

/* SMC Power Mode Protection register */

#define SMC_PMPROT_AVLP             (1 << 5)  /* Bit 5:  Allow Very-Low-Power Modes */
#define SMC_PMPROT_AHSRUN           (1 << 7)  /* Bit 7:  Allow High Speed Run mode */

/* SMC Power Mode Control register */

#define SMC_PMCTRL_STOPM_SHIFT      (0)       /* Bits 0-2: Stop Mode Control */
#define SMC_PMCTRL_STOPM_MASK       (7 << SMC_PMCTRL_STOPM_SHIFT)
#  define SMC_PMCTRL_STOPM_STOP     (0 << SMC_PMCTRL_STOPM_SHIFT) /* Normal Stop */
#  define SMC_PMCTRL_STOPM_VLPS     (2 << SMC_PMCTRL_STOPM_SHIFT) /* Very-Low-Power Stop */
#define SMC_PMCTRL_VLPSA            (1 << 3)  /* Bit 3:  Very Low Power Stop Aborted */
#define SMC_PMCTRL_RUNM_SHIFT       (5)       /* Bits 5-6: Run Mode Control */
#define SMC_PMCTRL_RUNM_MASK        (3 << SMC_PMCTRL_RUNM_SHIFT)
#  define SMC_PMCTRL_RUNM_RUN       (0 << SMC_PMCTRL_RUNM_SHIFT) /* Normal Run mode */
#  define SMC_PMCTRL_RUNM_VLPR      (2 << SMC_PMCTRL_RUNM_SHIFT) /* Very-Low-Power Run mode */
#  define SMC_PMCTRL_RUNM_HSRUN     (3 << SMC_PMCTRL_RUNM_SHIFT) /* High Speed Run mode */

/* SMC Stop Control Register */

#define SMC_STOPCTRL_STOPO_SHIFT    (6)       /* Bits 6-7:  Stop Option */
#define SMC_STOPCTRL_STOPO_MASK     (3 << SMC_STOPCTRL_STOPO_SHIFT)
#  define SMC_STOPCTRL_STOPO_STOP1  (1 << SMC_STOPCTRL_STOPO_SHIFT) /* Stop with systclk+busclk disabled */
#  define SMC_STOPCTRL_STOPO_STOP2  (2 << SMC_STOPCTRL_STOPO_SHIFT) /* Stop with systclk disabled */

/* SMC Power Mode Status register */

#define SMC_PMSTAT_PMSTAT_SHIFT     (0)       /* Bits 0-7: Power Mode Status */
#define SMC_PMSTAT_PMSTAT_MASK      (0xff << SMC_PMSTAT_PMSTAT_SHIFT)
#  define SMC_PMSTAT_PMSTAT_RUN     (0x01 << SMC_PMSTAT_PMSTAT_SHIFT) /* Current power mode is RUN */
#  define SMC_PMSTAT_PMSTAT_VLPR    (0x04 << SMC_PMSTAT_PMSTAT_SHIFT) /* Current power mode is VLPR */
#  define SMC_PMSTAT_PMSTAT_VLPS    (0x10 << SMC_PMSTAT_PMSTAT_SHIFT) /* Current power mode is VLPS */
#  define SMC_PMSTAT_PMSTAT_HSRUN   (0x80 << SMC_PMSTAT_PMSTAT_SHIFT) /* Current power mode is HSRUN */

#endif /* __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_SMC_H */
