/****************************************************************************
 * arch/arm/src/s32k1xx/hardware/s32k1xx_smc.h
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

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_SMC_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_SMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <hardware/s32k1xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SMC Register Offsets *****************************************************/

#define S32K1XX_SMC_VERID_OFFSET    0x0000  /* SMC Version ID Register */
#define S32K1XX_SMC_PARAM_OFFSET    0x0004  /* SMC Parameter Register */
#define S32K1XX_SMC_PMPROT_OFFSET   0x0008  /* SMC Power Mode Protection register */
#define S32K1XX_SMC_PMCTRL_OFFSET   0x000c  /* SMC Power Mode Control register */
#define S32K1XX_SMC_STOPCTRL_OFFSET 0x0010  /* SMC Stop Control Register */
#define S32K1XX_SMC_PMSTAT_OFFSET   0x0014  /* SMC Power Mode Status register */

/* SMC Register Addresses ***************************************************/

#define S32K1XX_SMC_VERID           (S32K1XX_SMC_BASE + S32K1XX_SMC_VERID_OFFSET)
#define S32K1XX_SMC_PARAM           (S32K1XX_SMC_BASE + S32K1XX_SMC_PARAM_OFFSET)
#define S32K1XX_SMC_PMPROT          (S32K1XX_SMC_BASE + S32K1XX_SMC_PMPROT_OFFSET)
#define S32K1XX_SMC_PMCTRL          (S32K1XX_SMC_BASE + S32K1XX_SMC_PMCTRL_OFFSET)
#define S32K1XX_SMC_STOPCTRL        (S32K1XX_SMC_BASE + S32K1XX_SMC_STOPCTRL_OFFSET)
#define S32K1XX_SMC_PMSTAT          (S32K1XX_SMC_BASE + S32K1XX_SMC_PMSTAT_OFFSET)

/* SMC Register Bitfield Definitions ****************************************/

/* SMC Version ID Register */

#define SMC_VERID_FEATURE_SHIFT     (0)                            /* Bits 0-15: Feature Identification Number */
#define SMC_VERID_FEATURE_MASK      (0xffff << SMC_VERID_FEATURE_SHIFT)
#  define SMC_VERID_FEATURE_STD     (1 << SMC_VERID_FEATURE_SHIFT) /* Standard feature set */
#define SMC_VERID_MINOR_SHIFT       (16)                           /* Bits 16-23: Minor Version Number */
#define SMC_VERID_MINOR_MASK        (0xff << SMC_VERID_MINOR_SHIFT)
#define SMC_VERID_MAJOR_SHIFT       (24)                           /* Bits 24-31: Major Version Number */
#define SMC_VERID_MAJOR_MASK        (0xff << SMC_VERID_MAJOR_SHIFT)

/* SMC Parameter Register */

#define SMC_PARAM_EHSRUN            (1 << 0)  /* Bit 0:  Existence of HSRUN feature */
#define SMC_PARAM_ELLS              (1 << 3)  /* Bit 3:  Existence of LLS feature */
#define SMC_PARAM_ELLS2             (1 << 5)  /* Bit 5:  Existence of LLS2 feature */
#define SMC_PARAM_EVLLS0            (1 << 6)  /* Bit 6:  Existence of VLLS0 feature */

/* SMC Power Mode Protection register */

#define SMC_PMPROT_AVLP_SHIFT       (5)  /* Bit 5:  Allow Very-Low-Power Modes */
#define SMC_PMPROT_AVLP             (1 << SMC_PMPROT_AVLP_SHIFT)  
#define SMC_PMPROT_AHSRUN_SHIFT     (7)  /* Bit 7:  Allow High Speed Run mode */
#define SMC_PMPROT_AHSRUN           (1 << SMC_PMPROT_AHSRUN_SHIFT)  

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
