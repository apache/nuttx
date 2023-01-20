/****************************************************************************
 * arch/arm/src/stm32/hardware/stm32_adc_v2g4.h
 *
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.  The
 *  ASF licenses this file to you under the Apache License, Version 2.0 (the
 *  "License"); you may not use this file except in compliance with the
 *  License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32_ADC_V2G4_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32_ADC_V2G4_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define HAVE_IP_ADC_V2
#undef HAVE_IP_ADC_V1           /* No ADC IPv1 */
#undef HAVE_ADC_CLOCK_HSI       /* No ADC clock from HSI */
#undef HAVE_ADC_POWERDOWN       /* No ADC power down */
#define HAVE_ADC_VBAT           /* VBAT channel support */
#undef HAVE_BASIC_ADC

/* Base addresses ***********************************************************/

#define STM32_ADC1_OFFSET              0x0000
#define STM32_ADC2_OFFSET              0x0100
#define STM32_ADC3_OFFSET              0x0000
#define STM32_ADC4_OFFSET              0x0100
#define STM32_ADC5_OFFSET              0x0200
#define STM32_ADCCMN_OFFSET            0x0300

#define STM32_ADC1_BASE                (STM32_ADC12_BASE + STM32_ADC1_OFFSET)    /* ADC1 Master ADC */
#define STM32_ADC2_BASE                (STM32_ADC12_BASE + STM32_ADC2_OFFSET)    /* ADC2 Slave ADC */
#define STM32_ADC12CMN_BASE            (STM32_ADC12_BASE + STM32_ADCCMN_OFFSET)  /* ADC1, ADC2 common */
#define STM32_ADC3_BASE                (STM32_ADC345_BASE + STM32_ADC3_OFFSET)   /* ADC3 Master ADC */
#define STM32_ADC4_BASE                (STM32_ADC345_BASE + STM32_ADC4_OFFSET)   /* ADC4 Slave ADC */
#define STM32_ADC5_BASE                (STM32_ADC345_BASE + STM32_ADC5_OFFSET)   /* ADC4 Slave ADC */
#define STM32_ADC345CMN_BASE           (STM32_ADC345_BASE + STM32_ADCCMN_OFFSET) /* ADC3, ADC4 common */

/* Compatibility defines */

#define STM32_ADC34CMN_BASE            STM32_ADC345CMN_BASE

/* Register Offsets *********************************************************/

/* Registers for Each ADC */

#define STM32_ADC_ISR_OFFSET           0x0000                         /* ADC Interrupt and Status register */
#define STM32_ADC_IER_OFFSET           0x0004                         /* ADC Interrupt Enable register */
#define STM32_ADC_CR_OFFSET            0x0008                         /* ADC Control register */
#define STM32_ADC_CFGR1_OFFSET         0x000c                         /* ADC Configuration register 1 */
#define STM32_ADC_CFGR2_OFFSET         0x0010                         /* ADC Configuration register 2 */
#define STM32_ADC_SMPR1_OFFSET         0x0014                         /* ADC Sample Time register 1 */
#define STM32_ADC_SMPR2_OFFSET         0x0018                         /* ADC Sample Time register 2 */
#define STM32_ADC_TR1_OFFSET           0x0020                         /* ADC Watchdog Threshold register 1 */
#define STM32_ADC_TR2_OFFSET           0x0024                         /* ADC Watchdog Threshold register 2 */
#define STM32_ADC_TR3_OFFSET           0x0028                         /* ADC Watchdog Threshold register 3 */
#define STM32_ADC_SQR1_OFFSET          0x0030                         /* ADC Regular Sequence register 1 */
#define STM32_ADC_SQR2_OFFSET          0x0034                         /* ADC Regular Sequence register 2 */
#define STM32_ADC_SQR3_OFFSET          0x0038                         /* ADC Regular Sequence register 3 */
#define STM32_ADC_SQR4_OFFSET          0x003c                         /* ADC Regular Sequence register 4 */
#define STM32_ADC_DR_OFFSET            0x0040                         /* ADC Regular data register */
#define STM32_ADC_JSQR_OFFSET          0x004c                         /* ADC Injected Sequence register */
#define STM32_ADC_OFR1_OFFSET          0x0060                         /* ADC Offset register 1 */
#define STM32_ADC_OFR2_OFFSET          0x0064                         /* ADC Offset register 2 */
#define STM32_ADC_OFR3_OFFSET          0x0068                         /* ADC Offset register 3 */
#define STM32_ADC_OFR4_OFFSET          0x006c                         /* ADC Offset register 4 */
#define STM32_ADC_JDR1_OFFSET          0x0080                         /* ADC Injected Data register 1 */
#define STM32_ADC_JDR2_OFFSET          0x0084                         /* ADC Injected Data register 2 */
#define STM32_ADC_JDR3_OFFSET          0x0088                         /* ADC Injected Data register 3 */
#define STM32_ADC_JDR4_OFFSET          0x008c                         /* ADC Injected Data register 4 */
#define STM32_ADC_AWD2CR_OFFSET        0x00a0                         /* ADC Analog Watchdog 2 Configuration register */
#define STM32_ADC_AWD3CR_OFFSET        0x00a4                         /* ADC Analog Watchdog 3 Configuration register */
#define STM32_ADC_DIFSEL_OFFSET        0x00b0                         /* ADC Differential Mode Selection register */
#define STM32_ADC_CALFACT_OFFSET       0x00b4                         /* ADC Calibration Factors register */
#define STM32_ADC_GCOMP_OFFSET         0x00c0                         /* ADC Gain compensation register */

/* Master and Slave ADC Common Registers */

#define STM32_ADC_CSR_OFFSET           0x0000                         /* Common Status register */
#define STM32_ADC_CCR_OFFSET           0x0008                         /* Common Control register */
#define STM32_ADC_CDR_OFFSET           0x000c                         /* Common Regular Data Register for Dual Mode */

/* Register Addresses *******************************************************/

/* Registers for Each ADC */

#if STM32_NADC > 0
#  define STM32_ADC1_ISR               (STM32_ADC1_BASE + STM32_ADC_ISR_OFFSET)
#  define STM32_ADC1_IER               (STM32_ADC1_BASE + STM32_ADC_IER_OFFSET)
#  define STM32_ADC1_CR                (STM32_ADC1_BASE + STM32_ADC_CR_OFFSET)
#  define STM32_ADC1_CFGR1             (STM32_ADC1_BASE + STM32_ADC_CFGR1_OFFSET)
#  define STM32_ADC1_CFGR2             (STM32_ADC1_BASE + STM32_ADC_CFGR2_OFFSET)
#  define STM32_ADC1_SMPR1             (STM32_ADC1_BASE + STM32_ADC_SMPR1_OFFSET)
#  define STM32_ADC1_SMPR2             (STM32_ADC1_BASE + STM32_ADC_SMPR2_OFFSET)
#  define STM32_ADC1_TR1               (STM32_ADC1_BASE + STM32_ADC_TR1_OFFSET)
#  define STM32_ADC1_TR2               (STM32_ADC1_BASE + STM32_ADC_TR2_OFFSET)
#  define STM32_ADC1_TR3               (STM32_ADC1_BASE + STM32_ADC_TR3_OFFSET)
#  define STM32_ADC1_SQR1              (STM32_ADC1_BASE + STM32_ADC_SQR1_OFFSET)
#  define STM32_ADC1_SQR2              (STM32_ADC1_BASE + STM32_ADC_SQR2_OFFSET)
#  define STM32_ADC1_SQR3              (STM32_ADC1_BASE + STM32_ADC_SQR3_OFFSET)
#  define STM32_ADC1_SQR4              (STM32_ADC1_BASE + STM32_ADC_SQR4_OFFSET)
#  define STM32_ADC1_DR                (STM32_ADC1_BASE + STM32_ADC_DR_OFFSET)
#  define STM32_ADC1_JSQR              (STM32_ADC1_BASE + STM32_ADC_JSQR_OFFSET)
#  define STM32_ADC1_OFR1              (STM32_ADC1_BASE + STM32_ADC_OFR1_OFFSET)
#  define STM32_ADC1_OFR2              (STM32_ADC1_BASE + STM32_ADC_OFR2_OFFSET)
#  define STM32_ADC1_OFR3              (STM32_ADC1_BASE + STM32_ADC_OFR3_OFFSET)
#  define STM32_ADC1_OFR4              (STM32_ADC1_BASE + STM32_ADC_OFR4_OFFSET)
#  define STM32_ADC1_JDR1              (STM32_ADC1_BASE + STM32_ADC_JDR1_OFFSET)
#  define STM32_ADC1_JDR2              (STM32_ADC1_BASE + STM32_ADC_JDR2_OFFSET)
#  define STM32_ADC1_JDR3              (STM32_ADC1_BASE + STM32_ADC_JDR3_OFFSET)
#  define STM32_ADC1_JDR4              (STM32_ADC1_BASE + STM32_ADC_JDR4_OFFSET)
#  define STM32_ADC1_AWD2CR            (STM32_ADC1_BASE + STM32_ADC_AWD2CR_OFFSET)
#  define STM32_ADC1_AWD3CR            (STM32_ADC1_BASE + STM32_ADC_AWD3CR_OFFSET)
#  define STM32_ADC1_DIFSEL            (STM32_ADC1_BASE + STM32_ADC_DIFSEL_OFFSET)
#  define STM32_ADC1_CALFACT           (STM32_ADC1_BASE + STM32_ADC_CALFACT_OFFSET)
#  define STM32_ADC1_GCOMP             (STM32_ADC1_BASE + STM32_ADC_GCOMP_OFFSET)
#endif

#if STM32_NADC > 1
#  define STM32_ADC2_ISR               (STM32_ADC2_BASE + STM32_ADC_ISR_OFFSET)
#  define STM32_ADC2_IER               (STM32_ADC2_BASE + STM32_ADC_IER_OFFSET)
#  define STM32_ADC2_CR                (STM32_ADC2_BASE + STM32_ADC_CR_OFFSET)
#  define STM32_ADC2_CFGR1             (STM32_ADC2_BASE + STM32_ADC_CFGR1_OFFSET)
#  define STM32_ADC2_CFGR2             (STM32_ADC2_BASE + STM32_ADC_CFGR2_OFFSET)
#  define STM32_ADC2_SMPR1             (STM32_ADC2_BASE + STM32_ADC_SMPR1_OFFSET)
#  define STM32_ADC2_SMPR2             (STM32_ADC2_BASE + STM32_ADC_SMPR2_OFFSET)
#  define STM32_ADC2_TR1               (STM32_ADC2_BASE + STM32_ADC_TR1_OFFSET)
#  define STM32_ADC2_TR2               (STM32_ADC2_BASE + STM32_ADC_TR2_OFFSET)
#  define STM32_ADC2_TR3               (STM32_ADC2_BASE + STM32_ADC_TR3_OFFSET)
#  define STM32_ADC2_SQR1              (STM32_ADC2_BASE + STM32_ADC_SQR1_OFFSET)
#  define STM32_ADC2_SQR2              (STM32_ADC2_BASE + STM32_ADC_SQR2_OFFSET)
#  define STM32_ADC2_SQR3              (STM32_ADC2_BASE + STM32_ADC_SQR3_OFFSET)
#  define STM32_ADC2_SQR4              (STM32_ADC2_BASE + STM32_ADC_SQR4_OFFSET)
#  define STM32_ADC2_DR                (STM32_ADC2_BASE + STM32_ADC_DR_OFFSET)
#  define STM32_ADC2_JSQR              (STM32_ADC2_BASE + STM32_ADC_JSQR_OFFSET)
#  define STM32_ADC2_OFR1              (STM32_ADC2_BASE + STM32_ADC_OFR1_OFFSET)
#  define STM32_ADC2_OFR2              (STM32_ADC2_BASE + STM32_ADC_OFR2_OFFSET)
#  define STM32_ADC2_OFR3              (STM32_ADC2_BASE + STM32_ADC_OFR3_OFFSET)
#  define STM32_ADC2_OFR4              (STM32_ADC2_BASE + STM32_ADC_OFR4_OFFSET)
#  define STM32_ADC2_JDR1              (STM32_ADC2_BASE + STM32_ADC_JDR1_OFFSET)
#  define STM32_ADC2_JDR2              (STM32_ADC2_BASE + STM32_ADC_JDR2_OFFSET)
#  define STM32_ADC2_JDR3              (STM32_ADC2_BASE + STM32_ADC_JDR3_OFFSET)
#  define STM32_ADC2_JDR4              (STM32_ADC2_BASE + STM32_ADC_JDR4_OFFSET)
#  define STM32_ADC2_AWD2CR            (STM32_ADC2_BASE + STM32_ADC_AWD2CR_OFFSET)
#  define STM32_ADC2_AWD3CR            (STM32_ADC2_BASE + STM32_ADC_AWD3CR_OFFSET)
#  define STM32_ADC2_DIFSEL            (STM32_ADC2_BASE + STM32_ADC_DIFSEL_OFFSET)
#  define STM32_ADC2_CALFACT           (STM32_ADC2_BASE + STM32_ADC_CALFACT_OFFSET)
#  define STM32_ADC2_GCOMP             (STM32_ADC2_BASE + STM32_ADC_GCOMP_OFFSET)
#endif

#if STM32_NADC > 2
#  define STM32_ADC3_ISR               (STM32_ADC3_BASE + STM32_ADC_ISR_OFFSET)
#  define STM32_ADC3_IER               (STM32_ADC3_BASE + STM32_ADC_IER_OFFSET)
#  define STM32_ADC3_CR                (STM32_ADC3_BASE + STM32_ADC_CR_OFFSET)
#  define STM32_ADC3_CFGR1             (STM32_ADC3_BASE + STM32_ADC_CFGR1_OFFSET)
#  define STM32_ADC3_CFGR2             (STM32_ADC3_BASE + STM32_ADC_CFGR2_OFFSET)
#  define STM32_ADC3_SMPR1             (STM32_ADC3_BASE + STM32_ADC_SMPR1_OFFSET)
#  define STM32_ADC3_SMPR2             (STM32_ADC3_BASE + STM32_ADC_SMPR2_OFFSET)
#  define STM32_ADC3_TR1               (STM32_ADC3_BASE + STM32_ADC_TR1_OFFSET)
#  define STM32_ADC3_TR2               (STM32_ADC3_BASE + STM32_ADC_TR2_OFFSET)
#  define STM32_ADC3_TR3               (STM32_ADC3_BASE + STM32_ADC_TR3_OFFSET)
#  define STM32_ADC3_SQR1              (STM32_ADC3_BASE + STM32_ADC_SQR1_OFFSET)
#  define STM32_ADC3_SQR2              (STM32_ADC3_BASE + STM32_ADC_SQR2_OFFSET)
#  define STM32_ADC3_SQR3              (STM32_ADC3_BASE + STM32_ADC_SQR3_OFFSET)
#  define STM32_ADC3_SQR4              (STM32_ADC3_BASE + STM32_ADC_SQR4_OFFSET)
#  define STM32_ADC3_DR                (STM32_ADC3_BASE + STM32_ADC_DR_OFFSET)
#  define STM32_ADC3_JSQR              (STM32_ADC3_BASE + STM32_ADC_JSQR_OFFSET)
#  define STM32_ADC3_OFR1              (STM32_ADC3_BASE + STM32_ADC_OFR1_OFFSET)
#  define STM32_ADC3_OFR2              (STM32_ADC3_BASE + STM32_ADC_OFR2_OFFSET)
#  define STM32_ADC3_OFR3              (STM32_ADC3_BASE + STM32_ADC_OFR3_OFFSET)
#  define STM32_ADC3_OFR4              (STM32_ADC3_BASE + STM32_ADC_OFR4_OFFSET)
#  define STM32_ADC3_JDR1              (STM32_ADC3_BASE + STM32_ADC_JDR1_OFFSET)
#  define STM32_ADC3_JDR2              (STM32_ADC3_BASE + STM32_ADC_JDR2_OFFSET)
#  define STM32_ADC3_JDR3              (STM32_ADC3_BASE + STM32_ADC_JDR3_OFFSET)
#  define STM32_ADC3_JDR4              (STM32_ADC3_BASE + STM32_ADC_JDR4_OFFSET)
#  define STM32_ADC3_AWD2CR            (STM32_ADC3_BASE + STM32_ADC_AWD2CR_OFFSET)
#  define STM32_ADC3_AWD3CR            (STM32_ADC3_BASE + STM32_ADC_AWD3CR_OFFSET)
#  define STM32_ADC3_DIFSEL            (STM32_ADC3_BASE + STM32_ADC_DIFSEL_OFFSET)
#  define STM32_ADC3_CALFACT           (STM32_ADC3_BASE + STM32_ADC_CALFACT_OFFSET)
#  define STM32_ADC3_GCOMP             (STM32_ADC3_BASE + STM32_ADC_GCOMP_OFFSET)
#endif

#if STM32_NADC > 3
#  define STM32_ADC4_ISR               (STM32_ADC4_BASE + STM32_ADC_ISR_OFFSET)
#  define STM32_ADC4_IER               (STM32_ADC4_BASE + STM32_ADC_IER_OFFSET)
#  define STM32_ADC4_CR                (STM32_ADC4_BASE + STM32_ADC_CR_OFFSET)
#  define STM32_ADC4_CFGR1             (STM32_ADC4_BASE + STM32_ADC_CFGR1_OFFSET)
#  define STM32_ADC4_CFGR2             (STM32_ADC4_BASE + STM32_ADC_CFGR2_OFFSET)
#  define STM32_ADC4_SMPR1             (STM32_ADC4_BASE + STM32_ADC_SMPR1_OFFSET)
#  define STM32_ADC4_SMPR2             (STM32_ADC4_BASE + STM32_ADC_SMPR2_OFFSET)
#  define STM32_ADC4_TR1               (STM32_ADC4_BASE + STM32_ADC_TR1_OFFSET)
#  define STM32_ADC4_TR2               (STM32_ADC4_BASE + STM32_ADC_TR2_OFFSET)
#  define STM32_ADC4_TR3               (STM32_ADC4_BASE + STM32_ADC_TR3_OFFSET)
#  define STM32_ADC4_SQR1              (STM32_ADC4_BASE + STM32_ADC_SQR1_OFFSET)
#  define STM32_ADC4_SQR2              (STM32_ADC4_BASE + STM32_ADC_SQR2_OFFSET)
#  define STM32_ADC4_SQR3              (STM32_ADC4_BASE + STM32_ADC_SQR3_OFFSET)
#  define STM32_ADC4_SQR4              (STM32_ADC4_BASE + STM32_ADC_SQR4_OFFSET)
#  define STM32_ADC4_DR                (STM32_ADC4_BASE + STM32_ADC_DR_OFFSET)
#  define STM32_ADC4_JSQR              (STM32_ADC4_BASE + STM32_ADC_JSQR_OFFSET)
#  define STM32_ADC4_OFR1              (STM32_ADC4_BASE + STM32_ADC_OFR1_OFFSET)
#  define STM32_ADC4_OFR2              (STM32_ADC4_BASE + STM32_ADC_OFR2_OFFSET)
#  define STM32_ADC4_OFR3              (STM32_ADC4_BASE + STM32_ADC_OFR3_OFFSET)
#  define STM32_ADC4_OFR4              (STM32_ADC4_BASE + STM32_ADC_OFR4_OFFSET)
#  define STM32_ADC4_JDR1              (STM32_ADC4_BASE + STM32_ADC_JDR1_OFFSET)
#  define STM32_ADC4_JDR2              (STM32_ADC4_BASE + STM32_ADC_JDR2_OFFSET)
#  define STM32_ADC4_JDR3              (STM32_ADC4_BASE + STM32_ADC_JDR3_OFFSET)
#  define STM32_ADC4_JDR4              (STM32_ADC4_BASE + STM32_ADC_JDR4_OFFSET)
#  define STM32_ADC4_AWD2CR            (STM32_ADC4_BASE + STM32_ADC_AWD2CR_OFFSET)
#  define STM32_ADC4_AWD3CR            (STM32_ADC4_BASE + STM32_ADC_AWD3CR_OFFSET)
#  define STM32_ADC4_DIFSEL            (STM32_ADC4_BASE + STM32_ADC_DIFSEL_OFFSET)
#  define STM32_ADC4_CALFACT           (STM32_ADC4_BASE + STM32_ADC_CALFACT_OFFSET)
#  define STM32_ADC4_GCOMP             (STM32_ADC4_BASE + STM32_ADC_GCOMP_OFFSET)
#endif

#if STM32_NADC > 4
#  define STM32_ADC5_ISR               (STM32_ADC5_BASE + STM32_ADC_ISR_OFFSET)
#  define STM32_ADC5_IER               (STM32_ADC5_BASE + STM32_ADC_IER_OFFSET)
#  define STM32_ADC5_CR                (STM32_ADC5_BASE + STM32_ADC_CR_OFFSET)
#  define STM32_ADC5_CFGR1             (STM32_ADC5_BASE + STM32_ADC_CFGR1_OFFSET)
#  define STM32_ADC5_CFGR2             (STM32_ADC5_BASE + STM32_ADC_CFGR2_OFFSET)
#  define STM32_ADC5_SMPR1             (STM32_ADC5_BASE + STM32_ADC_SMPR1_OFFSET)
#  define STM32_ADC5_SMPR2             (STM32_ADC5_BASE + STM32_ADC_SMPR2_OFFSET)
#  define STM32_ADC5_TR1               (STM32_ADC5_BASE + STM32_ADC_TR1_OFFSET)
#  define STM32_ADC5_TR2               (STM32_ADC5_BASE + STM32_ADC_TR2_OFFSET)
#  define STM32_ADC5_TR3               (STM32_ADC5_BASE + STM32_ADC_TR3_OFFSET)
#  define STM32_ADC5_SQR1              (STM32_ADC5_BASE + STM32_ADC_SQR1_OFFSET)
#  define STM32_ADC5_SQR2              (STM32_ADC5_BASE + STM32_ADC_SQR2_OFFSET)
#  define STM32_ADC5_SQR3              (STM32_ADC5_BASE + STM32_ADC_SQR3_OFFSET)
#  define STM32_ADC5_SQR4              (STM32_ADC5_BASE + STM32_ADC_SQR4_OFFSET)
#  define STM32_ADC5_DR                (STM32_ADC5_BASE + STM32_ADC_DR_OFFSET)
#  define STM32_ADC5_JSQR              (STM32_ADC5_BASE + STM32_ADC_JSQR_OFFSET)
#  define STM32_ADC5_OFR1              (STM32_ADC5_BASE + STM32_ADC_OFR1_OFFSET)
#  define STM32_ADC5_OFR2              (STM32_ADC5_BASE + STM32_ADC_OFR2_OFFSET)
#  define STM32_ADC5_OFR3              (STM32_ADC5_BASE + STM32_ADC_OFR3_OFFSET)
#  define STM32_ADC5_OFR4              (STM32_ADC5_BASE + STM32_ADC_OFR4_OFFSET)
#  define STM32_ADC5_JDR1              (STM32_ADC5_BASE + STM32_ADC_JDR1_OFFSET)
#  define STM32_ADC5_JDR2              (STM32_ADC5_BASE + STM32_ADC_JDR2_OFFSET)
#  define STM32_ADC5_JDR3              (STM32_ADC5_BASE + STM32_ADC_JDR3_OFFSET)
#  define STM32_ADC5_JDR4              (STM32_ADC5_BASE + STM32_ADC_JDR4_OFFSET)
#  define STM32_ADC5_AWD2CR            (STM32_ADC5_BASE + STM32_ADC_AWD2CR_OFFSET)
#  define STM32_ADC5_AWD3CR            (STM32_ADC5_BASE + STM32_ADC_AWD3CR_OFFSET)
#  define STM32_ADC5_DIFSEL            (STM32_ADC5_BASE + STM32_ADC_DIFSEL_OFFSET)
#  define STM32_ADC5_CALFACT           (STM32_ADC5_BASE + STM32_ADC_CALFACT_OFFSET)
#  define STM32_ADC5_GCOMP             (STM32_ADC5_BASE + STM32_ADC_GCOMP_OFFSET)
#endif

/* Master and Slave ADC Common Registers */

#if STM32_NADC > 0
#  define STM32_ADC12_CSR              (STM32_ADC12CMN_BASE + STM32_ADC_CSR_OFFSET)
#  define STM32_ADC12_CCR              (STM32_ADC12CMN_BASE + STM32_ADC_CCR_OFFSET)
#  define STM32_ADC12_CDR              (STM32_ADC12CMN_BASE + STM32_ADC_CDR_OFFSET)
#endif

#if STM32_NADC > 2
#  define STM32_ADC345_CSR             (STM32_ADC345CMN_BASE + STM32_ADC_CSR_OFFSET)
#  define STM32_ADC345_CCR             (STM32_ADC345CMN_BASE + STM32_ADC_CCR_OFFSET)
#  define STM32_ADC345_CDR             (STM32_ADC345CMN_BASE + STM32_ADC_CDR_OFFSET)

/* Compatibility defines */

#  define STM32_ADC34_CSR              STM32_ADC345_CSR
#  define STM32_ADC34_CCR              STM32_ADC345_CCR
#  define STM32_ADC34_CDR              STM32_ADC345_CDR
#endif

/* Register Bitfield Definitions ********************************************/

/* ADC Interrupt and Status Register (ISR), and
 * ADC Interrupt Enable Register (IER)
 */

#define ADC_INT_ARDY                   (1 << 0)                       /* Bit 0:  ADC ready */
#define ADC_INT_EOSMP                  (1 << 1)                       /* Bit 1:  End of sampling flag */
#define ADC_INT_EOC                    (1 << 2)                       /* Bit 2:  End of conversion */
#define ADC_INT_EOS                    (1 << 3)                       /* Bit 3:  End of regular sequence flag */
#define ADC_INT_OVR                    (1 << 4)                       /* Bit 4:  Overrun */
#define ADC_INT_JEOC                   (1 << 5)                       /* Bit 5:  Injected channel end of conversion */
#define ADC_INT_JEOS                   (1 << 6)                       /* Bit 6:  Injected channel end of sequence flag */
#define ADC_INT_AWD1                   (1 << 7)                       /* Bit 7:  Analog watchdog 1 flag */
#define ADC_INT_AWD2                   (1 << 8)                       /* Bit 8:  Analog watchdog 2 flag */
#define ADC_INT_AWD3                   (1 << 9)                       /* Bit 9:  Analog watchdog 3 flag */
#define ADC_INT_JQOVF                  (1 << 10)                      /* Bit 10: Injected context queue overflow */

/* ADC Control Register (CR) */

#define ADC_CR_ADEN                    (1 << 0)                       /* Bit 0:  ADC enable control */
#define ADC_CR_ADDIS                   (1 << 1)                       /* Bit 1:  ADC disable command */
#define ADC_CR_ADSTART                 (1 << 2)                       /* Bit 2:  ADC start of regular conversion */
#define ADC_CR_JADSTART                (1 << 3)                       /* Bit 3:  ADC start of injected conversion */
#define ADC_CR_ADSTP                   (1 << 4)                       /* Bit 4:  ADC stop of regular conversion command */
#define ADC_CR_JADSTP                  (1 << 5)                       /* Bit 5:  ADC stop of injected conversion command */
#define ADC_CR_ADVREGEN                (1 << 28)                      /* Bit 28: ADC voltage regulator enable */
#define ADC_CR_DEEPPWD                 (1 << 29)                      /* Bit 29: ADC deep power-down enable */
#define ADC_CR_ADCALDIF                (1 << 30)                      /* Bit 30: Differential mode for calibration */
#define ADC_CR_ADCAL                   (1 << 31)                      /* Bit 31: ADC calibration */

/* For complaince with the ADC driver we also define ADVREGEN like
 * for previous chips. For new chips ST decided to better describe
 * the mechanism behind ADVREGEN bits.
 */

#define ADC_CR_ADVREGEN_SHIFT          (28)
#define ADC_CR_ADVREGEN_MASK           (3 << ADC_CR_ADVREGEN_SHIFT)
#  define ADC_CR_ADVREGEN_INTER        (0 << ADC_CR_ADVREGEN_SHIFT)
#  define ADC_CR_ADVREGEN_ENABLED      (1 << ADC_CR_ADVREGEN_SHIFT)
#  define ADC_CR_ADVREGEN_DISABLED     (2 << ADC_CR_ADVREGEN_SHIFT)

/* ADC configuration register 1 (CFGR1) */

#define ADC_CFGR1_DMAEN                (1 << 0)                       /* Bit 0:  Direct memory access enable */
#define ADC_CFGR1_DMACFG               (1 << 1)                       /* Bit 1:  Direct memory access configuration */
#define ADC_CFGR1_RES_SHIFT            (3)                            /* Bits 3-4: Data resolution */
#define ADC_CFGR1_RES_MASK             (0x3 << ADC_CFGR1_RES_SHIFT)
#  define ADC_CFGR1_RES_12BIT          (0x0 << ADC_CFGR1_RES_SHIFT)   /* 12-bit resolution */
#  define ADC_CFGR1_RES_10BIT          (0x1 << ADC_CFGR1_RES_SHIFT)   /* 10-bit resolution */
#  define ADC_CFGR1_RES_8BIT           (0x2 << ADC_CFGR1_RES_SHIFT)   /* 8-bit resolution */
#  define ADC_CFGR1_RES_6BIT           (0x3 << ADC_CFGR1_RES_SHIFT)   /* 6-bit resolution */
#define ADC_CFGR1_EXTSEL_SHIFT         (5)                            /* Bits 5-9: External trigger selection for regular group */
#define ADC_CFGR1_EXTSEL_MASK          (0x1f << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_T1CC1     (0 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_T1CC2     (1 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_T1CC3     (2 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_T2CC2     (3 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_T3TRGO    (4 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_T4CC4     (5 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_EXTI11    (6 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_T8TRGO    (7 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_T8TRGO2   (8 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_T1TRGO    (9 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_T1TRGO2   (10 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_T2TRGO    (11 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_T4TRGO    (12 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_T6TRGO    (13 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_T15TRGO   (14 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_T3CC4     (15 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC34_CFGR1_EXTSEL_T20TRGO   (16 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC34_CFGR1_EXTSEL_T20TRGO2  (17 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_T20CC1    (18 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_T20CC2    (19 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_T20CC3    (20 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_HRT1TRG1  (21 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_HRT1TRG3  (22 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_HRT1TRG5  (23 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_HRT1TRG6  (24 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_HRT1TRG7  (25 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_HRT1TRG8  (26 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_HRT1TRG9  (27 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_HRT1TRG10 (28 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_LPTIMOUT  (29 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_T7TRGO    (30 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC12_CFGR1_EXTSEL_RSVD1     (31 << ADC_CFGR1_EXTSEL_SHIFT) /* 11111: Reserved */
#  define ADC34_CFGR1_EXTSEL_T3CC1     (0 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC34_CFGR1_EXTSEL_T2CC3     (1 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC34_CFGR1_EXTSEL_T1CC3     (2 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC34_CFGR1_EXTSEL_T8CC1     (3 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC34_CFGR1_EXTSEL_T3TRGO    (4 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC34_CFGR1_EXTSEL_EXTI2     (5 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC34_CFGR1_EXTSEL_T4CC1     (6 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC34_CFGR1_EXTSEL_T8TRGO    (7 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC34_CFGR1_EXTSEL_T8TRGO2   (8 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC34_CFGR1_EXTSEL_T1TRGO    (9 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC34_CFGR1_EXTSEL_T1TRGO2   (10 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC34_CFGR1_EXTSEL_T2TRGO    (11 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC34_CFGR1_EXTSEL_T4TRGO    (12 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC34_CFGR1_EXTSEL_T6TRGO    (13 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC34_CFGR1_EXTSEL_T15TRGO   (14 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC34_CFGR1_EXTSEL_T2CC1     (15 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC34_CFGR1_EXTSEL_T20TRGO   (16 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC34_CFGR1_EXTSEL_T20TRGO2  (17 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC34_CFGR1_EXTSEL_T20CC1    (18 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC34_CFGR1_EXTSEL_HRT1TRG2  (19 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC34_CFGR1_EXTSEL_HRT1TRG4  (20 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC34_CFGR1_EXTSEL_HRT1TRG1  (21 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC34_CFGR1_EXTSEL_HRT1TRG3  (22 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC34_CFGR1_EXTSEL_HRT1TRG5  (23 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC34_CFGR1_EXTSEL_HRT1TRG6  (24 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC34_CFGR1_EXTSEL_HRT1TRG7  (25 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC34_CFGR1_EXTSEL_HRT1TRG8  (26 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC34_CFGR1_EXTSEL_HRT1TRG9  (27 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC34_CFGR1_EXTSEL_HRT1TRG10 (28 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC34_CFGR1_EXTSEL_LPTIMOUT  (29 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC34_CFGR1_EXTSEL_T7TRGO    (30 << ADC_CFGR1_EXTSEL_SHIFT)
#  define ADC34_CFGR1_EXTSEL_RSVD1     (31 << ADC_CFGR1_EXTSEL_SHIFT) /* 11111: Reserved */
#define ADC_CFGR1_EXTEN_SHIFT          (10)                           /* Bits 10-11: External trigger/polarity selection regular channels */
#define ADC_CFGR1_EXTEN_MASK           (0x3 << ADC_CFGR1_EXTEN_SHIFT)
#  define ADC_CFGR1_EXTEN_NONE         (0x0 << ADC_CFGR1_EXTEN_SHIFT) /* Trigger detection disabled */
#  define ADC_CFGR1_EXTEN_RISING       (0x1 << ADC_CFGR1_EXTEN_SHIFT) /* Trigger detection on the rising edge */
#  define ADC_CFGR1_EXTEN_FALLING      (0x2 << ADC_CFGR1_EXTEN_SHIFT) /* Trigger detection on the falling edge */
#  define ADC_CFGR1_EXTEN_BOTH         (0x3 << ADC_CFGR1_EXTEN_SHIFT) /* Trigger detection on both edges */
#define ADC_CFGR1_OVRMOD               (1 << 12)                      /* Bit 12: Overrun Mode */
#define ADC_CFGR1_CONT                 (1 << 13)                      /* Bit 13: Continuous mode for regular conversions */
#define ADC_CFGR1_AUTDLY               (1 << 14)                      /* Bit 14: Delayed conversion mode */
#define ADC_CFGR1_ALIGN                (1 << 15)                      /* Bit 15: Data Alignment */
#define ADC_CFGR1_DISCEN               (1 << 16)                      /* Bit 16: Discontinuous mode for regular channels */
#define ADC_CFGR1_DISCNUM_SHIFT        (17)                           /* Bits 17-19: Discontinuous mode channel count */
#define ADC_CFGR1_DISCNUM_MASK         (0x7 << ADC_CFGR1_DISCNUM_SHIFT)
#  define ADC_CFGR1_DISCNUM(n)         (((n) - 1) << ADC_CFGR1_DISCNUM_SHIFT)
#  define ADC_CFGR1_DISCNUM_1          (0 << ADC_CFGR1_DISCNUM_SHIFT) /* 1 channel */
#  define ADC_CFGR1_DISCNUM_2          (1 << ADC_CFGR1_DISCNUM_SHIFT) /* 2 channels */
#  define ADC_CFGR1_DISCNUM_3          (2 << ADC_CFGR1_DISCNUM_SHIFT) /* 3 channels */
#  define ADC_CFGR1_DISCNUM_4          (3 << ADC_CFGR1_DISCNUM_SHIFT) /* 4 channels */
#  define ADC_CFGR1_DISCNUM_5          (4 << ADC_CFGR1_DISCNUM_SHIFT) /* 5 channels */
#  define ADC_CFGR1_DISCNUM_6          (5 << ADC_CFGR1_DISCNUM_SHIFT) /* 6 channels */
#  define ADC_CFGR1_DISCNUM_7          (6 << ADC_CFGR1_DISCNUM_SHIFT) /* 7 channels */
#  define ADC_CFGR1_DISCNUM_8          (7 << ADC_CFGR1_DISCNUM_SHIFT) /* 8 channels */
#define ADC_CFGR1_JDISCEN              (1 << 20)                      /* Bit 20: Discontinuous mode on injected channels */
#define ADC_CFGR1_JQM                  (1 << 21)                      /* Bit 21: JSQR queue mode */
#define ADC_CFGR1_AWD1SGL              (1 << 22)                      /* Bit 22: Enable watchdog on single/all channels */
#define ADC_CFGR1_AWD1EN               (1 << 23)                      /* Bit 23: Analog watchdog enable 1 regular channels */
#define ADC_CFGR1_JAWD1EN              (1 << 24)                      /* Bit 24: Analog watchdog enable 1 injected channels */
#define ADC_CFGR1_JAUTO                (1 << 25)                      /* Bit 25: Automatic injected group conversion */
#define ADC_CFGR1_AWD1CH_SHIFT         (26)                           /* Bits 26-30: Analog watchdog 1 channel select bits */
#define ADC_CFGR1_AWD1CH_MASK          (0x1f << ADC_CFGR1_AWD1CH_SHIFT)
#  define ADC_CFGR1_AWD1CH_DISABLED    (0 << ADC_CFGR1_AWD1CH_SHIFT)
#define ADC_CFGR1_JQDIS                (1 << 31)                      /* Bit 31: Injected queue disable */

/* ADC configuration register 2 (CFGR2) */

#define ADC_CFGR2_ROVSE                (1 << 0)                       /* Bit 0: Regular oversampling enable */
#define ADC_CFGR2_JOVSE                (1 << 1)                       /* Bit 1: Injected oversampling enable */
#define ADC_CFGR2_OVSR_SHIFT           (2)                            /* Bits 2-4: Oversampling ratio */
#define ADC_CFGR2_OVSR_MASK            (0x7 << ADC_CFGR2_OVSR_SHIFT)
#  define ADC_CFGR2_OVSR_2X            (0x0 << ADC_CFGR2_OVSR_SHIFT)  /* 000: 2x oversampling */
#  define ADC_CFGR2_OVSR_4X            (0x1 << ADC_CFGR2_OVSR_SHIFT)  /* 001: 4x oversampling */
#  define ADC_CFGR2_OVSR_8X            (0x2 << ADC_CFGR2_OVSR_SHIFT)  /* 010: 8x oversampling */
#  define ADC_CFGR2_OVSR_16X           (0x3 << ADC_CFGR2_OVSR_SHIFT)  /* 011: 16x oversampling */
#  define ADC_CFGR2_OVSR_32X           (0x4 << ADC_CFGR2_OVSR_SHIFT)  /* 100: 32x oversampling */
#  define ADC_CFGR2_OVSR_64X           (0x5 << ADC_CFGR2_OVSR_SHIFT)  /* 101: 64x oversampling */
#  define ADC_CFGR2_OVSR_128X          (0x6 << ADC_CFGR2_OVSR_SHIFT)  /* 110: 128x oversampling */
#  define ADC_CFGR2_OVSR_256X          (0x7 << ADC_CFGR2_OVSR_SHIFT)  /* 111: 256x oversampling */
#define ADC_CFGR2_OVSS_SHIFT           (5)                            /* Bits 5-8: Oversampling shift */
#define ADC_CFGR2_OVSS_MASK            (0xf << ADC_CFGR2_OVSS_SHIFT)
#  define ADC_CFGR2_OVSS(n)            ((n) << ADC_CFGR2_OVSS_SHIFT)
#  define ADC_CFGR2_OVSS_NONE          (0x0 << ADC_CFGR2_OVSS_SHIFT)  /* 0000: No shift  */
#  define ADC_CFGR2_OVSS_1_BIT         (0x1 << ADC_CFGR2_OVSS_SHIFT)  /* 0001: Shift 1 bits */
#  define ADC_CFGR2_OVSS_2_BIT         (0x2 << ADC_CFGR2_OVSS_SHIFT)  /* 0010: Shift 2 bits */
#  define ADC_CFGR2_OVSS_3_BIT         (0x3 << ADC_CFGR2_OVSS_SHIFT)  /* 0011: Shift 3 bits */
#  define ADC_CFGR2_OVSS_4_BIT         (0x4 << ADC_CFGR2_OVSS_SHIFT)  /* 0100: Shift 4 bits */
#  define ADC_CFGR2_OVSS_5_BIT         (0x5 << ADC_CFGR2_OVSS_SHIFT)  /* 0101: Shift 5 bits */
#  define ADC_CFGR2_OVSS_6_BIT         (0x6 << ADC_CFGR2_OVSS_SHIFT)  /* 0110: Shift 6 bits */
#  define ADC_CFGR2_OVSS_7_BIT         (0x7 << ADC_CFGR2_OVSS_SHIFT)  /* 0111: Shift 7 bits */
#  define ADC_CFGR2_OVSS_8_BIT         (0x8 << ADC_CFGR2_OVSS_SHIFT)  /* 1000: Shift 8 bits */
#define ADC_CFGR2_TROVS                (1 << 9)                       /* Bit 9: Triggered regular oversampling */
#define ADC_CFGR2_ROVSM                (1 << 10)                      /* Bit 10: Regular oversampling mode */
#define ADC_CFGR2_GCOMP                (1 << 16)                      /* Bit 16: Gain compensation mode */
#define ADC_CFGR2_SWTRIG               (1 << 25)                      /* Bit 25: Software trigger for sampling time control trigger mode */
#define ADC_CFGR2_BULB                 (1 << 26)                      /* Bit 26: Bulb sampling mode */
#define ADC_CFGR2_SMPTRIG              (1 << 27)                      /* Bit 27: Sampling time control trigger mode */

/* ADC sample time values for use with SMPR1 and SMPR2 bitfields */

#define ADC_SMPR_2p5                   0x0                            /* 000: Sample for 2.5 cycles */
#define ADC_SMPR_6p5                   0x1                            /* 001: Sample for 6.5 cycles */
#define ADC_SMPR_12p5                  0x2                            /* 010: Sample for 12.5 cycles */
#define ADC_SMPR_24p5                  0x3                            /* 011: Sample for 24.5 cycles */
#define ADC_SMPR_47p5                  0x4                            /* 100: Sample for 47.5 cycles */
#define ADC_SMPR_92p5                  0x5                            /* 101: Sample for 92.5 cycles */
#define ADC_SMPR_247p5                 0x6                            /* 110: Sample for 247.5 cycles */
#define ADC_SMPR_640p5                 0x7                            /* 111: Sample for 640.5 cycles */

/* ADC sample time register 1 (SMPR1) */

#define ADC_SMPR1_SMP0_SHIFT           (0)                            /* Bits 0-2: Channel 0 Sample time selection */
#define ADC_SMPR1_SMP0_MASK            (0x7 << ADC_SMPR1_SMP1_SHIFT)
#define ADC_SMPR1_SMP1_SHIFT           (3)                            /* Bits 3-5: Channel 1 Sample time selection */
#define ADC_SMPR1_SMP1_MASK            (0x7 << ADC_SMPR1_SMP1_SHIFT)
#define ADC_SMPR1_SMP2_SHIFT           (6)                            /* Bits 6-8: Channel 2 Sample time selection */
#define ADC_SMPR1_SMP2_MASK            (0x7 << ADC_SMPR1_SMP2_SHIFT)
#define ADC_SMPR1_SMP3_SHIFT           (9)                            /* Bits 9-11: Channel 3 Sample time selection */
#define ADC_SMPR1_SMP3_MASK            (0x7 << ADC_SMPR1_SMP3_SHIFT)
#define ADC_SMPR1_SMP4_SHIFT           (12)                           /* Bits 12-14: Channel 4 Sample time selection */
#define ADC_SMPR1_SMP4_MASK            (0x7 << ADC_SMPR1_SMP4_SHIFT)
#define ADC_SMPR1_SMP5_SHIFT           (15)                           /* Bits 15-17: Channel 5 Sample time selection */
#define ADC_SMPR1_SMP5_MASK            (0x7 << ADC_SMPR1_SMP5_SHIFT)
#define ADC_SMPR1_SMP6_SHIFT           (18)                           /* Bits 18-20: Channel 6 Sample time selection */
#define ADC_SMPR1_SMP6_MASK            (0x7 << ADC_SMPR1_SMP6_SHIFT)
#define ADC_SMPR1_SMP7_SHIFT           (21)                           /* Bits 21-23: Channel 7 Sample time selection */
#define ADC_SMPR1_SMP7_MASK            (0x7 << ADC_SMPR1_SMP7_SHIFT)
#define ADC_SMPR1_SMP8_SHIFT           (24)                           /* Bits 24-26: Channel 8 Sample time selection */
#define ADC_SMPR1_SMP8_MASK            (0x7 << ADC_SMPR1_SMP8_SHIFT)
#define ADC_SMPR1_SMP9_SHIFT           (27)                           /* Bits 27-29: Channel 9 Sample time selection */
#define ADC_SMPR1_SMP9_MASK            (0x7 << ADC_SMPR1_SMP9_SHIFT)
#define ADC_SMPR1_SMPPLUS              (1 << 31)                      /* Bit 31: Addition of one clock cycle to the sampling time */

/* ADC sample time register 2 (SMPR2) */

#define ADC_SMPR2_SMP10_SHIFT          (0)                            /* Bits 0-2: Channel 10 Sample time selection */
#define ADC_SMPR2_SMP10_MASK           (0x7 << ADC_SMPR2_SMP10_SHIFT)
#define ADC_SMPR2_SMP11_SHIFT          (3)                            /* Bits 3-5: Channel 11 Sample time selection */
#define ADC_SMPR2_SMP11_MASK           (0x7 << ADC_SMPR2_SMP11_SHIFT)
#define ADC_SMPR2_SMP12_SHIFT          (6)                            /* Bits 6-8: Channel 12 Sample time selection */
#define ADC_SMPR2_SMP12_MASK           (0x7 << ADC_SMPR2_SMP12_SHIFT)
#define ADC_SMPR2_SMP13_SHIFT          (9)                            /* Bits 9-11: Channel 13 Sample time selection */
#define ADC_SMPR2_SMP13_MASK           (0x7 << ADC_SMPR2_SMP13_SHIFT)
#define ADC_SMPR2_SMP14_SHIFT          (12)                           /* Bits 12-14: Channel 14 Sample time selection */
#define ADC_SMPR2_SMP14_MASK           (0x7 << ADC_SMPR2_SMP14_SHIFT)
#define ADC_SMPR2_SMP15_SHIFT          (15)                           /* Bits 15-17: Channel 15 Sample time selection */
#define ADC_SMPR2_SMP15_MASK           (0x7 << ADC_SMPR2_SMP15_SHIFT)
#define ADC_SMPR2_SMP16_SHIFT          (18)                           /* Bits 18-20: Channel 16 Sample time selection */
#define ADC_SMPR2_SMP16_MASK           (0x7 << ADC_SMPR2_SMP16_SHIFT)
#define ADC_SMPR2_SMP17_SHIFT          (21)                           /* Bits 21-23: Channel 17 Sample time selection */
#define ADC_SMPR2_SMP17_MASK           (0x7 << ADC_SMPR2_SMP17_SHIFT)
#define ADC_SMPR2_SMP18_SHIFT          (24)                           /* Bits 24-26: Channel 18 Sample time selection */
#define ADC_SMPR2_SMP18_MASK           (0x7 << ADC_SMPR2_SMP18_SHIFT)

/* ADC watchdog threshold register 1 (TR1) */

#define ADC_TR1_LT_SHIFT               (0)                            /* Bits 0-11: Analog watchdog 1 lower threshold */
#define ADC_TR1_LT_MASK                (0xfff << ADC_TR1_LT_SHIFT)
#define ADC_TR1_AWDFILT_SHIFT          (12)                           /* Bits 12-14: Analog watchdog filtering parameter */
#define ADC_TR1_AWDFILT_MASK           (0x7 << ADC_TR1_AWDFILT_SHIFT)
#  define ADC_TR1_AWDFILT(n)           (((n) - 1) << ADC_TR1_AWDFILT_SHIFT)
#  define ADC_TR1_AWDFILT_NONE         (0x0 << ADC_TR1_AWDFILT_SHIFT) /* 000: No filtering */
#  define ADC_TR1_AWDFILT_2            (0x1 << ADC_TR1_AWDFILT_SHIFT) /* 001: Two consecutive detections generate an AWDx flag or interrupt */
#  define ADC_TR1_AWDFILT_3            (0x2 << ADC_TR1_AWDFILT_SHIFT) /* 010: Three consecutive detections generate an AWDx flag or interrupt */
#  define ADC_TR1_AWDFILT_4            (0x3 << ADC_TR1_AWDFILT_SHIFT) /* 011: Four consecutive detections generate an AWDx flag or interrupt */
#  define ADC_TR1_AWDFILT_5            (0x4 << ADC_TR1_AWDFILT_SHIFT) /* 100: Five consecutive detections generate an AWDx flag or interrupt */
#  define ADC_TR1_AWDFILT_6            (0x5 << ADC_TR1_AWDFILT_SHIFT) /* 101: Six consecutive detections generate an AWDx flag or interrupt */
#  define ADC_TR1_AWDFILT_7            (0x6 << ADC_TR1_AWDFILT_SHIFT) /* 110: Seven consecutive detections generate an AWDx flag or interrupt */
#  define ADC_TR1_AWDFILT_8            (0x7 << ADC_TR1_AWDFILT_SHIFT) /* 111: Eight consecutive detections generate an AWDx flag or interrupt */
#define ADC_TR1_HT_SHIFT               (16)                           /* Bits 16-27: Analog watchdog 1 higher threshold */
#define ADC_TR1_HT_MASK                (0xfff << ADC_TR1_HT_SHIFT)

/* ADC watchdog threshold register 2 (TR2) */

#define ADC_TR2_LT_SHIFT               (0)                            /* Bits 0-7: Analog watchdog 2 lower threshold */
#define ADC_TR2_LT_MASK                (0xff << ADC_TR2_LT_SHIFT)
#define ADC_TR2_HT_SHIFT               (16)                           /* Bits 16-23: Analog watchdog 2 higher threshold */
#define ADC_TR2_HT_MASK                (0xff << ADC_TR2_HT_SHIFT)

/* ADC watchdog threshold register 3 (TR3) */

#define ADC_TR3_LT_SHIFT               (0)                            /* Bits 0-7: Analog watchdog 3 lower threshold */
#define ADC_TR3_LT_MASK                (0xff << ADC_TR3_LT_SHIFT)
#define ADC_TR3_HT_SHIFT               (16)                           /* Bits 16-23: Analog watchdog 3 higher threshold */
#define ADC_TR3_HT_MASK                (0xff << ADC_TR3_HT_SHIFT)

/* ADC regular sequence register 1 (SQR1) */

#define ADC_SQ_OFFSET                  (6)                            /* Offset between SQ bitfields in SQR1..SQR4 */

#define ADC_SQR1_L_SHIFT               (0)                            /* Bits 0-3:   Regular channel sequence length */
#define ADC_SQR1_L_MASK                (0x0f << ADC_SQR1_L_SHIFT)
#  define ADC_SQR1_L(n)                (((n) - 1) << ADC_SQR1_L_SHIFT)
#  define ADC_SQR1_L_1                 (0x0 << ADC_SQR1_L_SHIFT)      /* 1 conversion */
#  define ADC_SQR1_L_2                 (0x1 << ADC_SQR1_L_SHIFT)      /* 2 conversions */
#  define ADC_SQR1_L_3                 (0x2 << ADC_SQR1_L_SHIFT)      /* 3 conversions */
#  define ADC_SQR1_L_4                 (0x3 << ADC_SQR1_L_SHIFT)      /* 4 conversions */
#  define ADC_SQR1_L_5                 (0x4 << ADC_SQR1_L_SHIFT)      /* 5 conversion */
#  define ADC_SQR1_L_6                 (0x5 << ADC_SQR1_L_SHIFT)      /* 6 conversions */
#  define ADC_SQR1_L_7                 (0x6 << ADC_SQR1_L_SHIFT)      /* 7 conversions */
#  define ADC_SQR1_L_8                 (0x7 << ADC_SQR1_L_SHIFT)      /* 8 conversions */
#  define ADC_SQR1_L_9                 (0x8 << ADC_SQR1_L_SHIFT)      /* 9 conversion */
#  define ADC_SQR1_L_10                (0x9 << ADC_SQR1_L_SHIFT)      /* 10 conversions */
#  define ADC_SQR1_L_11                (0xa << ADC_SQR1_L_SHIFT)      /* 11 conversions */
#  define ADC_SQR1_L_12                (0xb << ADC_SQR1_L_SHIFT)      /* 12 conversions */
#  define ADC_SQR1_L_13                (0xc << ADC_SQR1_L_SHIFT)      /* 13 conversion */
#  define ADC_SQR1_L_14                (0xd << ADC_SQR1_L_SHIFT)      /* 14 conversions */
#  define ADC_SQR1_L_15                (0xe << ADC_SQR1_L_SHIFT)      /* 15 conversions */
#  define ADC_SQR1_L_16                (0xf << ADC_SQR1_L_SHIFT)      /* 16 conversions */
#define ADC_SQR1_SQ1_SHIFT             (6)                            /* Bits 6-10:  1st conversion in regular sequence */
#define ADC_SQR1_SQ1_MASK              (0x1f << ADC_SQR1_SQ1_SHIFT)
#define ADC_SQR1_SQ2_SHIFT             (12)                           /* Bits 12-16: 2nd conversion in regular sequence */
#define ADC_SQR1_SQ2_MASK              (0x1f << ADC_SQR1_SQ2_SHIFT)
#define ADC_SQR1_SQ3_SHIFT             (18)                           /* Bits 18-22: 3rd conversion in regular sequence */
#define ADC_SQR1_SQ3_MASK              (0x1f << ADC_SQR1_SQ3_SHIFT)
#define ADC_SQR1_SQ4_SHIFT             (24)                           /* Bits 24-28: 4th conversion in regular sequence */
#define ADC_SQR1_SQ4_MASK              (0x1f << ADC_SQR1_SQ4_SHIFT)
#define ADC_SQR1_RESERVED              (0xe0820830)                   /* Mask of all reserved bits: 4, 5, 11, 17, 23, 29, 30, 31 */
#define ADC_SQR1_FIRST                 (1)
#define ADC_SQR1_LAST                  (4)
#define ADC_SQR1_SQ_OFFSET             (1 * ADC_SQ_OFFSET)            /* Offset to first SQ bitfield in the register */

/* ADC regular sequence register 2 (SQR2) */

#define ADC_SQR2_SQ5_SHIFT             (0)                            /* Bits 0-4:   5th conversion in regular sequence */
#define ADC_SQR2_SQ5_MASK              (0x1f << ADC_SQR2_SQ5_SHIFT)
#define ADC_SQR2_SQ6_SHIFT             (6)                            /* Bits 6-10:  6th conversion in regular sequence */
#define ADC_SQR2_SQ6_MASK              (0x1f << ADC_SQR2_SQ6_SHIFT)
#define ADC_SQR2_SQ7_SHIFT             (12)                           /* Bits 12-16: 7th conversion in regular sequence */
#define ADC_SQR2_SQ7_MASK              (0x1f << ADC_SQR2_SQ7_SHIFT)
#define ADC_SQR2_SQ8_SHIFT             (18)                           /* Bits 18-22: 8th conversion in regular sequence */
#define ADC_SQR2_SQ8_MASK              (0x1f << ADC_SQR2_SQ8_SHIFT)
#define ADC_SQR2_SQ9_SHIFT             (24)                           /* Bits 24-28: 9th conversion in regular sequence */
#define ADC_SQR2_SQ9_MASK              (0x1f << ADC_SQR2_SQ9_SHIFT)
#define ADC_SQR2_RESERVED              (0xe0820820)                   /* Mask of all reserved bits: 5, 11, 17, 23, 29, 30, 31 */
#define ADC_SQR2_FIRST                 (5)
#define ADC_SQR2_LAST                  (9)
#define ADC_SQR2_SQ_OFFSET             (0)                            /* Offset to first SQ bitfield in the register */

/* ADC regular sequence register 3 (SQR3) */

#define ADC_SQR3_SQ10_SHIFT            (0)                            /* Bits 0-4:   10th conversion in regular sequence */
#define ADC_SQR3_SQ10_MASK             (0x1f << ADC_SQR3_SQ10_SHIFT)
#define ADC_SQR3_SQ11_SHIFT            (6)                            /* Bits 6-10:  11th conversion in regular sequence */
#define ADC_SQR3_SQ11_MASK             (0x1f << ADC_SQR3_SQ11_SHIFT)
#define ADC_SQR3_SQ12_SHIFT            (12)                           /* Bits 12-16: 12th conversion in regular sequence */
#define ADC_SQR3_SQ12_MASK             (0x1f << ADC_SQR3_SQ12_SHIFT)
#define ADC_SQR3_SQ13_SHIFT            (18)                           /* Bits 18-22: 13th conversion in regular sequence */
#define ADC_SQR3_SQ13_MASK             (0x1f << ADC_SQR3_SQ13_SHIFT)
#define ADC_SQR3_SQ14_SHIFT            (24)                           /* Bits 24-28: 14th conversion in regular sequence */
#define ADC_SQR3_SQ14_MASK             (0x1f << ADC_SQR3_SQ14_SHIFT)
#define ADC_SQR3_RESERVED              (0xe0820820)                   /* Mask of all reserved bits: 5, 11, 17, 23, 29, 30, 31 */
#define ADC_SQR3_FIRST                 (10)
#define ADC_SQR3_LAST                  (14)
#define ADC_SQR3_SQ_OFFSET             (0)                            /* Offset to first SQ bitfield in the register */

/* ADC regular sequence register 4 (SQR4) */

#define ADC_SQR4_SQ15_SHIFT            (0)                            /* Bits 0-4:   15th conversion in regular sequence */
#define ADC_SQR4_SQ15_MASK             (0x1f << ADC_SQR4_SQ15_SHIFT)
#define ADC_SQR4_SQ16_SHIFT            (6)                            /* Bits 6-10:  16th conversion in regular sequence */
#define ADC_SQR4_SQ16_MASK             (0x1f << ADC_SQR4_SQ16_SHIFT)
#define ADC_SQR4_RESERVED              (0xfffff820)                   /* Mask of all reserved bits: 5, 11-31 */
#define ADC_SQR4_FIRST                 (15)
#define ADC_SQR4_LAST                  (16)
#define ADC_SQR4_SQ_OFFSET             (0)                            /* Offset to first SQ bitfield in the register */

/* ADC regular data register (DR) */

#define ADC_DR_RDATA_SHIFT             (0)                            /* Bits 0-15: Regular data converted */
#define ADC_DR_RDATA_MASK              (0xffff << ADC_DR_RDATA_SHIFT)

/* ADC injected sequence register (JSQR) */

#define ADC_JSQR_JL_SHIFT              (0)                            /* Bits 0-1: Injected channel sequence length */
#define ADC_JSQR_JL_MASK               (0x3 << ADC_JSQR_JL_SHIFT)
#  define ADC_JSQR_JL(n)               (((n) - 1) << ADC_JSQR_JL_SHIFT)
#  define ADC_JSQR_JL_1                (0x0 << ADC_JSQR_JL_SHIFT)     /* 1 conversion */
#  define ADC_JSQR_JL_2                (0x1 << ADC_JSQR_JL_SHIFT)     /* 2 conversions */
#  define ADC_JSQR_JL_3                (0x2 << ADC_JSQR_JL_SHIFT)     /* 3 conversions */
#  define ADC_JSQR_JL_4                (0x3 << ADC_JSQR_JL_SHIFT)     /* 4 conversions */
#define ADC_JSQR_JEXTSEL_SHIFT         (2)                            /* Bits 2-6: External trigger selection for injected group */
#define ADC_JSQR_JEXTSEL_MASK          (0x1f << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC12_JSQR_JEXTSEL_T1TRGO    (0 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC12_JSQR_JEXTSEL_T1CC4     (1 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC12_JSQR_JEXTSEL_T2TRGO    (2 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC12_JSQR_JEXTSEL_T2CC1     (3 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC12_JSQR_JEXTSEL_T3CC4     (4 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC12_JSQR_JEXTSEL_T4TRGO    (5 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC12_JSQR_JEXTSEL_EXTI15    (6 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC12_JSQR_JEXTSEL_T8CC4     (7 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC12_JSQR_JEXTSEL_T1TRGO2   (8 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC12_JSQR_JEXTSEL_T8TRGO    (9 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC12_JSQR_JEXTSEL_T8TRGO2   (10 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC12_JSQR_JEXTSEL_T3CC3     (11 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC12_JSQR_JEXTSEL_T3TRGO    (12 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC12_JSQR_JEXTSEL_T3CC1     (13 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC12_JSQR_JEXTSEL_T6TRGO    (14 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC12_JSQR_JEXTSEL_T15TRGO   (15 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC12_JSQR_JEXTSEL_T20TRGO   (16 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC12_JSQR_JEXTSEL_T20TRGO2  (17 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC12_JSQR_JEXTSEL_T20CC4    (18 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC12_JSQR_JEXTSEL_HRT1TRG2  (19 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC12_JSQR_JEXTSEL_HRT1TRG4  (20 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC12_JSQR_JEXTSEL_HRT1TRG5  (21 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC12_JSQR_JEXTSEL_HRT1TRG6  (22 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC12_JSQR_JEXTSEL_HRT1TRG7  (23 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC12_JSQR_JEXTSEL_HRT1TRG8  (24 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC12_JSQR_JEXTSEL_HRT1TRG9  (25 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC12_JSQR_JEXTSEL_HRT1TRG10 (26 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC12_JSQR_JEXTSEL_T16CC1    (27 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC12_JSQR_JEXTSEL_RSVD1     (28 << ADC_JSQR_JEXTSEL_SHIFT) /* 11100: Reserved */
#  define ADC12_JSQR_JEXTSEL_LPTIMOUT  (29 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC12_JSQR_JEXTSEL_T7TRGO    (30 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC12_JSQR_JEXTSEL_RSVD2     (31 << ADC_JSQR_JEXTSEL_SHIFT) /* 11111: Reserved */
#  define ADC34_JSQR_JEXTSEL_T1TRGO    (0 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC34_JSQR_JEXTSEL_T1CC4     (1 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC34_JSQR_JEXTSEL_T2TRGO    (2 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC34_JSQR_JEXTSEL_T8CC2     (3 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC34_JSQR_JEXTSEL_T4CC3     (4 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC34_JSQR_JEXTSEL_T4TRGO    (5 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC34_JSQR_JEXTSEL_T4CC4     (6 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC34_JSQR_JEXTSEL_T8CC4     (7 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC34_JSQR_JEXTSEL_T1TRGO2   (8 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC34_JSQR_JEXTSEL_T8TRGO    (9 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC34_JSQR_JEXTSEL_T8TRGO2   (10 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC34_JSQR_JEXTSEL_T1CC3     (11 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC34_JSQR_JEXTSEL_T3TRGO    (12 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC34_JSQR_JEXTSEL_EXTI3     (13 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC34_JSQR_JEXTSEL_T6TRGO    (14 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC34_JSQR_JEXTSEL_T15TRGO   (15 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC34_JSQR_JEXTSEL_T20TRGO   (16 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC34_JSQR_JEXTSEL_T20TRGO2  (17 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC34_JSQR_JEXTSEL_T20CC2    (18 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC34_JSQR_JEXTSEL_HRT1TRG2  (19 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC34_JSQR_JEXTSEL_HRT1TRG4  (20 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC34_JSQR_JEXTSEL_HRT1TRG5  (21 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC34_JSQR_JEXTSEL_HRT1TRG6  (22 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC34_JSQR_JEXTSEL_HRT1TRG7  (23 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC34_JSQR_JEXTSEL_HRT1TRG8  (24 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC34_JSQR_JEXTSEL_HRT1TRG9  (25 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC34_JSQR_JEXTSEL_HRT1TRG10 (26 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC34_JSQR_JEXTSEL_HRT1TRG1  (27 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC34_JSQR_JEXTSEL_HRT1TRG3  (28 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC34_JSQR_JEXTSEL_LPTIMOUT  (29 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC34_JSQR_JEXTSEL_T7TRGO    (30 << ADC_JSQR_JEXTSEL_SHIFT)
#  define ADC34_JSQR_JEXTSEL_RSVD1     (31 << ADC_JSQR_JEXTSEL_SHIFT) /* 11111: Reserved */
#define ADC_JSQR_JEXTEN_SHIFT          (7)                            /* Bits 7-8: External trigger enable and polarity selection for injected channels */
#define ADC_JSQR_JEXTEN_MASK           (0x3 << ADC_JSQR_JEXTEN_SHIFT)
#  define ADC_JSQR_JEXTEN_NONE         (0x0 << ADC_JSQR_JEXTEN_SHIFT) /* 00: Trigger detection disabled */
#  define ADC_JSQR_JEXTEN_RISING       (0x1 << ADC_JSQR_JEXTEN_SHIFT) /* 01: Trigger detection on the rising edge */
#  define ADC_JSQR_JEXTEN_FALLING      (0x2 << ADC_JSQR_JEXTEN_SHIFT) /* 10: Trigger detection on the falling edge */
#  define ADC_JSQR_JEXTEN_BOTH         (0x3 << ADC_JSQR_JEXTEN_SHIFT) /* 11: Trigger detection on both the rising and falling edges */
#define ADC_JSQR_JSQ1_SHIFT            (9)                            /* Bits 9-13: 1st conversion in injected sequence */
#define ADC_JSQR_JSQ1_MASK             (0x1f << ADC_JSQR_JSQ1_SHIFT)
#  define ADC_JSQR_JSQ1(ch)            ((ch) << ADC_JSQR_JSQ1_SHIFT)  /* Channel number 1..18 */
#define ADC_JSQR_JSQ2_SHIFT            (15)                           /* Bits 15-19: 2nd conversion in injected sequence */
#define ADC_JSQR_JSQ2_MASK             (0x1f << ADC_JSQR_JSQ2_SHIFT)
#  define ADC_JSQR_JSQ2(ch)            ((ch) << ADC_JSQR_JSQ2_SHIFT)  /* Channel number 1..18 */
#define ADC_JSQR_JSQ3_SHIFT            (21)                           /* Bits 21-25: 3rd conversion in injected sequence */
#define ADC_JSQR_JSQ3_MASK             (0x1f << ADC_JSQR_JSQ3_SHIFT)
#  define ADC_JSQR_JSQ3(ch)            ((ch) << ADC_JSQR_JSQ3_SHIFT)  /* Channel number 1..18 */
#define ADC_JSQR_JSQ4_SHIFT            (27)                           /* Bits 27-31: 4th conversion in injected sequence */
#define ADC_JSQR_JSQ4_MASK             (0x1f << ADC_JSQR_JSQ4_SHIFT)
#  define ADC_JSQR_JSQ4(ch)            ((ch) << ADC_JSQR_JSQ4_SHIFT)  /* Channel number 1..18 */
#define ADC_JSQR_JSQ_SHIFT             (6)                            /* Shift between JSQx bits */

/* ADC offset register 1 thru 4 (OFR1, OFR2, OFR3, OFR4) */

#define ADC_OFR_OFFSETY_SHIFT          (0)                            /* Bits 0-11: Data offset y for channel OFFSETY_CH */
#define ADC_OFR_OFFSETY_MASK           (0x0fff << ADC_OFR_OFFSETY_SHIFT)
#  define ADC_OFR_OFFSETY(offset)      ((offset) << ADC_OFR_OFFSETY_SHIFT)
#define ADC_OFR_OFFSETPOS              (1 << 24)                      /* Bit 24: Positive offset (0=negative, 1=positive) */
#define ADC_OFR_SATEN                  (1 << 25)                      /* Bit 25: Saturation enable */
#define ADC_OFR_OFFSETY_CH_SHIFT       (26)                           /* Bits 26-30: Channel selection for data offset y */
#define ADC_OFR_OFFSETY_CH_MASK        (31 << ADC_OFR_OFFSETY_CH_SHIFT)
#  define ADC_OFR_OFFSETY_CH(ch)       ((ch) << ADC_OFR_OFFSETY_CH_SHIFT)
#define ADC_OFR_OFFSETY_EN             (1 << 31)                      /* Bit 31: Offset y enable */

/* ADC injected data register 1 and 2 (JDR1, JDR2) */

#define ADC_JDR_JDATA_SHIFT            (0)
#define ADC_JDR_JDATA_MASK             (0xffff << ADC_JDR_JDATA_SHIFT)

/* ADC analog watchdog 2 configuration register (AWD2CR) */

#define ADC_AWD2CR_CH_SHIFT            (0)                            /* Bits 0-18: Analog watchdog 2 channel selection */
#define ADC_AWD2CR_CH_MASK             (0x7ffff << ADC_AWD2CR_CH_SHIFT)
#  define ADC_AWD2CR_CH(n)             (1 << (n))                     /* Channel n=0..18 */

/* ADC analog watchdog 3 configuration register (AWD3CR) */

#define ADC_AWD3CR_CH_SHIFT            (0)                            /* Bits 0-18: Analog watchdog 2 channel selection */
#define ADC_AWD3CR_CH_MASK             (0x7ffff << ADC_AWD3CR_CH_SHIFT)
#  define ADC_AWD3CR_CH(n)             (1 << (n))                     /* Channel n=0..18 */

/* ADC differential mode selection register (DIFSEL) */

#define ADC_DIFSEL_CH_SHIFT            (0)                            /* Bits 0-18: Analog watchdog 2 channel selection */
#define ADC_DIFSEL_CH_MASK             (0x7ffff << ADC_DIFSEL_CH_SHIFT)
#  define ADC_DIFSEL_CH(n)             (1 << (n))                     /* Channel n=0..18 */

/* ADC calibration factors register (CALFACT) */

#define ADC_CALFACT_S_SHIFT            (0)                            /* Bits 0-6: Calibration factors in single-ended mode */
#define ADC_CALFACT_S_MASK             (0x7f << ADC_CALFACT_S_SHIFT)
#define ADC_CALFACT_D_SHIFT            (16)                           /* Bits 16-22: Calibration factors in differential mode */
#define ADC_CALFACT_D_MASK             (0x7f << ADC_CALFACT_D_SHIFT)

/* ADC gain compensation register (GCOMP) */

#define ADC_GCOMP_SHIFT                (0)                            /* Bits 0-13: Gain compensation coefficient */
#define ADC_GCOMP_MASK                 (0x3fff << ADC_GCOMP_SHIFT)

/* ADC12, ADC345 - Common status register (CSR) */

#define ADC_CSR_ADRDY_MST              (1 << 0)                       /* Bit 0: Master ADC ready */
#define ADC_CSR_EOSMP_MST              (1 << 1)                       /* Bit 1: End of Sampling phase flag (master ADC) */
#define ADC_CSR_EOC_MST                (1 << 2)                       /* Bit 2: End of regular conversion (master ADC) */
#define ADC_CSR_EOS_MST                (1 << 3)                       /* Bit 3: End of regular sequence flag (master ADC) */
#define ADC_CSR_OVR_MST                (1 << 4)                       /* Bit 4: Overrun flag (master ADC) */
#define ADC_CSR_JEOC_MST               (1 << 5)                       /* Bit 5: End of injected conversion flag (master ADC) */
#define ADC_CSR_JEOS_MST               (1 << 6)                       /* Bit 6: End of injected sequence flag (master ADC) */
#define ADC_CSR_AWD1_MST               (1 << 7)                       /* Bit 7: Analog watchdog 1 flag (master ADC) */
#define ADC_CSR_AWD2_MST               (1 << 8)                       /* Bit 8: Analog watchdog 2 flag (master ADC) */
#define ADC_CSR_AWD3_MST               (1 << 9)                       /* Bit 9: Analog watchdog 3 flag (master ADC) */
#define ADC_CSR_JQOVF_MST              (1 << 10)                      /* Bit 10: Injected Context Queue Overflow flag (master ADC) */
#define ADC_CSR_ADRDY_SLV              (1 << 16)                      /* Bit 16: Slave ADC ready */
#define ADC_CSR_EOSMP_SLV              (1 << 17)                      /* Bit 17: End of Sampling phase flag (slave ADC) */
#define ADC_CSR_EOC_SLV                (1 << 18)                      /* Bit 18: End of regular conversion (slave ADC) */
#define ADC_CSR_EOS_SLV                (1 << 19)                      /* Bit 19: End of regular sequence flag (slave ADC) */
#define ADC_CSR_OVR_SLV                (1 << 20)                      /* Bit 20: Overrun flag (slave ADC) */
#define ADC_CSR_JEOC_SLV               (1 << 21)                      /* Bit 21: End of injected conversion flag (slave ADC) */
#define ADC_CSR_JEOS_SLV               (1 << 22)                      /* Bit 22: End of injected sequence flag (slave ADC) */
#define ADC_CSR_AWD1_SLV               (1 << 23)                      /* Bit 23: Analog watchdog 1 flag (slave ADC) */
#define ADC_CSR_AWD2_SLV               (1 << 24)                      /* Bit 24: Analog watchdog 2 flag (slave ADC) */
#define ADC_CSR_AWD3_SLV               (1 << 25)                      /* Bit 25: Analog watchdog 3 flag (slave ADC) */
#define ADC_CSR_JQOVF_SLV              (1 << 26)                      /* Bit 26: Injected Context Queue Overflow flag (slave ADC) */

/* ADC12, ADC345 - Common control register (CCR) */

#define ADC_CCR_DUAL_SHIFT             (0)                            /* Bits 0-4: Dual ADC mode selection */
#define ADC_CCR_DUAL_MASK              (0x1f << ADC_CCR_DUAL_SHIFT)
#  define ADC_CCR_DUAL_IND             (0x0 << ADC_CCR_DUAL_SHIFT)    /* 00000: Independent mode */
#  define ADC_CCR_DUAL_DUAL            (0x1 << ADC_CCR_DUAL_SHIFT)    /* 00001: Dual mode, master/slave ADCs together */
#  define ADC_CCR_DUAL_SIMINJ          (0x1 << ADC_CCR_DUAL_SHIFT)    /* 00001: Combined regular sim. + injected sim. */
#  define ADC_CCR_DUAL_SIMALT          (0x2 << ADC_CCR_DUAL_SHIFT)    /* 00010: Combined regular sim. + alternate trigger */
#  define ADC_CCR_DUAL_INTINJ          (0x3 << ADC_CCR_DUAL_SHIFT)    /* 00011: Combined interl. mode + injected sim. */
#  define ADC_CCR_DUAL_INJECTED        (0x5 << ADC_CCR_DUAL_SHIFT)    /* 00101: Injected simultaneous mode only */
#  define ADC_CCR_DUAL_SIM             (0x6 << ADC_CCR_DUAL_SHIFT)    /* 00110: Regular simultaneous mode only */
#  define ADC_CCR_DUAL_INTERLEAVE      (0x7 << ADC_CCR_DUAL_SHIFT)    /* 00111: Interleaved mode only */
#  define ADC_CCR_DUAL_ALT             (0x9 << ADC_CCR_DUAL_SHIFT)    /* 01001: Alternate trigger mode only */
#define ADC_CCR_DELAY_SHIFT            (8)                            /* Bits 8-11: Delay between 2 sampling phases */
#define ADC_CCR_DELAY_MASK             (0xf << ADC_CCR_DELAY_SHIFT)   /* n * TADCCLK, 1-13 */
#  define ADC_CCR_DELAY(n)             (((n) - 1) << ADC_CCR_DELAY_SHIFT)
#define ADC_CCR_DMACFG                 (1 << 13)                      /* Bit 13: DMA configuration (for dual ADC mode) */
#define ADC_CCR_MDMA_SHIFT             (14)                           /* Bits 14-15: Direct memory access mode for dual ADC mode */
#define ADC_CCR_MDMA_MASK              (0x3 << ADC_CCR_MDMA_SHIFT)
#  define ADC_CCR_MDMA_DISABLED        (0x0 << ADC_CCR_MDMA_SHIFT)    /* MDMA mode disabled */
#  define ADC_CCR_MDMA_10_12           (0x2 << ADC_CCR_MDMA_SHIFT)    /* MDMA mode enabled (12 / 10-bit) */
#  define ADC_CCR_MDMA_6_8             (0x3 << ADC_CCR_MDMA_SHIFT)    /* MDMA mode enabled (8 / 6-bit) */
#define ADC_CCR_CKMODE_SHIFT           (16)                           /* Bits 16-17: ADC clock mode */
#define ADC_CCR_CKMODE_MASK            (0x3 << ADC_CCR_CKMODE_SHIFT)
#  define ADC_CCR_CKMODE_ASYNCH        (0x0 << ADC_CCR_CKMODE_SHIFT)  /* Asynchronous clock mode */
#  define ADC_CCR_CKMODE_SYNCH_DIV1    (0x1 << ADC_CCR_CKMODE_SHIFT)  /* Synchronous clock mode divided by 1 */
#  define ADC_CCR_CKMODE_SYNCH_DIV2    (0x2 << ADC_CCR_CKMODE_SHIFT)  /* Synchronous clock mode divided by 2 */
#  define ADC_CCR_CKMODE_SYNCH_DIV4    (0x3 << ADC_CCR_CKMODE_SHIFT)  /* Synchronous clock mode divided by 4 */
#define ADC_CCR_PRESC_SHIFT            (18)                           /* Bits 18-21: ADC prescaler */
#define ADC_CCR_PRESC_MASK             (0xf << ADC_CCR_PRESC_SHIFT)
#  define ADC_CCR_PRESC_1              (0x0 << ADC_CCR_PRESC_SHIFT)   /* 0000: Input ADC clock not divided */
#  define ADC_CCR_PRESC_2              (0x1 << ADC_CCR_PRESC_SHIFT)   /* 0001: Input ADC clock divided by 2 */
#  define ADC_CCR_PRESC_4              (0x2 << ADC_CCR_PRESC_SHIFT)   /* 0010: Input ADC clock divided by 4 */
#  define ADC_CCR_PRESC_6              (0x3 << ADC_CCR_PRESC_SHIFT)   /* 0011: Input ADC clock divided by 6 */
#  define ADC_CCR_PRESC_8              (0x4 << ADC_CCR_PRESC_SHIFT)   /* 0100: Input ADC clock divided by 8 */
#  define ADC_CCR_PRESC_10             (0x5 << ADC_CCR_PRESC_SHIFT)   /* 0101: Input ADC clock divided by 10 */
#  define ADC_CCR_PRESC_12             (0x6 << ADC_CCR_PRESC_SHIFT)   /* 0110: Input ADC clock divided by 12 */
#  define ADC_CCR_PRESC_16             (0x7 << ADC_CCR_PRESC_SHIFT)   /* 0111: Input ADC clock divided by 16 */
#  define ADC_CCR_PRESC_32             (0x8 << ADC_CCR_PRESC_SHIFT)   /* 1000: Input ADC clock divided by 32 */
#  define ADC_CCR_PRESC_64             (0x9 << ADC_CCR_PRESC_SHIFT)   /* 1001: Input ADC clock divided by 64 */
#  define ADC_CCR_PRESC_128            (0xa << ADC_CCR_PRESC_SHIFT)   /* 1010: Input ADC clock divided by 128 */
#  define ADC_CCR_PRESC_256            (0xb << ADC_CCR_PRESC_SHIFT)   /* 1011: Input ADC clock divided by 256 */
#define ADC_CCR_VREFEN                 (1 << 22)                      /* Bit 22: VREFINT enable */
#define ADC_CCR_TSEN                   (1 << 23)                      /* Bit 23: Temperature sensor enable */
#define ADC_CCR_VBATEN                 (1 << 24)                      /* Bit 24: VBAT enable */

/* Common regular data register for dual mode */

#define ADC_CDR_RDATA_MST_SHIFT        (0)                            /* Bits 0-15: Regular data of the Master ADC */
#define ADC_CDR_RDATA_MST_MASK         (0xffff << ADC_CDR_RDATA_MST_SHIFT)
#define ADC_CDR_RDATA_SLV_SHIFT        (16)                           /* Bits 16-31: Regular data of the Slave ADC */
#define ADC_CDR_RDATA_SLV_MASK         (0xffff << ADC_CDR_RDATA_SLV_SHIFT)

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32_ADC_V2G4_H */
