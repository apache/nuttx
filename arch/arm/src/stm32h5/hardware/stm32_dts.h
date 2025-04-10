/****************************************************************************
 * arch/arm/src/stm32h5/hardware/stm32_dts.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_DTS_H
#define __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_DTS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Preprocessor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_DTS_CFGR1_OFFSET     0x0000
#define STM32_DTS_T0VALR1_OFFSET   0x0008
#define STM32_DTS_RAMPVALR_OFFSET  0x0010
#define STM32_DTS_ITR1_OFFSET      0x0014
#define STM32_DTS_DR_OFFSET        0x001C
#define STM32_DTS_SR_OFFSET        0x0020
#define STM32_DTS_ITENR_OFFSET     0x0024
#define STM32_DTS_ICIFR_OFFSET     0x0028
#define STM32_DTS_OR_OFFSET        0x002C

/* Register Addresses *******************************************************/

#define STM32_DTS_CFGR1     (STM32_DTS_BASE+STM32_DTS_CFGR1_OFFSET)
#define STM32_DTS_T0VALR1   (STM32_DTS_BASE+STM32_DTS_T0VALR1_OFFSET)
#define STM32_DTS_RAMPVALR  (STM32_DTS_BASE+STM32_DTS_RAMPVALR_OFFSET)
#define STM32_DTS_ITR1      (STM32_DTS_BASE+STM32_DTS_ITR1_OFFSET)
#define STM32_DTS_DR        (STM32_DTS_BASE+STM32_DTS_DR_OFFSET)
#define STM32_DTS_SR        (STM32_DTS_BASE+STM32_DTS_SR_OFFSET)
#define STM32_DTS_ITENR     (STM32_DTS_BASE+STM32_DTS_ITENR_OFFSET)
#define STM32_DTS_ICIFR     (STM32_DTS_BASE+STM32_DTS_ICIFR_OFFSET)
#define STM32_DTS_OR        (STM32_DTS_BASE+STM32_DTS_OR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Temperature sensor configuration register 1 */

#define DTS_CFGR1_TS1_EN                      (1 << 0)  /* Temperature sensor 1 enable */
#define DTS_CFGR1_TS1_START                   (1 << 4)  /* Start frequency measurement on temp sensor 1*/

#define DTS_CFGR1_TS1_INTRIG_SEL_SHIFT        (8)       /* Bits 8-11: Input trigger selection */
#define DTS_CFGR1_TS1_INTRIG_SEL_MASK         (0xf << DTS_CFGR1_TS1_INTRIG_SEL_SHIFT)
#  define DTS_CFGR1_TS1_INTRIG_SEL_LPTIM1_CH1 (0x1 << DTS_CFGR1_TS1_INTRIG_SEL_SHIFT)
#  define DTS_CFGR1_TS1_INTRIG_SEL_LPTIM2_CH1 (0x2 << DTS_CFGR1_TS1_INTRIG_SEL_SHIFT)
#  define DTS_CFGR1_TS1_INTRIG_SEL_LPTIM3_CH1 (0x3 << DTS_CFGR1_TS1_INTRIG_SEL_SHIFT)
#  define DTS_CFGR1_TS1_INTRIG_SEL_EXTI13     (0x4 << DTS_CFGR1_TS1_INTRIG_SEL_SHIFT)

#define DTS_CFGR1_TS1_SMP_TIME_SHIFT          (16)      /* Bits 16-19: Sampling time */
#define DTS_CFGR1_TS1_SMP_TIME_MASK           (0xf << DTS_CFGR1_TS1_SMP_TIME_SHIFT)
#  define DTS_CFGR1_TS1_SMP_TIME(n)           ((n & 0xf) << DTS_CFGR1_TS1_SMP_TIME_SHIFT)

#define DTS_CFGR1_REFCLK_SEL                  (1 << 20) /* Reference clock selection */
#define DTS_CFGR1_Q_MEAS_OPT                  (1 << 21) /* Quick measurement option */

#define DTS_CFGR1_HSREF_CLK_DIV_SHIFT         (24)      /* Bits 24-30: High speed clock division ratio */
#define DTS_CFGR1_HSREF_CLK_DIV_MASK          (0x7f << DTS_CFGR1_HSREF_CLK_DIV_SHIFT)
#  define DTS_CFGR1_HSREF_CLK_DIV_RATIO(r)    ((r & 0x7f) << DTS_CFGR1_HSREF_CLK_DIV_SHIFT) /* Division ration is 1/r, r=0,1 means no divider */

/* Temperature sensor T0 value register 1 */

#define DTS_T0VALR1_TS1_FMT0_SHIFT            (0)       /* Engineering value for frequency at T0 */
#define DTS_T0VALR1_TS1_FMT0_MASK             (0xff << DTS_T0VALR1_TS1_FMT0_SHIFT)
#define DTS_T0VALR1_TS1_T0_SHIFT              (16)      /* Engineering value of T0 */
#define DTS_T0VALR1_TS1_T0_MASK               (0b11 << DTS_T0VALR1_TS1_T0_SHIFT)

/* Temperature sensor ramp value register */

#define DTS_RAMPVALR_TS1_RAMP_COEFF_SHIFT     (0)       /* Engineering value of ramp coefficient */
#define DTS_RAMPVALR_TS1_RAMP_COEFF_MASK      (0xff << DTS_RAMPVALR_TS1_RAMP_COEFF_SHIFT)

/* Temperature sensor interrupt threshold register 1 */

#define DTS_ITR1_TS1_LITTHD_SHIFT             (0)       /* High interrupt threshold */
#define DTS_ITR1_TS1_LITTHD_MASK              (0xff << DTS_ITR1_TS1_LITTHD_SHIFT)
#define DTS_ITR1_TS1_HITTHD_SHIFT             (16)      /* Low interrupt threshold */
#define DTS_ITR1_TS1_HITTHD_MASK              (0xff << DTS_ITR1_TS1_HITTHD_SHIFT)

/* Temperature sensor data register */

#define DTS_DR_TS1_MFREQ_SHIFT  (0)
#define DTS_DR_TS1_MFREQ_MASK   (0xff << DTS_DR_TS1_MFREQ_SHIFT)

/* Temperature sensor status register */

#define DTS_SR_TS1_ITEF         (1 << 0)      /* Synchronized Interrupt flag: end of measurement */
#define DTS_SR_TS1_ITLF         (1 << 1)      /* Synchronized Interrupt flag: low threshold */
#define DTS_SR_TS1_ITHF         (1 << 2)      /* Synchronized Interrupt flag: high threshold */
#define DTS_SR_TS1_AITEF        (1 << 4)      /* Asynchronous Interrupt flag: end of measurement */
#define DTS_SR_TS1_AITLF        (1 << 5)      /* Asynchronous Interrupt flag: low threshold */
#define DTS_SR_TS1_AITHF        (1 << 6)      /* Asynchronous Interrupt flag: high threshold */
#define DTS_SR_TS1_RDY          (1 << 15)     /* Temperature sensor ready flag */

/* Temperature sensor interrupt enable register */

#define DTS_ITENR_ITEEN         (1 << 0)      /* Synchronized Interrupt Enable: end of measurement */
#define DTS_ITENR_ITLEN         (1 << 1)      /* Synchronized Interrupt Enable: low threshold */
#define DTS_ITENR_ITHEN         (1 << 2)      /* Synchronized Interrupt Enable: high threshold */
#define DTS_ITENR_AITEEN        (1 << 4)      /* Asynchronous Interrupt Enable: end of measurement */
#define DTS_ITENR_AITLEN        (1 << 5)      /* Asynchronous Interrupt Enable: low threshold */
#define DTS_ITENR_AITHEN        (1 << 6)      /* Asynchronous Interrupt Enable: high threshold*/

/* Temperature sensor clear interrupt flag register */

#define DTS_ICIFR_CITEF         (1 << 0)      /* Interrupt clear flag: end of measurement */
#define DTS_ICIFR_CITLF         (1 << 1)      /* Interrupt clear flag: low threshold */
#define DTS_ICIFR_CITHF         (1 << 2)      /* Interrupt clear flag: high threshold */
#define DTS_ICIFR_CAITEF        (1 << 4)      /* Asynchronous interrupt clear flag: end of measurement */
#define DTS_ICIFR_CAITLF        (1 << 5)      /* Asynchronous interrupt clear flag: low threshold */
#define DTS_ICIFR_CAITHF        (1 << 6)      /* Asynchronous interrupt clear flag: high threshold */

/* Temperature sensor option register */

#define DTS_OR_TS_OP_SHIFT      (0)           /* Bits 0-31: General purpose option bits */
#define DTS_OR_TS_OP_MASK       (0xffff)

#endif /* __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_DTS_H */