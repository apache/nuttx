/****************************************************************************
 * arch/arm/src/tlsr82/hardware/tlsr82_adc.h
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

#ifndef __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_ADC_H
#define __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/tlsr82/chip.h>

#include "hardware/tlsr82_register.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ADC_VREF_REG                       (0xea)
#define ADC_CHAN_REG                       (0xeb)
#define ADC_MODE_REG                       (0xec)
#define ADC_SAMP0_REG                      (0xee)
#define ADC_SAMP1_REG                      (0xef)
#define ADC_SAMP2_REG                      (0xf0)
#define ADC_SAMP3_REG                      (0xf1)
#define ADC_CTRL0_REG                      (0xf2)
#define ADC_CTRL1_REG                      (0xf3)
#define ADC_CLKDIV_REG                     (0xf4)
#define ADC_STATUS_REG                     (0xf6)
#define ADC_DATAL_REG                      (0xf7)
#define ADC_DATAH_REG                      (0xf8)
#define ADC_DIVIDER_REG                    (0xf9)
#define ADC_SCALE_REG                      (0xfa)
#define ADC_PAG_CTRL_REG                   (0xfc)

#define ADC_CLK_REG                        (0x82)

/* ADC Reference Voltage definition */

#define ADC_VREF_SHIFT                     0
#define ADC_VREF_MASK                      (0x3 << ADC_VREF_SHIFT)
#define ADC_VREF_RSVD1                     (0x0 << ADC_VREF_SHIFT)
#define ADC_VREF_0P9V                      (0x1 << ADC_VREF_SHIFT)
#define ADC_VREF_1P2V                      (0x2 << ADC_VREF_SHIFT)
#define ADC_VREF_RSVD2                     (0x3 << ADC_VREF_SHIFT)

/* ADC analog input positive and negative channel definition */

#define ADC_CHAN_POS_SHIFT                 4
#define ADC_CHAN_POS_MASK                  (0xf << ADC_CHAN_POS_SHIFT)
#define ADC_CHAN_POS_NOINPUT               (0  << ADC_CHAN_POS_SHIFT)
#define ADC_CHAN_POS_B0                    (1  << ADC_CHAN_POS_SHIFT)
#define ADC_CHAN_POS_B1                    (2  << ADC_CHAN_POS_SHIFT)
#define ADC_CHAN_POS_B2                    (3  << ADC_CHAN_POS_SHIFT)
#define ADC_CHAN_POS_B3                    (4  << ADC_CHAN_POS_SHIFT)
#define ADC_CHAN_POS_B4                    (5  << ADC_CHAN_POS_SHIFT)
#define ADC_CHAN_POS_B5                    (6  << ADC_CHAN_POS_SHIFT)
#define ADC_CHAN_POS_B6                    (7  << ADC_CHAN_POS_SHIFT)
#define ADC_CHAN_POS_B7                    (8  << ADC_CHAN_POS_SHIFT)
#define ADC_CHAN_POS_C4                    (9  << ADC_CHAN_POS_SHIFT)
#define ADC_CHAN_POS_C5                    (10 << ADC_CHAN_POS_SHIFT)
#define ADC_CHAN_POS_PGA0                  (11 << ADC_CHAN_POS_SHIFT)
#define ADC_CHAN_POS_PGA1                  (12 << ADC_CHAN_POS_SHIFT)
#define ADC_CHAN_POS_TEMSENSOR             (13 << ADC_CHAN_POS_SHIFT)
#define ADC_CHAN_POS_TEMSENSOR_EE          (14 << ADC_CHAN_POS_SHIFT)
#define ADC_CHAN_POS_VBAT                  (15 << ADC_CHAN_POS_SHIFT)

#define ADC_CHAN_NEG_SHIFT                 0
#define ADC_CHAN_NEG_MASK                  (0xf << ADC_CHAN_NEG_SHIFT)
#define ADC_CHAN_NEG_NOINPUT               (0  << ADC_CHAN_NEG_SHIFT)
#define ADC_CHAN_NEG_B0                    (1  << ADC_CHAN_NEG_SHIFT)
#define ADC_CHAN_NEG_B1                    (2  << ADC_CHAN_NEG_SHIFT)
#define ADC_CHAN_NEG_B2                    (3  << ADC_CHAN_NEG_SHIFT)
#define ADC_CHAN_NEG_B3                    (4  << ADC_CHAN_NEG_SHIFT)
#define ADC_CHAN_NEG_B4                    (5  << ADC_CHAN_NEG_SHIFT)
#define ADC_CHAN_NEG_B5                    (6  << ADC_CHAN_NEG_SHIFT)
#define ADC_CHAN_NEG_B6                    (7  << ADC_CHAN_NEG_SHIFT)
#define ADC_CHAN_NEG_B7                    (8  << ADC_CHAN_NEG_SHIFT)
#define ADC_CHAN_NEG_C4                    (9  << ADC_CHAN_NEG_SHIFT)
#define ADC_CHAN_NEG_C5                    (10 << ADC_CHAN_NEG_SHIFT)
#define ADC_CHAN_NEG_PGA0                  (11 << ADC_CHAN_NEG_SHIFT)
#define ADC_CHAN_NEG_PGA1                  (12 << ADC_CHAN_NEG_SHIFT)
#define ADC_CHAN_NEG_TEMSENSOR             (13 << ADC_CHAN_NEG_SHIFT)
#define ADC_CHAN_NEG_TEMSENSOR_EE          (14 << ADC_CHAN_NEG_SHIFT)
#define ADC_CHAN_NEG_GND                   (15 << ADC_CHAN_NEG_SHIFT)

/* ADC Mode, Resolution definition */

#define ADC_MODE_RES_SHIFT                 0
#define ADC_MODE_RES_MASK                  (0x3 << ADC_MODE_RES_SHIFT)
#define ADC_MODE_RES_8BIT                  (0x0 << ADC_MODE_RES_SHIFT)
#define ADC_MODE_RES_10BIT                 (0x1 << ADC_MODE_RES_SHIFT)
#define ADC_MODE_RES_12BIT                 (0x2 << ADC_MODE_RES_SHIFT)
#define ADC_MODE_RES_14BIT                 (0x3 << ADC_MODE_RES_SHIFT)

#define ADC_MODE_INPUT_SHIFT               6
#define ADC_MODE_INPUT_MASK                (0x1 << ADC_MODE_INPUT_SHIFT)
#define ADC_MODE_INPUT_RSVD                (0x0 << ADC_MODE_INPUT_SHIFT)
#define ADC_MODE_INPUT_DIFF                (0x1 << ADC_MODE_INPUT_SHIFT)

/* ADC Sample 0 defnition
 * - Sample cycle
 */

/* ADC Sample cycle definition */

#define ADC_SAMP0_CYCLE_SHIFT              0
#define ADC_SAMP0_CYCLE_MASK               (0xf << ADC_SAMP0_CYCLE_SHIFT)
#define ADC_SAMP0_CYCLE_3                  (0x0 << ADC_SAMP0_CYCLE_SHIFT)
#define ADC_SAMP0_CYCLE_6                  (0x1 << ADC_SAMP0_CYCLE_SHIFT)
#define ADC_SAMP0_CYCLE_9                  (0x2 << ADC_SAMP0_CYCLE_SHIFT)
#define ADC_SAMP0_CYCLE_48                 (0xf << ADC_SAMP0_CYCLE_SHIFT)

/* ADC Control 0 definition
 * - Channal enable
 * - Sample length
 */

/* ADC Channel enabel definition */

#define ADC_CTRL0_CHANEN_SHIFT             2
#define ADC_CTRL0_CHANEN_MASK              (0x1 << ADC_CTRL0_CHANEN_SHIFT)
#define ADC_CTRL0_CHANEN_ENABLE            (0x1 << ADC_CTRL0_CHANEN_SHIFT)
#define ADC_CTRL0_CHANEN_DISABLE           (0x0 << ADC_CTRL0_CHANEN_SHIFT)

/* ADC Sample length definition */

#define ADC_CTRL0_SAMPLEN_SHIFT            4
#define ADC_CTRL0_SAMPLEN_MASK             (0x3 << ADC_CTRL0_SAMPLEN_SHIFT)

/* ADC Control 1 definition
 * - Adc Sample Control
 */

#define ADC_CTRL1_SAMP_SHIFT               0
#define ADC_CTRL1_SAMP_MASK                (0x1 << ADC_CTRL1_SAMP_SHIFT)
#define ADC_CTRL1_SAMP_ON                  (0x0 << ADC_CTRL1_SAMP_SHIFT)
#define ADC_CTRL1_SAMP_OFF                 (0x1 << ADC_CTRL1_SAMP_SHIFT)

/* ADC clock divide definition */

#define ADC_CLKDIV_SHIFT                   0
#define ADC_CLKDIV_MASK                    (0x3 << ADC_CLKDIV_SHIFT)
#define ADC_CLKDIV_0                       (0x0 << ADC_CLKDIV_SHIFT)
#define ADC_CLKDIV_1                       (0x1 << ADC_CLKDIV_SHIFT)
#define ADC_CLKDIV_2                       (0x2 << ADC_CLKDIV_SHIFT)
#define ADC_CLKDIV_3                       (0x3 << ADC_CLKDIV_SHIFT)

/* The adc vbat divider specification can not found in datasheet, below
 * definition follows the sdk code.
 */

#define ADC_DIVIDER_SEL_SHIFT              2
#define ADC_DIVIDER_SEL_MASK               (0x3 << ADC_DIVIDER_SEL_SHIFT)
#define ADC_DIVIDER_SEL_OFF                (0x0 << ADC_DIVIDER_SEL_SHIFT)
#define ADC_DIVIDER_SEL_1F3                (0x2 << ADC_DIVIDER_SEL_SHIFT)

/* ADC input pre-scaline select */

#define ADC_SCALE_SHIFT                    6
#define ADC_SCALE_MASK                     (0x3 << ADC_SCALE_SHIFT)
#define ADC_SCALE_1                        (0x0 << ADC_SCALE_SHIFT)
#define ADC_SCALE_RSVD1                    (0x1 << ADC_SCALE_SHIFT)
#define ADC_SCALE_RSVD2                    (0x2 << ADC_SCALE_SHIFT)
#define ADC_SCALE_1F8                      (0x3 << ADC_SCALE_SHIFT)

#define ADC_POWER_SHIFT                    5
#define ADC_POWER_MASK                     (0x1 << ADC_POWER_SHIFT)
#define ADC_POWER_UP                       (0x1 << ADC_POWER_SHIFT)
#define ADC_POWER_DOWN                     (0x0 << ADC_POWER_SHIFT)

#define ADC_CLK_24M_EN_SHIFT               6
#define ADC_CLK_24M_EN_MASK                (0x1 << ADC_CLK_24M_EN_SHIFT)
#define ADC_CLK_24M_EN                     (0x1 << ADC_CLK_24M_EN_SHIFT)

#endif /* __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_ADC_H */
