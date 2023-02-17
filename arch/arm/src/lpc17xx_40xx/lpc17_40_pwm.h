/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc17_40_pwm.h
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

#ifndef __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_PWM_H
#define __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/lpc17_40_pwm.h"
#include "hardware/lpc17_40_mcpwm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PLL0CLK = CCLK * CCLK divider */

#define LPC17_40_PWM_CLOCK (LPC17_40_CCLK * BOARD_CCLKCFG_DIVIDER)

#ifdef CONFIG_LPC17_40_PWM1_CHANNEL1
#  define LPC17_40_PWM1_CHANNEL1 1
#else
#define LPC17_40_PWM1_CHANNEL1 0
#endif

#ifdef CONFIG_LPC17_40_PWM1_CHANNEL2
#define LPC17_40_PWM1_CHANNEL2 1
#else
#define LPC17_40_PWM1_CHANNEL2 0
#endif

#ifdef CONFIG_LPC17_40_PWM1_CHANNEL3
#define LPC17_40_PWM1_CHANNEL3 1
#else
#define LPC17_40_PWM1_CHANNEL3 0
#endif

#ifdef CONFIG_LPC17_40_PWM1_CHANNEL4
#define LPC17_40_PWM1_CHANNEL4 1
#else
#define LPC17_40_PWM1_CHANNEL4 0
#endif

#ifdef CONFIG_LPC17_40_PWM1_CHANNEL5
#define LPC17_40_PWM1_CHANNEL5 1
#else
#define LPC17_40_PWM1_CHANNEL5 0
#endif

#ifdef CONFIG_LPC17_40_PWM1_CHANNEL6
#define LPC17_40_PWM1_CHANNEL6 1
#else
#define LPC17_40_PWM1_CHANNEL6 0
#endif

#define LPC17_40_PWM1_NCHANNELS (LPC17_40_PWM1_CHANNEL1 + \
                                 LPC17_40_PWM1_CHANNEL2 + \
                                 LPC17_40_PWM1_CHANNEL3 + \
                                 LPC17_40_PWM1_CHANNEL4 + \
                                 LPC17_40_PWM1_CHANNEL5 + \
                                 LPC17_40_PWM1_CHANNEL6)

#if CONFIG_PWM_NCHANNELS > LPC17_40_PWM1_NCHANNELS
#  error "PWM subsystem has more channels then physical channels enabled"
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_PWM_H */
