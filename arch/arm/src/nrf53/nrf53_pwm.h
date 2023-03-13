/****************************************************************************
 * arch/arm/src/nrf53/nrf53_pwm.h
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

#ifndef __ARCH_ARM_SRC_NRF53_NRF53_PWM_H
#define __ARCH_ARM_SRC_NRF53_NRF53_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/timers/pwm.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Enable the specified PWM channel if multichannel PWM is disabled */

#ifndef CONFIG_NRF53_PWM_MULTICHAN

#  ifdef CONFIG_NRF53_PWM0
#    if !defined(CONFIG_NRF53_PWM0_CHANNEL)
#      error "CONFIG_NRF53_PWM0_CHANNEL must be provided"
#    elif CONFIG_NRF53_PWM0_CHANNEL == 0
#      define CONFIG_NRF53_PWM0_CH0 1
#    elif CONFIG_NRF53_PWM0_CHANNEL == 1
#      define CONFIG_NRF53_PWM0_CH1 1
#    elif CONFIG_NRF53_PWM0_CHANNEL == 2
#      define CONFIG_NRF53_PWM0_CH2 1
#    elif CONFIG_NRF53_PWM0_CHANNEL == 3
#      define CONFIG_NRF53_PWM0_CH3 1
#    else
#      error "Unsupported value of CONFIG_NRF53_PWM0_CHANNEL"
#    endif
#  endif

#  ifdef CONFIG_NRF53_PWM1
#    if !defined(CONFIG_NRF53_PWM1_CHANNEL)
#      error "CONFIG_NRF53_PWM1_CHANNEL must be provided"
#    elif CONFIG_NRF53_PWM1_CHANNEL == 0
#      define CONFIG_NRF53_PWM1_CH0 1
#    elif CONFIG_NRF53_PWM1_CHANNEL == 1
#      define CONFIG_NRF53_PWM1_CH1 1
#    elif CONFIG_NRF53_PWM1_CHANNEL == 2
#      define CONFIG_NRF53_PWM1_CH2 1
#    elif CONFIG_NRF53_PWM1_CHANNEL == 3
#      define CONFIG_NRF53_PWM1_CH3 1
#    else
#      error "Unsupported value of CONFIG_NRF53_PWM1_CHANNEL"
#    endif
#  endif

#  ifdef CONFIG_NRF53_PWM2
#    if !defined(CONFIG_NRF53_PWM2_CHANNEL)
#      error "CONFIG_NRF53_PWM2_CHANNEL must be provided"
#    elif CONFIG_NRF53_PWM2_CHANNEL == 0
#      define CONFIG_NRF53_PWM2_CH0 1
#    elif CONFIG_NRF53_PWM2_CHANNEL == 1
#      define CONFIG_NRF53_PWM2_CH1 1
#    elif CONFIG_NRF53_PWM2_CHANNEL == 2
#      define CONFIG_NRF53_PWM2_CH2 1
#    elif CONFIG_NRF53_PWM2_CHANNEL == 3
#      define CONFIG_NRF53_PWM2_CH3 1
#    else
#      error "Unsupported value of CONFIG_NRF53_PWM2_CHANNEL"
#    endif
#  endif

#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_pwminitialize
 *
 * Description:
 *   Initialize one timer for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   pwm - A number identifying the pwm instance.
 *
 * Returned Value:
 *   On success, a pointer to the NRF53 lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *nrf53_pwminitialize(int pwm);

#endif /* __ARCH_ARM_SRC_NRF53_NRF53_PWM_H */
