/****************************************************************************
 * arch/arm/src/samv7/sam_pwm.h
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

#ifndef __ARCH_ARM_SRC_SAMV7_SAM_PWM_H
#define __ARCH_ARM_SRC_SAMV7_SAM_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/timers/pwm.h>

#include "hardware/sam_pwm.h"

#ifdef CONFIG_SAMV7_PWM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SAMV7_PWM0_CH0
#define PWM0_CH0 1
#else
#define PWM0_CH0 0
#endif

#ifdef CONFIG_SAMV7_PWM0_CH1
#define PWM0_CH1 1
#else
#define PWM0_CH1 0
#endif

#ifdef CONFIG_SAMV7_PWM0_CH2
#define PWM0_CH2 1
#else
#define PWM0_CH2 0
#endif

#ifdef CONFIG_SAMV7_PWM0_CH3
#define PWM0_CH3 1
#else
#define PWM0_CH3 0
#endif

#define PWM0_NCHANNELS (PWM0_CH0 + PWM0_CH1 + PWM0_CH2 + PWM0_CH3)

#ifdef CONFIG_SAMV7_PWM1_CH0
#define PWM1_CH0 1
#else
#define PWM1_CH0 0
#endif

#ifdef CONFIG_SAMV7_PWM1_CH1
#define PWM1_CH1 1
#else
#define PWM1_CH1 0
#endif

#ifdef CONFIG_SAMV7_PWM1_CH2
#define PWM1_CH2 1
#else
#define PWM1_CH2 0
#endif

#ifdef CONFIG_SAMV7_PWM1_CH3
#define PWM1_CH3 1
#else
#define PWM1_CH3 0
#endif

#define PWM1_NCHANNELS (PWM1_CH0 + PWM1_CH1 + PWM1_CH2 + PWM1_CH3)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Function: sam_pwminitialize
 *
 * Description:
 *   Initialize the PWM module for use with the upper level PWM driver.
 *
 * Input Parameters:
 *   pwm - a number identifying the PWM driver.
 *
 * Returned Value:
 *   A pointer to the lower half PWM driver is returned on success,
 *   NULL on failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *sam_pwminitialize(int pwm);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_SAMV7_PWM */
#endif /* __ARCH_ARM_SRC_SAMV7_SAM_PWM_H */
