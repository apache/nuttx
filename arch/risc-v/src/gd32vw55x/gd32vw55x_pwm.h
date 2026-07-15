/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_pwm.h
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

#ifndef __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_PWM_H
#define __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/timers/pwm.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The GPIO pin configurations used by the PWM outputs are provided by the
 * board logic in board.h, using the following names.  A pin configuration
 * that is not defined simply means that the corresponding output is not
 * routed to a pin on this board and it will not be configured:
 *
 *   GPIO_TIMER0_CH0OUT  ... GPIO_TIMER0_CH3OUT
 *   GPIO_TIMER0_CH0NOUT ... GPIO_TIMER0_CH2NOUT  (complementary outputs)
 *   GPIO_TIMER1_CH0OUT  ... GPIO_TIMER1_CH3OUT
 *   GPIO_TIMER2_CH0OUT  ... GPIO_TIMER2_CH3OUT
 *   GPIO_TIMER15_CH0OUT, GPIO_TIMER15_CH0NOUT
 *   GPIO_TIMER16_CH0OUT, GPIO_TIMER16_CH0NOUT
 *
 * TIMER5 is a basic timer without any channel and cannot generate PWM.
 */

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: gd32_pwminitialize
 *
 * Description:
 *   Initialize one timer for use with the upper level PWM driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer use: {0,1,2,15,16}
 *
 * Returned Value:
 *   On success, a pointer to the lower half PWM driver is returned.  NULL
 *   is returned on any failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *gd32_pwminitialize(int timer);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_PWM_H */
