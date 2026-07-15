/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_capture.h
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

#ifndef __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_CAPTURE_H
#define __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_CAPTURE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/timers/capture.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The capture driver uses the PWM input mode of the timer:  channel 0 is
 * mapped on CI0 and captures the period, channel 1 is mapped on the same
 * input and captures the pulse width, while the slave mode controller
 * resets the counter on every rising edge.  Only TIMER0, TIMER1 and TIMER2
 * have a slave mode controller and two capture channels, so only those
 * timers can be used.
 *
 * The input pin of channel 0 is provided by the board logic in board.h:
 *
 *   GPIO_TIMER0_CH0IN, GPIO_TIMER1_CH0IN, GPIO_TIMER2_CH0IN
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
 * Name: gd32_cap_initialize
 *
 * Description:
 *   Initialize one timer for use with the upper level capture driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer use: {0,1,2}
 *
 * Returned Value:
 *   On success, a pointer to the lower half capture driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct cap_lowerhalf_s *gd32_cap_initialize(int timer);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_CAPTURE_H */
