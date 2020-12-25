/****************************************************************************
 * arch/arm/src/armv7-m/systick.h
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

#ifndef __ARCH_ARM_SRC_ARMV7_M_SYSTICK_H
#define __ARCH_ARM_SRC_ARMV7_M_SYSTICK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/timers/timer.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: systick_initialize
 *
 * Description:
 *   This function initialize SYSTICK hardware module which is part of NVIC
 *   and return an instance of a "lower half" timer interface.
 *
 * Input parameters:
 *   coreclk - false if SYSTICK working clock come from the external
 *     reference clock, otherwise true.
 *   freq - The clock frequency in Hz. If freq is zero, calculate the value
 *     from NVIC_SYSTICK_CALIB register.
 *   minor - The timer driver minor number, skip the registration if minor
 *     < 0.
 *
 * Returned Value:
 *   On success, a non-NULL timer_lowerhalf_s is returned to the caller.
 *   In the event of any failure, a NULL value is returned.
 *
 ****************************************************************************/

#ifdef CONFIG_ARMV7M_SYSTICK
struct timer_lowerhalf_s *systick_initialize(bool coreclk, unsigned int freq,
                                             int minor);
#else
#  define systick_initialize(coreclk, freq, minor) NULL
#endif /* CONFIG_ARMV7M_SYSTICK */

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_ARMV7_M_SYSTICK_H */
