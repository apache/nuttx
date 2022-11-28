/****************************************************************************
 * arch/arm/src/armv7-r/arm_timer.h
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

#ifndef __ARCH_ARM_SRC_ARMV7_R_ARM_TIMER_H
#define __ARCH_ARM_SRC_ARMV7_R_ARM_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/timers/oneshot.h>

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
 * Name: arm_timer_initialize
 *
 * Description:
 *   This function initialize generic timer hardware module
 *   and return an instance of a "lower half" timer interface.
 *
 * Input parameters:
 *   freq - The clock frequency in Hz. If freq is zero, get the value
 *     from CNTFRQ register.
 *
 * Returned Value:
 *   On success, a non-NULL oneshot_lowerhalf_s is returned to the caller.
 *   In the event of any failure, a NULL value is returned.
 *
 ****************************************************************************/

#ifdef CONFIG_ARMV7R_HAVE_PTM
struct oneshot_lowerhalf_s *arm_timer_initialize(unsigned int freq);
#else
#  define arm_timer_initialize(freq) NULL
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_ARMV7_R_ARM_TIMER_H */
