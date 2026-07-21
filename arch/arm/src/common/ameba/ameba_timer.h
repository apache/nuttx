/****************************************************************************
 * arch/arm/src/common/ameba/ameba_timer.h
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

#ifndef __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_TIMER_H
#define __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

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
 * Name: ameba_timer_initialize
 *
 * Description:
 *   Bind one Ameba general-purpose timer instance to the NuttX timer
 *   character driver and register it at the given path (typically
 *   "/dev/timerN").  The instance index selects a row of the per-chip
 *   AMEBA_TIMER_CONFIG_TABLE (0 == TIM1, 1 == TIM2, both @ 32.768 kHz on the
 *   pke8721daf); its base, clock, RCC masks and IRQ come from that table, so
 *   a board only names the device and the instance it wants.
 *
 * Input Parameters:
 *   devpath  - The device path to register, e.g. "/dev/timer0".
 *   instance - Index into the per-chip timer instance table
 *              (0 .. AMEBA_TIMER_NINSTANCES - 1).
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ameba_timer_initialize(const char *devpath, int instance);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_TIMER_H */
