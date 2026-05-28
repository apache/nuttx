/****************************************************************************
 * arch/arm/src/common/stm32/stm32_freerun.h
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

#ifndef __ARCH_ARM_SRC_COMMON_STM32_STM32_FREERUN_H
#define __ARCH_ARM_SRC_COMMON_STM32_STM32_FREERUN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>
#include <nuttx/debug.h>

#include "stm32_tim.h"

#ifdef CONFIG_STM32_FREERUN

struct stm32_freerun_s
{
  uint8_t chan;
  uint8_t width;
  struct stm32_tim_dev_s *tch;
  uint32_t frequency;
#ifndef CONFIG_CLOCK_TIMEKEEPING
  uint32_t overflow;
#endif
#ifdef CONFIG_CLOCK_TIMEKEEPING
  uint64_t counter_mask;
#endif
};

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

int stm32_freerun_initialize(struct stm32_freerun_s *freerun, int chan,
                             uint16_t resolution);
#ifndef CONFIG_CLOCK_TIMEKEEPING
int stm32_freerun_counter(struct stm32_freerun_s *freerun,
                          struct timespec *ts);
#else
int stm32_freerun_counter(struct stm32_freerun_s *freerun,
                          uint64_t *counter);
#endif
int stm32_freerun_uninitialize(struct stm32_freerun_s *freerun);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_STM32_FREERUN */

#endif /* __ARCH_ARM_SRC_COMMON_STM32_STM32_FREERUN_H */
