/****************************************************************************
 * arch/arm/src/common/stm32/stm32_oneshot.h
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

#ifndef __ARCH_ARM_SRC_COMMON_STM32_STM32_ONESHOT_H
#define __ARCH_ARM_SRC_COMMON_STM32_STM32_ONESHOT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>

#include <nuttx/irq.h>

#include "stm32_tim.h"

#ifdef CONFIG_STM32_ONESHOT

#if !defined(CONFIG_STM32_ONESHOT_MAXTIMERS) || \
    CONFIG_STM32_ONESHOT_MAXTIMERS < 1
#  undef CONFIG_STM32_ONESHOT_MAXTIMERS
#  define CONFIG_STM32_ONESHOT_MAXTIMERS 1
#endif

#if CONFIG_STM32_ONESHOT_MAXTIMERS > 8
#  warning Additional logic required to handle more than 8 timers
#  undef CONFIG_STM32_ONESHOT_MAXTIMERS
#  define CONFIG_STM32_ONESHOT_MAXTIMERS 8
#endif

typedef void (*oneshot_handler_t)(void *arg);

struct stm32_oneshot_s
{
  uint8_t chan;
#if CONFIG_STM32_ONESHOT_MAXTIMERS > 1
  uint8_t cbndx;
#endif
  volatile bool running;
  struct stm32_tim_dev_s *tch;
  volatile oneshot_handler_t handler;
  volatile void *arg;
  uint32_t frequency;
  uint32_t period;
};

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

int stm32_oneshot_initialize(struct stm32_oneshot_s *oneshot, int chan,
                             uint16_t resolution);
int stm32_oneshot_max_delay(struct stm32_oneshot_s *oneshot, uint64_t *usec);
int stm32_oneshot_start(struct stm32_oneshot_s *oneshot,
                        oneshot_handler_t handler, void *arg,
                        const struct timespec *ts);
int stm32_oneshot_cancel(struct stm32_oneshot_s *oneshot,
                         struct timespec *ts);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_STM32_ONESHOT */

#endif /* __ARCH_ARM_SRC_COMMON_STM32_STM32_ONESHOT_H */
