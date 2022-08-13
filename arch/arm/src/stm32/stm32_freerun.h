/****************************************************************************
 * arch/arm/src/stm32/stm32_freerun.h
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

#ifndef __ARCH_ARM_SRC_STM32_STM32_FREERUN_H
#define __ARCH_ARM_SRC_STM32_STM32_FREERUN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>
#include <debug.h>

#include "stm32_tim.h"

#ifdef CONFIG_STM32_FREERUN

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The freerun client must allocate an instance of this structure and called
 * stm32_freerun_initialize() before using the freerun facilities.  The
 * client should not access the contents of this structure directly since
 * the contents are subject to change.
 */

struct stm32_freerun_s
{
  uint8_t chan;                /* The timer/counter in use */
  uint8_t width;               /* Width of timer (16- or 32-bits) */
  struct stm32_tim_dev_s *tch; /* Handle returned by stm32_tim_init() */
  uint32_t frequency;

#ifndef CONFIG_CLOCK_TIMEKEEPING
  uint32_t overflow;               /* Timer counter overflow */
#endif

#ifdef CONFIG_CLOCK_TIMEKEEPING
  uint64_t counter_mask;
#endif
};

/****************************************************************************
 * Public Data
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
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_freerun_initialize
 *
 * Description:
 *   Initialize the freerun timer wrapper
 *
 * Input Parameters:
 *   freerun    Caller allocated instance of the freerun state structure
 *   chan       Timer counter channel to be used.
 *   resolution The required resolution of the timer in units of
 *              microseconds.  NOTE that the range is restricted to the
 *              range of uint16_t (excluding zero).
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int stm32_freerun_initialize(struct stm32_freerun_s *freerun, int chan,
                             uint16_t resolution);

/****************************************************************************
 * Name: stm32_freerun_counter
 *
 * Description:
 *   Read the counter register of the free-running timer.
 *
 * Input Parameters:
 *   freerun Caller allocated instance of the freerun state structure.  This
 *           structure must have been previously initialized via a call to
 *           stm32_freerun_initialize();
 *   ts      The location in which to return the time remaining on the
 *           oneshot timer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

#ifndef CONFIG_CLOCK_TIMEKEEPING

int stm32_freerun_counter(struct stm32_freerun_s *freerun,
                          struct timespec *ts);

#else /* CONFIG_CLOCK_TIMEKEEPING */

int stm32_freerun_counter(struct stm32_freerun_s *freerun,
                          uint64_t *counter);

#endif /* CONFIG_CLOCK_TIMEKEEPING */

/****************************************************************************
 * Name: stm32_freerun_uninitialize
 *
 * Description:
 *   Stop the free-running timer and release all resources that it uses.
 *
 * Input Parameters:
 *   freerun Caller allocated instance of the freerun state structure.  This
 *           structure must have been previously initialized via a call to
 *           stm32_freerun_initialize();
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int stm32_freerun_uninitialize(struct stm32_freerun_s *freerun);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_STM32_FREERUN */
#endif /* __ARCH_ARM_SRC_STM32_STM32_FREERUN_H */
