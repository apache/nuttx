/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_freerun.h
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

#ifndef __ARCH_ARM_SRC_STM32L4_STM32L4_FREERUN_H
#define __ARCH_ARM_SRC_STM32L4_STM32L4_FREERUN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>
#include <debug.h>

#include "stm32l4_tim.h"

#ifdef CONFIG_STM32L4_FREERUN

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The freerun client must allocate an instance of this structure and called
 * stm32l4_freerun_initialize() before using the freerun facilities.  The
 * client should not access the contents of this structure directly since the
 * contents are subject to change.
 */

struct stm32l4_freerun_s
{
  uint8_t chan;                  /* The timer/counter in use */
  uint32_t overflow;             /* Timer counter overflow */
  struct stm32l4_tim_dev_s *tch; /* Handle returned by stm32l4_tim_init() */
  uint32_t frequency;
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
 * Name: stm32l4_freerun_initialize
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

int stm32l4_freerun_initialize(struct stm32l4_freerun_s *freerun, int chan,
                             uint16_t resolution);

/****************************************************************************
 * Name: stm32l4_freerun_counter
 *
 * Description:
 *   Read the counter register of the free-running timer.
 *
 * Input Parameters:
 *   freerun Caller allocated instance of the freerun state structure.  This
 *           structure must have been previously initialized via a call to
 *           stm32l4_freerun_initialize();
 *   ts      The location in which to return the time remaining on the
 *           oneshot timer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int stm32l4_freerun_counter(struct stm32l4_freerun_s *freerun,
                          struct timespec *ts);

/****************************************************************************
 * Name: stm32l4_freerun_uninitialize
 *
 * Description:
 *   Stop the free-running timer and release all resources that it uses.
 *
 * Input Parameters:
 *   freerun Caller allocated instance of the freerun state structure.  This
 *           structure must have been previously initialized via a call to
 *           stm32l4_freerun_initialize();
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int stm32l4_freerun_uninitialize(struct stm32l4_freerun_s *freerun);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_STM32L4_FREERUN */
#endif /* __ARCH_ARM_SRC_STM32L4_STM32L4_FREERUN_H */
