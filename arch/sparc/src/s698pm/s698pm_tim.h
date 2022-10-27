/****************************************************************************
 * arch/sparc/src/s698pm/s698pm_tim.h
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

#ifndef __ARCH_SPARC_SRC_S698PM_S698PM_TIM_H
#define __ARCH_SPARC_SRC_S698PM_S698PM_TIM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define S698PM_TIM1_BASE       0x80000310
#define S698PM_TIM2_BASE       0x80000320
#define S698PM_TIM3_BASE       0x80000330
#define S698PM_TIM4_BASE       0x80000340

#define S698PM_TIMPRE_BASE     0x80000300

#define S698PM_TIM_CR_OFFSET     0x0008  /* Control register 1 (16-bit) */
#define S698PM_TIM_CNT_OFFSET    0x0000  /* Counter (16-bit) */
#define S698PM_TIM_ARR_OFFSET    0x0004  /* Auto-reload register (16-bit) */

#define S698PM_TIM_PSCLOAD_OFFSET    0x0004  /* Prescaler load (16-bit) */
#define S698PM_TIM_PSCCONT_OFFSET    0x0000  /* Prescaler count (16-bit) */

#define TIMER_LOADCOUNT       0x4
#define TIMER_RELOADCOUNT     0x2
#define TIMER_ENABLE          0x1

/* Helpers ******************************************************************/
#define S698PM_TIM_SETMODE(d,mode)       ((d)->ops->setmode(d,mode))
#define S698PM_TIM_SETCLOCK(d,freq)      ((d)->ops->setclock(d,freq))
#define S698PM_TIM_GETCLOCK(d)           ((d)->ops->getclock(d))
#define S698PM_TIM_SETPERIOD(d,period)   ((d)->ops->setperiod(d,period))
#define S698PM_TIM_GETPERIOD(d)          ((d)->ops->getperiod(d))
#define S698PM_TIM_GETCOUNTER(d)         ((d)->ops->getcounter(d))
#define S698PM_TIM_SETISR(d,hnd,arg,s)   ((d)->ops->setisr(d,hnd,arg,s))
#define S698PM_TIM_CLRINT(d,s)           ((d)->ops->clrint(d,s))
#define S698PM_TIM_CHECKINT(d,s)         ((d)->ops->checkint(d,s))
/****************************************************************************
 * Public Types
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

/* TIM Device Structure */

struct s698pm_tim_dev_s
{
  struct s698pm_tim_ops_s *ops;
};

enum s698pm_tim_mode_e
{
  S698PM_TIM_MODE_UNUSED       = -1,

  /* One of the following */

  S698PM_TIM_MODE_MASK         = 0x3,
  S698PM_TIM_MODE_DISABLED     = 0x1,
  S698PM_TIM_MODE_DOWN         = 0x2,
};

/* TIM Operations */

struct s698pm_tim_ops_s
{
  /* Basic Timers */

  int  (*setmode)(struct s698pm_tim_dev_s *dev,
                  enum s698pm_tim_mode_e mode);
  int  (*setclock)(struct s698pm_tim_dev_s *dev, uint32_t freq);
  uint32_t (*getclock)(struct s698pm_tim_dev_s *dev);
  void (*setperiod)(struct s698pm_tim_dev_s *dev, uint32_t period);
  uint32_t (*getperiod)(struct s698pm_tim_dev_s *dev);
  uint32_t (*getcounter)(struct s698pm_tim_dev_s *dev);

  /* Timer interrupts */

  int  (*setisr)(struct s698pm_tim_dev_s *dev,
                 xcpt_t handler, void *arg, int source);
  void (*clrint)(struct s698pm_tim_dev_s *dev, int source);
  int  (*checkint)(struct s698pm_tim_dev_s *dev, int source);
};

/* Power-up timer and get its structure */

struct s698pm_tim_dev_s *s698pm_tim_init(int timer);

/* Power-down timer, mark it as unused */

int s698pm_tim_deinit(struct s698pm_tim_dev_s *dev);

/****************************************************************************
 * Name: s698pm_timer_initialize
 *
 * Description:
 *   Bind the configuration timer to a timer lower half instance and
 *   register the timer drivers at 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the timer device. This should be of the form
 *   /dev/timer0
 *   timer - the timer number.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_TIMER
int s698pm_timer_initialize(const char *devpath, int timer);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_SPARC_SRC_S698PM_S698PM_TIM_H */
