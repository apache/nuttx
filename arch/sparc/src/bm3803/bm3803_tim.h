/****************************************************************************
 * arch/sparc/src/bm3803/bm3803_tim.h
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

#ifndef __ARCH_SPARC_SRC_BM3803_BM3803_TIM_H
#define __ARCH_SPARC_SRC_BM3803_BM3803_TIM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define BM3803_TIM1_BASE       0x80000040
#define BM3803_TIM2_BASE       0x80000050
#define BM3803_TIM12PRE_BASE   0x80000060

#define BM3803_TIM_WDG_OFFSET    0x000C  /* wdg register 1 (16-bit) */
#define BM3803_TIM_CR_OFFSET     0x0008  /* Control register 1 (16-bit) */
#define BM3803_TIM_CNT_OFFSET    0x0000  /* Counter (16-bit) */
#define BM3803_TIM_ARR_OFFSET    0x0004  /* Auto-reload register (16-bit) */

#define BM3803_TIM_PSCLOAD_OFFSET    0x0004  /* Prescaler load (16-bit) */
#define BM3803_TIM_PSCCONT_OFFSET    0x0000  /* Prescaler count (16-bit) */

#define TIMER_WDG             0x10
#define TIMER_LOADCOUNT       0x4
#define TIMER_RELOADCOUNT     0x2
#define TIMER_ENABLE          0x1

#  define UINT24_MAX        16777215u
/* Helpers ******************************************************************/
#define BM3803_TIM_SETMODE(d,mode)       ((d)->ops->setmode(d,mode))
#define BM3803_TIM_SETCLOCK(d,freq)      ((d)->ops->setclock(d,freq))
#define BM3803_TIM_GETCLOCK(d)           ((d)->ops->getclock(d))
#define BM3803_TIM_SETPERIOD(d,period)   ((d)->ops->setperiod(d,period))
#define BM3803_TIM_GETPERIOD(d)          ((d)->ops->getperiod(d))
#define BM3803_TIM_GETCOUNTER(d)         ((d)->ops->getcounter(d))
#define BM3803_TIM_SETISR(d,hnd,arg,s)   ((d)->ops->setisr(d,hnd,arg,s))
#define BM3803_TIM_CLRINT(d,s)           ((d)->ops->clrint(d,s))
#define BM3803_TIM_CHECKINT(d,s)         ((d)->ops->checkint(d,s))
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

struct bm3803_tim_dev_s
{
  struct bm3803_tim_ops_s *ops;
};

enum bm3803_tim_mode_e
{
  BM3803_TIM_MODE_UNUSED       = -1,

  /* One of the following */

  BM3803_TIM_MODE_MASK         = 0x3,
  BM3803_TIM_MODE_DISABLED     = 0x1,
  BM3803_TIM_MODE_DOWN         = 0x2,
};

/* TIM Operations */

struct bm3803_tim_ops_s
{
  /* Basic Timers */

  int  (*setmode)(struct bm3803_tim_dev_s *dev,
                  enum bm3803_tim_mode_e mode);
  int  (*setclock)(struct bm3803_tim_dev_s *dev, uint32_t freq);
  uint32_t (*getclock)(struct bm3803_tim_dev_s *dev);
  void (*setperiod)(struct bm3803_tim_dev_s *dev, uint32_t period);
  uint32_t (*getperiod)(struct bm3803_tim_dev_s *dev);
  uint32_t (*getcounter)(struct bm3803_tim_dev_s *dev);

  /* Timer interrupts */

  int  (*setisr)(struct bm3803_tim_dev_s *dev,
                 xcpt_t handler, void *arg, int source);
  void (*clrint)(struct bm3803_tim_dev_s *dev, int source);
  int  (*checkint)(struct bm3803_tim_dev_s *dev, int source);
};

/* Power-up timer and get its structure */

struct bm3803_tim_dev_s *bm3803_tim_init(int timer);

/* Power-down timer, mark it as unused */

int bm3803_tim_deinit(struct bm3803_tim_dev_s *dev);

/****************************************************************************
 * Name: bm3803_timer_initialize
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
int bm3803_timer_initialize(const char *devpath, int timer);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_SPARC_SRC_BM3803_BM3803_TIM_H */
