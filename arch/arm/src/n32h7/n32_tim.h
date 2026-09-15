/****************************************************************************
 * arch/arm/src/n32h7/n32_tim.h
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

#ifndef __ARCH_ARM_SRC_N32H7_N32_TIM_H
#define __ARCH_ARM_SRC_N32H7_N32_TIM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/n32h7_tim.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Helpers for timer driver operations **************************************/

#define N32_TIM_SETMODE(d, mode)      ((d)->ops->setmode(d, mode))
#define N32_TIM_SETCLOCK(d, freq)     ((d)->ops->setclock(d, freq))
#define N32_TIM_SETPERIOD(d, period)  ((d)->ops->setperiod(d, period))
#define N32_TIM_GETPERIOD(d)          ((d)->ops->getperiod(d))
#define N32_TIM_GETCOUNTER(d)         ((d)->ops->getcounter(d))
#define N32_TIM_SETCOUNTER(d, c)      ((d)->ops->setcounter(d, c))
#define N32_TIM_GETWIDTH(d)           ((d)->ops->getwidth(d))
#define N32_TIM_SETCHANNEL(d, ch, m)  ((d)->ops->setchannel(d, ch, m))
#define N32_TIM_SETCOMPARE(d, ch, c)  ((d)->ops->setcompare(d, ch, c))
#define N32_TIM_GETCAPTURE(d, ch)     ((d)->ops->getcapture(d, ch))
#define N32_TIM_SETISR(d, h, a, s)    ((d)->ops->setisr(d, h, a, s))
#define N32_TIM_ENABLEINT(d, s)       ((d)->ops->enableint(d, s))
#define N32_TIM_DISABLEINT(d, s)      ((d)->ops->disableint(d, s))
#define N32_TIM_ACKINT(d, s)          ((d)->ops->ackint(d, s))
#define N32_TIM_CHECKINT(d, s)        ((d)->ops->checkint(d, s))
#define N32_TIM_ENABLE(d)             ((d)->ops->enable(d))
#define N32_TIM_DISABLE(d)            ((d)->ops->disable(d))

/* Timer interrupt sources for use with N32_TIM_SETISR */
#define N32_TIM_ISR_UPDATE      (1 << 0)  /* Update interrupt (overflow/underflow) */
#define N32_TIM_ISR_CC          (1 << 1)  /* Capture/Compare interrupt (CC1-CC4) */
#define N32_TIM_ISR_TRG_COM     (1 << 2)  /* Trigger and COM interrupt */
#define N32_TIM_ISR_BRK         (1 << 3)  /* Break interrupt (only ATIM) */

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

/* Forward reference */

struct n32_tim_dev_s;

/* Timer modes of operation */

typedef enum
{
  N32_TIM_MODE_UNUSED       = -1,

  /* One of the following */

  N32_TIM_MODE_MASK         = 0x0310,
  N32_TIM_MODE_DISABLED     = 0x0000,
  N32_TIM_MODE_UP           = 0x0100,
  N32_TIM_MODE_DOWN         = 0x0110,
  N32_TIM_MODE_UPDOWN       = 0x0200,
  N32_TIM_MODE_PULSE        = 0x0300,

  /* Clock source options (not fully supported in this version) */

  N32_TIM_MODE_CK_INT       = 0x0000,
} n32_tim_mode_t;

/* Timer channel modes */

typedef enum
{
  N32_TIM_CH_DISABLED       = 0x00,

  /* Common configuration */

  N32_TIM_CH_POLARITY_POS   = 0x00,
  N32_TIM_CH_POLARITY_NEG   = 0x01,

  /* MODES: */

  N32_TIM_CH_MODE_MASK      = 0x0e,

  /* Output compare modes */

  N32_TIM_CH_OUTPWM         = 0x04,   /* PWM mode 1 (active high) */
  N32_TIM_CH_OUTTOGGLE      = 0x08,   /* Toggle on match */
} n32_tim_channel_t;

/* Timer operations structure */

struct n32_tim_ops_s
{
  /* Basic timer operations */

  void (*enable)(struct n32_tim_dev_s *dev);
  void (*disable)(struct n32_tim_dev_s *dev);
  int  (*setmode)(struct n32_tim_dev_s *dev, n32_tim_mode_t mode);
  int  (*setclock)(struct n32_tim_dev_s *dev, uint32_t freq);
  void (*setperiod)(struct n32_tim_dev_s *dev, uint32_t period);
  uint32_t (*getperiod)(struct n32_tim_dev_s *dev);
  uint32_t (*getcounter)(struct n32_tim_dev_s *dev);
  void (*setcounter)(struct n32_tim_dev_s *dev, uint32_t count);

  /* General and advanced timers */

  int  (*getwidth)(struct n32_tim_dev_s *dev);
  int  (*setchannel)(struct n32_tim_dev_s *dev, uint8_t channel,
                     n32_tim_channel_t mode);
  int  (*setcompare)(struct n32_tim_dev_s *dev, uint8_t channel,
                     uint32_t compare);
  int  (*getcapture)(struct n32_tim_dev_s *dev, uint8_t channel);

  /* Timer interrupts */

  int  (*setisr)(struct n32_tim_dev_s *dev, xcpt_t handler, void *arg,
                 int source);
  void (*enableint)(struct n32_tim_dev_s *dev, int source);
  void (*disableint)(struct n32_tim_dev_s *dev, int source);
  void (*ackint)(struct n32_tim_dev_s *dev, int source);
  int  (*checkint)(struct n32_tim_dev_s *dev, int source);
};

/* Timer device structure (public) */

struct n32_tim_dev_s
{
  const struct n32_tim_ops_s *ops;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: n32_tim_init
 *
 * Description:
 *   Initialize a timer device and enable its clock.
 *
 * Input Parameters:
 *   timer - Timer number (1-18, as defined by the configuration).
 *           The mapping is determined by CONFIG_N32H7_ATIMx,
 *           CONFIG_N32H7_GTIMAx, CONFIG_N32H7_GTIMBx, CONFIG_N32H7_BTIMx.
 *
 * Returned Value:
 *   Pointer to the timer device structure, or NULL on failure.
 *
 ****************************************************************************/

struct n32_tim_dev_s *n32_tim_init(int timer);

/****************************************************************************
 * Name: n32_tim_deinit
 *
 * Description:
 *   De-initialize a timer device, disable its clock, and reset it.
 *
 * Input Parameters:
 *   dev - Timer device structure.
 *
 * Returned Value:
 *   OK on success, negated errno on failure.
 *
 ****************************************************************************/

int n32_tim_deinit(struct n32_tim_dev_s *dev);

/****************************************************************************
 * Name: n32_timer_initialize
 *
 * Description:
 *   Bind the configuration timer to a timer lower half instance and
 *   register the timer drivers at 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the timer device. This should be of the
 *             form /dev/timer0
 *   timer   - The timer number.
 *
 * Returned Values:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_TIMER
int n32_timer_initialize(const char *devpath, int timer);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_N32H7_N32_TIM_H */
