/****************************************************************************
 * arch/arm/src/tlsr82/tlsr82_timer.h
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

#ifndef __ARCH_ARM_SRC_TLSR82_TLSR82_TIMER_H
#define __ARCH_ARM_SRC_TLSR82_TLSR82_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

#include <stdint.h>

#include "hardware/tlsr82_timer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Helpers ******************************************************************/

#define TLSR82_TIMER_START(d)                         ((d)->ops->start(d))
#define TLSR82_TIMER_STOP(d)                          ((d)->ops->stop(d))
#define TLSR82_TIMER_CLEAR(d)                         ((d)->ops->clear(d))
#define TLSR82_TIMER_SETMODE(d, m)                    ((d)->ops->setmode(d, m))
#define TLSR82_TIMER_GETCTR(d, v)                     ((d)->ops->getcounter(d, v))
#define TLSR82_TIMER_SETCTR(d, v)                     ((d)->ops->setcounter(d, v))
#define TLSR82_TIMER_GETCAPTURE(d, v)                 ((d)->ops->getcapture(d, v))
#define TLSR82_TIMER_SETCAPTURE(d, v)                 ((d)->ops->setcapture(d, v))
#define TLSR82_TIMER_GETCLK(d, v)                     ((d)->ops->getclock(d, v))
#define TLSR82_TIMER_SETISR(d, hnd, arg)              ((d)->ops->setisr(d, hnd, arg))
#define TLSR82_TIMER_ENABLEINT(d)                     ((d)->ops->enableint(d))
#define TLSR82_TIMER_DISABLEINT(d)                    ((d)->ops->disableint(d))
#define TLSR82_TIMER_ACKINT(d)                        ((d)->ops->ackint(d))
#define TLSR82_TIMER_CHECKINT(d)                      ((d)->ops->checkint(d))

#define TLSR82_TIMER0                                 0
#define TLSR82_TIMER1                                 1
#define TLSR82_TIMER2                                 2

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Instances of Timer  */

enum tlsr82_timer_inst_e
{
  TLSR82_INST_TIMER1 = 0,  /* Timer 1 */
  TLSR82_INST_TIMER2,      /* Timer 2 */
  TLSR82_INST_NUM,         /* Timer number */
};

/* Timer mode */

enum tlsr82_timer_mode_e
{
  TLSR82_TIMER_MODE_SYS_CLK,
  TLSR82_TIMER_MODE_GPIO_TRIG,
  TLSR82_TIMER_MODE_GPIO_PWM,
  TLSR82_TIMER_MODE_TICK,
  TLSR82_TIMER_MODE_NUM,
};

/* TIMER device */

struct tlsr82_timer_dev_s
{
  struct tlsr82_timer_ops_s *ops;
};

/* TIMER ops */

/* This is a struct containing the pointers to the timer operations */

struct tlsr82_timer_ops_s
{
  /* Timer tasks */

  void (*start)(struct tlsr82_timer_dev_s *dev);
  void (*stop)(struct tlsr82_timer_dev_s *dev);
  void (*clear)(struct tlsr82_timer_dev_s *dev);

  /* Timer operations */

  void (*setmode)(struct tlsr82_timer_dev_s *dev,
                  enum tlsr82_timer_mode_e mode);
  void (*getcounter)(struct tlsr82_timer_dev_s *dev, uint32_t *value);
  void (*setcounter)(struct tlsr82_timer_dev_s *dev, uint32_t value);
  void (*getcapture)(struct tlsr82_timer_dev_s *dev, uint32_t *value);
  void (*setcapture)(struct tlsr82_timer_dev_s *dev, uint32_t value);
  void (*getclock)(struct tlsr82_timer_dev_s *dev, uint32_t *value);

  /* Timer interrupts */

  int  (*setisr)(struct tlsr82_timer_dev_s *dev, xcpt_t handler,
                 void * arg);
  void (*enableint)(struct tlsr82_timer_dev_s *dev);
  void (*disableint)(struct tlsr82_timer_dev_s *dev);
  void (*ackint)(struct tlsr82_timer_dev_s *dev);
  int  (*checkint)(struct tlsr82_timer_dev_s *dev);
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline uint32_t tlsr82_time(void)
{
  return SYSTIMER_TICK_REG;
}

static inline bool tlsr82_time_exceed(uint32_t start, uint32_t us)
{
  return ((uint32_t)(tlsr82_time() - start) > us * 16);
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct tlsr82_timer_dev_s *tlsr82_timer_init(int timer);
void tlsr82_timer_deinit(struct tlsr82_timer_dev_s *dev);

#endif /* __ARCH_RISCV_SRC_ESP32C3_ESP32C3_TIM_H */
