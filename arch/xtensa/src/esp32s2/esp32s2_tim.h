/****************************************************************************
 * arch/xtensa/src/esp32s2/esp32s2_tim.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S2_ESP32S2_TIM_H
#define __ARCH_XTENSA_SRC_ESP32S2_ESP32S2_TIM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Helpers ******************************************************************/

#define ESP32S2_TIM_START(d)                         ((d)->ops->start(d))
#define ESP32S2_TIM_STOP(d)                          ((d)->ops->stop(d))
#define ESP32S2_TIM_CLEAR(d)                         ((d)->ops->clear(d))
#define ESP32S2_TIM_SETMODE(d, m)                    ((d)->ops->setmode(d, m))
#define ESP32S2_TIM_SETPRE(d, p)                     ((d)->ops->setpre(d, p))
#define ESP32S2_TIM_SETSTEP(d, s, t)                 ((d)->ops->setstep(d, s, t))
#define ESP32S2_TIM_GETCTR(d, v)                     ((d)->ops->getcounter(d, v))
#define ESP32S2_TIM_CLK_SRC(d, s)                    ((d)->ops->setclksrc(d, s))
#define ESP32S2_TIM_SETCTR(d, v)                     ((d)->ops->setcounter(d, v))
#define ESP32S2_TIM_RLD_NOW(d)                       ((d)->ops->reloadnow(d))
#define ESP32S2_TIM_GETALRVL(d, v)                   ((d)->ops->getalarmvalue(d, v))
#define ESP32S2_TIM_GETPERIOD(d, v)                  ((d)->ops->getperiod(d, v))
#define ESP32S2_TIM_SETALRVL(d, v)                   ((d)->ops->setalarmvalue(d, v))
#define ESP32S2_TIM_SETPERIOD(d, v)                  ((d)->ops->setperiod(d, v))
#define ESP32S2_TIM_SETWORKMODE(d, v)                ((d)->ops->setworkmode(d, v))
#define ESP32S2_TIM_SETALRM(d, e)                    ((d)->ops->setalarm(d, e))
#define ESP32S2_TIM_SETARLD(d, e)                    ((d)->ops->setautoreload(d, e))
#define ESP32S2_TIM_SETISR(d, hnd, arg)              ((d)->ops->setisr(d, hnd, arg))
#define ESP32S2_TIM_ENABLEINT(d)                     ((d)->ops->enableint(d))
#define ESP32S2_TIM_DISABLEINT(d)                    ((d)->ops->disableint(d))
#define ESP32S2_TIM_ACKINT(d)                        ((d)->ops->ackint(d))
#define ESP32S2_TIM_CHECKINT(d)                      ((d)->ops->checkint(d))

#define TIMER0 0          /* Timer group 0 timer 0 */
#define TIMER1 1          /* Timer group 0 timer 1 */
#define TIMER2 2          /* Timer group 1 timer 0 */
#define TIMER3 3          /* Timer group 1 timer 1 */
#define SYSTIMER_COMP0 4  /* Systimer comparator 0 */
#define SYSTIMER_COMP1 5  /* Systimer comparator 1 */
#define SYSTIMER_COMP2 6  /* Systimer comparator 2 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Timer mode */

enum esp32s2_tim_mode_e
{
  ESP32S2_TIM_MODE_DOWN,
  ESP32S2_TIM_MODE_UP,
};

/* Systimer Work modes */

enum esp32s2_tim_work_mode_e
{
  ESP32S2_TIM_DELAY_ALRM,
  ESP32S2_TIM_PERIOD_ALRM,
};

/* Timer mode */

enum esp32s2_tim_clksrc_e
{
  ESP32S2_TIM_APB_CLK,  /* Available only for Generic Timers */
  ESP32S2_TIM_XTAL_CLK, /* Available for both Generic Timers and Systimer */
  ESP32S2_TIM_PLL_CLK   /* Available only for Systimer */
};

/* ESP32-S2 TIM device */

struct esp32s2_tim_dev_s
{
  struct esp32s2_tim_ops_s *ops;
};

/* ESP32-S2 TIM ops */

/* This is a struct containing the pointers to the timer operations */

struct esp32s2_tim_ops_s
{
  /* Timer tasks */

  void (*start)(struct esp32s2_tim_dev_s *dev);
  void (*stop)(struct esp32s2_tim_dev_s *dev);
  void (*clear)(struct esp32s2_tim_dev_s *dev);

  /* Timer operations */

  void (*setmode)(struct esp32s2_tim_dev_s *dev,
                       enum esp32s2_tim_mode_e mode);
  void (*setpre)(struct esp32s2_tim_dev_s *dev, uint16_t pre);
  void (*setstep)(struct esp32s2_tim_dev_s *dev,
                      enum esp32s2_tim_clksrc_e src,
                      uint16_t ticks);
  void (*getcounter)(struct esp32s2_tim_dev_s *dev,
                          uint64_t *value);
  void (*setclksrc)(struct esp32s2_tim_dev_s *dev,
                         enum esp32s2_tim_clksrc_e src);
  void (*setcounter)(struct esp32s2_tim_dev_s *dev, uint64_t value);
  void (*reloadnow)(struct esp32s2_tim_dev_s *dev);
  void (*getalarmvalue)(struct esp32s2_tim_dev_s *dev,
                             uint64_t *value);
  void (*getperiod)(struct esp32s2_tim_dev_s *dev, uint32_t *value);
  void (*setalarmvalue)(struct esp32s2_tim_dev_s *dev,
                             uint64_t value);
  void (*setperiod)(struct esp32s2_tim_dev_s *dev, uint32_t value);
  void (*setworkmode)(struct esp32s2_tim_dev_s *dev,
                           enum esp32s2_tim_work_mode_e);
  void (*setalarm)(struct esp32s2_tim_dev_s *dev, bool enable);
  void (*setautoreload)(struct esp32s2_tim_dev_s *dev, bool enable);

  /* Timer interrupts */

  int (*setisr)(struct esp32s2_tim_dev_s *dev, xcpt_t handler,
                     void * arg);
  void (*enableint)(struct esp32s2_tim_dev_s *dev);
  void (*disableint)(struct esp32s2_tim_dev_s *dev);
  void (*ackint)(struct esp32s2_tim_dev_s *dev);
  int (*checkint)(struct esp32s2_tim_dev_s *dev);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct esp32s2_tim_dev_s *esp32s2_tim_init(int timer);
void esp32s2_tim_deinit(struct esp32s2_tim_dev_s *dev);

#endif /* __ARCH_XTENSA_SRC_ESP32S2_ESP32S2_TIM_H */
