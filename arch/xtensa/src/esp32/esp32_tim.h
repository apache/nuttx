/****************************************************************************
 * arch/xtensa/src/esp32/esp32_tim.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_ESP32_TIM_H
#define __ARCH_XTENSA_SRC_ESP32_ESP32_TIM_H

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

#define ESP32_TIM_START(d)                         ((d)->ops->start(d))
#define ESP32_TIM_STOP(d)                          ((d)->ops->stop(d))
#define ESP32_TIM_CLEAR(d)                         ((d)->ops->clear(d))
#define ESP32_TIM_CONFIGURE(d, p, m, c, av, a, ar) ((d)->ops->configure(d, m, c, av, a, ar))
#define ESP32_TIM_SETMODE(d, m)                    ((d)->ops->setmode(d, m))
#define ESP32_TIM_SETPRE(d, p)                     ((d)->ops->setpre(d, p))
#define ESP32_TIM_GETCONFIG(d, v)                  ((d)->ops->getconfig(d, v))
#define ESP32_TIM_GETCTR(d, v)                     ((d)->ops->getcounter(d, v))
#define ESP32_TIM_SETCTR(d, v)                     ((d)->ops->setcounter(d, v))
#define ESP32_TIM_GETALRVL(d, v)                   ((d)->ops->getalarmvalue(d, v))
#define ESP32_TIM_SETALRVL(d, v)                   ((d)->ops->setalarmvalue(d, v))
#define ESP32_TIM_SETALRM(d, e)                    ((d)->ops->setalarm(d, e))
#define ESP32_TIM_SETARLD(d, e)                    ((d)->ops->setautoreload(d, e))
#define ESP32_TIM_SETISR(d, hnd, arg)              ((d)->ops->setisr(d, hnd, arg))
#define ESP32_TIM_ENABLEINT(d)                     ((d)->ops->enableint(d))
#define ESP32_TIM_DISABLEINT(d)                    ((d)->ops->disableint(d))
#define ESP32_TIM_CHECKINT(d)                      ((d)->ops->checkint(d))
#define ESP32_TIM_ACKINT(d)                        ((d)->ops->ackint(d))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Timer mode */

enum esp32_tim_mode_e
{
  ESP32_TIM_MODE_DOWN,
  ESP32_TIM_MODE_UP,
};

/* ESP32 TIM device */

struct esp32_tim_dev_s
{
  struct esp32_tim_ops_s *ops;
};

/* ESP32 TIM ops */

/* This is a struct containing the pointers to the timer operations */

struct esp32_tim_ops_s
{
  /* Timer tasks */

  CODE int (*start)(FAR struct esp32_tim_dev_s *dev);
  CODE int (*stop)(FAR struct esp32_tim_dev_s *dev);
  CODE int (*clear)(FAR struct esp32_tim_dev_s *dev);

  /* Timer configuration */

  CODE int (*configure)(FAR struct esp32_tim_dev_s *dev, uint16_t pre,
                        uint8_t mode, uint64_t counter_value,
                        uint64_t alarm_value, bool alarm,
                        bool autoreload);

  /* Timer operations */

  CODE int (*setmode)(FAR struct esp32_tim_dev_s *dev, uint8_t mode);
  CODE int (*setpre)(FAR struct esp32_tim_dev_s *dev, uint16_t pre);
  CODE int (*getconfig)(FAR struct esp32_tim_dev_s *dev, uint32_t *value);
  CODE int (*getcounter)(FAR struct esp32_tim_dev_s *dev, uint64_t *value);
  CODE int (*setcounter)(FAR struct esp32_tim_dev_s *dev, uint64_t value);
  CODE int (*getalarmvalue)(FAR struct esp32_tim_dev_s *dev,
                            uint64_t *value);
  CODE int (*setalarmvalue)(FAR struct esp32_tim_dev_s *dev, uint64_t value);
  CODE int (*setalarm)(FAR struct esp32_tim_dev_s *dev, bool enable);
  CODE int (*setautoreload)(FAR struct esp32_tim_dev_s *dev, bool enable);

  /* Timer interrupts */

  CODE int (*setisr)(FAR struct esp32_tim_dev_s *dev, xcpt_t handler,
                     FAR void * arg);
  CODE int (*enableint)(FAR struct esp32_tim_dev_s *dev);
  CODE int (*disableint)(FAR struct esp32_tim_dev_s *dev);
  CODE int (*checkint)(FAR struct esp32_tim_dev_s *dev);
  CODE int (*ackint)(FAR struct esp32_tim_dev_s *dev);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

FAR struct esp32_tim_dev_s *esp32_tim_init(int timer);
int esp32_tim_deinit(FAR struct esp32_tim_dev_s *dev);

#endif /* __ARCH_XTENSA_SRC_ESP32_ESP32_TIM_H */
