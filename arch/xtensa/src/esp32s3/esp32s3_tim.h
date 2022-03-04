/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_tim.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_TIM_H
#define __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_TIM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_TIMER0
#  define ESP32S3_TIMER0 0
#endif

#ifdef CONFIG_ESP32S3_TIMER1
#  define ESP32S3_TIMER1 1
#endif

#ifdef CONFIG_ESP32S3_TIMER2
#  define ESP32S3_TIMER2 2
#endif

#ifdef CONFIG_ESP32S3_TIMER3
#  define ESP32S3_TIMER3 3
#endif

/* Helpers ******************************************************************/

#define ESP32S3_TIM_START(d)                  ((d)->ops->start(d))
#define ESP32S3_TIM_STOP(d)                   ((d)->ops->stop(d))
#define ESP32S3_TIM_CLEAR(d)                  ((d)->ops->clear(d))
#define ESP32S3_TIM_SETMODE(d, m)             ((d)->ops->setmode(d, m))
#define ESP32S3_TIM_CLK_SRC(d, s)             ((d)->ops->setclksrc(d, s))
#define ESP32S3_TIM_SETPRE(d, p)              ((d)->ops->setpre(d, p))
#define ESP32S3_TIM_GETCTR(d, v)              ((d)->ops->getcounter(d, v))
#define ESP32S3_TIM_SETCTR(d, v)              ((d)->ops->setcounter(d, v))
#define ESP32S3_TIM_RLD_NOW(d)                ((d)->ops->reloadnow(d))
#define ESP32S3_TIM_GETALRVL(d, v)            ((d)->ops->getalarmvalue(d, v))
#define ESP32S3_TIM_SETALRVL(d, v)            ((d)->ops->setalarmvalue(d, v))
#define ESP32S3_TIM_SETALRM(d, e)             ((d)->ops->setalarm(d, e))
#define ESP32S3_TIM_SETARLD(d, e)             ((d)->ops->setautoreload(d, e))
#define ESP32S3_TIM_SETISR(d, hnd, arg)       ((d)->ops->setisr(d, hnd, arg))
#define ESP32S3_TIM_ENABLEINT(d)              ((d)->ops->enableint(d))
#define ESP32S3_TIM_DISABLEINT(d)             ((d)->ops->disableint(d))
#define ESP32S3_TIM_ACKINT(d)                 ((d)->ops->ackint(d))
#define ESP32S3_TIM_CHECKINT(d)               ((d)->ops->checkint(d))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Timer clock source */

enum esp32s3_tim_clksrc_e
{
  ESP32S3_TIM_APB_CLK,
  ESP32S3_TIM_XTAL_CLK,
};

/* Timer mode */

enum esp32s3_tim_mode_e
{
  ESP32S3_TIM_MODE_DOWN,
  ESP32S3_TIM_MODE_UP,
};

/* ESP32-S3 TIM device */

struct esp32s3_tim_dev_s
{
  struct esp32s3_tim_ops_s *ops;
};

/* ESP32-S3 TIM ops
 *
 * This is a struct containing the pointers to the timer operations.
 */

struct esp32s3_tim_ops_s
{
  /* Timer tasks */

  void (*start)(struct esp32s3_tim_dev_s *dev);
  void (*stop)(struct esp32s3_tim_dev_s *dev);
  void (*clear)(struct esp32s3_tim_dev_s *dev);

  /* Timer operations */

  void (*setmode)(struct esp32s3_tim_dev_s *dev,
                  enum esp32s3_tim_mode_e mode);
  void (*setclksrc)(struct esp32s3_tim_dev_s *dev,
                    enum esp32s3_tim_clksrc_e src);
  void (*setpre)(struct esp32s3_tim_dev_s *dev, uint16_t pre);
  void (*getcounter)(struct esp32s3_tim_dev_s *dev, uint64_t *value);
  void (*setcounter)(struct esp32s3_tim_dev_s *dev, uint64_t value);
  void (*reloadnow)(struct esp32s3_tim_dev_s *dev);
  void (*getalarmvalue)(struct esp32s3_tim_dev_s *dev, uint64_t *value);
  void (*setalarmvalue)(struct esp32s3_tim_dev_s *dev, uint64_t value);
  void (*setalarm)(struct esp32s3_tim_dev_s *dev, bool enable);
  void (*setautoreload)(struct esp32s3_tim_dev_s *dev, bool enable);

  /* Timer interrupts */

  int  (*setisr)(struct esp32s3_tim_dev_s *dev, xcpt_t handler, void *arg);
  void (*enableint)(struct esp32s3_tim_dev_s *dev);
  void (*disableint)(struct esp32s3_tim_dev_s *dev);
  void (*ackint)(struct esp32s3_tim_dev_s *dev);
  int  (*checkint)(struct esp32s3_tim_dev_s *dev);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct esp32s3_tim_dev_s *esp32s3_tim_init(int timer);
void esp32s3_tim_deinit(struct esp32s3_tim_dev_s *dev);

#endif /* __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_TIM_H */
