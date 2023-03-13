/****************************************************************************
 * arch/arm/src/nrf53/nrf53_tim.h
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

#ifndef __ARCH_ARM_SRC_NRF53_NRF53_TIM_H
#define __ARCH_ARM_SRC_NRF53_NRF53_TIM_H

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

#define NRF53_TIM_START(d)                ((d)->ops->start(d))
#define NRF53_TIM_STOP(d)                 ((d)->ops->stop(d))
#define NRF53_TIM_CLEAR(d)                ((d)->ops->clear(d))
#define NRF53_TIM_CONFIGURE(d, m, w)      ((d)->ops->configure(d, m, w))
#define NRF53_TIM_SHORTS(d, s, i, e)      ((d)->ops->shorts(d, s, i, e))
#define NRF53_TIM_COUNT(d)                ((d)->ops->count(d))
#define NRF53_TIM_SETCC(d, i, cc)         ((d)->ops->setcc(d, i, cc))
#define NRF53_TIM_GETCC(d, i, cc)         ((d)->ops->getcc(d, i, cc))
#define NRF53_TIM_SETPRE(d, pre)          ((d)->ops->setpre(d, pre))
#define NRF53_TIM_SETISR(d, hnd, arg)     ((d)->ops->setisr(d, hnd, arg))
#define NRF53_TIM_ENABLEINT(d, s)         ((d)->ops->enableint(d, s))
#define NRF53_TIM_DISABLEINT(d, s)        ((d)->ops->disableint(d, s))
#define NRF53_TIM_CHECKINT(d, s)          ((d)->ops->checkint(d, s))
#define NRF53_TIM_ACKINT(d, s)            ((d)->ops->ackint(d, s))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Timer mode */

enum nrf53_tim_mode_e
{
  NRF53_TIM_MODE_UNUSED   = 0,
  NRF53_TIM_MODE_TIMER    = 1,
  NRF53_TIM_MODE_COUNTER  = 2,
  NRF53_TIM_MODE_LOWPOWER = 3,
};

/* Timer bit width */

enum nrf53_tim_width_e
{
  NRF53_TIM_WIDTH_16B    = 0,
  NRF53_TIM_WIDTH_8B     = 1,
  NRF53_TIM_WIDTH_24B    = 2,
  NRF53_TIM_WIDTH_32B    = 3,
};

/* Timer CC index */

enum nrf53_tim_cc_e
{
  NRF53_TIM_CC0 = 0,
  NRF53_TIM_CC1 = 1,
  NRF53_TIM_CC2 = 2,
  NRF53_TIM_CC3 = 3,
  NRF53_TIM_CC4 = 4,
  NRF53_TIM_CC5 = 5
};

/* Timer IRQ source */

enum nrf53_tim_irq_e
{
  NRF53_TIM_INT_COMPARE0 = 0,
  NRF53_TIM_INT_COMPARE1 = 1,
  NRF53_TIM_INT_COMPARE2 = 2,
  NRF53_TIM_INT_COMPARE3 = 3,
  NRF53_TIM_INT_COMPARE4 = 4,
  NRF53_TIM_INT_COMPARE5 = 5,
};

/* Timer shorts type */

enum nrf53_tim_shorts_e
{
  NRF53_TIM_SHORT_COMPARE_CLEAR = 1,
  NRF53_TIM_SHORT_COMPARE_STOP  = 2
};

/* Timer frequency prescaler */

enum nrf53_tim_pre_e
{
  NRF53_TIM_PRE_16000000 = 0,
  NRF53_TIM_PRE_8000000  = 1,
  NRF53_TIM_PRE_4000000  = 2,
  NRF53_TIM_PRE_2000000  = 3,
  NRF53_TIM_PRE_1000000  = 4,
  NRF53_TIM_PRE_500000   = 5,
  NRF53_TIM_PRE_250000   = 6,
  NRF53_TIM_PRE_125000   = 7,
  NRF53_TIM_PRE_62500    = 8,
  NRF53_TIM_PRE_31250    = 9
};

/* NRF53 TIM device */

struct nrf53_tim_dev_s
{
  struct nrf53_tim_ops_s *ops;
};

/* NRF53 TIM ops */

struct nrf53_tim_ops_s
{
  /* Timer tasks */

  int (*start)(struct nrf53_tim_dev_s *dev);
  int (*stop)(struct nrf53_tim_dev_s *dev);
  int (*clear)(struct nrf53_tim_dev_s *dev);

  /* Timer configuration */

  int (*configure)(struct nrf53_tim_dev_s *dev, uint8_t mode, uint8_t width);
  int (*shorts)(struct nrf53_tim_dev_s *dev, uint8_t s, uint8_t i, bool en);

  /* Timer operations */

  int (*count)(struct nrf53_tim_dev_s *dev);
  int (*setcc)(struct nrf53_tim_dev_s *dev, uint8_t i, uint32_t cc);
  int (*getcc)(struct nrf53_tim_dev_s *dev, uint8_t i, uint32_t *cc);
  int (*setpre)(struct nrf53_tim_dev_s *dev, uint8_t pre);

  /* Timer interrupts */

  int (*setisr)(struct nrf53_tim_dev_s *dev, xcpt_t handler, void *arg);
  int (*enableint)(struct nrf53_tim_dev_s *dev, uint8_t source);
  int (*disableint)(struct nrf53_tim_dev_s *dev, uint8_t source);
  int (*checkint)(struct nrf53_tim_dev_s *dev, uint8_t source);
  int (*ackint)(struct nrf53_tim_dev_s *dev, uint8_t source);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct nrf53_tim_dev_s *nrf53_tim_init(int timer);
int nrf53_tim_deinit(struct nrf53_tim_dev_s *dev);

#endif /* __ARCH_ARM_SRC_NRF53_NRF53_TIM_H */
