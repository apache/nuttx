/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_tim.h
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

#ifndef __ARCH_ARM_SRC_NRF54L_NRF54L_TIM_H
#define __ARCH_ARM_SRC_NRF54L_NRF54L_TIM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Helpers ******************************************************************/

#define NRF54L_TIM_START(d)                ((d)->ops->start(d))
#define NRF54L_TIM_STOP(d)                 ((d)->ops->stop(d))
#define NRF54L_TIM_CLEAR(d)                ((d)->ops->clear(d))
#define NRF54L_TIM_CONFIGURE(d, m, w)      ((d)->ops->configure(d, m, w))
#define NRF54L_TIM_SHORTS(d, s, i, e)      ((d)->ops->shorts(d, s, i, e))
#define NRF54L_TIM_COUNT(d)                ((d)->ops->count(d))
#define NRF54L_TIM_SETCC(d, i, cc)         ((d)->ops->setcc(d, i, cc))
#define NRF54L_TIM_GETCC(d, i, cc)         ((d)->ops->getcc(d, i, cc))
#define NRF54L_TIM_SETPRE(d, pre)          ((d)->ops->setpre(d, pre))
#define NRF54L_TIM_SETISR(d, hnd, arg)     ((d)->ops->setisr(d, hnd, arg))
#define NRF54L_TIM_ENABLEINT(d, s)         ((d)->ops->enableint(d, s))
#define NRF54L_TIM_DISABLEINT(d, s)        ((d)->ops->disableint(d, s))
#define NRF54L_TIM_CHECKINT(d, s)          ((d)->ops->checkint(d, s))
#define NRF54L_TIM_ACKINT(d, s)            ((d)->ops->ackint(d, s))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Timer mode */

enum nrf54l_tim_mode_e
{
  NRF54L_TIM_MODE_UNUSED   = 0,
  NRF54L_TIM_MODE_TIMER    = 1,
  NRF54L_TIM_MODE_COUNTER  = 2,
  NRF54L_TIM_MODE_LOWPOWER = 3,
};

/* Timer bit width */

enum nrf54l_tim_width_e
{
  NRF54L_TIM_WIDTH_16B    = 0,
  NRF54L_TIM_WIDTH_8B     = 1,
  NRF54L_TIM_WIDTH_24B    = 2,
  NRF54L_TIM_WIDTH_32B    = 3,
};

/* Timer CC index */

enum nrf54l_tim_cc_e
{
  NRF54L_TIM_CC0 = 0,
  NRF54L_TIM_CC1 = 1,
  NRF54L_TIM_CC2 = 2,
  NRF54L_TIM_CC3 = 3,
  NRF54L_TIM_CC4 = 4,
  NRF54L_TIM_CC5 = 5,
  NRF54L_TIM_CC6 = 6,
  NRF54L_TIM_CC7 = 7
};

/* Timer IRQ source */

enum nrf54l_tim_irq_e
{
  NRF54L_TIM_INT_COMPARE0 = 0,
  NRF54L_TIM_INT_COMPARE1 = 1,
  NRF54L_TIM_INT_COMPARE2 = 2,
  NRF54L_TIM_INT_COMPARE3 = 3,
  NRF54L_TIM_INT_COMPARE4 = 4,
  NRF54L_TIM_INT_COMPARE5 = 5,
  NRF54L_TIM_INT_COMPARE6 = 6,
  NRF54L_TIM_INT_COMPARE7 = 7,
};

/* Timer shorts type */

enum nrf54l_tim_shorts_e
{
  NRF54L_TIM_SHORT_COMPARE_CLEAR = 1,
  NRF54L_TIM_SHORT_COMPARE_STOP  = 2
};

/* Timer frequency prescaler (input frequency / 2^prescaler) */

enum nrf54l_tim_pre_e
{
  NRF54L_TIM_PRE_DIV1   = 0,
  NRF54L_TIM_PRE_DIV2   = 1,
  NRF54L_TIM_PRE_DIV4   = 2,
  NRF54L_TIM_PRE_DIV8   = 3,
  NRF54L_TIM_PRE_DIV16  = 4,
  NRF54L_TIM_PRE_DIV32  = 5,
  NRF54L_TIM_PRE_DIV64  = 6,
  NRF54L_TIM_PRE_DIV128 = 7,
  NRF54L_TIM_PRE_DIV256 = 8,
  NRF54L_TIM_PRE_DIV512 = 9
};

/* NRF54L TIM device */

struct nrf54l_tim_dev_s
{
  struct nrf54l_tim_ops_s *ops;
};

/* NRF54L TIM ops */

struct nrf54l_tim_ops_s
{
  /* Timer tasks */

  int (*start)(struct nrf54l_tim_dev_s *dev);
  int (*stop)(struct nrf54l_tim_dev_s *dev);
  int (*clear)(struct nrf54l_tim_dev_s *dev);

  /* Timer configuration */

  int (*configure)(struct nrf54l_tim_dev_s *dev, uint8_t mode,
                   uint8_t width);
  int (*shorts)(struct nrf54l_tim_dev_s *dev, uint8_t s, uint8_t i, bool en);

  /* Timer operations */

  int (*count)(struct nrf54l_tim_dev_s *dev);
  int (*setcc)(struct nrf54l_tim_dev_s *dev, uint8_t i, uint32_t cc);
  int (*getcc)(struct nrf54l_tim_dev_s *dev, uint8_t i, uint32_t *cc);
  int (*setpre)(struct nrf54l_tim_dev_s *dev, uint8_t pre);

  /* Timer interrupts */

  int (*setisr)(struct nrf54l_tim_dev_s *dev, xcpt_t handler, void *arg);
  int (*enableint)(struct nrf54l_tim_dev_s *dev, uint8_t source);
  int (*disableint)(struct nrf54l_tim_dev_s *dev, uint8_t source);
  int (*checkint)(struct nrf54l_tim_dev_s *dev, uint8_t source);
  int (*ackint)(struct nrf54l_tim_dev_s *dev, uint8_t source);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct nrf54l_tim_dev_s *nrf54l_tim_init(int timer);
int nrf54l_tim_deinit(struct nrf54l_tim_dev_s *dev);
uint32_t nrf54l_tim_getbasefreq(struct nrf54l_tim_dev_s *dev);

#endif /* __ARCH_ARM_SRC_NRF54L_NRF54L_TIM_H */
