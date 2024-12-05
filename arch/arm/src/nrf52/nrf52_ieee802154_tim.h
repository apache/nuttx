/****************************************************************************
 * arch/arm/src/nrf52/nrf52_ieee802154_tim.h
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

#ifndef __ARCH_ARM_SRC_NRF52_NRF52_IEEE802154_TIM_H
#define __ARCH_ARM_SRC_NRF52_NRF52_IEEE802154_TIM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "nrf52_tim.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum nrf52_ieee802154_timer_e
{
  NRF52_TIMER_CHAN_ACK       = 0, /* Handle ACK time */
  NRF52_TIMER_CHAN_TXDELAY   = 1, /* Handle TX delay */
  NRF52_TIMER_CHAN_WAITACK   = 2, /* Handle ACK wait */
  NRF52_TIMER_CHAN_CSMADELAY = 3, /* Handle CSMA wait */
};

/* Forward reference */

struct nrf52_radioi8_tim_s;
struct nrf52_radioi8_dev_s;

/* Timer ops  */

struct nrf52_radioi8_tim_ops_s
{
  /* Configure TIMER event */

  int (*setup)(struct nrf52_radioi8_dev_s *dev, uint8_t chan, uint32_t val);

  /* Stop timer */

  void (*stop)(struct nrf52_radioi8_dev_s *dev);

  /* Reset timer */

  void (*reset)(struct nrf52_radioi8_dev_s *dev);
};

/* Timer interface */

struct nrf52_radioi8_tim_s
{
  /* Timer lower-half */

  struct nrf52_tim_dev_s *tim;

  /* IEEE 802.15.4 timer operations */

  struct nrf52_radioi8_tim_ops_s *ops;

  /* Timer state */

  int8_t tim_now;
  bool   tim_pending;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_radioi8_tim_init
 *
 * Description:
 *   Initialize high resoluton timer for IEEE802154 operations.
 *   Used to handle short radio timeouts like ACK, IFS or delayed
 *   transmitions.
 *
 ****************************************************************************/

struct nrf52_radioi8_tim_s *
nrf52_radioi8_tim_init(struct nrf52_radioi8_dev_s *dev);

#endif  /* __ARCH_ARM_SRC_NRF52_NRF52_IEEE802154_TIM_H */
