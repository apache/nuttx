/****************************************************************************
 * arch/arm/src/nrf52/nrf52_ieee802154_rtc.h
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

#ifndef __ARCH_ARM_SRC_NRF52_NRF52_IEEE802154_RTC_H
#define __ARCH_ARM_SRC_NRF52_NRF52_IEEE802154_RTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "nrf52_rtc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* RTC events */

enum nrf52_ieee802154_rtc_e
{
  NRF52_RTC_BI       = 0,       /* Beacon Interval interval (BI) */
  NRF52_RTC_TIMESLOT = 1,       /* Time slot */
  NRF52_RTC_CAP      = 2,       /* Contention Access Period (CAP) */
  NRF52_RTC_SD       = 3        /* Super Frame Duration (SD) */
};

/* Forward reference */

struct nrf52_radioi8_rtc_s;
struct nrf52_radioi8_dev_s;

/* RTC ops  */

struct nrf52_radioi8_rtc_ops_s
{
  /* Configure RTC events according to superframe spec */

  int (*setup)(struct nrf52_radioi8_dev_s *dev,
               struct ieee802154_superframespec_s *sfspec);

  /* Start RTC */

  int (*start)(struct nrf52_radioi8_dev_s *dev);

  /* Stop RTC */

  int (*stop)(struct nrf52_radioi8_dev_s *dev);

  /* Reset RTC */

  void (*reset)(struct nrf52_radioi8_dev_s *dev);
};

/* RTC interface */

struct nrf52_radioi8_rtc_s
{
  /* RTC lower-half */

  struct nrf52_rtc_dev_s *rtc;

  /* IEEE 802.15.4 RTC operations */

  struct nrf52_radioi8_rtc_ops_s *ops;

#ifdef CONFIG_NRF52_RADIO_IEEE802154_SUPERFRAME
  /* For deferring inactive state work to the work queue */

  struct work_s inactive_work;

  /* RTC state */

  uint32_t rtc_timeslot;
#endif
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_radioi8_rtc_init
 *
 * Description:
 *   Initialize low resoluton, low power timer for IEEE802154 operations.
 *   Used to handle superframe timings.
 *
 ****************************************************************************/

struct nrf52_radioi8_rtc_s *
nrf52_radioi8_rtc_init(struct nrf52_radioi8_dev_s *dev);

#endif  /* __ARCH_ARM_SRC_NRF52_NRF52_IEEE802154_RTC_H */
