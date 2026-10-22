/****************************************************************************
 * arch/arm/src/nrf52/nrf52_ieee802154_rtc.h
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

#ifndef __ARCH_ARM_SRC_NRF52_NRF52_IEEE802154_RTC_H
#define __ARCH_ARM_SRC_NRF52_NRF52_IEEE802154_RTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/wqueue.h>

#include "nrf52_rtc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Superframe events */

enum nrf52_ieee802154_sfevt_e
{
  NRF52_SFEVT_NONE  = 0,        /* No event scheduled */
  NRF52_SFEVT_START = 1,        /* Superframe start, beacon TX */
  NRF52_SFEVT_SDEND = 2         /* End of the active portion */
};

/* Forward reference */

struct ieee802154_superframespec_s;
struct nrf52_radioi8_rtc_s;
struct nrf52_radioi8_dev_s;

/* RTC ops  */

struct nrf52_radioi8_rtc_ops_s
{
  /* Configure the superframe timings according to superframe spec */

  int (*setup)(struct nrf52_radioi8_dev_s *dev,
               struct ieee802154_superframespec_s *sfspec);

  /* Start transmitting beacons */

  int (*start)(struct nrf52_radioi8_dev_s *dev);

  /* A beacon with the given PHR was just received */

  int (*sync)(struct nrf52_radioi8_dev_s *dev, uint8_t phr);

  /* Stop the superframe */

  int (*stop)(struct nrf52_radioi8_dev_s *dev);

  /* Reset RTC */

  void (*reset)(struct nrf52_radioi8_dev_s *dev);

  /* Time in us since the superframe start, false if not synchronized */

  bool (*elapsed)(struct nrf52_radioi8_dev_s *dev, uint32_t *us);
};

/* RTC interface */

struct nrf52_radioi8_rtc_s
{
  /* RTC lower-half */

  struct nrf52_rtc_dev_s *rtc;

  /* IEEE 802.15.4 RTC operations */

  struct nrf52_radioi8_rtc_ops_s *ops;

  /* For deferring inactive state work to the work queue */

  struct work_s inactive_work;

  /* Superframe timings in us */

  uint64_t sf_start;            /* Start of the current superframe */
  uint64_t evt_time;            /* Time of the scheduled event */
  uint32_t bi;                  /* Beacon interval */
  uint32_t sd;                  /* Superframe duration */
  uint32_t cap;                 /* End of the CAP */
  uint32_t capstart;            /* Start of the CAP, after the beacon IFS */

  /* RTC state */

  uint32_t ovf;                 /* Counter overflows */
  uint8_t  evt;                 /* Scheduled event */
  bool     running;             /* Superframe timing valid */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_radioi8_rtc_init
 *
 * Description:
 *   Initialize low resolution, low power timer for IEEE802154 operations.
 *   Used to handle superframe timings.
 *
 ****************************************************************************/

struct nrf52_radioi8_rtc_s *
nrf52_radioi8_rtc_init(struct nrf52_radioi8_dev_s *dev);

#endif  /* __ARCH_ARM_SRC_NRF52_NRF52_IEEE802154_RTC_H */
