/****************************************************************************
 * arch/arm/src/nrf52/nrf52_rtc.h
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

#ifndef __ARCH_ARM_SRC_NRF52_NRF52_RTC_H
#define __ARCH_ARM_SRC_NRF52_NRF52_RTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Helpers ******************************************************************/

#define NRF52_RTC_START(d)                ((d)->ops->start(d))
#define NRF52_RTC_STOP(d)                 ((d)->ops->stop(d))
#define NRF52_RTC_CLEAR(d)                ((d)->ops->clear(d))
#define NRF52_RTC_TRGOVRFLW(d)            ((d)->ops->trgovrflw(d))
#define NRF52_RTC_GETCOUNTER(d, c)        ((d)->ops->getcounter(d, c))
#define NRF52_RTC_SETCC(d, i, cc)         ((d)->ops->setcc(d, i, cc))
#define NRF52_RTC_GETCC(d, i, cc)         ((d)->ops->setcc(d, i, cc))
#define NRF52_RTC_SETPRE(d, pre)          ((d)->ops->setpre(d, pre))
#define NRF52_RTC_SETISR(d, hnd, arg)     ((d)->ops->setisr(d, hnd, arg))
#define NRF52_RTC_ENABLEINT(d, s)         ((d)->ops->enableint(d, s))
#define NRF52_RTC_DISABLEINT(d, s)        ((d)->ops->disableint(d, s))
#define NRF52_RTC_CHECKINT(d, s)          ((d)->ops->checkint(d, s))
#define NRF52_RTC_ACKINT(d, s)            ((d)->ops->ackint(d, s))
#define NRF52_RTC_ENABLEEVT(d, s)         ((d)->ops->enableevt(d, s))
#define NRF52_RTC_DISABLEEVT(d, s)        ((d)->ops->disableevt(d, s))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* RTC CC index */

enum nrf52_rtc_cc_e
{
  NRF52_RTC_CC0 = 0,
  NRF52_RTC_CC1 = 1,
  NRF52_RTC_CC2 = 2,
  NRF52_RTC_CC3 = 3,
};

/* RTC Interrupts/Events */

enum nrf52_rtc_evt_e
{
  NRF52_RTC_EVT_TICK     = 0,
  NRF52_RTC_EVT_OVRFLW   = 1,
  NRF52_RTC_EVT_COMPARE0 = 2,
  NRF52_RTC_EVT_COMPARE1 = 3,
  NRF52_RTC_EVT_COMPARE2 = 4,
  NRF52_RTC_EVT_COMPARE3 = 5,
};

/* NRF52 RTC device */

struct nrf52_rtc_dev_s
{
  struct nrf52_rtc_ops_s *ops;
};

/* NRF52 RTC ops */

struct nrf52_rtc_ops_s
{
  /* RTC tasks */

  CODE int (*start)(FAR struct nrf52_rtc_dev_s *dev);
  CODE int (*stop)(FAR struct nrf52_rtc_dev_s *dev);
  CODE int (*clear)(FAR struct nrf52_rtc_dev_s *dev);
  CODE int (*trgovrflw)(FAR struct nrf52_rtc_dev_s *dev);

  /* RTC operations */

  CODE int (*getcounter)(FAR struct nrf52_rtc_dev_s *dev, FAR uint32_t *cc);
  CODE int (*setcc)(FAR struct nrf52_rtc_dev_s *dev, uint8_t i, uint32_t cc);
  CODE int (*getcc)(FAR struct nrf52_rtc_dev_s *dev, uint8_t i,
                    FAR uint32_t *cc);
  CODE int (*setpre)(FAR struct nrf52_rtc_dev_s *dev, uint16_t pre);

  /* RTC interrupts */

  CODE int (*setisr)(FAR struct nrf52_rtc_dev_s *dev, xcpt_t handler,
                     FAR void * arg);
  CODE int (*enableint)(FAR struct nrf52_rtc_dev_s *dev, uint8_t source);
  CODE int (*disableint)(FAR struct nrf52_rtc_dev_s *dev, uint8_t source);
  CODE int (*checkint)(FAR struct nrf52_rtc_dev_s *dev, uint8_t source);
  CODE int (*ackint)(FAR struct nrf52_rtc_dev_s *dev, uint8_t source);

  /* RTC events */

  CODE int (*enableevt)(FAR struct nrf52_rtc_dev_s *dev, uint8_t evt);
  CODE int (*disableevt)(FAR struct nrf52_rtc_dev_s *dev, uint8_t evt);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

FAR struct nrf52_rtc_dev_s *nrf52_rtc_init(int rtc);
int nrf52_rtc_deinit(FAR struct nrf52_rtc_dev_s *dev);

#endif /* __ARCH_ARM_SRC_NRF52_NRF52_RTC_H */
