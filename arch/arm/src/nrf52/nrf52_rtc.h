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
#define NRF52_RTC_GETBASE(d)              ((d)->ops->getbase(d))

/* These are defined for direct access to registers, which is needed in some
 * critical parts where access speed is important
 */

#define NRF52_RTC_GETCOUNTER_REG(base)    (getreg32(base + NRF52_RTC_COUNTER_OFFSET))
#define NRF52_RTC_SETCC_REG(base, ch, cc) (putreg32(cc, base + NRF52_RTC_CC_OFFSET(ch)))
#define NRF52_RTC_GETCC_REG(base, ch)     (getreg32(base + NRF52_RTC_CC_OFFSET(ch)))

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

  int (*start)(struct nrf52_rtc_dev_s *dev);
  int (*stop)(struct nrf52_rtc_dev_s *dev);
  int (*clear)(struct nrf52_rtc_dev_s *dev);
  int (*trgovrflw)(struct nrf52_rtc_dev_s *dev);

  /* RTC operations */

  int (*getcounter)(struct nrf52_rtc_dev_s *dev, uint32_t *cc);
  int (*setcc)(struct nrf52_rtc_dev_s *dev, uint8_t i, uint32_t cc);
  int (*getcc)(struct nrf52_rtc_dev_s *dev, uint8_t i, uint32_t *cc);
  int (*setpre)(struct nrf52_rtc_dev_s *dev, uint16_t pre);

  /* RTC interrupts */

  int (*setisr)(struct nrf52_rtc_dev_s *dev, xcpt_t handler, void *arg);
  int (*enableint)(struct nrf52_rtc_dev_s *dev, uint8_t source);
  int (*disableint)(struct nrf52_rtc_dev_s *dev, uint8_t source);
  int (*checkint)(struct nrf52_rtc_dev_s *dev, uint8_t source);
  int (*ackint)(struct nrf52_rtc_dev_s *dev, uint8_t source);

  /* RTC events */

  int (*enableevt)(struct nrf52_rtc_dev_s *dev, uint8_t evt);
  int (*disableevt)(struct nrf52_rtc_dev_s *dev, uint8_t evt);

  /* Utility */

  uint32_t (*getbase)(struct nrf52_rtc_dev_s *dev);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct nrf52_rtc_dev_s *nrf52_rtc_init(int rtc);
int nrf52_rtc_deinit(struct nrf52_rtc_dev_s *dev);

#endif /* __ARCH_ARM_SRC_NRF52_NRF52_RTC_H */
