/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_grtc.h
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

#ifndef __ARCH_ARM_SRC_NRF54L_NRF54L_GRTC_H
#define __ARCH_ARM_SRC_NRF54L_NRF54L_GRTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/irq.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Helpers ******************************************************************/

#define NRF54L_GRTC_START(d)                ((d)->ops->start(d))
#define NRF54L_GRTC_STOP(d)                 ((d)->ops->stop(d))
#define NRF54L_GRTC_CLEAR(d)                ((d)->ops->clear(d))
#define NRF54L_GRTC_GETCOUNTER(d, c)        ((d)->ops->getcounter(d, c))
#define NRF54L_GRTC_SETCC(d, i, cc)         ((d)->ops->setcc(d, i, cc))
#define NRF54L_GRTC_GETCC(d, i, cc)         ((d)->ops->getcc(d, i, cc))
#define NRF54L_GRTC_DISABLECC(d, i)         ((d)->ops->disablecc(d, i))
#define NRF54L_GRTC_SETISR(d, hnd, arg)     ((d)->ops->setisr(d, hnd, arg))
#define NRF54L_GRTC_ENABLEINT(d, s)         ((d)->ops->enableint(d, s))
#define NRF54L_GRTC_DISABLEINT(d, s)        ((d)->ops->disableint(d, s))
#define NRF54L_GRTC_CHECKINT(d, s)          ((d)->ops->checkint(d, s))
#define NRF54L_GRTC_ACKINT(d, s)            ((d)->ops->ackint(d, s))
#define NRF54L_GRTC_GETBASE(d)              ((d)->ops->getbase(d))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* GRTC CC index */

enum nrf54l_grtc_cc_e
{
  NRF54L_GRTC_CC0 = 0,
  NRF54L_GRTC_CC1 = 1,
  NRF54L_GRTC_CC2 = 2,
  NRF54L_GRTC_CC3 = 3,
  NRF54L_GRTC_CC4 = 4,
  NRF54L_GRTC_CC5 = 5,
  NRF54L_GRTC_CC6 = 6,
  NRF54L_GRTC_CC7 = 7,
  NRF54L_GRTC_CC8 = 8,
  NRF54L_GRTC_CC9 = 9,
  NRF54L_GRTC_CC10 = 10,
  NRF54L_GRTC_CC11 = 11,
};

/* GRTC Interrupts/Events */

enum nrf54l_grtc_evt_e
{
  NRF54L_GRTC_EVT_COMPARE0 = 0,
  NRF54L_GRTC_EVT_COMPARE1 = 1,
  NRF54L_GRTC_EVT_COMPARE2 = 2,
  NRF54L_GRTC_EVT_COMPARE3 = 3,
  NRF54L_GRTC_EVT_COMPARE4 = 4,
  NRF54L_GRTC_EVT_COMPARE5 = 5,
  NRF54L_GRTC_EVT_COMPARE6 = 6,
  NRF54L_GRTC_EVT_COMPARE7 = 7,
  NRF54L_GRTC_EVT_COMPARE8 = 8,
  NRF54L_GRTC_EVT_COMPARE9 = 9,
  NRF54L_GRTC_EVT_COMPARE10 = 10,
  NRF54L_GRTC_EVT_COMPARE11 = 11,
};

/* NRF54L GRTC device */

struct nrf54l_grtc_dev_s
{
  struct nrf54l_grtc_ops_s *ops;
};

/* NRF54L GRTC ops */

struct nrf54l_grtc_ops_s
{
  /* GRTC tasks */

  int (*start)(struct nrf54l_grtc_dev_s *dev);
  int (*stop)(struct nrf54l_grtc_dev_s *dev);
  int (*clear)(struct nrf54l_grtc_dev_s *dev);

  /* GRTC operations */

  int (*getcounter)(struct nrf54l_grtc_dev_s *dev, uint64_t *cc);
  int (*setcc)(struct nrf54l_grtc_dev_s *dev, uint8_t i, uint64_t cc);
  int (*getcc)(struct nrf54l_grtc_dev_s *dev, uint8_t i, uint64_t *cc);
  int (*disablecc)(struct nrf54l_grtc_dev_s *dev, uint8_t i);

  /* GRTC interrupts */

  int (*setisr)(struct nrf54l_grtc_dev_s *dev, xcpt_t handler, void *arg);
  int (*enableint)(struct nrf54l_grtc_dev_s *dev, uint8_t source);
  int (*disableint)(struct nrf54l_grtc_dev_s *dev, uint8_t source);
  int (*checkint)(struct nrf54l_grtc_dev_s *dev, uint8_t source);
  int (*ackint)(struct nrf54l_grtc_dev_s *dev, uint8_t source);

  /* Utility */

  uint32_t (*getbase)(struct nrf54l_grtc_dev_s *dev);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct nrf54l_grtc_dev_s *nrf54l_grtc_init(int grtc);
int nrf54l_grtc_deinit(struct nrf54l_grtc_dev_s *dev);

#endif /* __ARCH_ARM_SRC_NRF54L_NRF54L_GRTC_H */
