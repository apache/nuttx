/****************************************************************************
 * arch/arm/src/nrf52/nrf52_ieee802154_trace.h
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

#ifndef __ARCH_ARM_SRC_NRF52_NRF52_IEEE802154_TRACE_H
#define __ARCH_ARM_SRC_NRF52_NRF52_IEEE802154_TRACE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Trace item */

struct radio_trace_s
{
  clock_t     time;     /* Event timestamp */
  const char *str;      /* Event string */
  uint32_t    arg;      /* Optional data */
};

/* This must match char *g_radio_trace_str[] */

enum radio_trace_type_e
{
  /* Radio interupts */

  RADIO_TRACE_IRQ_RADIO,
  RADIO_TRACE_IRQ_RXDONE,
  RADIO_TRACE_IRQ_TXDONE,
  RADIO_TRACE_IRQ_ACKTX,
  RADIO_TRACE_IRQ_WAITACK,
  RADIO_TRACE_IRQ_RXACKDONE,
  RADIO_TRACE_IRQ_TXCCABUSY,

  /* Timer interrupts */

  RADIO_TRACE_IRQ_TIMACKTX,
  RADIO_TRACE_IRQ_TIMTXDELAY,
  RADIO_TRACE_IRQ_TIMWAITACK,
  RADIO_TRACE_IRQ_TIMCSMADELAY,
  RADIO_TRACE_IRQ_RTCSD,
  RADIO_TRACE_IRQ_RTCCAP,
  RADIO_TRACE_IRQ_RTCTIMESLOT,
  RADIO_TRACE_IRQ_RTCBI,

  /* Works */

  RADIO_TRACE_WORK_RX,
  RADIO_TRACE_WORK_TX,
  RADIO_TRACE_WORK_BUSY,
  RADIO_TRACE_WORK_ED,
  RADIO_TRACE_WORK_NOACK,

  /* Various events */

  RADIO_TRACE_ACKTX,
  RADIO_TRACE_TIMSTART,
  RADIO_TRACE_RXENABLE,
  RADIO_TRACE_RXDISABLE,
  RADIO_TRACE_CSMASETUP,
  RADIO_TRACE_CSMATRIGGER,
  RADIO_TRACE_NOCSMATRIGGER,
  RADIO_TRACE_NOACK,
  RADIO_TRACE_DROPFRAME,
  RADIO_TRACE_TXRETRY,
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_NRF52_RADIO_IEEE802154_TRACE
void nrf52_radioi8_trace_init(void);
void nrf52_radioi8_trace_put(uint8_t type, uint32_t arg);
void nrf52_radioi8_trace_dump(void);
#else
#  define nrf52_radioi8_trace_put(...)
#endif

#endif  /* __ARCH_ARM_SRC_NRF52_NRF52_IEEE802154_TRACE_H */
