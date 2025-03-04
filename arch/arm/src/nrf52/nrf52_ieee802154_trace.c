/****************************************************************************
 * arch/arm/src/nrf52/nrf52_ieee802154_trace.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <stdint.h>
#include <stdio.h>

#include <arch/board/board.h>

#include "nrf52_ieee802154_trace.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This must match enum radio_trace_type_e */

static const char *g_radio_trace_str[] =
{
  "RADIO IRQ",
  "RADIO IRQ: RX DONE",
  "RADIO IRQ: TX DONE",
  "RADIO IRQ: ACK TX",
  "RADIO IRQ: WAIT ACK",
  "RADIO IRQ: RX ACK DONE",
  "RADIO IRQ: TX CCA BUSY",

  "TIM IRQ: TIM ACK TX",
  "TIM IRQ; TIM TX DELAY",
  "TIM IRQ: TIM WAIT ACK",
  "TIM IRQ: TIM CSMA DELAY",
  "RTC IRQ: RTC SD",
  "RTC IRQ: RTC CAP",
  "RTC IRQ: RTC TIMESLOT",
  "RTC IRQ: RTC BI",

  "WORK: RX",
  "WORK: TX",
  "WORK: BUSY",
  "WORK: ED",
  "WORK: NO ACK",

  "ACKTX",
  "TIMSTART",
  "RXENABLE",
  "RXDISABLE",
  "CSMASETUP",
  "CSMATRIGGER",
  "NOCSMATRIGGER",
  "NOACK",
  "DROPFRAME",
  "TXTRYAGAIN",
};

/* Trace data */

static volatile size_t g_radio_trace_cntr = 0;
static volatile struct radio_trace_s
g_radio_trace[CONFIG_NRF52_RADIO_IEEE802154_TRACE_BUFSIZE];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_radioi8_trace_init
 *
 * Description:
 *   Initialize trace interface.
 *
 ****************************************************************************/

void nrf52_radioi8_trace_init(void)
{
  g_radio_trace_cntr = 0;
  memset((void *)g_radio_trace, 0, sizeof(g_radio_trace));
}

/****************************************************************************
 * Name: nrf52_radioi8_trace_put
 *
 * Description:
 *   Put trace event.
 *
 ****************************************************************************/

void nrf52_radioi8_trace_put(uint8_t type, uint32_t arg)
{
  if (g_radio_trace_cntr >= CONFIG_NRF52_RADIO_IEEE802154_TRACE_BUFSIZE)
    {
      g_radio_trace_cntr = 0;
    }

  g_radio_trace[g_radio_trace_cntr].time = perf_gettime();
  g_radio_trace[g_radio_trace_cntr].str  = g_radio_trace_str[type];
  g_radio_trace[g_radio_trace_cntr].arg  = arg;
  g_radio_trace_cntr += 1;
}

/****************************************************************************
 * Name: nrf52_radioi8_trace_dump
 *
 * Description:
 *   Dump radio trace.
 *
 ****************************************************************************/

void nrf52_radioi8_trace_dump(void)
{
  int i = 0;

  printf("NRF52 Radio IEEE 802.15.4 Trace dump:\n");
  printf("CPU freq: %d\n\n", BOARD_SYSTICK_CLOCK);

  for (i = 0; i < g_radio_trace_cntr; i += 1)
    {
      printf("[%lu]: %s %lu\n\n",
             g_radio_trace[i].time,
             g_radio_trace[i].str,
             g_radio_trace[i].arg);
    }

  /* Reset trace buffer */

  g_radio_trace_cntr = 0;
  memset((void *)g_radio_trace, 0, sizeof(g_radio_trace));
}
