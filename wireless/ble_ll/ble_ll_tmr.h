/****************************************************************************
 * wireless/ble_ll/ble_ll_tmr.h
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
 * Derived from Apache Mynewt NimBLE
 * (nimble/controller/include/controller/ble_ll_tmr.h).
 *
 ****************************************************************************/

#ifndef __WIRELESS_BLE_LL_BLE_LL_TMR_H
#define __WIRELESS_BLE_LL_BLE_LL_TMR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "ble_ll_os.h"
#include "ble_phy.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define USECS_PER_TICK \
  ((1000000 + CONFIG_BLE_LL_TMR_FREQ - 1) / CONFIG_BLE_LL_TMR_FREQ)

#define LL_TMR_LT(_t1, _t2)  ((int32_t)((_t1) - (_t2)) < 0)
#define LL_TMR_GT(_t1, _t2)  ((int32_t)((_t1) - (_t2)) > 0)
#define LL_TMR_GEQ(_t1, _t2) ((int32_t)((_t1) - (_t2)) >= 0)
#define LL_TMR_LEQ(_t1, _t2) ((int32_t)((_t1) - (_t2)) <= 0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef void (ble_ll_tmr_cb)(void *arg);

/* Link Layer timer.  All timers are multiplexed on the single compare
 * channel of the lower-half timer and their callbacks run in interrupt
 * context.
 */

struct ble_ll_tmr
{
  TAILQ_ENTRY(ble_ll_tmr) link;
  ble_ll_tmr_cb *cb;
  void *arg;
  uint32_t expiry;
  bool queued;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void ble_ll_tmr_init(struct ble_ll_tmr *tmr, ble_ll_tmr_cb *cb, void *arg);
void ble_ll_tmr_start(struct ble_ll_tmr *tmr, uint32_t tgt);
void ble_ll_tmr_stop(struct ble_ll_tmr *tmr);

/* Lower-half timer compare event handler */

void ble_ll_tmr_isr(void);

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ble_ll_tmr_get
 *
 ****************************************************************************/

static inline uint32_t ble_ll_tmr_get(void)
{
  return BLE_PHY_OP(tmr_get);
}

/****************************************************************************
 * Name: ble_ll_tmr_t2u
 *
 ****************************************************************************/

static inline uint32_t ble_ll_tmr_t2u(uint32_t ticks)
{
#if CONFIG_BLE_LL_TMR_FREQ == 31250
  return ticks * 32;
#elif CONFIG_BLE_LL_TMR_FREQ == 32768
  return ((ticks >> 9) * 15625) + (((ticks & 0x1ff) * 15625) >> 9);
#else
  return ((uint64_t)ticks * 1000000) / CONFIG_BLE_LL_TMR_FREQ;
#endif
}

/****************************************************************************
 * Name: ble_ll_tmr_u2t
 *
 ****************************************************************************/

static inline uint32_t ble_ll_tmr_u2t(uint32_t usecs)
{
#if CONFIG_BLE_LL_TMR_FREQ == 31250
  return usecs / 32;
#endif
#if CONFIG_BLE_LL_TMR_FREQ == 32768
#ifdef __ARM_FEATURE_IDIV
  if (usecs < 131072)
    {
      return (usecs * 32768) / 1000000;
    }
  else
    {
      return ((uint64_t)usecs * 32768) / 1000000;
    }
#else
  if (usecs <= 31249)
    {
      return (usecs * 137439) / 4194304;
    }
#endif
#endif

  return ((uint64_t)usecs * CONFIG_BLE_LL_TMR_FREQ) / 1000000;
}

/****************************************************************************
 * Name: ble_ll_tmr_u2t_up
 *
 ****************************************************************************/

static inline uint32_t ble_ll_tmr_u2t_up(uint32_t usecs)
{
  return ble_ll_tmr_u2t(usecs + (USECS_PER_TICK - 1));
}

/****************************************************************************
 * Name: ble_ll_tmr_u2t_r
 *
 ****************************************************************************/

static inline uint32_t ble_ll_tmr_u2t_r(uint32_t usecs, uint8_t *rem_us)
{
  uint32_t ticks;

  ticks = ble_ll_tmr_u2t(usecs);
  *rem_us = usecs - ble_ll_tmr_t2u(ticks);
  if (*rem_us == USECS_PER_TICK)
    {
      *rem_us = 0;
      ticks++;
    }

  return ticks;
}

/****************************************************************************
 * Name: ble_ll_tmr_add
 *
 ****************************************************************************/

static inline void ble_ll_tmr_add(uint32_t *ticks, uint8_t *rem_us,
                                  uint32_t usecs)
{
  uint32_t t_ticks;
  uint8_t t_rem_us;

  t_ticks = ble_ll_tmr_u2t_r(usecs, &t_rem_us);

  *ticks += t_ticks;
  *rem_us += t_rem_us;
  if (*rem_us >= USECS_PER_TICK)
    {
      *rem_us -= USECS_PER_TICK;
      *ticks += 1;
    }
}

/****************************************************************************
 * Name: ble_ll_tmr_add_u
 *
 ****************************************************************************/

static inline void ble_ll_tmr_add_u(uint32_t *ticks, uint8_t *rem_us,
                                    uint8_t usecs)
{
  DEBUGASSERT(usecs < USECS_PER_TICK);

  *rem_us += usecs;
  if (*rem_us >= USECS_PER_TICK)
    {
      *rem_us -= USECS_PER_TICK;
      *ticks += 1;
    }
}

/****************************************************************************
 * Name: ble_ll_tmr_sub
 *
 ****************************************************************************/

static inline void ble_ll_tmr_sub(uint32_t *ticks, uint8_t *rem_us,
                                  uint32_t usecs)
{
  uint32_t t_ticks;
  uint8_t t_rem_us;

  if (usecs <= *rem_us)
    {
      *rem_us -= usecs;
      return;
    }

  usecs -= *rem_us;
  *rem_us = 0;

  t_ticks = ble_ll_tmr_u2t_r(usecs, &t_rem_us);
  if (t_rem_us)
    {
      t_ticks += 1;
      *rem_us = USECS_PER_TICK - t_rem_us;
    }

  *ticks -= t_ticks;
}

#endif /* __WIRELESS_BLE_LL_BLE_LL_TMR_H */
