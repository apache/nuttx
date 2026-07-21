/****************************************************************************
 * arch/arm/src/common/ameba/ameba_timer.c
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

/* NuttX timer lower half for the Realtek Ameba general-purpose timers.  The
 * chip carries twelve identical 32-bit up-counting timers (TIM0..TIM11);
 * this driver is a parameterised lower half that binds any of them, by
 * an instance index into the per-chip AMEBA_TIMER_CONFIG_TABLE.  The board
 * calls ameba_timer_initialize() once per device, so several timers can be
 * exposed at once -- on the pke8721daf /dev/timer0 is TIM1 (32.768 kHz, long
 * low-power intervals) and /dev/timer1 is TIM2 (also 32.768 kHz).  TIM0 is
 * reserved by the ROM as the system timer (SYSTIMER, 31 us/tick) and is
 * never exposed as a general-purpose timer here.
 *
 * Each timer counts up from 0 to its auto-reload value ARR, raises the
 * update interrupt on overflow and reloads to 0 automatically, so a running
 * timer is inherently periodic.  A timeout in microseconds maps to ARR with
 * the single SDK formula ARR = us * clkfreq / 1e6 - 1 (verified against the
 * vendor timer_api.c gtimer_reload for the 32.768 kHz basic timers); the
 * arithmetic is done in 64 bits so a large microsecond timeout scaled by the
 * clock does not overflow.
 *
 * The timer is driven through the SDK fwlib RTIM API.  The time-base setup
 * (RTIM_TimeBaseStructInit/RTIM_TimeBaseInit/RTIM_Cmd/RTIM_INTConfig/
 * RTIM_GetCount) resolves to the on-chip ROM symbol table (_LONG_CALL_
 * entries), while the interrupt-clear and hot period-change helpers
 * (RTIM_INTClear/RTIM_ChangePeriodImmediate) come from the fwlib RAM source
 * ameba_tim.c and linked in.  The NuttX interrupt is attached with the
 * standard irq_attach()/up_enable_irq() (RTIM_TimeBaseInit is passed a NULL
 * user callback so the vendor HAL installs nothing of its own), matching the
 * other Ameba drivers on this core.
 *
 * The chip-specific wiring (per-instance register base, input clock, RCC
 * clock masks and IRQ number) lives entirely in the per-chip
 * ameba_timer_chip.h instance table; the shared driver reads only that table
 * and is never edited per chip.  As in the PWM driver the one fwlib init
 * struct layout used here is mirrored locally rather than pulling in the
 * vendor <ameba_pwmtimer.h>.
 */

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <string.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/timers/timer.h>

#include "ameba_timer.h"
#include "ameba_timer_chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Second/third argument to fwlib "state" style APIs. */

#define AMEBA_DISABLE               0x0
#define AMEBA_ENABLE                0x1

/* fwlib time-base field values (from ameba_pwmtimer.h). */

#define AMEBA_TIM_IT_UPDATE         0x00000001  /* TIM_IT_Update             */
#define AMEBA_TIM_UPDATESRC_OVER    0x00000004  /* TIM_UpdateSource_Overflow */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* One row of the per-chip AMEBA_TIMER_CONFIG_TABLE. */

struct ameba_timer_config_s
{
  uintptr_t base;              /* Non-secure timer register base */
  uint32_t  periph;            /* RCC periph function mask (arg 1) */
  uint32_t  clk;               /* RCC periph clock mask (arg 2) */
  uint32_t  clkfreq;           /* Timer input clock (Hz) */
  int       irq;               /* NuttX IRQ number */
  uint8_t   idx;               /* Timer index (TIM_Idx) */
};

/* Layout-compatible mirror of the fwlib RTIM_TimeBaseInitTypeDef (same field
 * order and types); passed by address to RTIM_TimeBaseStructInit()/
 * RTIM_TimeBaseInit().
 */

struct ameba_tim_timebase_s
{
  uint32_t prescaler;          /* TIM_Prescaler    */
  uint32_t period;             /* TIM_Period       */
  uint32_t updateevent;        /* TIM_UpdateEvent  */
  uint32_t updatesource;       /* TIM_UpdateSource */
  uint32_t arrprotection;      /* TIM_ARRProtection */
  uint8_t  idx;                /* TIM_Idx          */
  uint32_t securetimer;        /* TIM_SecureTimer  */
};

struct ameba_timer_lowerhalf_s
{
  const struct timer_ops_s *ops; /* Lower half operations (must be first) */
  struct ameba_timer_config_s cfg;
  tccb_t    callback;          /* Upper-half expiration callback */
  void     *arg;               /* Argument for the callback */
  uint32_t  timeout;           /* Programmed timeout (microseconds) */
  uint32_t  arr;               /* Auto-reload value for that timeout */
  bool      started;           /* Time base is running */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SDK fwlib RTIM/clock API (ROM for the time base, RAM ameba_tim.c for the
 * interrupt-clear and period-change helpers).  The RTIM_TypeDef pointer is
 * passed as the raw register base cast to void *.
 */

extern void RCC_PeriphClockCmd(uint32_t periph, uint32_t clock,
                               uint8_t newstate);
extern void RTIM_TimeBaseStructInit(struct ameba_tim_timebase_s *init);
extern void RTIM_TimeBaseInit(void *timx,
                              struct ameba_tim_timebase_s *init,
                              int irqnum, void *usercb, uint32_t cbdata);
extern void RTIM_Cmd(void *timx, uint32_t newstate);
extern void RTIM_INTConfig(void *timx, uint32_t tim_it, uint32_t newstate);
extern void RTIM_INTClear(void *timx);
extern void RTIM_ChangePeriodImmediate(void *timx, uint32_t autoreload);
extern uint32_t RTIM_GetCount(void *timx);

/* Timer lower-half operations. */

static int ameba_timer_start(struct timer_lowerhalf_s *lower);
static int ameba_timer_stop(struct timer_lowerhalf_s *lower);
static int ameba_timer_getstatus(struct timer_lowerhalf_s *lower,
                                 struct timer_status_s *status);
static int ameba_timer_settimeout(struct timer_lowerhalf_s *lower,
                                  uint32_t timeout);
static void ameba_timer_setcallback(struct timer_lowerhalf_s *lower,
                                    tccb_t callback, void *arg);
static int ameba_timer_maxtimeout(struct timer_lowerhalf_s *lower,
                                  uint32_t *maxtimeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct timer_ops_s g_ameba_timer_ops =
{
  .start      = ameba_timer_start,
  .stop       = ameba_timer_stop,
  .getstatus  = ameba_timer_getstatus,
  .settimeout = ameba_timer_settimeout,
  .setcallback = ameba_timer_setcallback,
  .maxtimeout = ameba_timer_maxtimeout,
};

/* Per-chip instance table: { base, periph, clk, clkfreq, irq, idx }. */

static const struct ameba_timer_config_s
  g_ameba_timer_config[AMEBA_TIMER_NINSTANCES] = AMEBA_TIMER_CONFIG_TABLE;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ameba_timer_us2arr
 *
 * Description:
 *   Convert a timeout in microseconds to the timer's auto-reload value using
 *   the SDK formula ARR = us * clkfreq / 1e6 - 1, in 64-bit arithmetic so a
 *   large microsecond timeout scaled by the clock does not overflow.
 *
 ****************************************************************************/

static uint32_t ameba_timer_us2arr(uint32_t clkfreq, uint32_t us)
{
  uint64_t ticks = ((uint64_t)us * clkfreq) / 1000000ull;

  if (ticks == 0)
    {
      ticks = 1;
    }

  return (uint32_t)(ticks - 1);
}

/****************************************************************************
 * Name: ameba_timer_interrupt
 *
 * Description:
 *   Update-event ISR.  Clears the flag, invokes the upper-half callback and
 *   honours its verdict: a false return stops the timer, a true return keeps
 *   it periodic and may hot-change the period to the callback's next
 *   interval (deferred by the ARR protection to the reload just started).
 *
 ****************************************************************************/

/* Dummy handler passed to RTIM_TimeBaseInit so the ROM performs its full
 * timer configuration (some register setup is skipped when UserCB is NULL).
 * NuttX installs its own vector table, so this is never actually invoked.
 */

static uint32_t ameba_timer_dummy_cb(void *data)
{
  UNUSED(data);
  return 0;
}

static int ameba_timer_interrupt(int irq, void *context, void *arg)
{
  struct ameba_timer_lowerhalf_s *priv = arg;

  /* Clear the pending update flag via the ROM helper (it polls the timer
   * clock domain to guarantee the write took effect).
   */

  RTIM_INTClear((void *)priv->cfg.base);

  if (priv->callback != NULL)
    {
      uint32_t next = priv->timeout;

      if (priv->callback(&next, priv->arg))
        {
          if (next != priv->timeout && next > 0)
            {
              /* The callback asked for a different next interval. */

              priv->timeout = next;
              priv->arr     = ameba_timer_us2arr(priv->cfg.clkfreq, next);
              RTIM_ChangePeriodImmediate((void *)priv->cfg.base, priv->arr);
            }
        }
      else
        {
          /* The callback asked to stop after this expiration. */

          RTIM_INTConfig((void *)priv->cfg.base, AMEBA_TIM_IT_UPDATE,
                         AMEBA_DISABLE);
          RTIM_Cmd((void *)priv->cfg.base, AMEBA_DISABLE);
          priv->started = false;
        }
    }

  /* Clear once more (vendor double-clear guard). */

  RTIM_INTClear((void *)priv->cfg.base);

  return OK;
}

/****************************************************************************
 * Name: ameba_timer_start
 ****************************************************************************/

static int ameba_timer_start(struct timer_lowerhalf_s *lower)
{
  struct ameba_timer_lowerhalf_s *priv =
    (struct ameba_timer_lowerhalf_s *)lower;
  struct ameba_tim_timebase_s tb;

  if (priv->started)
    {
      return -EPERM;
    }

  if (priv->arr == 0 && priv->timeout == 0)
    {
      /* No timeout has been programmed yet. */

      return -EINVAL;
    }

  memset(&tb, 0, sizeof(tb));
  RTIM_TimeBaseStructInit(&tb);

  /* Match the vendor gtimer_init exactly: TIM_Period is NOT supplied here.
   * TimeBaseInit with ARR preload enabled would leave the auto-reload shadow
   * at its reset value (0) until the first overflow, so arming the interrupt
   * immediately floods the CPU.  The period is instead programmed by the
   * RTIM_ChangePeriodImmediate() below, which clears ARPE, writes ARR and
   * issues a UG so the shadow is loaded before the timer runs.
   */

  tb.idx           = priv->cfg.idx;
  tb.updateevent   = AMEBA_ENABLE;
  tb.updatesource  = AMEBA_TIM_UPDATESRC_OVER;
  tb.arrprotection = AMEBA_ENABLE;

  /* NULL user callback: the NuttX ISR is attached separately, so the vendor
   * HAL installs no handler of its own.
   */

  RTIM_TimeBaseInit((void *)priv->cfg.base, &tb, priv->cfg.irq,
                    (void *)ameba_timer_dummy_cb, 0);

  /* Program the period and force an immediate shadow reload (vendor
   * gtimer_reload).  The overflow reload is periodic on this IP whatever the
   * ARR-preload bit is, so no further CR fix-up is needed here.
   */

  RTIM_ChangePeriodImmediate((void *)priv->cfg.base, priv->arr);

  /* Only fire the update interrupt when an upper-half callback wants it; a
   * bare timer just free-runs and reloads for getstatus() polling (vendor
   * gtimer_start: INTConfig then Cmd).
   */

  RTIM_INTConfig((void *)priv->cfg.base, AMEBA_TIM_IT_UPDATE,
                 priv->callback != NULL ? AMEBA_ENABLE : AMEBA_DISABLE);

  RTIM_Cmd((void *)priv->cfg.base, AMEBA_ENABLE);

  priv->started = true;
  return OK;
}

/****************************************************************************
 * Name: ameba_timer_stop
 ****************************************************************************/

static int ameba_timer_stop(struct timer_lowerhalf_s *lower)
{
  struct ameba_timer_lowerhalf_s *priv =
    (struct ameba_timer_lowerhalf_s *)lower;

  if (!priv->started)
    {
      return -EPERM;
    }

  RTIM_INTConfig((void *)priv->cfg.base, AMEBA_TIM_IT_UPDATE, AMEBA_DISABLE);
  RTIM_Cmd((void *)priv->cfg.base, AMEBA_DISABLE);
  priv->started = false;
  return OK;
}

/****************************************************************************
 * Name: ameba_timer_getstatus
 ****************************************************************************/

static int ameba_timer_getstatus(struct timer_lowerhalf_s *lower,
                                 struct timer_status_s *status)
{
  struct ameba_timer_lowerhalf_s *priv =
    (struct ameba_timer_lowerhalf_s *)lower;
  uint32_t count;
  uint32_t remaining;

  status->flags = 0;
  if (priv->started)
    {
      status->flags |= TCFLAGS_ACTIVE;
    }

  if (priv->callback != NULL)
    {
      status->flags |= TCFLAGS_HANDLER;
    }

  status->timeout = priv->timeout;

  /* Time left = (ARR + 1 - current up-count) converted back to microseconds
   * in 64-bit arithmetic; a stopped timer has the full timeout left.
   */

  if (priv->started)
    {
      count     = RTIM_GetCount((void *)priv->cfg.base);
      remaining = (count > priv->arr) ? 0 : (priv->arr + 1 - count);
      status->timeleft =
        (uint32_t)(((uint64_t)remaining * 1000000ull) / priv->cfg.clkfreq);
    }
  else
    {
      status->timeleft = priv->timeout;
    }

  return OK;
}

/****************************************************************************
 * Name: ameba_timer_settimeout
 ****************************************************************************/

static int ameba_timer_settimeout(struct timer_lowerhalf_s *lower,
                                  uint32_t timeout)
{
  struct ameba_timer_lowerhalf_s *priv =
    (struct ameba_timer_lowerhalf_s *)lower;

  if (timeout == 0)
    {
      return -EINVAL;
    }

  priv->timeout = timeout;
  priv->arr     = ameba_timer_us2arr(priv->cfg.clkfreq, timeout);

  /* Hot-update a running time base; the auto-reload protection defers the
   * change to the next update event so the count does not glitch.
   */

  if (priv->started)
    {
      RTIM_ChangePeriodImmediate((void *)priv->cfg.base, priv->arr);
    }

  return OK;
}

/****************************************************************************
 * Name: ameba_timer_setcallback
 ****************************************************************************/

static void ameba_timer_setcallback(struct timer_lowerhalf_s *lower,
                                    tccb_t callback, void *arg)
{
  struct ameba_timer_lowerhalf_s *priv =
    (struct ameba_timer_lowerhalf_s *)lower;
  irqstate_t flags;

  flags = enter_critical_section();

  priv->callback = callback;
  priv->arg      = arg;

  /* Track the interrupt enable to the callback while the timer runs, so a
   * callback set or cleared after start() takes effect immediately.
   */

  if (priv->started)
    {
      RTIM_INTConfig((void *)priv->cfg.base, AMEBA_TIM_IT_UPDATE,
                     callback != NULL ? AMEBA_ENABLE : AMEBA_DISABLE);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: ameba_timer_maxtimeout
 ****************************************************************************/

static int ameba_timer_maxtimeout(struct timer_lowerhalf_s *lower,
                                  uint32_t *maxtimeout)
{
  struct ameba_timer_lowerhalf_s *priv =
    (struct ameba_timer_lowerhalf_s *)lower;
  uint64_t maxus;

  /* Largest microsecond value whose ARR still fits the 32-bit counter:
   * us = (ARR_max + 1) * 1e6 / clkfreq, capped to the uint32_t API range.
   */

  maxus = ((uint64_t)0xffffffffull * 1000000ull) / priv->cfg.clkfreq;
  if (maxus > 0xffffffffull)
    {
      maxus = 0xffffffffull;
    }

  *maxtimeout = (uint32_t)maxus;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ameba_timer_initialize
 *
 * Description:
 *   See ameba_timer.h.
 *
 ****************************************************************************/

int ameba_timer_initialize(const char *devpath, int instance)
{
  struct ameba_timer_lowerhalf_s *priv;
  void *handle;

  if (instance < 0 || instance >= AMEBA_TIMER_NINSTANCES)
    {
      return -EINVAL;
    }

  priv = kmm_zalloc(sizeof(struct ameba_timer_lowerhalf_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->ops = &g_ameba_timer_ops;
  priv->cfg = g_ameba_timer_config[instance];

  /* Gate the timer's peripheral clock (function and clock masks are equal on
   * this chip) and wire the NuttX interrupt.  The time base itself is left
   * stopped until start(); settimeout() supplies the period first.
   */

  RCC_PeriphClockCmd(priv->cfg.periph, priv->cfg.clk, AMEBA_ENABLE);

  irq_attach(priv->cfg.irq, ameba_timer_interrupt, priv);
  up_enable_irq(priv->cfg.irq);

  handle = timer_register(devpath, (struct timer_lowerhalf_s *)priv);
  if (handle == NULL)
    {
      up_disable_irq(priv->cfg.irq);
      irq_detach(priv->cfg.irq);
      _err("ERROR: timer_register(%s) failed\n", devpath);
      kmm_free(priv);
      return -EEXIST;
    }

  return OK;
}
