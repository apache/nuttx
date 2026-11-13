/****************************************************************************
 * drivers/timers/sp804_timer.c
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
 * SP804 "timer" lower-half (periodic /dev/timerN)
 *
 * sp804_oneshot.c binds SP804 Timer 2 as a oneshot alarm for the OS.  This
 * file exposes SP804 Timer 1 as a standard NuttX timer lower-half so it can
 * be registered as /dev/timerN and exercised by apps/examples/timer.  The
 * two are independent register windows (Timer 1 at base+0x00) and only one
 * of them is used on a given board.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/timers/sp804.h>

#include "sp804.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sp804_timer_lowerhalf_s
{
  struct timer_lowerhalf_s lower;  /* Publicly visible portion */
  uintptr_t base;                  /* Timer 1 register base */
  uint32_t  freq;                  /* Input clock frequency (Hz) */
  uint32_t  load;                  /* Reload value for the configured period */
  tccb_t    callback;              /* Upper-half timeout callback */
  FAR void *arg;                   /* Argument for the callback */
  bool      started;               /* True if the timer is running */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sp804_timer_us_to_ticks / sp804_timer_ticks_to_us
 ****************************************************************************/

static uint32_t
sp804_timer_us_to_ticks(FAR struct sp804_timer_lowerhalf_s *p, uint32_t usec)
{
  uint64_t ticks = ((uint64_t)usec * p->freq) / USEC_PER_SEC;

  if (ticks < 1)
    {
      ticks = 1;
    }
  else if (ticks > UINT32_MAX)
    {
      ticks = UINT32_MAX;
    }

  return (uint32_t)ticks;
}

static uint32_t
sp804_timer_ticks_to_us(FAR struct sp804_timer_lowerhalf_s *p,
                        uint32_t ticks)
{
  return (uint32_t)(((uint64_t)ticks * USEC_PER_SEC) / p->freq);
}

/****************************************************************************
 * Name: sp804_timer_interrupt
 ****************************************************************************/

static int sp804_timer_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct sp804_timer_lowerhalf_s *priv = arg;
  uint32_t next = 0;

  /* Acknowledge the interrupt (write to INTCLR). */

  sp804_putreg(1, priv->base, SP804_INTCLR);

  if (priv->callback != NULL &&
      priv->callback(&next, priv->arg))
    {
      /* The upper half wants to keep running.  The hardware reloads
       * automatically in periodic mode, but honour a requested change of
       * period if one was supplied.
       */

      if (next > 0)
        {
          uint32_t load = sp804_timer_us_to_ticks(priv, next);

          priv->load = load;
          sp804_putreg(load, priv->base, SP804_LOAD);
        }
    }
  else
    {
      /* No callback or callback asked to stop: halt the timer. */

      sp804_putreg(0, priv->base, SP804_CONTROL);
      priv->started = false;
    }

  return 0;
}

/****************************************************************************
 * Name: sp804_timer_start
 ****************************************************************************/

static int sp804_timer_start(FAR struct timer_lowerhalf_s *lower)
{
  FAR struct sp804_timer_lowerhalf_s *priv =
    (FAR struct sp804_timer_lowerhalf_s *)lower;

  sp804_putreg(0, priv->base, SP804_CONTROL);
  sp804_putreg(priv->load, priv->base, SP804_LOAD);
  sp804_putreg(1, priv->base, SP804_INTCLR);
  sp804_putreg(SP804_CTRL_ENABLE | SP804_CTRL_32BIT |
               SP804_CTRL_PERIODIC | SP804_CTRL_IE,
               priv->base, SP804_CONTROL);
  priv->started = true;

  return OK;
}

/****************************************************************************
 * Name: sp804_timer_stop
 ****************************************************************************/

static int sp804_timer_stop(FAR struct timer_lowerhalf_s *lower)
{
  FAR struct sp804_timer_lowerhalf_s *priv =
    (FAR struct sp804_timer_lowerhalf_s *)lower;

  sp804_putreg(0, priv->base, SP804_CONTROL);
  sp804_putreg(1, priv->base, SP804_INTCLR);
  priv->started = false;

  return OK;
}

/****************************************************************************
 * Name: sp804_timer_getstatus
 ****************************************************************************/

static int sp804_timer_getstatus(FAR struct timer_lowerhalf_s *lower,
                                 FAR struct timer_status_s *status)
{
  FAR struct sp804_timer_lowerhalf_s *priv =
    (FAR struct sp804_timer_lowerhalf_s *)lower;

  status->flags = 0;
  if (priv->started)
    {
      status->flags |= TCFLAGS_ACTIVE;
    }

  if (priv->callback != NULL)
    {
      status->flags |= TCFLAGS_HANDLER;
    }

  status->timeout  = sp804_timer_ticks_to_us(priv, priv->load);
  status->timeleft = sp804_timer_ticks_to_us(priv,
                       sp804_getreg(priv->base, SP804_VALUE));

  return OK;
}

/****************************************************************************
 * Name: sp804_timer_settimeout
 ****************************************************************************/

static int sp804_timer_settimeout(FAR struct timer_lowerhalf_s *lower,
                                  uint32_t timeout)
{
  FAR struct sp804_timer_lowerhalf_s *priv =
    (FAR struct sp804_timer_lowerhalf_s *)lower;

  priv->load = sp804_timer_us_to_ticks(priv, timeout);
  sp804_putreg(priv->load, priv->base, SP804_LOAD);

  return OK;
}

/****************************************************************************
 * Name: sp804_timer_setcallback
 ****************************************************************************/

static void sp804_timer_setcallback(FAR struct timer_lowerhalf_s *lower,
                                    tccb_t callback, FAR void *arg)
{
  FAR struct sp804_timer_lowerhalf_s *priv =
    (FAR struct sp804_timer_lowerhalf_s *)lower;
  irqstate_t flags;

  flags = up_irq_save();
  priv->callback = callback;
  priv->arg      = arg;
  up_irq_restore(flags);
}

/****************************************************************************
 * Name: sp804_timer_maxtimeout
 ****************************************************************************/

static int sp804_timer_maxtimeout(FAR struct timer_lowerhalf_s *lower,
                                  FAR uint32_t *maxtimeout)
{
  FAR struct sp804_timer_lowerhalf_s *priv =
    (FAR struct sp804_timer_lowerhalf_s *)lower;

  *maxtimeout = sp804_timer_ticks_to_us(priv, UINT32_MAX);

  return OK;
}

static const struct timer_ops_s g_sp804_timer_ops =
{
  .start      = sp804_timer_start,
  .stop       = sp804_timer_stop,
  .getstatus  = sp804_timer_getstatus,
  .settimeout = sp804_timer_settimeout,
  .setcallback = sp804_timer_setcallback,
  .ioctl      = NULL,
  .maxtimeout = sp804_timer_maxtimeout,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sp804_timer_initialize
 ****************************************************************************/

FAR struct timer_lowerhalf_s *
sp804_timer_initialize(uintptr_t base, int irq, uint32_t freq)
{
  FAR struct sp804_timer_lowerhalf_s *priv;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return NULL;
    }

  priv->lower.ops = &g_sp804_timer_ops;
  priv->base      = base;
  priv->freq      = freq;
  priv->load      = freq;   /* Default 1s period until settimeout() */
  priv->started   = false;

  /* Leave the timer disabled until started. */

  sp804_putreg(0, priv->base, SP804_CONTROL);
  sp804_putreg(1, priv->base, SP804_INTCLR);

  irq_attach(irq, sp804_timer_interrupt, priv);
  up_enable_irq(irq);

  return &priv->lower;
}
