/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_rit.c
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
 * WARNING -- This code is currently *NOT* thread safe.  No checking is done
 *            that one alarm call might not override an already enabled one.
 *            You might be able to handle this with some kind of cascading
 *            scheme where alarm receives the next value in a list of alarms
 *            all in the future.
 *
 *            Brandon Warhurst
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/lpc43_rit.h"
#include "lpc43_rit.h"

#ifdef CONFIG_LPC43_RIT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Set the timer resolution at 1uS
 *
 * This is a battery waster, but I need this kind of resolution for 1-wire
 * protocols... Until I can find a better way to implement them.
 *
 * This should probable be Kconfig material.
  */

#define RIT_TIMER_RESOLUTION CONFIG_LPC43_RIT_RES

/****************************************************************************
 * Private Data
 ****************************************************************************/

static double sec_per_tick;
static uint64_t g_internal_timer;
static uint64_t g_alarm;
struct timespec g_ts;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int lpc43_rit_isr(int irq, void *context, void *arg)
{
  irqstate_t flags;

  flags = enter_critical_section();

  putreg32(RIT_CTRL_INT, LPC43_RIT_CTRL);

  g_internal_timer += (uint64_t)RIT_TIMER_RESOLUTION;
  if (g_alarm > 0 && g_internal_timer >= g_alarm)
    {
      /* handle expired alarm */

      g_ts.tv_sec = (uint32_t)(g_internal_timer / 1000000000);
      g_ts.tv_nsec = (uint32_t)(g_internal_timer % 1000000000);
      nxsched_alarm_expiration(&g_ts);
    }

  leave_critical_section(flags);
  return OK;
}

static inline void lpc43_load_rit_timer(uint32_t value)
{
  putreg32(value, LPC43_RIT_COUNTER);
}

static inline void lpc43_load_rit_compare(uint32_t value)
{
  putreg32(value, LPC43_RIT_COMPVAL);
}

static inline void lpc43_set_rit_timer_mask(uint32_t value)
{
  putreg32(value, LPC43_RIT_MASK);
}

static inline uint32_t lpc43_read_rit_timer(void)
{
  return getreg32(LPC43_RIT_COUNTER);
}

static inline void lpc43_rit_timer_start(void)
{
  uint32_t regval;
  regval = getreg32(LPC43_RIT_CTRL);

  /* Since a cycle timer is in use here, the timer clear is enabled. If
   * using the timer directly, this likely should not be used.  Also, clear
   * the interrupt flag.
   */

  regval |= RIT_CTRL_INT | RIT_CTRL_EN | RIT_CTRL_ENCLR;
  putreg32(regval, LPC43_RIT_CTRL);
}

static inline void lpc43_rit_timer_stop(void)
{
  uint32_t regval;
  regval = getreg32(LPC43_RIT_CTRL);
  regval &= ~RIT_CTRL_EN;
  putreg32(regval, LPC43_RIT_CTRL);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_timer_initialize(void)
{
  uint32_t ticks_per_int;
  uint32_t mask_bits = 0;
  uint32_t mask_test = 0x80000000;

  lpc43_rit_timer_stop();
  lpc43_load_rit_timer(0);
  g_internal_timer = 0;

  /* Set up the IRQ here */

  irq_attach(LPC43M4_IRQ_RITIMER, lpc43_rit_isr, NULL);

  /* Compute how many seconds per tick we have on the main clock.  If it is
   * 204MHz for example, then there should be about 4.90ns per tick
   */

  sec_per_tick = (double)1.0 / (double)LPC43_CCLK;

  /* Given an RIT_TIMER_RESOLUTION, compute how many ticks it will take to
   * reach that resolution.  For example, if we wanted a 1/4uS timer
   * resolution, that would be 250ns resolution.  The timer is an integer
   * value, although maybe this should change, but that means
   * 250/1000000000*0.00000000490 = 51.02 ticks or 51 ticks, roughly.
   * We round up by 1 tick.
   */

  ticks_per_int = RIT_TIMER_RESOLUTION / (1000000000 * sec_per_tick) + 1;

  /* Now we need to compute the mask that will let us set up to generate an
   * interrupt every 1/4uS.  This isn't "tickless" per-se, and probably
   * should be implemented differently, however it allows me to create a
   * 64 bit nanosecond timer than can "free-run" by being updated every
   * RIT_TIMER_RESOLUTION cycles.  I would have implemented the better
   * approach, but I didn't have a good way to determine how to manage a
   * 32 bit ns timer.  Every 21 seconds the thing rolls over@ 204MHz, so
   * you'd have to set up the compare interrupt to handle the roll over.  It
   * WOULD be fewer interrupts, but it seemed to make things more
   * complicated.  When I have a better idea, I'll change this.
   */

  while (!((mask_test >> mask_bits) & ticks_per_int))
    {
      mask_bits++;
    }

  tmrinfo("mask_bits = %d, mask = %X, ticks_per_int = %d\n",
          mask_bits, (0xffffffff << (32 - mask_bits)), ticks_per_int);

  /* Set the mask and compare value so we get interrupts every
   * RIT_TIMER_RESOLUTION cycles.
   */

  lpc43_set_rit_timer_mask((0xffffffff << (32 - mask_bits)));
  lpc43_load_rit_compare(ticks_per_int);

  /* Turn on the IRQ */

  up_enable_irq(LPC43M4_IRQ_RITIMER);

  /* Start the timer */

  lpc43_rit_timer_start();
}

int up_timer_gettime(struct timespec *ts)
{
  ts->tv_sec = (uint32_t)(g_internal_timer / 1000000000);
  ts->tv_nsec = (uint32_t)(g_internal_timer % 1000000000);

  return OK;
}

int up_alarm_cancel(struct timespec *ts)
{
  ts->tv_sec = (uint32_t)(g_internal_timer / 1000000000);
  ts->tv_nsec = (uint32_t)(g_internal_timer % 1000000000);
  g_alarm = 0;
  return OK;
}

int up_alarm_start(const struct timespec *ts)
{
  /* According to the docs, this version should expect to receive the time
   * in the future when the alarm should expire. So that's the way it's
   * coded.
   */

  g_alarm = (uint64_t)ts->tv_sec * (uint64_t)1000000000 +
            (uint64_t)ts->tv_nsec;
  return OK;
}

int up_timer_cancel(struct timespec *ts)
{
  /* Currently this is just an alarm and both are implemented.  This is *NOT*
   * how it is supposed to be and will be corrected, but for now, this is a
   * simple way to implement both.
   * FIXME
   */

  return up_alarm_cancel(ts);
}

int up_timer_start(const struct timespec *ts)
{
  /* According to the docs, this version should basically compute the time
   * in the future when an alarm should go off.  That is the way it could
   * potentially be implemented, so that's the way I did it.
   */

  g_alarm = g_internal_timer;
  g_alarm += (uint64_t)ts->tv_sec * (uint64_t)1000000000 +
             (uint64_t)ts->tv_nsec;
  return OK;
}

#endif /* CONFIG_LPC43_RIT */
