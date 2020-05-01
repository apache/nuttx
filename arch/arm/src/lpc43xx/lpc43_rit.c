/****************************************************************************
 *  arch/arm/src/lpc43/lpc43_rit.c
 *
 *   Copyright (C) 2012, 2016 Gregory Nutt. All rights reserved.
 *   Author: Brandon Warhurst <warhurst_002@yahoo.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

#include "arm_arch.h"
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
static uint64_t internal_timer, alarm;
struct timespec g_ts;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int lpc43_RIT_isr(int irq, FAR void *context, FAR void *arg)
{
  irqstate_t flags;

  flags = enter_critical_section();

  putreg32(RIT_CTRL_INT, LPC43_RIT_CTRL);

  internal_timer += (uint64_t)RIT_TIMER_RESOLUTION;
  if (alarm > 0 && internal_timer >= alarm)
    {
      /* handle expired alarm */

      g_ts.tv_sec = (uint32_t)(internal_timer / 1000000000);
      g_ts.tv_nsec = (uint32_t)(internal_timer % 1000000000);
      nxsched_alarm_expiration(&g_ts);
    }

  leave_critical_section(flags);
  return OK;
}

static inline void lpc43_load_RIT_timer(uint32_t value)
{
  putreg32(value, LPC43_RIT_COUNTER);
}

static inline void lpc43_load_RIT_compare(uint32_t value)
{
  putreg32(value, LPC43_RIT_COMPVAL);
}

static inline void lpc43_set_RIT_timer_mask(uint32_t value)
{
  putreg32(value, LPC43_RIT_MASK);
}

static inline uint32_t lpc43_read_RIT_timer(void)
{
  return getreg32(LPC43_RIT_COUNTER);
}

static inline void lpc43_RIT_timer_start(void)
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

static inline void lpc43_RIT_timer_stop(void)
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

  lpc43_RIT_timer_stop();
  lpc43_load_RIT_timer(0);
  internal_timer = 0;

  /* Set up the IRQ here */

  irq_attach(LPC43M4_IRQ_RITIMER, lpc43_RIT_isr, NULL);

  /* Compute how many seconds per tick we have on the main clock.  If it is
   * 204MHz for example, then there should be about 4.90ns per tick
   */

  sec_per_tick = (double)1.0/(double)LPC43_CCLK;

  /* Given an RIT_TIMER_RESOLUTION, compute how many ticks it will take to
   * reach that resolution.  For example, if we wanted a 1/4uS timer
   * resolution, that would be 250ns resolution.  The timer is an integer
   * value, although maybe this should change, but that means
   * 250/1000000000*0.00000000490 = 51.02 ticks or 51 ticks, roughly.
   * We round up by 1 tick.
   */

  ticks_per_int = RIT_TIMER_RESOLUTION/(1000000000*sec_per_tick)+1;

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

  tmrinfo("mask_bits = %d, mask = %X, ticks_per_int = %d\r\n",
          mask_bits, (0xffffffff << (32 - mask_bits)), ticks_per_int);

  /* Set the mask and compare value so we get interrupts every
   * RIT_TIMER_RESOLUTION cycles.
   */

  lpc43_set_RIT_timer_mask((0xFFFFFFFF << (32 - mask_bits)));
  lpc43_load_RIT_compare(ticks_per_int);

  /* Turn on the IRQ */

  up_enable_irq(LPC43M4_IRQ_RITIMER);

  /* Start the timer */

  lpc43_RIT_timer_start();
}

int up_timer_gettime(FAR struct timespec *ts)
{
  ts->tv_sec = (uint32_t)(internal_timer / 1000000000);
  ts->tv_nsec = (uint32_t)(internal_timer % 1000000000);

  return OK;
}

int up_alarm_cancel(FAR struct timespec *ts)
{
  ts->tv_sec = (uint32_t)(internal_timer / 1000000000);
  ts->tv_nsec = (uint32_t)(internal_timer % 1000000000);
  alarm = 0;
  return OK;
}

int up_alarm_start(FAR const struct timespec *ts)
{
  /* According to the docs, this version should expect to receive the time
   * in the future when the alarm should expire. So that's the way it's
   * coded.
   */

  alarm = (uint64_t)ts->tv_sec * (uint64_t)1000000000 + (uint64_t)ts->tv_nsec;
  return OK;
}

int up_timer_cancel(FAR struct timespec *ts)
{
  /* Currently this is just an alarm and both are implemented.  This is *NOT*
   * how it is supposed to be and will be corrected, but for now, this is a
   * simple way to implement both.
   * FIXME
   */

  return up_alarm_cancel(ts);
}

int up_timer_start(FAR const struct timespec *ts)
{
  /* According to the docs, this version should basically compute the time
   * in the future when an alarm should go off.  That is the way it could
   * potentially be implemented, so that's the way I did it.
   */

  alarm = internal_timer;
  alarm += (uint64_t)ts->tv_sec * (uint64_t)1000000000 + (uint64_t)ts->tv_nsec;
  return OK;
}

#endif /* CONFIG_LPC43_RIT */
