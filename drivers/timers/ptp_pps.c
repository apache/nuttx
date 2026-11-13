/****************************************************************************
 * drivers/timers/ptp_pps.c
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

/* PPS disciplined software PTP clock.
 *
 * The time of the clock is a linear function of a free running 32-bit
 * hardware counter:
 *
 *   t(cnt) = base_ns + ((cnt - base_cnt) * mult + base_frac) >> shift
 *
 * mult is the length of one counter period in units of 2^-shift ns.  shift
 * is chosen so that mult is just below 2^31, which gives a frequency
 * resolution of better than 1 ppb for any counter frequency and keeps the
 * 32x32 bit product within 64 bits.  The remainder of the conversion is
 * kept in base_frac, so no time is lost when the clock is re-based.
 *
 * Frequency changes re-base the clock first, so they are phase
 * continuous.  A watchdog re-bases the clock periodically so that the
 * counter difference never exceeds 2^30, whatever the counter frequency.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/clock.h>
#include <nuttx/debug.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spinlock.h>
#include <nuttx/wdog.h>
#include <nuttx/timers/ptp_clock.h>
#include <nuttx/timers/ptp_pps.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The clock is re-based at least every 2^30 counter periods, a quarter of
 * the wrap around period of the counter.
 */

#define PTP_PPS_REBASE_COUNTS   (UINT32_C(1) << 30)

/* Largest mult, leaves room for any frequency adjustment up to 100% */

#define PTP_PPS_MULT_MAX        UINT32_C(0x7fffffff)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ptp_pps_s
{
  struct ptp_lowerhalf_s           ptp;    /* Must be first */
  FAR struct ptp_pps_lowerhalf_s  *lower;  /* Counter and capture inputs */
  spinlock_t                       lock;   /* Protects the time base */
  struct wdog_s                    wdog;   /* Periodic re-base */
  clock_t                          period; /* Re-base period, in ticks */

  /* Time base, see the description at the top of the file */

  int64_t                          base_ns;
  uint32_t                         base_cnt;
  uint32_t                         base_frac;
  uint32_t                         mult;
  uint32_t                         mult_nom;
  uint8_t                          shift;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ptp_pps_adjfine(FAR struct ptp_lowerhalf_s *lower, long ppb);
static int ptp_pps_adjtime(FAR struct ptp_lowerhalf_s *lower,
                           int64_t delta);
static int ptp_pps_gettime(FAR struct ptp_lowerhalf_s *lower,
                           FAR struct timespec *ts,
                           FAR struct ptp_system_timestamp *sts);
static int ptp_pps_getcrosststamp(FAR struct ptp_lowerhalf_s *lower,
                                  FAR struct system_device_crosststamp *cts);
static int ptp_pps_settime(FAR struct ptp_lowerhalf_s *lower,
                           FAR const struct timespec *ts);
static int ptp_pps_getres(FAR struct ptp_lowerhalf_s *lower,
                          FAR struct timespec *res);
static int ptp_pps_enable(FAR struct ptp_lowerhalf_s *lower,
                          FAR struct ptp_clock_request *rq, bool on);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct ptp_ops_s g_ptp_pps_ops =
{
  ptp_pps_adjfine,        /* adjfine */
  NULL,                   /* adjphase */
  ptp_pps_adjtime,        /* adjtime */
  ptp_pps_gettime,        /* gettime */
  ptp_pps_getcrosststamp, /* getcrosststamp */
  ptp_pps_settime,        /* settime */
  ptp_pps_getres,         /* getres */
  NULL,                   /* control */
  ptp_pps_enable,         /* enable */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ptp_pps_ns2ts
 ****************************************************************************/

static void ptp_pps_ns2ts(int64_t ns, FAR struct timespec *ts)
{
  int64_t sec = ns / NSEC_PER_SEC;
  long nsec = (long)(ns - sec * NSEC_PER_SEC);

  if (nsec < 0)
    {
      nsec += NSEC_PER_SEC;
      sec--;
    }

  ts->tv_sec  = (time_t)sec;
  ts->tv_nsec = nsec;
}

/****************************************************************************
 * Name: ptp_pps_cnt2ns
 *
 * Description:
 *   Convert a counter value to the time of the clock.  The counter value
 *   may be slightly older than the time base (a capture that is reported
 *   after a re-base), so the difference is signed.
 *
 *   Must be called with the lock held.
 *
 ****************************************************************************/

static int64_t ptp_pps_cnt2ns(FAR struct ptp_pps_s *dev, uint32_t cnt)
{
  int32_t diff = (int32_t)(cnt - dev->base_cnt);
  uint64_t prod;

  if (diff >= 0)
    {
      prod = (uint64_t)(uint32_t)diff * dev->mult + dev->base_frac;
      return dev->base_ns + (int64_t)(prod >> dev->shift);
    }

  /* The counter value is before the time base, round toward -infinity */

  prod = (uint64_t)(uint32_t)(-(int64_t)diff) * dev->mult;
  if (prod <= dev->base_frac)
    {
      return dev->base_ns;
    }

  prod -= dev->base_frac;
  return dev->base_ns -
         (int64_t)((prod + (UINT64_C(1) << dev->shift) - 1) >> dev->shift);
}

/****************************************************************************
 * Name: ptp_pps_rebase
 *
 * Description:
 *   Move the time base to the current counter value.  Must be called with
 *   the lock held.
 *
 ****************************************************************************/

static void ptp_pps_rebase(FAR struct ptp_pps_s *dev)
{
  uint32_t cnt = dev->lower->ops->read(dev->lower);
  uint32_t diff = cnt - dev->base_cnt;
  uint64_t prod;

  /* The counter only moves forward and the watchdog keeps the difference
   * below 2^31, so the unsigned difference is the elapsed counter periods.
   */

  if (diff == 0)
    {
      return;
    }

  prod           = (uint64_t)diff * dev->mult + dev->base_frac;
  dev->base_ns  += (int64_t)(prod >> dev->shift);
  dev->base_frac = (uint32_t)(prod & ((UINT64_C(1) << dev->shift) - 1));
  dev->base_cnt  = cnt;
}

/****************************************************************************
 * Name: ptp_pps_wdog
 *
 * Description:
 *   Re-base the clock periodically so that the counter difference used in
 *   the conversion never overflows, even if no PPS is received.
 *
 ****************************************************************************/

static void ptp_pps_wdog(wdparm_t arg)
{
  FAR struct ptp_pps_s *dev = (FAR struct ptp_pps_s *)arg;
  irqstate_t flags;

  flags = spin_lock_irqsave(&dev->lock);
  ptp_pps_rebase(dev);
  spin_unlock_irqrestore(&dev->lock, flags);

  wd_start(&dev->wdog, dev->period, ptp_pps_wdog, arg);
}

/****************************************************************************
 * Name: ptp_pps_adjfine
 *
 * Description:
 *   Set the frequency offset of the clock, in ppb, relative to the nominal
 *   counter frequency.  Positive values make the clock run faster.
 *
 ****************************************************************************/

static int ptp_pps_adjfine(FAR struct ptp_lowerhalf_s *lower, long ppb)
{
  FAR struct ptp_pps_s *dev = (FAR struct ptp_pps_s *)lower;
  int64_t mult;
  irqstate_t flags;

  mult = (int64_t)dev->mult_nom +
         ((int64_t)dev->mult_nom * ppb) / NSEC_PER_SEC;
  if (mult <= 0 || mult > UINT32_MAX)
    {
      return -ERANGE;
    }

  flags = spin_lock_irqsave(&dev->lock);
  ptp_pps_rebase(dev);
  dev->mult = (uint32_t)mult;
  spin_unlock_irqrestore(&dev->lock, flags);

  ptpinfo("adjfine %ld ppb, mult %" PRIu32 "\n", ppb, (uint32_t)mult);
  return OK;
}

/****************************************************************************
 * Name: ptp_pps_adjtime
 ****************************************************************************/

static int ptp_pps_adjtime(FAR struct ptp_lowerhalf_s *lower, int64_t delta)
{
  FAR struct ptp_pps_s *dev = (FAR struct ptp_pps_s *)lower;
  irqstate_t flags;

  flags = spin_lock_irqsave(&dev->lock);
  dev->base_ns += delta;
  spin_unlock_irqrestore(&dev->lock, flags);

  ptpinfo("adjtime %" PRId64 " ns\n", delta);
  return OK;
}

/****************************************************************************
 * Name: ptp_pps_gettime
 ****************************************************************************/

static int ptp_pps_gettime(FAR struct ptp_lowerhalf_s *lower,
                           FAR struct timespec *ts,
                           FAR struct ptp_system_timestamp *sts)
{
  FAR struct ptp_pps_s *dev = (FAR struct ptp_pps_s *)lower;
  irqstate_t flags;
  int64_t ns;

  flags = spin_lock_irqsave(&dev->lock);

  if (sts != NULL)
    {
      nxclock_gettime(CLOCK_REALTIME, &sts->pre_ts);
    }

  ns = ptp_pps_cnt2ns(dev, dev->lower->ops->read(dev->lower));

  if (sts != NULL)
    {
      nxclock_gettime(CLOCK_REALTIME, &sts->post_ts);
    }

  spin_unlock_irqrestore(&dev->lock, flags);

  ptp_pps_ns2ts(ns, ts);
  return OK;
}

/****************************************************************************
 * Name: ptp_pps_getcrosststamp
 *
 * Description:
 *   Read the clock and the system clocks with interrupts disabled, as
 *   close together as possible.
 *
 ****************************************************************************/

static int ptp_pps_getcrosststamp(FAR struct ptp_lowerhalf_s *lower,
                                  FAR struct system_device_crosststamp *cts)
{
  FAR struct ptp_pps_s *dev = (FAR struct ptp_pps_s *)lower;
  irqstate_t flags;
  int64_t ns;

  flags = spin_lock_irqsave(&dev->lock);
  ns = ptp_pps_cnt2ns(dev, dev->lower->ops->read(dev->lower));
  nxclock_gettime(CLOCK_REALTIME, &cts->realtime);
  nxclock_gettime(CLOCK_MONOTONIC, &cts->monoraw);
  spin_unlock_irqrestore(&dev->lock, flags);

  ptp_pps_ns2ts(ns, &cts->device);
  return OK;
}

/****************************************************************************
 * Name: ptp_pps_settime
 ****************************************************************************/

static int ptp_pps_settime(FAR struct ptp_lowerhalf_s *lower,
                           FAR const struct timespec *ts)
{
  FAR struct ptp_pps_s *dev = (FAR struct ptp_pps_s *)lower;
  irqstate_t flags;

  if (ts->tv_nsec < 0 || ts->tv_nsec >= NSEC_PER_SEC)
    {
      return -EINVAL;
    }

  flags = spin_lock_irqsave(&dev->lock);
  ptp_pps_rebase(dev);
  dev->base_ns   = (int64_t)ts->tv_sec * NSEC_PER_SEC + ts->tv_nsec;
  dev->base_frac = 0;
  spin_unlock_irqrestore(&dev->lock, flags);

  return OK;
}

/****************************************************************************
 * Name: ptp_pps_getres
 ****************************************************************************/

static int ptp_pps_getres(FAR struct ptp_lowerhalf_s *lower,
                          FAR struct timespec *res)
{
  FAR struct ptp_pps_s *dev = (FAR struct ptp_pps_s *)lower;
  long nsec = (long)((NSEC_PER_SEC + dev->lower->freq - 1) /
                     dev->lower->freq);

  res->tv_sec  = 0;
  res->tv_nsec = nsec > 0 ? nsec : 1;
  return OK;
}

/****************************************************************************
 * Name: ptp_pps_enable
 ****************************************************************************/

static int ptp_pps_enable(FAR struct ptp_lowerhalf_s *lower,
                          FAR struct ptp_clock_request *rq, bool on)
{
  FAR struct ptp_pps_s *dev = (FAR struct ptp_pps_s *)lower;

  if (rq->type != PTP_CLK_REQ_EXTTS || dev->lower->ops->enable == NULL)
    {
      return -EOPNOTSUPP;
    }

  return dev->lower->ops->enable(dev->lower, rq->extts.index,
                                 rq->extts.flags & PTP_EXTTS_EDGES, on);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ptp_pps_capture
 *
 * Description:
 *   Report a counter value latched by a capture input.  See
 *   include/nuttx/timers/ptp_pps.h.
 *
 ****************************************************************************/

void ptp_pps_capture(FAR struct ptp_pps_lowerhalf_s *lower,
                     unsigned int index, unsigned int flags,
                     uint32_t count)
{
  FAR struct ptp_pps_s *dev = lower->priv;
  struct ptp_clock_event event;
  irqstate_t irqflags;
  int64_t ns;

  if (dev == NULL)
    {
      return;
    }

  irqflags = spin_lock_irqsave(&dev->lock);
  ns = ptp_pps_cnt2ns(dev, count);
  spin_unlock_irqrestore(&dev->lock, irqflags);

  event.type  = PTP_CLK_REQ_EXTTS;
  event.index = index;
  event.flags = flags;
  ptp_pps_ns2ts(ns, &event.ts);

  ptp_clock_event(&dev->ptp, &event);
}

/****************************************************************************
 * Name: ptp_pps_register
 *
 * Description:
 *   Create a PTP clock on top of the counter of the lower half and register
 *   it as /dev/ptp<devno>.  See include/nuttx/timers/ptp_pps.h.
 *
 ****************************************************************************/

int ptp_pps_register(FAR struct ptp_pps_lowerhalf_s *lower, int devno)
{
  FAR struct ptp_pps_s *dev;
  uint64_t mult = 0;
  uint64_t period;
  int shift;
  int ret;

  DEBUGASSERT(lower != NULL && lower->ops != NULL &&
              lower->ops->read != NULL && lower->freq > 0);

  dev = kmm_zalloc(sizeof(struct ptp_pps_s));
  if (dev == NULL)
    {
      ptperr("ERROR: Allocation failed\n");
      return -ENOMEM;
    }

  /* Find the largest shift that keeps mult below 2^31 */

  for (shift = 31; shift > 0; shift--)
    {
      mult = (((uint64_t)NSEC_PER_SEC << shift) + lower->freq / 2) /
             lower->freq;
      if (mult <= PTP_PPS_MULT_MAX)
        {
          break;
        }
    }

  if (mult == 0 || mult > PTP_PPS_MULT_MAX)
    {
      kmm_free(dev);
      return -EINVAL;
    }

  /* Re-base at least every 2^30 counter periods */

  period = (uint64_t)PTP_PPS_REBASE_COUNTS * TICK_PER_SEC / lower->freq;
  if (period < 1)
    {
      period = 1;
    }

  dev->lower        = lower;
  dev->shift        = shift;
  dev->mult_nom     = (uint32_t)mult;
  dev->mult         = (uint32_t)mult;
  dev->period       = (clock_t)period;
  dev->base_cnt     = lower->ops->read(lower);
  dev->ptp.ops      = &g_ptp_pps_ops;
  dev->ptp.n_ext_ts = lower->n_ext_ts;
  spin_lock_init(&dev->lock);

  ptpinfo("freq %" PRIu32 " Hz, shift %d, mult %" PRIu32 "\n",
          lower->freq, shift, dev->mult);

  ret = ptp_clock_register(&dev->ptp, CONFIG_PTP_PPS_MAX_ADJ, devno);
  if (ret < 0)
    {
      ptperr("ERROR: ptp_clock_register failed: %d\n", ret);
      kmm_free(dev);
      return ret;
    }

  lower->priv = dev;
  wd_start(&dev->wdog, dev->period, ptp_pps_wdog, (wdparm_t)dev);
  return OK;
}
