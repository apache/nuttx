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
 * The clock is built on a capture lower half (struct cap_lowerhalf_s)
 * whose free running 32-bit counter is read with getcount().  The time of
 * the clock is a linear function of the counter:
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
 *
 * The capture channel is the external time stamp (EXTTS) input 0: the
 * capture notification callback reads the counter value latched by the
 * hardware on the edge with getcapture(), converts it to the time of the
 * clock and queues it as a struct ptp_extts_event.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/clock.h>
#include <nuttx/debug.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/lib/math32.h>
#include <nuttx/spinlock.h>
#include <nuttx/wdog.h>
#include <nuttx/timers/capture.h>
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

/* Counter differences at or above this value are before the time base */

#define PTP_PPS_DIFF_NEG        (UINT32_C(1) << 31)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ptp_pps_s
{
  struct ptp_lowerhalf_s      ptp;    /* Must be first */
  FAR struct cap_lowerhalf_s *lower;  /* Counter and capture input */
  spinlock_t                  lock;   /* Protects the time base */
  struct wdog_s               wdog;   /* Periodic re-base */
  clock_t                     period; /* Re-base period, in ticks */
  uint32_t                    freq;   /* Nominal counter frequency (Hz) */
  unsigned int                edges;  /* Edge reported in EXTTS events */

  /* Time base, see the description at the top of the file */

  int64_t                     base_ns;
  uint32_t                    base_cnt;
  uint32_t                    base_frac;
  uint32_t                    mult;
  uint32_t                    mult_nom;
  uint8_t                     shift;
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

static inline void ptp_pps_ns2ts(int64_t ns, FAR struct timespec *ts)
{
  ts->tv_sec  = ns / NSEC_PER_SEC;
  ts->tv_nsec = ns % NSEC_PER_SEC;

  if (ts->tv_nsec < 0)
    {
      ts->tv_nsec += NSEC_PER_SEC;
      ts->tv_sec--;
    }
}

/****************************************************************************
 * Name: ptp_pps_count
 *
 * Description:
 *   Read the free running counter of the lower half.
 *
 ****************************************************************************/

static inline uint32_t ptp_pps_count(FAR struct ptp_pps_s *dev)
{
  uint32_t cnt = 0;

  dev->lower->ops->getcount(dev->lower, &cnt);
  return cnt;
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
  uint32_t diff = cnt - dev->base_cnt;
  uint64_t prod;
  int64_t ns;

  if (diff < PTP_PPS_DIFF_NEG)
    {
      prod = diff;
      prod = prod * dev->mult + dev->base_frac;
      ns   = prod >> dev->shift;
      return dev->base_ns + ns;
    }

  /* The counter value is before the time base, round toward -infinity */

  diff = dev->base_cnt - cnt;
  prod = diff;
  prod = prod * dev->mult;
  if (prod <= dev->base_frac)
    {
      return dev->base_ns;
    }

  prod -= dev->base_frac;
  ns    = (prod + (UINT64_C(1) << dev->shift) - 1) >> dev->shift;
  return dev->base_ns - ns;
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
  uint32_t cnt = ptp_pps_count(dev);
  uint32_t diff = cnt - dev->base_cnt;
  uint64_t prod;
  int64_t ns;

  /* The counter only moves forward and the watchdog keeps the difference
   * below 2^31, so the unsigned difference is the elapsed counter periods.
   */

  if (diff == 0)
    {
      return;
    }

  prod           = diff;
  prod           = prod * dev->mult + dev->base_frac;
  ns             = prod >> dev->shift;
  dev->base_ns  += ns;
  dev->base_frac = prod & ((UINT64_C(1) << dev->shift) - 1);
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
 * Name: ptp_pps_notify
 *
 * Description:
 *   Capture notification, called by the lower half on each captured edge,
 *   normally from its interrupt handler.  The counter value latched by
 *   the hardware is converted to the time of the clock and queued as an
 *   EXTTS event.
 *
 ****************************************************************************/

static void ptp_pps_notify(FAR struct cap_lowerhalf_s *lower,
                           FAR void *priv)
{
  FAR struct ptp_pps_s *dev = priv;
  struct ptp_clock_event event;
  irqstate_t flags;
  uint32_t cnt;
  int64_t ns;

  if (lower->ops->getcapture(lower, &cnt) < 0)
    {
      return;
    }

  flags = spin_lock_irqsave(&dev->lock);
  ns = ptp_pps_cnt2ns(dev, cnt);
  spin_unlock_irqrestore(&dev->lock, flags);

  event.type  = PTP_CLK_REQ_EXTTS;
  event.index = 0;
  event.flags = dev->edges;
  ptp_pps_ns2ts(ns, &event.ts);

  ptp_clock_event(&dev->ptp, &event);
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
  irqstate_t flags;
  int64_t mult;

  mult  = dev->mult_nom;
  mult += mult * ppb / NSEC_PER_SEC;
  if (mult <= 0 || mult > UINT32_MAX)
    {
      return -ERANGE;
    }

  flags = spin_lock_irqsave(&dev->lock);
  ptp_pps_rebase(dev);
  dev->mult = mult;
  spin_unlock_irqrestore(&dev->lock, flags);

  ptpinfo("adjfine %ld ppb, mult %" PRId64 "\n", ppb, mult);
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

  ns = ptp_pps_cnt2ns(dev, ptp_pps_count(dev));

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
  ns = ptp_pps_cnt2ns(dev, ptp_pps_count(dev));
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
  int64_t ns;

  if (ts->tv_nsec < 0 || ts->tv_nsec >= NSEC_PER_SEC)
    {
      return -EINVAL;
    }

  /* tv_sec may be 32-bit, do the multiplication in 64-bit */

  ns = ts->tv_sec;
  ns = ns * NSEC_PER_SEC + ts->tv_nsec;

  flags = spin_lock_irqsave(&dev->lock);
  ptp_pps_rebase(dev);
  dev->base_ns   = ns;
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

  res->tv_sec  = 0;
  res->tv_nsec = div_round_up(NSEC_PER_SEC, dev->freq);
  return OK;
}

/****************************************************************************
 * Name: ptp_pps_enable
 *
 * Description:
 *   Enable or disable the external time stamp input 0, the capture channel
 *   of the lower half.
 *
 ****************************************************************************/

static int ptp_pps_enable(FAR struct ptp_lowerhalf_s *lower,
                          FAR struct ptp_clock_request *rq, bool on)
{
  FAR struct ptp_pps_s *dev = (FAR struct ptp_pps_s *)lower;
  FAR struct cap_lowerhalf_s *cap = dev->lower;
  enum cap_type_e type;

  if (rq->type != PTP_CLK_REQ_EXTTS)
    {
      return -EOPNOTSUPP;
    }

  if (rq->extts.index != 0)
    {
      return -EINVAL;
    }

  if (!on)
    {
      return cap->ops->unbind(cap);
    }

  switch (rq->extts.flags & PTP_EXTTS_EDGES)
    {
      case PTP_EXTTS_EDGES:
        type       = CAP_TYPE_BOTH;
        dev->edges = 0;
        break;

      case PTP_FALLING_EDGE:
        type       = CAP_TYPE_FALLING;
        dev->edges = PTP_FALLING_EDGE;
        break;

      default:
        type       = CAP_TYPE_RISING;
        dev->edges = PTP_RISING_EDGE;
        break;
    }

  return cap->ops->bind(cap, type, ptp_pps_notify, dev);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ptp_pps_register
 *
 * Description:
 *   Create a PPS disciplined PTP clock on top of a capture lower half and
 *   register it as /dev/ptp<devno>.  See include/nuttx/timers/ptp_pps.h.
 *
 ****************************************************************************/

int ptp_pps_register(FAR struct cap_lowerhalf_s *lower, uint32_t freq,
                     int devno)
{
  FAR const struct cap_ops_s *ops;
  FAR struct ptp_pps_s *dev;
  uint64_t mult = 0;
  uint64_t period;
  int shift;
  int ret;

  DEBUGASSERT(lower != NULL && lower->ops != NULL);

  ops = lower->ops;
  if (freq == 0 || ops->start == NULL || ops->getcount == NULL ||
      ops->getcapture == NULL || ops->bind == NULL || ops->unbind == NULL)
    {
      return -EINVAL;
    }

  /* Find the largest shift that keeps mult below 2^31 */

  for (shift = 31; shift > 0; shift--)
    {
      mult = NSEC_PER_SEC;
      mult = ((mult << shift) + freq / 2) / freq;
      if (mult <= PTP_PPS_MULT_MAX)
        {
          break;
        }
    }

  if (mult == 0 || mult > PTP_PPS_MULT_MAX)
    {
      return -EINVAL;
    }

  dev = kmm_zalloc(sizeof(struct ptp_pps_s));
  if (dev == NULL)
    {
      ptperr("ERROR: Allocation failed\n");
      return -ENOMEM;
    }

  /* Re-base at least every 2^30 counter periods */

  period = PTP_PPS_REBASE_COUNTS;
  period = period * TICK_PER_SEC / freq;

  dev->lower        = lower;
  dev->freq         = freq;
  dev->shift        = shift;
  dev->mult_nom     = mult;
  dev->mult         = mult;
  dev->period       = period > 0 ? period : 1;
  dev->ptp.ops      = &g_ptp_pps_ops;
  dev->ptp.n_ext_ts = 1;
  spin_lock_init(&dev->lock);

  /* Start the counter */

  ret = ops->start(lower);
  if (ret < 0)
    {
      ptperr("ERROR: Failed to start the capture: %d\n", ret);
      kmm_free(dev);
      return ret;
    }

  dev->base_cnt = ptp_pps_count(dev);

  ptpinfo("freq %" PRIu32 " Hz, shift %d, mult %" PRIu32 "\n",
          freq, shift, dev->mult);

  ret = ptp_clock_register(&dev->ptp, CONFIG_PTP_PPS_MAX_ADJ, devno);
  if (ret < 0)
    {
      ptperr("ERROR: ptp_clock_register failed: %d\n", ret);
      ops->stop(lower);
      kmm_free(dev);
      return ret;
    }

  wd_start(&dev->wdog, dev->period, ptp_pps_wdog, (wdparm_t)dev);
  return OK;
}
