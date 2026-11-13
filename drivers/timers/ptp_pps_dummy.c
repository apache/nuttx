/****************************************************************************
 * drivers/timers/ptp_pps_dummy.c
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

/* Simulated counter and PPS input for the PPS disciplined PTP clock.
 *
 * up_perf_gettime() is used as the "true" time.  The simulated counter
 * runs CONFIG_PTP_PPS_DUMMY_ERROR_PPB faster than its nominal frequency,
 * like a real crystal would, and a simulated PPS edge is produced at every
 * whole second of the true time.  As with a hardware input capture, the
 * counter value is latched exactly at the edge (plus a configurable
 * jitter) and reported later from a watchdog, so the latency of the
 * watchdog does not affect the time stamps.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/debug.h>
#include <nuttx/wdog.h>
#include <nuttx/timers/ptp_clock.h>
#include <nuttx/timers/ptp_pps.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Report the simulated edge this long after the true second */

#define PTP_PPS_DUMMY_REPORT_NS  (2 * NSEC_PER_MSEC)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ptp_pps_dummy_s
{
  struct ptp_pps_lowerhalf_s lower; /* Must be first */
  struct wdog_s              wdog;  /* Simulated PPS */
  int64_t                    whole; /* Actual frequency, integer Hz */
  int64_t                    milli; /* Actual frequency, mHz remainder */
  time_t                     last;  /* Second of the last edge */
  uint32_t                   rand;  /* Jitter pseudo random state */
  bool                       enabled;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint32_t ptp_pps_dummy_read(FAR struct ptp_pps_lowerhalf_s *lower);
static int ptp_pps_dummy_enable(FAR struct ptp_pps_lowerhalf_s *lower,
                                unsigned int index, unsigned int flags,
                                bool on);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct ptp_pps_ops_s g_ptp_pps_dummy_ops =
{
  ptp_pps_dummy_read,   /* read */
  ptp_pps_dummy_enable, /* enable */
};

static struct ptp_pps_dummy_s g_ptp_pps_dummy;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ptp_pps_dummy_truetime
 ****************************************************************************/

static void ptp_pps_dummy_truetime(FAR struct timespec *ts)
{
  up_perf_convert(up_perf_gettime(), ts);
}

/****************************************************************************
 * Name: ptp_pps_dummy_count
 *
 * Description:
 *   The value of the simulated counter at a given true time.
 *
 ****************************************************************************/

static int64_t ptp_pps_dummy_count(FAR struct ptp_pps_dummy_s *priv,
                                   time_t sec, long nsec)
{
  return (int64_t)sec * priv->whole +
         (int64_t)sec * priv->milli / 1000 +
         (int64_t)nsec * priv->whole / NSEC_PER_SEC;
}

/****************************************************************************
 * Name: ptp_pps_dummy_jitter
 *
 * Description:
 *   Uniform jitter in [-JITTER, +JITTER] ns, converted to counts.
 *
 ****************************************************************************/

static int64_t ptp_pps_dummy_jitter(FAR struct ptp_pps_dummy_s *priv)
{
#if CONFIG_PTP_PPS_DUMMY_JITTER_NS > 0
  int64_t ns;

  priv->rand = priv->rand * 1664525u + 1013904223u;
  ns = (int64_t)(priv->rand >> 8) % (2 * CONFIG_PTP_PPS_DUMMY_JITTER_NS + 1)
       - CONFIG_PTP_PPS_DUMMY_JITTER_NS;
  return ns * priv->whole / NSEC_PER_SEC;
#else
  UNUSED(priv);
  return 0;
#endif
}

/****************************************************************************
 * Name: ptp_pps_dummy_read
 ****************************************************************************/

static uint32_t ptp_pps_dummy_read(FAR struct ptp_pps_lowerhalf_s *lower)
{
  FAR struct ptp_pps_dummy_s *priv = (FAR struct ptp_pps_dummy_s *)lower;
  struct timespec ts;

  ptp_pps_dummy_truetime(&ts);
  return (uint32_t)ptp_pps_dummy_count(priv, ts.tv_sec, ts.tv_nsec);
}

/****************************************************************************
 * Name: ptp_pps_dummy_enable
 ****************************************************************************/

static int ptp_pps_dummy_enable(FAR struct ptp_pps_lowerhalf_s *lower,
                                unsigned int index, unsigned int flags,
                                bool on)
{
  FAR struct ptp_pps_dummy_s *priv = (FAR struct ptp_pps_dummy_s *)lower;

  if (index != 0)
    {
      return -EINVAL;
    }

  if (on && (flags & PTP_FALLING_EDGE) != 0)
    {
      return -EOPNOTSUPP;
    }

  priv->enabled = on;
  return OK;
}

/****************************************************************************
 * Name: ptp_pps_dummy_wdog
 *
 * Description:
 *   Called a little after each true second: report the counter value
 *   latched at the second.
 *
 ****************************************************************************/

static void ptp_pps_dummy_wdog(wdparm_t arg)
{
  FAR struct ptp_pps_dummy_s *priv = (FAR struct ptp_pps_dummy_s *)arg;
  struct timespec ts;
  int64_t delay;

  ptp_pps_dummy_truetime(&ts);

  if (ts.tv_sec != priv->last)
    {
      priv->last = ts.tv_sec;
      if (priv->enabled)
        {
          int64_t cnt = ptp_pps_dummy_count(priv, ts.tv_sec, 0) +
                        ptp_pps_dummy_jitter(priv);

          ptp_pps_capture(&priv->lower, 0, PTP_RISING_EDGE, (uint32_t)cnt);
        }
    }

  /* Wake up again just after the next true second */

  delay = NSEC_PER_SEC - ts.tv_nsec + PTP_PPS_DUMMY_REPORT_NS;
  wd_start(&priv->wdog, (clock_t)((delay + NSEC_PER_TICK - 1) /
                                  NSEC_PER_TICK),
           ptp_pps_dummy_wdog, arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ptp_pps_dummy_initialize
 *
 * Description:
 *   Register a PPS disciplined clock with a simulated counter and a
 *   simulated PPS input, for testing.
 *
 ****************************************************************************/

int ptp_pps_dummy_initialize(int devno)
{
  FAR struct ptp_pps_dummy_s *priv = &g_ptp_pps_dummy;
  int64_t freq_mhz;
  struct timespec ts;
  int ret;

  /* Actual frequency of the simulated oscillator, in mHz */

  freq_mhz = (int64_t)CONFIG_PTP_PPS_DUMMY_FREQ * 1000 +
             (int64_t)CONFIG_PTP_PPS_DUMMY_FREQ *
             CONFIG_PTP_PPS_DUMMY_ERROR_PPB / 1000000;

  priv->whole          = freq_mhz / 1000;
  priv->milli          = freq_mhz % 1000;
  priv->rand           = 0x2545f491;
  priv->lower.ops      = &g_ptp_pps_dummy_ops;
  priv->lower.freq     = CONFIG_PTP_PPS_DUMMY_FREQ;
  priv->lower.n_ext_ts = 1;

  ret = ptp_pps_register(&priv->lower, devno);
  if (ret < 0)
    {
      return ret;
    }

  ptp_pps_dummy_truetime(&ts);
  priv->last = ts.tv_sec;
  ptp_pps_dummy_wdog((wdparm_t)priv);
  return OK;
}
