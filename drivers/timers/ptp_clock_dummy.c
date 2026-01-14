/****************************************************************************
 * drivers/timers/ptp_clock_dummy.c
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

#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/timers/ptp_clock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ptp_clock_dummy_s
{
  struct ptp_lowerhalf_s lower;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ptp_clock_dummy_adjfine(FAR struct ptp_lowerhalf_s *lower,
                                   long scaled_ppm);
static int ptp_clock_dummy_adjphase(FAR struct ptp_lowerhalf_s *lower,
                                    int32_t phase);
static int ptp_clock_dummy_adjtime(FAR struct ptp_lowerhalf_s *lower,
                                   int64_t delta);
static int ptp_clock_dummy_gettime(FAR struct ptp_lowerhalf_s *lower,
                                   FAR struct timespec *ts,
                                   FAR struct ptp_system_timestamp *sts);
static int ptp_clock_dummy_getcrosststamp(FAR struct ptp_lowerhalf_s *lower,
                                FAR struct system_device_crosststamp *cts);
static int ptp_clock_dummy_settime(FAR struct ptp_lowerhalf_s *lower,
                                   FAR const struct timespec *ts);
static int ptp_clock_dummy_getres(FAR struct ptp_lowerhalf_s *lower,
                                  FAR struct timespec *res);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct ptp_ops_s g_ptp_clock_dummy_ops =
{
  ptp_clock_dummy_adjfine,        /* adjfine         */
  ptp_clock_dummy_adjphase,       /* adjphase        */
  ptp_clock_dummy_adjtime,        /* adjtime         */
  ptp_clock_dummy_gettime,        /* gettime         */
  ptp_clock_dummy_getcrosststamp, /* getcroasststamp */
  ptp_clock_dummy_settime,        /* settime         */
  ptp_clock_dummy_getres,         /* getres          */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int ptp_clock_dummy_adjfine(FAR struct ptp_lowerhalf_s *lower,
                                   long scaled_ppm)
{
  ptpinfo("ptp_clock_dummy_adjfine ppm:%ld\n", scaled_ppm);
  return 0;
}

static int ptp_clock_dummy_adjphase(FAR struct ptp_lowerhalf_s *lower,
                                    int32_t phase)
{
  ptpinfo("ptp_clock_dummy_adjphase phase:%"PRIi32"\n", phase);
  return 0;
}

static int ptp_clock_dummy_adjtime(FAR struct ptp_lowerhalf_s *lower,
                                   int64_t delta)
{
  ptpinfo("ptp_clock_dummy_adjtime delta:%"PRIi64"\n", delta);
  return 0;
}

static int ptp_clock_dummy_gettime(FAR struct ptp_lowerhalf_s *lower,
                                   FAR struct timespec *ts,
                                   FAR struct ptp_system_timestamp *sts)
{
  clock_gettime(CLOCK_REALTIME, ts);

  if (sts != NULL)
    {
      sts->pre_ts.tv_sec = ts->tv_sec;
      sts->pre_ts.tv_nsec = ts->tv_nsec;
      sts->post_ts.tv_sec = ts->tv_sec;
      sts->post_ts.tv_nsec = ts->tv_nsec;
    }

  ptpinfo("ptp_clock_dummy_gettime sec:%ld, nsec:%ld\n", ts->tv_sec,
          ts->tv_nsec);
  return 0;
}

static int
ptp_clock_dummy_getcrosststamp(FAR struct ptp_lowerhalf_s *lower,
                               FAR struct system_device_crosststamp *cts)
{
  struct timespec ts;

  if (cts == NULL)
    {
      return -EINVAL;
    }

  clock_systime_timespec(&ts);
  cts->device.tv_sec = ts.tv_sec;
  cts->device.tv_nsec = ts.tv_nsec;

  clock_gettime(CLOCK_REALTIME, &ts);
  cts->realtime.tv_sec = ts.tv_sec;
  cts->realtime.tv_nsec = ts.tv_nsec;

  clock_gettime(CLOCK_MONOTONIC, &ts);
  cts->monoraw.tv_sec = ts.tv_sec;
  cts->monoraw.tv_nsec = ts.tv_sec;

  ptpinfo("ptp_clock_dummy_getcrosststamp sec:%ld, nsec:%ld\n",
          (long)ts.tv_sec, ts.tv_nsec);
  return 0;
}

static int ptp_clock_dummy_settime(FAR struct ptp_lowerhalf_s *lower,
                                   FAR const struct timespec *ts)
{
  ptpinfo("ptp_clock_dummy_settime sec:%ld, nsec:%ld\n", (long)ts->tv_sec,
          ts->tv_nsec);
  return 0;
}

static int ptp_clock_dummy_getres(FAR struct ptp_lowerhalf_s *lower,
                                  FAR struct timespec *res)
{
  res->tv_sec = 0;
  res->tv_nsec = 1;
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ptp_clock_dummy_initialize
 *
 * Description:
 *   This function will initialize dummy ptp device driver for test.
 *
 * Input Parameters:
 *   devno - The user specifies number of device. ex: /dev/ptpX.
 *
 * Returned Value:
 *   OK if the driver was successfully initialize; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int ptp_clock_dummy_initialize(int devno)
{
  FAR struct ptp_clock_dummy_s *priv;

  /* Allocate the upper-half data structure */

  priv = kmm_zalloc(sizeof(struct ptp_clock_dummy_s));
  if (!priv)
    {
      ptperr("ERROR: Allocation failed\n");
      return -ENOMEM;
    }

  priv->lower.ops = &g_ptp_clock_dummy_ops;

  return ptp_clock_register(&priv->lower, 1000000, devno);
}
