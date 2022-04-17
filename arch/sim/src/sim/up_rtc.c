/****************************************************************************
 * arch/sim/src/sim/up_rtc.c
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

#include <nuttx/arch.h>
#include <nuttx/timers/rtc.h>
#include <nuttx/timers/arch_rtc.h>
#include <nuttx/timers/rpmsg_rtc.h>

#include "up_internal.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int sim_rtc_rdtime(struct rtc_lowerhalf_s *lower,
                          struct rtc_time *rtctime);
static int sim_rtc_settime(struct rtc_lowerhalf_s *lower,
                           const struct rtc_time *rtctime);
static bool sim_rtc_havesettime(struct rtc_lowerhalf_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rtc_ops_s g_sim_rtc_ops =
{
  .rdtime      = sim_rtc_rdtime,
  .settime     = sim_rtc_settime,
  .havesettime = sim_rtc_havesettime,
};

static struct rtc_lowerhalf_s g_sim_rtc =
{
  .ops         = &g_sim_rtc_ops,
};

static int64_t g_sim_delta;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int sim_rtc_rdtime(struct rtc_lowerhalf_s *lower,
                          struct rtc_time *rtctime)
{
  uint64_t nsec;
  time_t sec;

  nsec  = host_gettime(true);
  nsec += g_sim_delta;
  sec   = nsec / NSEC_PER_SEC;
  nsec -= sec * NSEC_PER_SEC;

  gmtime_r(&sec, (struct tm *)rtctime);
  rtctime->tm_nsec = nsec;

  return OK;
}

static int sim_rtc_settime(struct rtc_lowerhalf_s *lower,
                           const struct rtc_time *rtctime)
{
  g_sim_delta = timegm((struct tm *)rtctime);
  g_sim_delta *= NSEC_PER_SEC;
  g_sim_delta += rtctime->tm_nsec;
  g_sim_delta -= host_gettime(true);

  return OK;
}

static bool sim_rtc_havesettime(struct rtc_lowerhalf_s *lower)
{
  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Initialize the builtin, MCU hardware RTC per the selected
 *   configuration.  This function is called once very early in the OS
 *   initialization sequence.
 *
 *   NOTE that initialization of external RTC hardware that depends on the
 *   availability of OS resources (such as SPI or I2C) must be deferred
 *   until the system has fully booted.  Other, RTC-specific initialization
 *   functions are used in that case.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_initialize(void)
{
  struct rtc_lowerhalf_s *rtc = &g_sim_rtc;
  bool sync = true;

#ifdef CONFIG_RTC_RPMSG_SERVER
  rtc = rpmsg_rtc_server_initialize(rtc);
#elif defined(CONFIG_RTC_RPMSG)
  rtc = rpmsg_rtc_initialize();
  sync = false;
#endif
  up_rtc_set_lowerhalf(rtc, sync);
  return rtc_initialize(0, rtc);
}
