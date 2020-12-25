/****************************************************************************
 * drivers/timers/rpmsg_rtc.c
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

#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/rptun/openamp.h>
#include <nuttx/semaphore.h>
#include <nuttx/timers/rpmsg_rtc.h>

#include <errno.h>
#include <string.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define RPMSG_RTC_EPT_NAME          "rpmsg-rtc"

#define RPMSG_RTC_SET               0
#define RPMSG_RTC_GET               1
#define RPMSG_RTC_ALARM_SET         2
#define RPMSG_RTC_ALARM_CANCEL      3
#define RPMSG_RTC_ALARM_FIRE        4

/****************************************************************************
 * Private Types
 ****************************************************************************/

begin_packed_struct struct rpmsg_rtc_header_s
{
  uint32_t command;
  int32_t  result;
  uint64_t cookie;
} end_packed_struct;

begin_packed_struct struct rpmsg_rtc_set_s
{
  struct rpmsg_rtc_header_s header;
  int64_t                   sec;
  int32_t                   nsec;
} end_packed_struct;

#define rpmsg_rtc_get_s rpmsg_rtc_set_s

begin_packed_struct struct rpmsg_rtc_alarm_set_s
{
  struct rpmsg_rtc_header_s header;
  int64_t                   sec;
  int32_t                   nsec;
  int32_t                   id;
} end_packed_struct;

begin_packed_struct struct rpmsg_rtc_alarm_cancel_s
{
  struct rpmsg_rtc_header_s header;
  int32_t                   id;
} end_packed_struct;

#define rpmsg_rtc_alarm_fire_s rpmsg_rtc_alarm_cancel_s

struct rpmsg_rtc_cookie_s
{
  FAR struct rpmsg_rtc_header_s *msg;
  sem_t                         sem;
};

/* This is the private type for the RTC state. It must be cast compatible
 * with struct rtc_lowerhalf_s.
 */

struct rpmsg_rtc_lowerhalf_s
{
  /* This is the contained reference to the read-only, lower-half
   * operations vtable (which may lie in FLASH or ROM)
   */

  FAR const struct rtc_ops_s *ops;

  /* Data following is private to this driver and not visible outside of
   * this file.
   */

  struct rpmsg_endpoint      ept;
  FAR const char             *cpuname;

#ifdef CONFIG_RTC_ALARM
  struct lower_setalarm_s    alarminfo[CONFIG_RTC_NALARMS];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void rpmsg_rtc_device_created(FAR struct rpmsg_device *rdev,
              FAR void *priv);
static void rpmsg_rtc_device_destroy(FAR struct rpmsg_device *rdev,
              FAR void *priv);
static void rpmsg_rtc_alarm_fire_handler(FAR struct rpmsg_endpoint *ept,
              FAR void *data, size_t len, uint32_t src, FAR void *priv);
static int rpmsg_rtc_ept_cb(FAR struct rpmsg_endpoint *ept, FAR void *data,
              size_t len, uint32_t src, FAR void *priv);

static int rpmsg_rtc_send_recv(FAR struct rpmsg_rtc_lowerhalf_s *lower,
              uint32_t command, FAR struct rpmsg_rtc_header_s *msg, int len);
static int rpmsg_rtc_rdtime(FAR struct rtc_lowerhalf_s *lower,
              FAR struct rtc_time *rtctime);
static int rpmsg_rtc_settime(FAR struct rtc_lowerhalf_s *lower,
              FAR const struct rtc_time *rtctime);
static bool rpmsg_rtc_havesettime(FAR struct rtc_lowerhalf_s *lower);
#ifdef CONFIG_RTC_ALARM
static int rpmsg_rtc_setalarm(FAR struct rtc_lowerhalf_s *lower_,
              FAR const struct lower_setalarm_s *alarminfo);
static int rpmsg_rtc_setrelative(FAR struct rtc_lowerhalf_s *lower,
              FAR const struct lower_setrelative_s *relinfo);
static int rpmsg_rtc_cancelalarm(FAR struct rtc_lowerhalf_s *lower,
              int alarmid);
static int rpmsg_rtc_rdalarm(FAR struct rtc_lowerhalf_s *lower_,
              FAR struct lower_rdalarm_s *alarminfo);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rtc_ops_s g_rpmsg_rtc_ops =
{
  .rdtime      = rpmsg_rtc_rdtime,
  .settime     = rpmsg_rtc_settime,
  .havesettime = rpmsg_rtc_havesettime,
#ifdef CONFIG_RTC_ALARM
  .setalarm    = rpmsg_rtc_setalarm,
  .setrelative = rpmsg_rtc_setrelative,
  .cancelalarm = rpmsg_rtc_cancelalarm,
  .rdalarm     = rpmsg_rtc_rdalarm,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void rpmsg_rtc_device_created(FAR struct rpmsg_device *rdev,
                                     FAR void *priv)
{
  FAR struct rpmsg_rtc_lowerhalf_s *lower = priv;

  if (strcmp(lower->cpuname, rpmsg_get_cpuname(rdev)) == 0)
    {
      lower->ept.priv = lower;

      rpmsg_create_ept(&lower->ept, rdev, RPMSG_RTC_EPT_NAME,
                       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                       rpmsg_rtc_ept_cb, NULL);
    }
}

static void rpmsg_rtc_device_destroy(FAR struct rpmsg_device *rdev,
                                     FAR void *priv)
{
  FAR struct rpmsg_rtc_lowerhalf_s *lower = priv;

  if (strcmp(lower->cpuname, rpmsg_get_cpuname(rdev)) == 0)
    {
      rpmsg_destroy_ept(&lower->ept);
    }
}

static void rpmsg_rtc_alarm_fire_handler(FAR struct rpmsg_endpoint *ept,
                                         FAR void *data, size_t len,
                                         uint32_t src, FAR void *priv)
{
#ifdef CONFIG_RTC_ALARM
  FAR struct rpmsg_rtc_lowerhalf_s *lower = priv;
  FAR struct rpmsg_rtc_alarm_fire_s *msg = data;
  FAR struct lower_setalarm_s *alarminfo = &lower->alarminfo[msg->id];

  alarminfo->cb(alarminfo->priv, alarminfo->id);
#endif
}

static int rpmsg_rtc_ept_cb(FAR struct rpmsg_endpoint *ept, FAR void *data,
                            size_t len, uint32_t src, FAR void *priv)
{
  FAR struct rpmsg_rtc_header_s *header = data;
  FAR struct rpmsg_rtc_cookie_s *cookie =
      (struct rpmsg_rtc_cookie_s *)(uintptr_t)header->cookie;

  switch (header->command)
    {
    case RPMSG_RTC_ALARM_FIRE:
      rpmsg_rtc_alarm_fire_handler(ept, data, len, src, priv);
      break;

    default:
      if (cookie)
        {
          memcpy(cookie->msg, data, len);
          nxsem_post(&cookie->sem);
        }
      break;
    }

  return 0;
}

static int rpmsg_rtc_send_recv(FAR struct rpmsg_rtc_lowerhalf_s *lower,
                               uint32_t command,
                               FAR struct rpmsg_rtc_header_s *msg, int len)
{
  FAR struct rpmsg_rtc_cookie_s cookie;
  int ret;

  nxsem_init(&cookie.sem, 0, 0);
  nxsem_set_protocol(&cookie.sem, SEM_PRIO_NONE);
  cookie.msg = msg;

  msg->command = command;
  msg->result  = -ENXIO;
  msg->cookie  = (uintptr_t)&cookie;

  ret = rpmsg_send(&lower->ept, msg, len);
  if (ret < 0)
    {
      goto fail;
    }

  ret = nxsem_wait_uninterruptible(&cookie.sem);
  if (ret == 0)
    {
      ret = msg->result;
    }

fail:
  nxsem_destroy(&cookie.sem);
  return ret;
}

static int rpmsg_rtc_rdtime(FAR struct rtc_lowerhalf_s *lower,
                            FAR struct rtc_time *rtctime)
{
  struct rpmsg_rtc_get_s msg;
  int ret;

  ret = rpmsg_rtc_send_recv((FAR struct rpmsg_rtc_lowerhalf_s *)lower,
          RPMSG_RTC_GET, (struct rpmsg_rtc_header_s *)&msg, sizeof(msg));
  if (ret >= 0)
    {
      time_t time = msg.sec;
      gmtime_r(&time, (FAR struct tm *)rtctime);
      rtctime->tm_nsec = msg.nsec;
    }

  return ret;
}

static int rpmsg_rtc_settime(FAR struct rtc_lowerhalf_s *lower,
                             FAR const struct rtc_time *rtctime)
{
  struct rpmsg_rtc_set_s msg =
  {
    .sec  = mktime((FAR struct tm *)rtctime),
    .nsec = rtctime->tm_nsec,
  };

  return rpmsg_rtc_send_recv((FAR struct rpmsg_rtc_lowerhalf_s *)lower,
          RPMSG_RTC_SET, (struct rpmsg_rtc_header_s *)&msg, sizeof(msg));
}

static bool rpmsg_rtc_havesettime(FAR struct rtc_lowerhalf_s *lower)
{
  return true;
}

#ifdef CONFIG_RTC_ALARM
static int rpmsg_rtc_setalarm(FAR struct rtc_lowerhalf_s *lower_,
                              FAR const struct lower_setalarm_s *alarminfo)
{
  FAR struct rpmsg_rtc_lowerhalf_s *lower =
    (FAR struct rpmsg_rtc_lowerhalf_s *)lower_;
  struct rpmsg_rtc_alarm_set_s msg =
  {
    .sec  = mktime((FAR struct tm *)&alarminfo->time),
    .nsec = alarminfo->time.tm_nsec,
    .id   = alarminfo->id,
  };

  int ret;

  ret = rpmsg_rtc_send_recv(lower, RPMSG_RTC_ALARM_SET,
          (struct rpmsg_rtc_header_s *)&msg, sizeof(msg));
  if (ret >= 0)
    {
      lower->alarminfo[alarminfo->id] = *alarminfo;
    }

  return ret;
}

static int
  rpmsg_rtc_setrelative(FAR struct rtc_lowerhalf_s *lower,
                        FAR const struct lower_setrelative_s *relinfo)
{
  struct lower_setalarm_s alarminfo =
  {
    .id   = relinfo->id,
    .cb   = relinfo->cb,
    .priv = relinfo->priv,
  };

  time_t time;

  rpmsg_rtc_rdtime(lower, &alarminfo.time);
  time = mktime((FAR struct tm *)&alarminfo.time);
  time = time + relinfo->reltime;
  gmtime_r(&time, (FAR struct tm *)&alarminfo.time);

  return rpmsg_rtc_setalarm(lower, &alarminfo);
}

static int rpmsg_rtc_cancelalarm(FAR struct rtc_lowerhalf_s *lower,
                                 int alarmid)
{
  struct rpmsg_rtc_alarm_cancel_s msg =
  {
    .id = alarmid,
  };

  return rpmsg_rtc_send_recv((FAR struct rpmsg_rtc_lowerhalf_s *)lower,
          RPMSG_RTC_ALARM_CANCEL, (struct rpmsg_rtc_header_s *)&msg,
          sizeof(msg));
}

static int rpmsg_rtc_rdalarm(FAR struct rtc_lowerhalf_s *lower_,
                             FAR struct lower_rdalarm_s *alarminfo)
{
  FAR struct rpmsg_rtc_lowerhalf_s *lower =
    (FAR struct rpmsg_rtc_lowerhalf_s *)lower_;

  *alarminfo->time = lower->alarminfo[alarminfo->id].time;
  return 0;
}
#endif

/****************************************************************************
 * Name: rpmsg_rtc_initialize
 *
 * Description:
 *
 *   Take remote core RTC as external RTC hardware through rpmsg.
 *
 * Input Parameters:
 *   cpuname - current cpu name
 *   minor  - device minor number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

FAR struct rtc_lowerhalf_s *rpmsg_rtc_initialize(FAR const char *cpuname,
                                                 int minor)
{
  FAR struct rpmsg_rtc_lowerhalf_s *lower;

  lower = kmm_zalloc(sizeof(*lower));
  if (lower)
    {
      lower->ops     = &g_rpmsg_rtc_ops;
      lower->cpuname = cpuname;

      rpmsg_register_callback(lower,
                              rpmsg_rtc_device_created,
                              rpmsg_rtc_device_destroy,
                              NULL);

      rtc_initialize(minor, (FAR struct rtc_lowerhalf_s *)lower);
    }

  return (FAR struct rtc_lowerhalf_s *)lower;
}
