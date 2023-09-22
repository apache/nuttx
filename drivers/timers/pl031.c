/****************************************************************************
 * drivers/timers/pl031.c
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

#include <nuttx/timers/rtc.h>
#include <nuttx/timers/pl031.h>
#include <stdio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define pl031_getreg(b,o)   (*(FAR volatile uint32_t *)((b) + (o)))
#define pl031_putreg(v,b,o) (*((FAR volatile uint32_t *)((b) + (o))) = (v))

#define PL031_RTCDR         0x00     /* RO Data read register */
#define PL031_RTCMR         0x04     /* RW Match register */
#define PL031_RTCLR         0x08     /* RW Data load register */
#define PL031_RTCCR         0x0c     /* RW Control register */
#define PL031_RTCIMSC       0x10     /* RW Interrupt mask and set register */
#define PL031_RTCRIS        0x14     /* RO Raw interrupt status register */
#define PL031_RTCMIS        0x18     /* RO Masked interrupt status register */
#define PL031_RTCICR        0x1c     /* WO Interrupt clear register */

/* ST variants have additional timer functionality */

#define PL031_RTCTDR        0x20     /* Timer data read register */
#define PL031_RTCTLR        0x24     /* Timer data load register */
#define PL031_RTCTCR        0x28     /* Timer control register */
#define PL031_RTCYDR        0x30     /* Year data read register */
#define PL031_RTCYMR        0x34     /* Year match register */
#define PL031_RTCYLR        0x38     /* Year data load register */

#define PL031_RTCBIT_AI     (1 << 0) /* Alarm interrupt bit */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int pl031_rdtime(FAR struct rtc_lowerhalf_s *lower,
                        FAR struct rtc_time *rtctime);
static int pl031_settime(FAR struct rtc_lowerhalf_s *lower,
                         FAR const struct rtc_time *rtctime);
static bool pl031_havesettime(FAR struct rtc_lowerhalf_s *lower);
#if defined(CONFIG_RTC_ALARM)
static int pl031_alarm_irq(int irq, FAR void *context, FAR void *arg);
static int pl031_setalarm(FAR struct rtc_lowerhalf_s *lower,
                          FAR const struct lower_setalarm_s *alarminfo);
static int
pl031_rdalarm(FAR struct rtc_lowerhalf_s *lower,
              FAR struct lower_rdalarm_s *alarminfo);
static int
pl031_cancelalarm(FAR struct rtc_lowerhalf_s *lower, int alarmid);
static int
pl031_setrelative(FAR struct rtc_lowerhalf_s *lower,
                  FAR const struct lower_setrelative_s *alarminfo);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rtc_ops_s g_rtc_ops =
{
  .rdtime         = pl031_rdtime,
  .settime        = pl031_settime,
  .havesettime    = pl031_havesettime,
#ifdef CONFIG_RTC_ALARM
  .setalarm       = pl031_setalarm,
  .setrelative    = pl031_setrelative,
  .cancelalarm    = pl031_cancelalarm,
  .rdalarm        = pl031_rdalarm,
#endif
};

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct pl031_lowerhalf_s
{
  FAR const struct rtc_ops_s *ops;
  uintptr_t base;
  struct lower_setalarm_s alarm;
};

/****************************************************************************
 * Name: pl031_rdtime
 ****************************************************************************/

static int pl031_rdtime(FAR struct rtc_lowerhalf_s *lower,
                        FAR struct rtc_time *rtctime)
{
  FAR struct pl031_lowerhalf_s *priv = (FAR struct pl031_lowerhalf_s *)lower;
  time_t time;

  DEBUGASSERT(priv != NULL && rtctime != NULL);

  time = pl031_getreg(priv->base, PL031_RTCDR);

  gmtime_r(&time, (FAR struct tm *)rtctime);

  return 0;
}

/****************************************************************************
 * Name: pl031_settime
 ****************************************************************************/

static int pl031_settime(FAR struct rtc_lowerhalf_s *lower,
                         FAR const struct rtc_time *rtctime)
{
  FAR struct pl031_lowerhalf_s *priv = (FAR struct pl031_lowerhalf_s *)lower;
  time_t time;

  DEBUGASSERT(priv != NULL && rtctime != NULL);

  time = mktime((FAR struct tm *)rtctime);
  pl031_putreg(time, priv->base, PL031_RTCLR);

  return 0;
}

/****************************************************************************
 * Name: pl031_havesettime
 ****************************************************************************/

static bool pl031_havesettime(FAR struct rtc_lowerhalf_s *lower)
{
  return true;
}

#ifdef CONFIG_RTC_ALARM
/****************************************************************************
 * Name: pl031_alarm_callback
 ****************************************************************************/

static int pl031_alarm_irq(int irq, FAR void *context, FAR void *arg)
{
  FAR struct pl031_lowerhalf_s *lower = (FAR struct pl031_lowerhalf_s *)arg;
  rtc_alarm_callback_t cb;
  FAR void *priv;

  cb = lower->alarm.cb;
  priv = lower->alarm.priv;

  if (cb != NULL)
    {
      cb(priv, 0);
    }

  pl031_putreg(1, lower->base, PL031_RTCICR);
  return OK;
}

/****************************************************************************
 * Name: pl031_setalarm
 ****************************************************************************/

static int pl031_setalarm(FAR struct rtc_lowerhalf_s *lower,
                          FAR const struct lower_setalarm_s *alarminfo)
{
  FAR struct pl031_lowerhalf_s *priv = (FAR struct pl031_lowerhalf_s *)lower;
  time_t time;

  DEBUGASSERT(priv != NULL && alarminfo != NULL);

  priv->alarm.time = alarminfo->time;
  priv->alarm.cb = alarminfo->cb;
  priv->alarm.priv = alarminfo->priv;

  time = mktime((FAR struct tm *)&alarminfo->time);

  pl031_putreg(time, priv->base, PL031_RTCMR);
  pl031_putreg(1, priv->base, PL031_RTCIMSC);

  return OK;
}

/****************************************************************************
 * Name: pl031_rdalarm
 ****************************************************************************/

static int pl031_rdalarm(FAR struct rtc_lowerhalf_s *lower,
                         FAR struct lower_rdalarm_s *alarminfo)
{
  FAR struct pl031_lowerhalf_s *priv = (FAR struct pl031_lowerhalf_s *)lower;

  DEBUGASSERT(priv != NULL && alarminfo != NULL);

  *alarminfo->time = priv->alarm.time;
  return 0;
}

/****************************************************************************
 * Name: pl031_cancelalarm
 ****************************************************************************/

int pl031_cancelalarm(FAR struct rtc_lowerhalf_s *lower, int alarmid)
{
  FAR struct pl031_lowerhalf_s *priv = (FAR struct pl031_lowerhalf_s *)lower;

  DEBUGASSERT(priv != NULL);

  pl031_putreg(0, priv->base, PL031_RTCIMSC);
  return OK;
}

/****************************************************************************
 * Name: pl031_setrelative
 ****************************************************************************/

static int
pl031_setrelative(FAR struct rtc_lowerhalf_s *lower,
                  FAR const struct lower_setrelative_s *alarminfo)
{
  FAR struct pl031_lowerhalf_s *priv = (FAR struct pl031_lowerhalf_s *)lower;
  struct lower_setalarm_s setalarm;
  time_t time;

  DEBUGASSERT(priv != NULL && alarminfo != NULL);

  setalarm.id = alarminfo->id;
  setalarm.cb = alarminfo->cb;
  setalarm.priv = alarminfo->priv;

  time = pl031_getreg(priv->base, PL031_RTCDR);
  time += alarminfo->reltime;

  gmtime_r(&time, (FAR struct tm *)&setalarm.time);

  return pl031_setalarm(lower, &setalarm);
}
#endif

/****************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Initialize the qemu RTC per the selected configuration.  This function
 *   is called once during the OS initialization sequence
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

FAR struct rtc_lowerhalf_s *pl031_initialize(uintptr_t base, int irq)
{
  FAR struct pl031_lowerhalf_s *rtc_lowerhalf =
     kmm_zalloc(sizeof(*rtc_lowerhalf));

  rtc_lowerhalf->ops = &g_rtc_ops;
  rtc_lowerhalf->base = base;

#ifdef CONFIG_RTC_ALARM
  irq_attach(irq, pl031_alarm_irq, rtc_lowerhalf);
  up_enable_irq(irq);
#endif

  return (FAR struct rtc_lowerhalf_s *)rtc_lowerhalf;
}
