/****************************************************************************
 * arch/risc-v/src/espressif/esp_rtc.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <string.h>
#include <sys/types.h>

#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/timers/rtc.h>

#include "clock/clock.h"

#include "esp_hr_timer.h"
#include "esp_rtc.h"
#include "riscv_internal.h"

#include "esp_attr.h"
#include "soc/rtc.h"

/* Chip-dependent headers from esp-hal-3rdparty */

#ifdef CONFIG_ESPRESSIF_ESP32C3
#include "esp32c3/rtc.h"
#include "esp32c3/rom/rtc.h"
#elif defined(CONFIG_ESPRESSIF_ESP32C6)
#include "esp32c6/rtc.h"
#include "esp32c6/rom/rtc.h"
#elif defined(CONFIG_ESPRESSIF_ESP32H2)
#include "esp32h2/rtc.h"
#include "esp32h2/rom/rtc.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The magic data for the struct esp_rtc_backup_s that is in RTC slow
 * memory.
 */

#define MAGIC_RTC_SAVE (UINT64_C(0x11223344556677))

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_RTC_DRIVER

#ifdef CONFIG_RTC_ALARM
struct alm_cbinfo_s
{
  struct esp_hr_timer_s  *alarm_hdl;    /* Timer id point to here */
  rtc_alarm_callback_t    ac_cb;        /* Client callback function */
  volatile void          *ac_arg;       /* Argument to pass with the callback function */
  uint64_t                deadline_us;
  uint8_t                 index;
};
#endif /* CONFIG_RTC_ALARM */

/* This is the private type for the RTC state. It must be cast compatible
 * with struct rtc_lowerhalf_s.
 */

struct esp_rtc_lowerhalf_s
{
  /* This is the contained reference to the read-only, lower-half
   * operations vtable (which may lie in FLASH or ROM)
   */

  const struct rtc_ops_s *ops;
#ifdef CONFIG_RTC_ALARM
  /* Alarm callback information */

  struct alm_cbinfo_s     alarmcb[CONFIG_RTC_NALARMS];
#endif /* CONFIG_RTC_ALARM */
};

#endif/* CONFIG_RTC_DRIVER */

struct esp_rtc_backup_s
{
  uint64_t magic;
  int64_t  offset;           /* Offset time from RTC HW value */
  int64_t  reserved0;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Prototypes for static methods in struct rtc_ops_s */

#ifdef CONFIG_RTC_DRIVER
static int esp_rtc_rdtime(struct rtc_lowerhalf_s *lower,
                          struct rtc_time *rtctime);
static int esp_rtc_settime(struct rtc_lowerhalf_s *lower,
                           const struct rtc_time *rtctime);
static bool esp_rtc_havesettime(struct rtc_lowerhalf_s *lower);

#ifdef CONFIG_RTC_ALARM
static void IRAM_ATTR rtc_hr_timer_cb(void *arg);
static int esp_rtc_setalarm(struct rtc_lowerhalf_s *lower,
                            const struct lower_setalarm_s *alarminfo);
static int esp_rtc_setrelative(struct rtc_lowerhalf_s *lower,
                               const struct lower_setrelative_s *alarminfo);
static int esp_rtc_cancelalarm(struct rtc_lowerhalf_s *lower,
                               int alarmid);
static int esp_rtc_rdalarm(struct rtc_lowerhalf_s *lower,
                           struct lower_rdalarm_s *alarminfo);
#endif /* CONFIG_RTC_ALARM */
#endif /* CONFIG_RTC_DRIVER */

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_RTC_DRIVER
/* RTC driver operations */

static const struct rtc_ops_s g_rtc_ops =
{
  .rdtime      = esp_rtc_rdtime,
  .settime     = esp_rtc_settime,
  .havesettime = esp_rtc_havesettime,
#ifdef CONFIG_RTC_ALARM
  .setalarm    = esp_rtc_setalarm,
  .setrelative = esp_rtc_setrelative,
  .cancelalarm = esp_rtc_cancelalarm,
  .rdalarm     = esp_rtc_rdalarm,
#endif /* CONFIG_RTC_ALARM */
};

/* RTC device state */

static struct esp_rtc_lowerhalf_s g_rtc_lowerhalf =
{
  .ops = &g_rtc_ops
};

/* Flag for tracking HR Timer enable status */

static bool g_hr_timer_enabled = false;

#endif /* CONFIG_RTC_DRIVER */

/* Saved data for persistent RTC time */

static RTC_DATA_ATTR struct esp_rtc_backup_s g_rtc_saved_data;
static struct esp_rtc_backup_s *g_rtc_save;

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile bool g_rtc_enabled = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_rtc_set_boot_time
 *
 * Description:
 *   Set time to RTC register to replace the original boot time.
 *
 * Input Parameters:
 *   time_us       - set time in microseconds.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void IRAM_ATTR esp_rtc_set_boot_time(uint64_t time_us)
{
  putreg32((uint32_t)(time_us & UINT32_MAX), RTC_BOOT_TIME_LOW_REG);
  putreg32((uint32_t)(time_us >> 32), RTC_BOOT_TIME_HIGH_REG);
}

/****************************************************************************
 * Name: esp_rtc_get_boot_time
 *
 * Description:
 *   Get time of RTC register to indicate the original boot time.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Boot time in microseconds.
 *
 ****************************************************************************/

static uint64_t IRAM_ATTR esp_rtc_get_boot_time(void)
{
  return ((uint64_t)getreg32(RTC_BOOT_TIME_LOW_REG))
        + (((uint64_t)getreg32(RTC_BOOT_TIME_HIGH_REG)) << 32);
}

/****************************************************************************
 * Name: rtc_hr_timer_cb
 *
 * Description:
 *   Callback to be called upon HR-Timer expiration.
 *
 * Input Parameters:
 *   arg           - Information about the HR-Timer configuration.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#if defined(CONFIG_RTC_DRIVER) && defined(CONFIG_RTC_ALARM)
static void IRAM_ATTR rtc_hr_timer_cb(void *arg)
{
  struct alm_cbinfo_s *cbinfo = (struct alm_cbinfo_s *)arg;
  rtc_alarm_callback_t cb;
  void *cb_arg;
  int alminfo_id;

  DEBUGASSERT(cbinfo != NULL);

  alminfo_id = cbinfo->index;

  if (cbinfo->ac_cb != NULL)
    {
      /* Alarm callback */

      cb = cbinfo->ac_cb;
      cb_arg = (void *)cbinfo->ac_arg;
      cbinfo->ac_cb  = NULL;
      cbinfo->ac_arg = NULL;
      cbinfo->deadline_us = 0;
      cb(cb_arg, alminfo_id);
    }
}
#endif /* CONFIG_RTC_DRIVER */

/****************************************************************************
 * Name: esp_rtc_rdtime
 *
 * Description:
 *   Return the current RTC time.
 *
 * Input Parameters:
 *   lower         - A reference to RTC lower half driver state structure.
 *   rcttime       - The location in which to return the current RTC time.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_DRIVER

#ifdef CONFIG_RTC_HIRES
static int esp_rtc_rdtime(struct rtc_lowerhalf_s *lower,
                          struct rtc_time *rtctime)
{
  struct timespec ts;
  int ret;

  /* Get the higher resolution time */

  ret = up_rtc_gettime(&ts);
  if (ret < 0)
    {
      goto errout;
    }

  /* Convert the one second epoch time to a struct tmThis operation
   * depends on the fact that struct rtc_time and struct tm are cast
   * compatible.
   */

  if (!gmtime_r(&ts.tv_sec, (struct tm *)rtctime))
    {
      ret = -get_errno();
      goto errout;
    }

  return OK;

errout:
  rtcerr("Failed to get RTC time: %d\n", ret);
  return ret;
}

#else /* !CONFIG_RTC_HIRES */

static int esp_rtc_rdtime(struct rtc_lowerhalf_s *lower,
                          struct rtc_time *rtctime)
{
  time_t timer;

  /* The resolution of time is only 1 second */

  timer = up_rtc_time();

  /* Convert the one second epoch time to a struct tm */

  if (gmtime_r(&timer, (struct tm *)rtctime) == 0)
    {
      int errcode = get_errno();
      DEBUGASSERT(errcode > 0);

      rtcerr("gmtime_r failed: %d\n", errcode);
      return -errcode;
    }

  return OK;
}
#endif /* CONFIG_RTC_HIRES */

#endif /* CONFIG_RTC_DRIVER */

/****************************************************************************
 * Name: esp_rtc_settime
 *
 * Description:
 *   Implement the settime() method of the RTC driver interface.
 *
 * Input Parameters:
 *   lower         - A reference to RTC lower half driver state structure.
 *   rcttime       - The new time to set.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_DRIVER
static int esp_rtc_settime(struct rtc_lowerhalf_s *lower,
                           const struct rtc_time *rtctime)
{
  struct timespec ts;

  /* Convert the struct rtc_time to a time_t. Here we assume that struct
   * rtc_time is cast compatible with struct tm.
   */

  ts.tv_sec  = mktime((struct tm *)rtctime);
  ts.tv_nsec = 0;

  /* Now set the time (with a accuracy of seconds) */

  return up_rtc_settime(&ts);
}
#endif /* CONFIG_RTC_DRIVER */

/****************************************************************************
 * Name: esp_rtc_havesettime
 *
 * Description:
 *   Implement the havesettime() method of the RTC driver interface.
 *
 * Input Parameters:
 *   lower         - A reference to RTC lower half driver state structure.
 *
 * Returned Value:
 *   True if RTC date-time have been previously set, false otherwise.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_DRIVER
static bool esp_rtc_havesettime(struct rtc_lowerhalf_s *lower)
{
  return esp_rtc_get_boot_time() != 0;
}
#endif /* CONFIG_RTC_DRIVER */

/****************************************************************************
 * Name: esp_rtc_setalarm
 *
 * Description:
 *   Set a new alarm. This function implements the setalarm() method of the
 *   RTC driver interface.
 *
 * Input Parameters:
 *   lower         - A reference to RTC lower half driver state structure.
 *   alarminfo     - Provided information needed to set the alarm.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#if defined(CONFIG_RTC_DRIVER) && defined(CONFIG_RTC_ALARM)
static int esp_rtc_setalarm(struct rtc_lowerhalf_s *lower,
                            const struct lower_setalarm_s *alarminfo)
{
  struct esp_rtc_lowerhalf_s *priv = (struct esp_rtc_lowerhalf_s *)lower;
  struct alm_cbinfo_s *cbinfo;
  uint64_t timeout;
  irqstate_t flags;
  int id;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(alarminfo != NULL);

  timeout = mktime((struct tm *)&alarminfo->time);

  /* Set the alarm in RT-Timer */

  id     = alarminfo->id;
  cbinfo = &priv->alarmcb[id];

  if (cbinfo->ac_cb != NULL)
    {
      return -EBUSY;
    }

  /* Create the RT-Timer alarm */

  flags = spin_lock_irqsave(NULL);

  if (cbinfo->alarm_hdl == NULL)
    {
      struct esp_hr_timer_args_s hr_timer_args;
      int ret;

      cbinfo->index          = id;
      hr_timer_args.arg      = cbinfo;
      hr_timer_args.callback = rtc_hr_timer_cb;

      ret = esp_hr_timer_create(&hr_timer_args, &cbinfo->alarm_hdl);
      if (ret < 0)
        {
          rtcerr("Failed to create HR Timer=%d\n", ret);
          spin_unlock_irqrestore(NULL, flags);
          return ret;
        }
    }

  cbinfo->ac_cb       = alarminfo->cb;
  cbinfo->ac_arg      = alarminfo->priv;
  cbinfo->deadline_us = timeout * USEC_PER_SEC;

  rtcinfo("Starting alarm ID %d\n", id);

  esp_hr_timer_start(cbinfo->alarm_hdl, cbinfo->deadline_us, false);

  spin_unlock_irqrestore(NULL, flags);

  return OK;
}
#endif /* CONFIG_RTC_DRIVER && CONFIG_RTC_ALARM */

/****************************************************************************
 * Name: esp_rtc_setrelative
 *
 * Description:
 *   Set a new alarm relative to the current time. This function implements
 *   the setrelative() method of the RTC driver interface.
 *
 * Input Parameters:
 *   lower         - A reference to RTC lower half driver state structure.
 *   alarminfo     - Provided information needed to set the alarm.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#if defined(CONFIG_RTC_DRIVER) && defined(CONFIG_RTC_ALARM)
static int esp_rtc_setrelative(struct rtc_lowerhalf_s *lower,
                               const struct lower_setrelative_s *alarminfo)
{
  struct lower_setalarm_s setalarm;
  time_t seconds;
  int ret = -EINVAL;
  irqstate_t flags;

  DEBUGASSERT(lower != NULL && alarminfo != NULL);

  if (alarminfo->reltime > 0)
    {
      flags = spin_lock_irqsave(NULL);

      seconds = alarminfo->reltime;
      gmtime_r(&seconds, (struct tm *)&setalarm.time);

      /* The set the alarm using this absolute time */

      setalarm.id   = alarminfo->id;
      setalarm.cb   = alarminfo->cb;
      setalarm.priv = alarminfo->priv;
      ret = esp_rtc_setalarm(lower, &setalarm);

      spin_unlock_irqrestore(NULL, flags);
    }

  return ret;
}
#endif /* CONFIG_RTC_DRIVER && CONFIG_RTC_ALARM */

/****************************************************************************
 * Name: esp_rtc_cancelalarm
 *
 * Description:
 *   Cancel the current alarm. This function implements the cancelalarm()
 *   method of the RTC driver interface.
 *
 * Input Parameters:
 *   lower         - A reference to RTC lower half driver state structure.
 *   alarmid       - ID of the alarm to be cancelled.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#if defined(CONFIG_RTC_DRIVER) && defined(CONFIG_RTC_ALARM)
static int esp_rtc_cancelalarm(struct rtc_lowerhalf_s *lower, int alarmid)
{
  struct esp_rtc_lowerhalf_s *priv;
  struct alm_cbinfo_s *cbinfo;
  irqstate_t flags;

  DEBUGASSERT(lower != NULL);

  priv = (struct esp_rtc_lowerhalf_s *)lower;

  /* Set the alarm in hardware and enable interrupts */

  cbinfo = &priv->alarmcb[alarmid];

  if (cbinfo->ac_cb == NULL)
    {
      return -ENODATA;
    }

  flags = spin_lock_irqsave(NULL);

  /* Stop and delete the alarm */

  rtcinfo("Cancelling alarm ID %d\n", alarmid);

  esp_hr_timer_stop(cbinfo->alarm_hdl);
  esp_hr_timer_delete(cbinfo->alarm_hdl);

  cbinfo->ac_cb = NULL;
  cbinfo->deadline_us = 0;
  cbinfo->alarm_hdl = NULL;

  spin_unlock_irqrestore(NULL, flags);

  return OK;
}
#endif /* CONFIG_RTC_DRIVER && CONFIG_RTC_ALARM */

/****************************************************************************
 * Name: esp_rtc_rdalarm
 *
 * Description:
 *   Query the RTC alarm.
 *
 * Input Parameters:
 *   lower         - A reference to RTC lower half driver state structure.
 *   alarminfo     - Provided information needed to query the alarm.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#if defined(CONFIG_RTC_DRIVER) && defined(CONFIG_RTC_ALARM)
static int esp_rtc_rdalarm(struct rtc_lowerhalf_s *lower,
                           struct lower_rdalarm_s *alarminfo)
{
  struct esp_rtc_lowerhalf_s *priv;
  struct timespec ts;
  struct alm_cbinfo_s *cbinfo;
  irqstate_t flags;

  DEBUGASSERT(lower != NULL);
  DEBUGASSERT(alarminfo != NULL);
  DEBUGASSERT(alarminfo->time != NULL);

  priv = (struct esp_rtc_lowerhalf_s *)lower;

  flags = spin_lock_irqsave(NULL);

  /* Get the alarm according to the alarm ID */

  cbinfo = &priv->alarmcb[alarminfo->id];

  ts.tv_sec = (esp_hr_timer_time_us() + g_rtc_save->offset +
              cbinfo->deadline_us) / USEC_PER_SEC;
  ts.tv_nsec = ((esp_hr_timer_time_us() + g_rtc_save->offset +
              cbinfo->deadline_us) % USEC_PER_SEC) * NSEC_PER_USEC;

  localtime_r((const time_t *)&ts.tv_sec,
              (struct tm *)alarminfo->time);

  spin_unlock_irqrestore(NULL, flags);

  return OK;
}
#endif /* CONFIG_RTC_DRIVER && CONFIG_RTC_ALARM */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_rtc_time
 *
 * Description:
 *   Get the current time in seconds. This is similar to the standard time()
 *   function. This interface is only required if the low-resolution
 *   RTC/counter hardware implementation is selected. It is only used by the
 *   RTOS during initialization to set up the system time when CONFIG_RTC is
 *   set but CONFIG_RTC_HIRES is not set.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Current time in seconds.
 *
 ****************************************************************************/

#ifndef CONFIG_RTC_HIRES
time_t up_rtc_time(void)
{
  uint64_t time_us;
  irqstate_t flags;

  flags = spin_lock_irqsave(NULL);

#ifdef CONFIG_RTC_DRIVER
  /* NOTE: HR-Timer starts to work after the board is initialized, and the
   * RTC controller starts works after up_rtc_initialize is initialized.
   * Since the system clock starts to work before the board is initialized,
   * if CONFIG_RTC is enabled, the system time must be matched by the time
   * of the RTC controller (up_rtc_initialize has already been initialized,
   * and HR-Timer cannot work).
   */

  /* Determine if HR-Timer is started */

  if (g_hr_timer_enabled)
    {
      /* Get the time from HR-Timer, the time interval between RTC
       * controller and HR-Timer is stored in g_rtc_save->offset.
       */

      time_us = esp_hr_timer_time_us() + g_rtc_save->offset +
                              esp_rtc_get_boot_time();
    }
  else
#endif
    {
      /* Get the time from RTC controller */

      time_us = esp_rtc_get_time_us() + esp_rtc_get_boot_time();
    }

  spin_unlock_irqrestore(NULL, flags);

  return (time_t)(time_us / USEC_PER_SEC);
}
#endif /* !CONFIG_RTC_HIRES */

/****************************************************************************
 * Name: up_rtc_gettime
 *
 * Description:
 *   Get the current time from the high resolution RTC time or HR-Timer. This
 *   interface is only supported by the high-resolution RTC/counter hardware
 *   implementation. It is used to replace the system timer.
 *
 * Input Parameters:
 *   tp            - The location to return the RTC time or HR-Timer value.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_HIRES
int up_rtc_gettime(struct timespec *tp)
{
  irqstate_t flags;
  uint64_t time_us;

  flags = spin_lock_irqsave(NULL);

#ifdef CONFIG_RTC_DRIVER
  if (g_hr_timer_enabled)
    {
      time_us = esp_hr_timer_time_us() + g_rtc_save->offset +
                              esp_rtc_get_boot_time();
    }
  else
#endif
    {
      time_us = esp_rtc_get_time_us() + esp_rtc_get_boot_time();
    }

  tp->tv_sec  = time_us / USEC_PER_SEC;
  tp->tv_nsec = (time_us % USEC_PER_SEC) * NSEC_PER_USEC;

  spin_unlock_irqrestore(NULL, flags);

  return OK;
}
#endif /* CONFIG_RTC_HIRES */

/****************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Set the RTC to the provided time. All RTC implementations must be
 *   able to set their time based on a standard timespec.
 *
 * Input Parameters:
 *   ts            - Time to set.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

int up_rtc_settime(const struct timespec *ts)
{
  uint64_t now_us;
  uint64_t rtc_offset_us;
  irqstate_t flags;

  DEBUGASSERT(ts != NULL && ts->tv_nsec < NSEC_PER_SEC);

  flags = spin_lock_irqsave(NULL);

  now_us = ((uint64_t) ts->tv_sec) * USEC_PER_SEC +
          ts->tv_nsec / NSEC_PER_USEC;

#ifdef CONFIG_RTC_DRIVER
  if (g_hr_timer_enabled)
    {
      /* Set based on HR-Timer offset value */

      rtc_offset_us = now_us - esp_hr_timer_time_us();
    }
  else
#endif
    {
      /* Set based on the offset value of the RTC controller */

      rtc_offset_us = now_us - esp_rtc_get_time_us();
    }

  g_rtc_save->offset = 0;

  esp_rtc_set_boot_time(rtc_offset_us);

  spin_unlock_irqrestore(NULL, flags);

  return OK;
}

/****************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Initialize the hardware RTC according to the selected configuration.
 *   This function is called once during the OS initialization sequence.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

int up_rtc_initialize(void)
{
  g_rtc_save = &g_rtc_saved_data;

  /* If saved data is invalid, clear offset information */

  if (g_rtc_save->magic != MAGIC_RTC_SAVE)
    {
      g_rtc_save->magic = MAGIC_RTC_SAVE;
      g_rtc_save->offset = 0;
      esp_rtc_set_boot_time(0);
    }

#ifdef CONFIG_RTC_HIRES
  /* Synchronize the base time to the RTC time */

  up_rtc_gettime(&g_basetime);
#endif

  g_rtc_enabled = true;

  return OK;
}

/****************************************************************************
 * Name: esp_rtc_driverinit
 *
 * Description:
 *   Initialize and register an RTC lower half driver.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_DRIVER
int esp_rtc_driverinit(void)
{
  struct rtc_lowerhalf_s *lower = (struct rtc_lowerhalf_s *)&g_rtc_lowerhalf;

  int ret = rtc_initialize(0, lower);
  if (ret < 0)
    {
      return ret;
    }

  /* Enable HR-Timer */

  VERIFY(esp_hr_timer_init());

  g_hr_timer_enabled = true;

  /* Get the time difference between HR Timer and RTC */

  g_rtc_save->offset = esp_rtc_get_time_us() - esp_hr_timer_time_us();

  return ret;
}
#endif /* CONFIG_RTC_DRIVER */
