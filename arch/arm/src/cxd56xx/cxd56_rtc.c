/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_rtc.c
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

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/wdog.h>
#include <nuttx/clock.h>

#include <arch/board/board.h>

#include "clock/clock.h"
#include "arm_internal.h"
#include "cxd56_rtc.h"

#include "hardware/cxd5602_topreg.h"
#include "hardware/cxd5602_memorymap.h"
#include "hardware/cxd5602_backupmem.h"
#include "hardware/cxd56_rtc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifdef CONFIG_RTC_HIRES
#  ifndef CONFIG_RTC_FREQUENCY
#    define CONFIG_RTC_FREQUENCY 32768
#  endif
#  if CONFIG_RTC_FREQUENCY != 32768
#    error "Only lo-res CONFIG_RTC_FREQUENCY of 32.768kHz is supported"
#  endif
#else
#  ifndef CONFIG_RTC_FREQUENCY
#    define CONFIG_RTC_FREQUENCY 1
#  endif
#  if CONFIG_RTC_FREQUENCY != 1
#    error "Only lo-res CONFIG_RTC_FREQUENCY of 1Hz is supported"
#  endif
#endif

/* convert seconds to 64bit counter value running at 32kHz */

#define SEC_TO_CNT(sec) ((uint64_t)(((uint64_t)(sec)) << 15))

/* convert nano-seconds to 32kHz counter less than 1 second */

#define NSEC_TO_PRECNT(nsec) \
  (((nsec) / (NSEC_PER_SEC / CONFIG_RTC_FREQUENCY)) & 0x7fff)

#define MAGIC_RTC_SAVE (0x12aae190077a80ull)

/* RTC clcok stable waiting time (interval x retry) */

#define RTC_CLOCK_CHECK_INTERVAL  (200) /* milliseconds */
#define RTC_CLOCK_CHECK_MAX_RETRY (15)

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
struct alm_cbinfo_s
{
  volatile alm_callback_t ac_cb; /* Client callback function */
  volatile void *ac_arg;         /* Argument to pass with the callback function */
};
#endif

struct rtc_backup_s
{
  uint64_t magic;
  int64_t  reserved0;
  int64_t  offset;              /* offset time from RTC HW value */
  int64_t  reserved1;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Callback to use when the alarm expires */

#ifdef CONFIG_RTC_ALARM
static struct alm_cbinfo_s g_alarmcb[RTC_ALARM_LAST];
#endif

/* Saved data for persistent RTC time */

static struct rtc_backup_s *g_rtc_save;

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile bool g_rtc_enabled = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtc_dumptime
 *
 * Description:
 *    Dump RTC
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_RTC
static void rtc_dumptime(const struct timespec *tp, const char *msg)
{
  struct tm tm;

  gmtime_r(&tp->tv_sec, &tm);

  rtcinfo("%s:\n", msg);
  rtcinfo("RTC %u.%09u\n", tp->tv_sec, tp->tv_nsec);
  rtcinfo("%4d/%02d/%02d %02d:%02d:%02d\n",
          tm.tm_year, tm.tm_mon, tm.tm_mday,
          tm.tm_hour, tm.tm_min, tm.tm_sec);
}
#else
#  define rtc_dumptime(tp, msg)
#endif

/****************************************************************************
 * Name: cxd56_rtc_interrupt
 *
 * Description:
 *    RTC interrupt service routine
 *
 * Input Parameters:
 *   irq - The IRQ number that generated the interrupt
 *   context - Architecture specific register save information.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int cxd56_rtc_interrupt(int irq, void *context, void *arg)
{
  struct alm_cbinfo_s *cbinfo;
  alm_callback_t cb;
  void *cb_arg;
  uint32_t source;
  uint32_t clear;
  int id;
  int ret = OK;

  /* interrupt clear */

  source = getreg32(CXD56_RTC0_ALMFLG);

  if (source & RTCREG_ALM0_MASK)
    {
      id = RTC_ALARM0;
      clear = source & RTCREG_ALM0_MASK;
    }
  else
    {
      rtcerr("ERROR: Invalid ALARM\n");
      return ret;
    }

  putreg32(clear, CXD56_RTC0_ALMCLR);
  putreg32(0, CXD56_RTC0_ALMOUTEN(id));

  cbinfo = &g_alarmcb[id];

  if (cbinfo->ac_cb != NULL)
    {
      /* Alarm callback */

      cb = cbinfo->ac_cb;
      cb_arg = (void *)cbinfo->ac_arg;

      cbinfo->ac_cb  = NULL;
      cbinfo->ac_arg = NULL;

      cb(cb_arg, id);
    }

  return 0;
}
#endif

/****************************************************************************
 * Name: cxd56_rtc_initialize
 *
 * Description:
 *   Actually initialize the hardware RTC. This function is called in the
 *   initialization sequence, thereafter may be called when wdog timer is
 *   expired.
 *
 * Input Parameters:
 *   arg: Not used
 *
 ****************************************************************************/

static void cxd56_rtc_initialize(wdparm_t arg)
{
  struct timespec ts;
#ifdef CONFIG_CXD56_RTC_LATEINIT
  static struct wdog_s s_wdog;
  static int s_retry = 0;

  /* Check whether RTC clock source selects the external RTC and the
   * synchronization from the external RTC is completed.
   */

  g_rtc_save = (struct rtc_backup_s *)BKUP->rtc_saved_data;

  if (((getreg32(CXD56_TOPREG_CKSEL_ROOT) & STATUS_RTC_MASK)
       != STATUS_RTC_SEL) ||
      (g_rtc_save->magic != MAGIC_RTC_SAVE))
    {
      /* Retry until RTC clock is stable */

      if (s_retry++ < RTC_CLOCK_CHECK_MAX_RETRY)
        {
          rtcinfo("retry count: %d\n", s_retry);

          if (OK == wd_start(&s_wdog, MSEC2TICK(RTC_CLOCK_CHECK_INTERVAL),
                             cxd56_rtc_initialize, 0))
            {
              /* Again, this function is called recursively */

              return;
            }
        }

      rtcerr("ERROR: Use inaccurate RCRTC instead of RTC\n");
    }

  /* RTC clock is stable, or give up using the external RTC */

  wd_cancel(&s_wdog);
#endif

#ifdef CONFIG_RTC_ALARM
  /* Configure RTC interrupt to catch overflow and alarm interrupts. */

  irq_attach(CXD56_IRQ_RTC0_A0, cxd56_rtc_interrupt, NULL);
  irq_attach(CXD56_IRQ_RTC_INT, cxd56_rtc_interrupt, NULL);
  up_enable_irq(CXD56_IRQ_RTC0_A0);
  up_enable_irq(CXD56_IRQ_RTC_INT);
#endif

  /* If saved data is invalid, clear offset information */

  if (g_rtc_save->magic != MAGIC_RTC_SAVE)
    {
      g_rtc_save->offset = 0;
    }

  if (g_rtc_save->offset == 0)
    {
      /* Keep the system operating time before RTC is enabled. */

      clock_systime_timespec(&ts);
    }

#ifdef CONFIG_RTC_HIRES
  /* Synchronize the base time to the RTC time */

  up_rtc_gettime(&g_basetime);
#endif

  if (g_rtc_save->offset == 0)
    {
      /* Reflect the system operating time to RTC offset data. */

      g_rtc_save->offset = SEC_TO_CNT(ts.tv_sec) |
                           NSEC_TO_PRECNT(ts.tv_nsec);
    }

  /* Make it possible to use the RTC timer functions */

  g_rtc_enabled = true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Initialize the hardware RTC per the selected configuration.  This
 *   function is called once during the OS initialization sequence
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
  cxd56_rtc_initialize(0);
  return OK;
}

/****************************************************************************
 * Name: up_rtc_time
 *
 * Description:
 *   Get the current time in seconds.  This is similar to the standard time()
 *   function.  This interface is only required if the low-resolution
 *   RTC/counter hardware implementation selected.  It is only used by the
 *   RTOS during initialization to set up the system time when CONFIG_RTC is
 *   set but neither CONFIG_RTC_HIRES nor CONFIG_RTC_DATETIME are set.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current time in seconds
 *
 ****************************************************************************/

#ifndef CONFIG_RTC_HIRES
time_t up_rtc_time(void)
{
  uint64_t count;

  count = cxd56_rtc_count();
  count += g_rtc_save->offset;
  count >>= 15; /* convert to 1sec resolution */

  return (time_t)count / CONFIG_RTC_FREQUENCY;
}
#endif

/****************************************************************************
 * Name: up_rtc_gettime
 *
 * Description:
 *   Get the current time from the high resolution RTC clock/counter.  This
 *   interface is only supported by the high-resolution RTC/counter hardware
 *   implementation.  It is used to replace the system timer.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_HIRES
int up_rtc_gettime(struct timespec *tp)
{
  uint64_t count;

  count = cxd56_rtc_count();
  count += g_rtc_save->offset;

  /* Then we can save the time in seconds and fractional seconds. */

  tp->tv_sec  = count / CONFIG_RTC_FREQUENCY;
  tp->tv_nsec = (count % CONFIG_RTC_FREQUENCY) *
                (NSEC_PER_SEC / CONFIG_RTC_FREQUENCY);

  rtc_dumptime(tp, "Getting time");

  return OK;
}
#endif

/****************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Set the RTC to the provided time.  All RTC implementations must be able
 *   to set their time based on a standard timespec.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_settime(const struct timespec *tp)
{
  irqstate_t flags;
  uint64_t count;

  flags = spin_lock_irqsave(NULL);

#ifdef RTC_DIRECT_CONTROL
  /* wait until previous write request is completed */

  while (RTCREG_WREQ_BUSYA_MASK & getreg32(CXD56_RTC0_WRREGREQ));

  putreg32(tp->tv_sec, CXD56_RTC0_WRREGPOSTCNT);
  putreg32(NSEC_TO_PRECNT(tp->tv_nsec), CXD56_RTC0_WRREGPRECNT);

  putreg32(RTCREG_WREQ_BUSYA_MASK, CXD56_RTC0_WRREGREQ);

  /* wait until write request reflected */

  while (RTCREG_WREQ_BUSYB_MASK & getreg32(CXD56_RTC0_WRREGREQ));

#else
  /* Only save the difference from HW raw value */

  count = SEC_TO_CNT(tp->tv_sec) | NSEC_TO_PRECNT(tp->tv_nsec);
  g_rtc_save->offset = (int64_t)count - (int64_t)cxd56_rtc_count();
#endif

  spin_unlock_irqrestore(NULL, flags);

  rtc_dumptime(tp, "Setting time");

  return OK;
}

/****************************************************************************
 * Name: cxd56_rtc_count
 *
 * Description:
 *   Get RTC raw counter value
 *
 * Returned Value:
 *   64bit counter value running at 32kHz
 *
 ****************************************************************************/

uint64_t cxd56_rtc_count(void)
{
  uint64_t val;
  irqstate_t flags;

  /* The pre register is latched with reading the post rtcounter register,
   * so these registers always have to been read in the below order,
   * 1st post -> 2nd pre, and should be operated in atomic.
   */

  flags = spin_lock_irqsave(NULL);

  val = (uint64_t)getreg32(CXD56_RTC0_RTPOSTCNT) << 15;
  val |= getreg32(CXD56_RTC0_RTPRECNT);

  spin_unlock_irqrestore(NULL, flags);

  return val;
}

/****************************************************************************
 * Name: cxd56_rtc_almcount
 *
 * Description:
 *   Get RTC raw alarm counter value
 *
 * Returned Value:
 *   64bit alarm counter value running at 32kHz
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
uint64_t cxd56_rtc_almcount(void)
{
  uint64_t val;
  irqstate_t flags;

  flags = spin_lock_irqsave(NULL);

  val = (uint64_t)getreg32(CXD56_RTC0_SETALMPOSTCNT(0)) << 15;
  val |= (getreg32(CXD56_RTC0_SETALMPRECNT(0)) & 0x7fff);

  spin_unlock_irqrestore(NULL, flags);

  return val;
}
#endif

/****************************************************************************
 * Name: cxd56_rtc_setalarm
 *
 * Description:
 *   Set up an alarm.
 *
 * Input Parameters:
 *   alminfo - Information about the alarm configuration.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int cxd56_rtc_setalarm(struct alm_setalarm_s *alminfo)
{
  struct alm_cbinfo_s *cbinfo;
  irqstate_t flags;
  int ret = -EBUSY;
  int id;
  uint64_t count;
  uint32_t mask;

  ASSERT(alminfo != NULL);
  DEBUGASSERT(RTC_ALARM_LAST > alminfo->as_id);

  /* Set the alarm in hardware and enable interrupts */

  id = alminfo->as_id;
  cbinfo = &g_alarmcb[id];

  if (cbinfo->ac_cb == NULL)
    {
      /* The set the alarm */

      flags = spin_lock_irqsave(NULL);

      cbinfo->ac_cb  = alminfo->as_cb;
      cbinfo->ac_arg = alminfo->as_arg;

      count = SEC_TO_CNT(alminfo->as_time.tv_sec) |
        NSEC_TO_PRECNT(alminfo->as_time.tv_nsec);

      count -= g_rtc_save->offset;

      /* clear previsous setting */

      mask = RTCREG_ALM0_ERR_FLAG_MASK | RTCREG_ALM0_FLAG_MASK;
      mask <<= id;

      putreg32(mask, CXD56_RTC0_ALMCLR);

      /* wait until previous alarm request is completed */

      while (RTCREG_ASET_BUSY_MASK & getreg32(CXD56_RTC0_SETALMPRECNT(id)));

      putreg32((uint32_t)(count >> 15), CXD56_RTC0_SETALMPOSTCNT(id));
      putreg32((uint32_t)(count & 0x7fff), CXD56_RTC0_SETALMPRECNT(id));

      while (RTCREG_ALM_BUSY_MASK & getreg32(CXD56_RTC0_ALMOUTEN(id)));

      putreg32(RTCREG_ALM_EN_MASK | RTCREG_ALM_ERREN_MASK,
               CXD56_RTC0_ALMOUTEN(id));

      while (RTCREG_ALM_BUSY_MASK & getreg32(CXD56_RTC0_ALMOUTEN(id)));

      spin_unlock_irqrestore(NULL, flags);

      rtc_dumptime(&alminfo->as_time, "New Alarm time");
      ret = OK;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: cxd56_rtc_cancelalarm
 *
 * Description:
 *   Cancel an alaram.
 *
 * Input Parameters:
 *  alarmid - Identifies the alarm to be cancelled
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int cxd56_rtc_cancelalarm(enum alm_id_e alarmid)
{
  struct alm_cbinfo_s *cbinfo;
  irqstate_t flags;
  int ret = -ENODATA;
  uint32_t mask;

  DEBUGASSERT(RTC_ALARM_LAST > alarmid);

  /* Cancel the alarm in hardware and clear interrupts */

  cbinfo = &g_alarmcb[alarmid];

  if (cbinfo->ac_cb != NULL)
    {
      /* Unset the alarm */

      flags = spin_lock_irqsave(NULL);

      cbinfo->ac_cb = NULL;

      while (RTCREG_ALM_BUSY_MASK & getreg32(CXD56_RTC0_ALMOUTEN(alarmid)));

      putreg32(0, CXD56_RTC0_ALMOUTEN(alarmid));

      while (RTCREG_ALM_BUSY_MASK & getreg32(CXD56_RTC0_ALMOUTEN(alarmid)));

      /* wait until previous alarm request is completed */

      while (RTCREG_ASET_BUSY_MASK &
             getreg32(CXD56_RTC0_SETALMPRECNT(alarmid)));

      /* clear the alarm counter */

      putreg32(0, CXD56_RTC0_SETALMPOSTCNT(alarmid));
      putreg32(0, CXD56_RTC0_SETALMPRECNT(alarmid));

      while (RTCREG_ASET_BUSY_MASK &
             getreg32(CXD56_RTC0_SETALMPRECNT(alarmid)));

      /* wait until the interrupt flag is clear */

      mask = RTCREG_ALM0_ERR_FLAG_MASK | RTCREG_ALM0_FLAG_MASK;
      mask <<= alarmid;

      while (mask & getreg32(CXD56_RTC0_ALMFLG))
        {
          putreg32(mask, CXD56_RTC0_ALMCLR);
        }

      spin_unlock_irqrestore(NULL, flags);

      ret = OK;
    }

  return ret;
}
#endif
