/****************************************************************************
 * include/nuttx/timers/rtc.h
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

#ifndef __INCLUDE_NUTTX_TIMERS_RTC_H
#define __INCLUDE_NUTTX_TIMERS_RTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdbool.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

#include <nuttx/fs/ioctl.h>

#ifdef CONFIG_RTC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* CONFIG_RTC - Enables general support for a hardware RTC.  Specific
 *   architectures may require other specific settings.
 *
 * CONFIG_RTC_DATETIME - There are two general types of RTC:  (1) A simple
 *   battery backed counter that keeps the time when power is down, and (2)
 *   A full date / time RTC the provides the date and time information, often
 *   in BCD format.  If CONFIG_RTC_DATETIME is selected, it specifies this
 *   second kind of RTC. In this case, the RTC is used to "seed" the normal
 *   NuttX timer and the NuttX system timer provides for higher resolution
 *   time.
 *
 * CONFIG_RTC_HIRES - If CONFIG_RTC_DATETIME not selected, then the simple,
 *   battery backed counter is used.  There are two different implementations
 *   of such simple counters based on the time resolution of the counter:
 *   The typical RTC keeps time to resolution of 1 second, usually
 *   supporting a 32-bit time_t value.  In this case, the RTC is used to
 *   "seed" the normal NuttX timer and the NuttX timer provides for higher
 *   resolution time.
 *
 *   If CONFIG_RTC_HIRES is enabled in the NuttX configuration, then the
 *   RTC provides higher resolution time and completely replaces the system
 *   timer for purpose of date and time.
 *
 * CONFIG_RTC_FREQUENCY - If CONFIG_RTC_HIRES is defined, then the frequency
 *   of the high resolution RTC must be provided.  If CONFIG_RTC_HIRES is
 *   not defined, CONFIG_RTC_FREQUENCY is assumed to be one.
 *
 * CONFIG_RTC_ALARM - Enable if the RTC hardware supports setting of an
 *   alarm.  A callback function will be executed when the alarm goes off
 *
 * CONFIG_RTC_PERIODIC - Enable if the RTC hardware supports setting a
 *  periodic wakeup. A callback function will be executed when the wakeup
 *  happens. This is an experimental feature.
 *
 * CONFIG_RTC_DRIVER - Enable building the upper-half RTC driver
 */

#ifdef CONFIG_RTC_HIRES
#  ifdef CONFIG_RTC_DATETIME
#    error "CONFIG_RTC_HIRES and CONFIG_RTC_DATETIME are both defined"
#  endif
#  ifndef CONFIG_RTC_FREQUENCY
#    error "CONFIG_RTC_FREQUENCY is required for CONFIG_RTC_HIRES"
#  endif
#else
#  ifndef CONFIG_RTC_FREQUENCY
#    define CONFIG_RTC_FREQUENCY 1
#  endif
#  if CONFIG_RTC_FREQUENCY != 1
#    error "The low resolution RTC must have frequency 1Hz"
#  endif
#endif

#if defined(CONFIG_RTC_ALARM) && !defined(CONFIG_RTC_NALARMS)
#  define CONFIG_RTC_NALARMS 1
#endif

/* The remainder of the contain of this header file is only valid if the
 * RTC upper half driver is built.
 */

#ifdef CONFIG_RTC_DRIVER

/* IOCTL Commands ***********************************************************/

/* RTC driver IOCTL commands.  These are Linux compatible command names, not
 * all of these commands are supported by all RTC drivers, however.
 */

/* RTC_RD_TIME returns the current RTC time.
 *
 * Argument: A writeable reference to a struct rtc_time to receive the RTC's
 *           time.
 */

#define RTC_RD_TIME        _RTCIOC(0x0001)

/* RTC_SET_TIME sets the RTC's time
 *
 * Argument: A read-only reference to a struct rtc_time containing the
 *           new time to be set.
 */

#define RTC_SET_TIME       _RTCIOC(0x0002)

/* RTC_HAVE_SET_TIME checks if RTC's time had been set
 *
 * Argument: A writable reference to a bool to receive true/false return
 *           value of the check.
 */

#define RTC_HAVE_SET_TIME  _RTCIOC(0x0003)

/* RTC_SET_ALARM sets the alarm time (for RTCs that support alarms).
 *
 * Argument: A read-only reference to a struct rtc_setalarm_s containing the
 *           new alarm time to be set.
 */

#define RTC_SET_ALARM      _RTCIOC(0x0004)

/* RTC_SET_RELATIVE sets the alarm time relative to the current time.
 *
 * Argument: A read-only reference to a struct rtc_setrelative_s containing
 *           the new relative alarm time to be set.
 */

#define RTC_SET_RELATIVE   _RTCIOC(0x0005)

/* RTC_CANCEL_ALARM cancel the alarm.
 *
 * Argument: An ALARM ID value that indicates which alarm should be canceled.
 */

#define RTC_CANCEL_ALARM   _RTCIOC(0x0006)

/* RTC_RD_ALARM query the alarm.
 *
 * Argument: An ALARM ID value that indicates which alarm should be queried.
 */

#define RTC_RD_ALARM       _RTCIOC(0x0007)

/* RTC_SET_PERIODIC set a periodic wakeup.
 *
 * Argument: A read-only reference to a struct rtc_setperiodic_s containing
 *           the new wakeup period to be set.
 */

#define RTC_SET_PERIODIC     _RTCIOC(0x0008)

/* RTC_CANCEL_PERIODIC cancel the periodic wakeup.
 *
 * Argument: An ID value that indicates which wakeup should be canceled.
 */

#define RTC_CANCEL_PERIODIC  _RTCIOC(0x0009)

/* Architecture-specific RTC IOCTLS should begin at RTC_USER_IOCBASE.  For
 * example:
 *
 *   #define MY_RTC_IOCTL1 _RTCIOC(RTC_USER_IOCBASE);
 *   #define MY_RTC_IOCTL2 _RTCIOC(RTC_USER_IOCBASE + 1);
 *   etc.
 */

#define RTC_USER_IOCBASE     0x000a

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* IOCTL data structures */

/* Broken-out time representation used with RTC IOCTL commands:
 *
 * The fields in this structure have the same meaning and ranges as for the
 * tm structure described in gmtime().  Further, it is REQUIRED that the
 * structure be cast compatible with struct tm! They must be interchangeable.
 */

struct rtc_time
{
  int tm_sec;          /* Seconds (0-61, allows for leap seconds) */
  int tm_min;          /* Minutes (0-59) */
  int tm_hour;         /* Hours (0-23) */
  int tm_mday;         /* Day of the month (1-31) */
  int tm_mon;          /* Month (0-11) */
  int tm_year;         /* Years since 1900 */
  int tm_wday;         /* Day of the week (0-6) (unused) */
  int tm_yday;         /* Day of the year (0-365) (unused) */
  int tm_isdst;        /* Non-0 if daylight savings time is in effect (unused) */
  long tm_gmtoff;      /* Offset from UTC in seconds */
  const char *tm_zone; /* Timezone abbreviation. */
#if defined(CONFIG_RTC_HIRES) || defined(CONFIG_ARCH_HAVE_RTC_SUBSECONDS)
  long tm_nsec;        /* Nanosecond (0-999999999) */
#endif
};

#ifdef CONFIG_RTC_ALARM
/* Structure used with the RTC_RD_ALARM IOCTL command and with
 * rdalarm() method.
 */

struct rtc_rdalarm_s
{
  uint8_t id;                /* Indicates the alarm being queried */
  bool active;               /* Alarm actively timing or disabled */
  struct rtc_time time;      /* Current RTC time (if enabled) */
};

/* Structure used with the RTC_SET_ALARM IOCTL command. */

struct rtc_setalarm_s
{
  uint8_t id;                /* Indicates the alarm to be set */
  pid_t pid;                 /* Identifies task to be notified (0=caller) */
  struct sigevent event;     /* Describe the way a task is to be notified */
  struct rtc_time time;      /* Alarm time */
};

/* Structure used with the RTC_SET_RELATIVE IOCTL command. */

struct rtc_setrelative_s
{
  uint8_t id;                /* Indicates the alarm to be set */
  pid_t pid;                 /* Identifies task to be notified (0=caller) */
  struct sigevent event;     /* Describe the way a task is to be notified */
  time_t reltime;            /* Relative time in seconds */
};

/* Callback type used by the RTC hardware to notify the RTC driver when the
 * alarm expires.
 */

typedef CODE void (*rtc_alarm_callback_t)(FAR void *priv, int alarmid);

/* Structure used with the setalarm method */

struct lower_setalarm_s
{
  uint8_t id;                /* Indicates the alarm to be set */
  rtc_alarm_callback_t cb;   /* Callback when the alarm expires */
  FAR void *priv;            /* Private argument to accompany callback */
  struct rtc_time time;      /* Alarm time */
};

/* Structure used with the setrelative method */

struct lower_setrelative_s
{
  uint8_t id;                /* Indicates the alarm to be set */
  rtc_alarm_callback_t cb;   /* Callback when the alarm expires */
  FAR void *priv;            /* Private argument to accompany callback */
  time_t reltime;            /* Relative time in seconds */
};

/* Structure used with the rdalarm method */

struct lower_rdalarm_s
{
  uint8_t id;                /* Indicates the alarm to be set */
  FAR void *priv;            /* Private argument to accompany callback */
  FAR struct rtc_time *time; /* Queried RTC time pointer */
};
#endif

#ifdef CONFIG_RTC_PERIODIC

/* Structure used with the RTC_SET_PERIODIC IOCTL command. */

struct rtc_setperiodic_s
{
  uint8_t id;                /* Indicates the alarm to be set */
  pid_t pid;                 /* Identifies task to be notified (0=caller) */
  struct sigevent event;     /* Describe the way a task is to be notified */
  struct timespec period;    /* Period between wakeups */
};

/* Callback type used by the RTC hardware to notify the RTC driver when the
 * wakeup period expires.
 */

typedef CODE void (*rtc_wakeup_callback_t)(FAR void *priv, int alarmid);

/* Structure used with the setperiodic method */

struct lower_setperiodic_s
{
  uint8_t id;                /* Indicates the wakeup to be set */
  rtc_wakeup_callback_t cb;  /* Callback when the wakeup expires */
  FAR void *priv;            /* Private argument to accompany callback */
  struct timespec period;    /* Period between wakeups */
};

#endif

/* The RTC driver is implemented as a common, upper-half character driver
 * that provides the RTC driver structure and a lower-level, hardware
 * specific implementation that performs the actual RTC operations.
 *
 * struct rtc_ops_s defines the callable interface from the common upper-
 * half driver into lower half implementation.  Each method in this
 * structure corresponds to one of the RTC IOCTL commands.  All IOCTL
 * command logic is implemented in the lower-half driver.
 *
 * A NULL value should be provided for any unsupported  methods.
 */

struct rtc_lowerhalf_s;
struct rtc_ops_s
{
  /* rdtime() returns the current RTC time. */

  CODE int (*rdtime)(FAR struct rtc_lowerhalf_s *lower,
                     FAR struct rtc_time *rtctime);

  /* settime sets the RTC's time */

  CODE int (*settime)(FAR struct rtc_lowerhalf_s *lower,
                      FAR const struct rtc_time *rtctime);

  /* havesettime checks if RTC time have been set */

  CODE bool (*havesettime)(FAR struct rtc_lowerhalf_s *lower);

#ifdef CONFIG_RTC_ALARM
  /* setalarm sets up a new alarm. */

  CODE int (*setalarm)(FAR struct rtc_lowerhalf_s *lower,
                       FAR const struct lower_setalarm_s *alarminfo);

  /* setalarm sets up a new alarm relative to the current time. */

  CODE int (*setrelative)(FAR struct rtc_lowerhalf_s *lower,
                          FAR const struct lower_setrelative_s *alarminfo);

  /* cancelalarm cancels the current alarm. */

  CODE int (*cancelalarm)(FAR struct rtc_lowerhalf_s *lower, int alarmid);

  /* rdalarm query the current alarm. */

  CODE int (*rdalarm)(FAR struct rtc_lowerhalf_s *lower,
                      FAR struct lower_rdalarm_s *alarminfo);
#endif

#ifdef CONFIG_RTC_PERIODIC
  /* setperiodic sets up a new periodic wakeup. */

  CODE int (*setperiodic)(FAR struct rtc_lowerhalf_s *lower,
                          FAR const struct lower_setperiodic_s *alarminfo);

  /* cancelperiodic cancels the current periodic wakeup. */

  CODE int (*cancelperiodic)(FAR struct rtc_lowerhalf_s *lower, int alarmid);
#endif

#ifdef CONFIG_RTC_IOCTL
  /* Support for architecture-specific RTC operations */

  CODE int (*ioctl)(FAR struct rtc_lowerhalf_s *lower, int cmd,
                    unsigned long arg);
#endif

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  /* The driver has been unlinked and there are no further open references
   * to the driver.
   */

  CODE int (*destroy)(FAR struct rtc_lowerhalf_s *lower);
#endif
};

/* When the RTC driver is instantiated, a reference to struct
 * rtc_lowerhalf_s is passed to the initialization function and bound to
 * the driver.  The actual content of the state structure used by different
 * lower half drivers will vary from implementation to implementation.  But
 * each implementation must be cast compatible with this definition of
 * struct rtc_lowerhalf_s that is understood by the upper half driver.
 */

struct rtc_lowerhalf_s
{
  /* This is the contained reference to the read-only, lower-half
   * operations vtable (which may lie in FLASH or ROM)
   */

  FAR const struct rtc_ops_s *ops;

  /* Data following this can vary from RTC driver-to-driver */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rtc_initialize
 *
 * Description:
 *   Create an RTC driver by binding to the lower half RTC driver instance
 *   provided to this function.  The resulting RTC driver will be registered
 *   at /dev/rtcN where N is the minor number provided to this function.
 *
 ****************************************************************************/

#ifdef __KERNEL__
int rtc_initialize(int minor, FAR struct rtc_lowerhalf_s *lower);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_RTC_DRIVER */
#endif /* CONFIG_RTC */
#endif /* __INCLUDE_NUTTX_TIMERS_RTC_H */
