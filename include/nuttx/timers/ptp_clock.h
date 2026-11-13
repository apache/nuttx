/****************************************************************************
 * include/nuttx/timers/ptp_clock.h
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

#ifndef __INCLUDE_NUTTX_TIMERS_PTP_CLOCK_H
#define __INCLUDE_NUTTX_TIMERS_PTP_CLOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <sys/timex.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/clock.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PTP_CLOCK_SETTIME        _PTPIOC(0x1)
#define PTP_CLOCK_GETTIME        _PTPIOC(0x2)
#define PTP_CLOCK_GETRES         _PTPIOC(0x3)
#define PTP_CLOCK_ADJTIME        _PTPIOC(0x4)

#define PTP_CLOCK_GETCAPS        _PTPIOC(0x5)
#define PTP_SYS_OFFSET           _PTPIOC(0x6)
#define PTP_SYS_OFFSET_PRECISE   _PTPIOC(0x7)
#define PTP_SYS_OFFSET_EXTENDED  _PTPIOC(0x8)

#define PTP_CLOCK_GETCAPS2       _PTPIOC(0x9)
#define PTP_SYS_OFFSET2          _PTPIOC(0xa)
#define PTP_SYS_OFFSET_PRECISE2  _PTPIOC(0xb)
#define PTP_SYS_OFFSET_EXTENDED2 _PTPIOC(0xc)

#define PTP_CLOCK_SETSTATS       _PTPIOC(0xd)
#define PTP_CLOCK_GETSTATS       _PTPIOC(0xe)

/* External timestamp (EXTTS), periodic output (PEROUT) and PPS requests.
 * These follow the Linux PTP hardware clock API: once an external
 * timestamp channel is enabled with PTP_EXTTS_REQUEST, every edge seen on
 * that input is time stamped with the PTP clock and can be read() from
 * the /dev/ptpX device as a struct ptp_extts_event.
 */

#define PTP_EXTTS_REQUEST        _PTPIOC(0xf)
#define PTP_PEROUT_REQUEST       _PTPIOC(0x10)
#define PTP_ENABLE_PPS           _PTPIOC(0x11)
#define PTP_EXTTS_REQUEST2       _PTPIOC(0x12)
#define PTP_PEROUT_REQUEST2      _PTPIOC(0x13)
#define PTP_ENABLE_PPS2          _PTPIOC(0x14)

/* Bits of the ptp_extts_request.flags field */

#define PTP_ENABLE_FEATURE       (1 << 0)
#define PTP_RISING_EDGE          (1 << 1)
#define PTP_FALLING_EDGE         (1 << 2)
#define PTP_STRICT_FLAGS         (1 << 3)
#define PTP_EXTTS_EDGES          (PTP_RISING_EDGE | PTP_FALLING_EDGE)

/* Bits of the ptp_perout_request.flags field */

#define PTP_PEROUT_ONE_SHOT      (1 << 0)
#define PTP_PEROUT_DUTY_CYCLE    (1 << 1)
#define PTP_PEROUT_PHASE         (1 << 2)

/* Request types passed to the lower half enable() method */

#define PTP_CLK_REQ_EXTTS        0
#define PTP_CLK_REQ_PEROUT       1
#define PTP_CLK_REQ_PPS          2

/* Maximum allowed offset measurement samples. */

#define PTP_MAX_SAMPLES          25

#define PTP_STATS_NUM            10

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/*  PTP statistics info */

struct ptp_statistics_s
{
  struct timespec origin_time[PTP_STATS_NUM];
  struct timespec correction_time[PTP_STATS_NUM];
  int64_t delta_ns[PTP_STATS_NUM];
  int64_t adjustment_ns[PTP_STATS_NUM];
  int32_t drift_ppb[PTP_STATS_NUM];
  int32_t follow_up_error;
  int32_t sync_error;
  int32_t announce;
  int32_t sync;
  int32_t follow_up;
  int32_t delay_resp;
  int32_t delay_req;
  int32_t unknown;
};

/* struct system_device_crosststamp - system/device cross-timestamp
 * (synchronized capture)
 */

struct system_device_crosststamp
{
  struct timespec device;   /* Device time */
  struct timespec realtime; /* Realtime simultaneous with device time */
  struct timespec monoraw;  /* Monotonic raw simultaneous with device time */
};

/* struct ptp_clock_time - represents a time value
 *
 * The sign of the seconds field applies to the whole value. The
 * nanoseconds field is always unsigned. The reserved field is
 * included for sub-nanosecond resolution, should the demand for
 * this ever appear.
 */

struct ptp_clock_time
{
  int64_t  sec;
  uint32_t nsec;
  uint32_t reserved;
};

struct ptp_clock_caps
{
  int max_adj;            /* Maximum frequency adjustment in parts per billion. */
  int cross_timestamping; /* Whether the clock supports precise system-device cross timestamps */
  int adjust_phase;       /* Whether the clock supports phase adjustment */
  int n_ext_ts;           /* Number of external time stamp channels */
  int n_per_out;          /* Number of programmable periodic signals */
  int pps;                /* Whether the clock supports a PPS callback */
};

/* struct ptp_extts_request - enable or disable an external timestamp
 * channel (PTP_EXTTS_REQUEST).
 */

struct ptp_extts_request
{
  unsigned int index;     /* Which channel to configure */
  unsigned int flags;     /* Bit field of PTP_ENABLE_FEATURE and edges */
  unsigned int rsv[2];    /* Reserved for future use */
};

/* struct ptp_extts_event - an external timestamp, returned by read() */

struct ptp_extts_event
{
  struct ptp_clock_time t; /* Time when the event occurred */
  unsigned int index;      /* Which channel produced the event */
  unsigned int flags;      /* Edge that produced the event, if known */
  unsigned int rsv[2];     /* Reserved for future use */
};

/* struct ptp_perout_request - configure a periodic output
 * (PTP_PEROUT_REQUEST).
 */

struct ptp_perout_request
{
  struct ptp_clock_time start;   /* Absolute start time, or phase offset
                                  * with PTP_PEROUT_PHASE */
  struct ptp_clock_time period;  /* Desired period, zero means disable */
  unsigned int index;            /* Which channel to configure */
  unsigned int flags;            /* PTP_PEROUT_xxx flags */
  struct ptp_clock_time on;      /* "On" time (PTP_PEROUT_DUTY_CYCLE) */
};

/* struct ptp_clock_request - request passed to the lower half enable()
 * method.
 */

struct ptp_clock_request
{
  int type;                         /* PTP_CLK_REQ_xxx */
  struct ptp_extts_request extts;   /* Valid for PTP_CLK_REQ_EXTTS */
  struct ptp_perout_request perout; /* Valid for PTP_CLK_REQ_PEROUT */
};

struct ptp_sys_offset
{
  unsigned int n_samples; /* Desired number of measurements. */
  unsigned int rsv[3];    /* Reserved for future use. */

  /* Array of interleaved system/phc time stamps. The kernel
   * will provide 2*n_samples + 1 time stamps, with the last
   * one as a system time stamp.
   */

  struct ptp_clock_time ts[2 * PTP_MAX_SAMPLES + 1];
};

struct ptp_sys_offset_extended
{
  unsigned int n_samples; /* Desired number of measurements. */
  unsigned int rsv[3];    /* Reserved for future use. */

  /* Array of [system, phc, system] time stamps. The kernel will provide
   * 3*n_samples time stamps.
   */

  struct ptp_clock_time ts[PTP_MAX_SAMPLES][3];
};

struct ptp_sys_offset_precise
{
  struct ptp_clock_time device;
  struct ptp_clock_time sys_realtime;
  struct ptp_clock_time sys_monoraw;
  unsigned int rsv[4];
};

/* struct ptp_system_timestamp - system time corresponding to a
 * PHC timestamp.
 */

struct ptp_system_timestamp
{
  struct timespec pre_ts;
  struct timespec post_ts;
};

struct ptp_lowerhalf_s;
struct ptp_ops_s
{
  /**************************************************************************
   * Name:  adjfine
   *
   * Description:
   *   Adjusts the frequency of the hardware clock.
   *
   * Input Parameters:
   *   lower - The instance of lower half ptp driver
   *   ppb   - Desired frequency offset from nominal frequency in parts
   *           per billion.
   *
   * Returned Value:
   *   Zero (OK) or positive on success; a negated errno value on failure.
   *
   **************************************************************************/

  CODE int (*adjfine)(FAR struct ptp_lowerhalf_s *lower, long ppb);

  /**************************************************************************
   * Name: adjphase
   *
   * Description:
   *   Adjusts the phase offset of the hardware clock.
   *
   * Input Parameters:
   *   lower - The instance of lower half ptp driver.
   *   phase - Desired change in nanoseconds.
   *
   * Returned Value:
   *   Zero (OK) or positive on success; a negated errno value on failure.
   *
   **************************************************************************/

  CODE int (*adjphase)(FAR struct ptp_lowerhalf_s *lower, int32_t phase);

  /**************************************************************************
   * Name: adjtime
   *
   * Description:
   *   Shifts the time of the hardware clock.
   *
   * Input Parameters:
   *   lower - The instance of lower half ptp driver.
   *   delta - Desired change in nanoseconds.
   *
   * Returned Value:
   *   Zero (OK) or positive on success; a negated errno value on failure.
   *
   **************************************************************************/

  CODE int (*adjtime)(FAR struct ptp_lowerhalf_s *lower, int64_t delta);

  /**************************************************************************
   * Name: gettime
   *
   * Description:
   *   Reads the current time from the hardware clock and optionally also
   *   also the system clock. The first reading is made right before reading
   *   the lowest bits of the PHC timestamp and the second reading
   *   immediately follows that.
   *
   * Input Parameters:
   *   lower - The instance of lower half ptp driver.
   *   ts    - Holds the PHC timestamp.
   *   sts   - If not NULL, it holds a pair of timestamps from the
   *           system clock.
   *
   * Returned Value:
   *   Zero (OK) or positive on success; a negated errno value on failure.
   *
   **************************************************************************/

  CODE int (*gettime)(FAR struct ptp_lowerhalf_s *lower,
                      FAR struct timespec *ts,
                      FAR struct ptp_system_timestamp *sts);

  /**************************************************************************
   * Name: getcrosststamp
   *
   * Description:
   *   Reads the current time from the hardware clock and system clock
   *   simultaneously.
   *
   * Input Parameters:
   *   lower - The instance of lower half ptp driver.
   *   cts   - Contains timestamp (device,system) pair, where system time is
   *           realtime and monotonic.
   *
   * Returned Value:
   *   Zero (OK) or positive on success; a negated errno value on failure.
   *
   **************************************************************************/

  CODE int (*getcrosststamp)(FAR struct ptp_lowerhalf_s *lower,
                             FAR struct system_device_crosststamp *cts);

  /**************************************************************************
   * Name: settime
   *
   * Description:
   *   Set the current time on the hardware clock.
   *
   * Input Parameters:
   *   lower  - The instance of lower half ptp driver
   *   ts     - Time value to set.
   *
   * Returned Value:
   *   Zero (OK) or positive on success; a negated errno value on failure.
   *
   **************************************************************************/

  CODE int (*settime)(FAR struct ptp_lowerhalf_s *lower,
                      FAR const struct timespec *ts);

  /**************************************************************************
   * Name: getres
   *
   * Description:
   *   Finds the resolution (precision) of the specified clock, and, if res
   *   is non-NULL, stores it in the struct timespec pointed to by res.
   *
   * Input Parameters:
   *   pc    - The instance of lower half ptp driver.
   *   res   - Holds the resolution.
   *
   * Returned Value:
   *   Zero (OK) or positive on success; a negated errno value on failure.
   *
   **************************************************************************/

  CODE int (*getres)(FAR struct ptp_lowerhalf_s *lower,
                     FAR struct timespec *res);

  /**************************************************************************
   * Name: control
   *
   * Description:
   *   Performs platform-specific operations.
   *
   * Input Parameters:
   *   lower - The instance of lower half ptp driver.
   *   cmd   - The command to perform.
   *   arg   - The argument of the command.
   *
   * Returned Value:
   *   Zero (OK) or positive on success; a negated errno value on failure.
   *   -ENOTTY - The cmd don't support.
   *
   **************************************************************************/

  CODE int (*control)(FAR struct ptp_lowerhalf_s *lower,
                      int cmd, unsigned long arg);

  /**************************************************************************
   * Name: enable
   *
   * Description:
   *   Enables or disables an ancillary feature of the clock: an external
   *   timestamp channel (EXTTS), a periodic output (PEROUT) or the PPS
   *   event.  Time stamps of an enabled EXTTS channel are reported to the
   *   upper half with ptp_clock_event().
   *
   * Input Parameters:
   *   lower - The instance of lower half ptp driver.
   *   rq    - The feature to change and its parameters.
   *   on    - true to enable the feature, false to disable it.
   *
   * Returned Value:
   *   Zero (OK) or positive on success; a negated errno value on failure.
   *   -EOPNOTSUPP - The feature is not supported.
   *
   **************************************************************************/

  CODE int (*enable)(FAR struct ptp_lowerhalf_s *lower,
                     FAR struct ptp_clock_request *rq, bool on);
};

/* The ptp lower half driver interface, describes a PTP hardware
 * clock driver.
 */

struct ptp_lowerhalf_s
{
  FAR const struct ptp_ops_s *ops; /* Lower half driver operations. */
  FAR void *upper;                 /* The upper handle */
  uint8_t n_ext_ts;                /* Number of external time stamp inputs */
  uint8_t n_per_out;               /* Number of periodic outputs */
  bool    pps;                     /* Whether PTP_ENABLE_PPS is supported */
};

/* struct ptp_clock_event - an event reported by the lower half */

struct ptp_clock_event
{
  int type;                        /* PTP_CLK_REQ_EXTTS only for now */
  unsigned int index;              /* Channel that produced the event */
  unsigned int flags;              /* Edge, PTP_RISING/FALLING_EDGE or 0 */
  struct timespec ts;              /* Time stamp in the PTP time scale */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Name: ptp_clockid_to_filep
 *
 * Description:
 *   Convert clockid to struct filep.
 *
 ****************************************************************************/

int ptp_clockid_to_filep(clockid_t clock_id, FAR struct file **filep);

/****************************************************************************
 * Name: ptp_clock_register
 *
 * Description:
 *   This function binds an instance of a "lower half" ptp driver with the
 *   "upper half" ptp device and registers that device so that can be used
 *   by application code.
 *
 * Input Parameters:
 *   lower   - A pointer to an instance of lower half ptp driver. This
 *             instance is bound to the ptp driver and must persists as long
 *             as the driver persists.
 *   mxa_adj - The maximum frequency adjustment in parts per billion.
 *   devno   - The user specifies number of device. ex: /dev/ptpX.
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int ptp_clock_register(FAR struct ptp_lowerhalf_s *lower, int32_t max_adj,
                       int devno);

/****************************************************************************
 * Name: ptp_clock_unregister
 *
 * Description:
 *   This function unregisters character node and releases all resource from
 *   upper half driver. This API corresponds to the ptp_register.
 *
 * Input Parameters:
 *   dev   - A pointer to an instance of lower half ptp driver. This
 *           instance is bound to the ptp driver and must persists as long
 *           as the driver persists.
 *   devno - The user specifies which device of this type, from 0.
 ****************************************************************************/

void ptp_clock_unregister(FAR struct ptp_lowerhalf_s *dev, int devno);

/****************************************************************************
 * Name: ptp_clock_event
 *
 * Description:
 *   Report an event (an external time stamp) to the upper half.  The event
 *   is queued and can be read() from /dev/ptpX by the application.
 *
 *   This function may be called from a normal interrupt handler.  It must
 *   NOT be called from a zero latency (CONFIG_ARCH_HIPRI_INTERRUPT)
 *   handler, which is not allowed to use OS services.
 *
 * Input Parameters:
 *   lower - A pointer to an instance of lower half ptp driver.
 *   event - The event to report.
 *
 ****************************************************************************/

void ptp_clock_event(FAR struct ptp_lowerhalf_s *lower,
                     FAR const struct ptp_clock_event *event);

#if defined(__cplusplus)
}
#endif
#endif /* __INCLUDE_NUTTX_TIMERS_PTP_CLOCK_H */
