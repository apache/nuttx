/****************************************************************************
 * include/nuttx/timers/pulsecount.h
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

#ifndef __INCLUDE_NUTTX_TIMERS_PULSECOUNT_H
#define __INCLUDE_NUTTX_TIMERS_PULSECOUNT_H

/* A pulsecount device generates a finite pulse train with controlled high
 * time, low time, and pulse count.  The driver is split into two parts:
 *
 * 1. An "upper half", generic character driver that provides the common
 *    pulsecount interface to application code.
 * 2. A "lower half", platform-specific driver that implements the timer
 *    programming needed to generate the pulse train.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/clock.h>

#include <stdint.h>
#include <fixedmath.h>

#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL Commands ***********************************************************/

/* PULSECOUNTIOC_SETCHARACTERISTICS - Set the pulse train characteristics.
 *
 *   ioctl argument: A read-only reference to struct pulsecount_info_s.
 *
 * PULSECOUNTIOC_GETCHARACTERISTICS - Get the currently selected pulse train
 *   characteristics.
 *
 *   ioctl argument: A writable reference to struct pulsecount_info_s.
 *
 * PULSECOUNTIOC_START - Start the configured pulse train.  The
 *   PULSECOUNTIOC_SETCHARACTERISTICS command must have previously been sent.
 *   By default this command blocks until the configured pulse count
 *   completes.  That blocking behavior can be overridden by opening the
 *   device with O_NONBLOCK.
 *
 *   ioctl argument: None.
 *
 * PULSECOUNTIOC_STOP - Stop pulse generation and return immediately.
 *
 *   TODO: Support cancelling a blocking PULSECOUNTIOC_START.
 *
 *   ioctl argument: None.
 */

#define PULSECOUNTIOC_SETCHARACTERISTICS _PULSECOUNTIOC(1)
#define PULSECOUNTIOC_GETCHARACTERISTICS _PULSECOUNTIOC(2)
#define PULSECOUNTIOC_START              _PULSECOUNTIOC(3)
#define PULSECOUNTIOC_STOP               _PULSECOUNTIOC(4)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct pulsecount_info_s
{
  uint32_t high_ns;        /* Pulse high time in nanoseconds */
  uint32_t low_ns;         /* Pulse low time in nanoseconds */
  uint32_t count;          /* Number of pulses to generate */
};

/****************************************************************************
 * Public Inline Functions
 ****************************************************************************/

static inline uint64_t
pulsecount_period_ns(FAR const struct pulsecount_info_s *info)
{
  return (uint64_t)info->high_ns + info->low_ns;
}

static inline uint32_t
pulsecount_frequency(FAR const struct pulsecount_info_s *info)
{
  return NSEC_PER_SEC / pulsecount_period_ns(info);
}

static inline ub16_t
pulsecount_duty(FAR const struct pulsecount_info_s *info)
{
  return (ub16_t)(((uint64_t)info->high_ns << 16) /
                  pulsecount_period_ns(info));
}

struct pulsecount_lowerhalf_s;
struct pulsecount_ops_s
{
  /* This method is called when the driver is opened.  The lower half driver
   * should configure and initialize the device so that it is ready for use.
   * It should not generate pulses until the start method is called.
   */

  CODE int (*setup)(FAR struct pulsecount_lowerhalf_s *dev);

  /* This method is called when the driver is closed.  The lower half driver
   * should stop pulse output, free resources, disable the timer, and put the
   * system into the lowest possible power usage state.
   */

  CODE int (*shutdown)(FAR struct pulsecount_lowerhalf_s *dev);

  /* Configure the timer and start the finite pulse train.  The lower half
   * must call pulsecount_expired() with the provided handle after the
   * programmed pulse count completes.
   */

  CODE int (*start)(FAR struct pulsecount_lowerhalf_s *dev,
                    FAR const struct pulsecount_info_s *info,
                    FAR void *handle);

  /* Stop the pulse train and reset the timer resources. */

  CODE int (*stop)(FAR struct pulsecount_lowerhalf_s *dev);

  /* Lower-half logic may support platform-specific ioctl commands. */

  CODE int (*ioctl)(FAR struct pulsecount_lowerhalf_s *dev,
                    int cmd, unsigned long arg);
};

struct pulsecount_lowerhalf_s
{
  /* The first field of this state structure must be a pointer to the
   * pulsecount callback structure.
   */

  FAR const struct pulsecount_ops_s *ops;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

int pulsecount_register(FAR const char *path,
                        FAR struct pulsecount_lowerhalf_s *dev);

/* This callback is called by the lower half after a finite pulse train has
 * completed.  The handle must be the value passed to the lower half start()
 * method.
 */

void pulsecount_expired(FAR void *handle);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_TIMERS_PULSECOUNT_H */
