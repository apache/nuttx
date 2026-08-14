/****************************************************************************
 * boards/arm/stm32u0/stm32u083c-dk/src/stm32_djoystick.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <unistd.h>
#include <nuttx/signal.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>

#include <nuttx/debug.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mutex.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/input/djoystick.h>

#include "stm32u083c-dk.h"

#if defined(CONFIG_INPUT_DJOYSTICK) && defined(CONFIG_ADC)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SCHED_WORKQUEUE
#  error "The joystick driver requires CONFIG_SCHED_WORKQUEUE"
#endif

/* The 4-direction joystick with selection is connected to a resistor
 * ladder on PC2 (ADC1 IN2).  The thresholds below decode the 12-bit
 * conversion result into joystick positions (values from the ST BSP).
 */

#define JOY_ADC_DEVPATH    "/dev/adc0"
#define JOY_ADC_CHANNEL    2
#define JOY_ADC_NCHANNELS  1

#define JOY_SEL_MAX        10
#define JOY_LEFT_MIN       709
#define JOY_LEFT_MAX       955
#define JOY_DOWN_MIN       1514
#define JOY_DOWN_MAX       1762
#define JOY_UP_MIN         2370
#define JOY_UP_MAX         2618
#define JOY_RIGHT_MIN      3000
#define JOY_RIGHT_MAX      3312

/* Joystick polling interval */

#define JOY_POLL_TICKS     MSEC2TICK(10)

/* Bitset of supported joystick discretes */

#define DJOY_SUPPORTED     (DJOY_UP_BIT | DJOY_DOWN_BIT | DJOY_LEFT_BIT | \
                            DJOY_RIGHT_BIT | DJOY_BUTTON_SELECT_BIT)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static djoy_buttonset_t
  djoy_supported(const struct djoy_lowerhalf_s *lower);
static djoy_buttonset_t djoy_sample_raw(void);
static djoy_buttonset_t
  djoy_sample(const struct djoy_lowerhalf_s *lower);
static void djoy_enable(const struct djoy_lowerhalf_s *lower,
                        djoy_buttonset_t press, djoy_buttonset_t release,
                        djoy_interrupt_t handler, void *arg);

static void djoy_worker(void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The joystick ADC device file and access serialization */

static struct file g_adcfile;
static mutex_t g_adclock = NXMUTEX_INITIALIZER;

/* Current interrupt handler, argument and event sets */

static djoy_interrupt_t g_djoyhandler;
static void *g_djoyarg;
static djoy_buttonset_t g_djoypress;
static djoy_buttonset_t g_djoyrelease;
static djoy_buttonset_t g_djoysample;
static bool g_djoyfirst;

/* Joystick polling work */

static struct work_s g_djoywork;

/* This is the discrete joystick lower half driver interface */

static const struct djoy_lowerhalf_s g_djoylower =
{
  .dl_supported  = djoy_supported,
  .dl_sample     = djoy_sample,
  .dl_enable     = djoy_enable,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: djoy_supported
 *
 * Description:
 *   Return the set of buttons supported on the discrete joystick device
 *
 ****************************************************************************/

static djoy_buttonset_t
  djoy_supported(const struct djoy_lowerhalf_s *lower)
{
  iinfo("Supported: %02x\n", DJOY_SUPPORTED);
  return (djoy_buttonset_t)DJOY_SUPPORTED;
}

/****************************************************************************
 * Name: djoy_decode
 *
 * Description:
 *   Decode the joystick ADC value into a set of joystick discretes.
 *
 ****************************************************************************/

static djoy_buttonset_t djoy_decode(int32_t data)
{
  if (data < JOY_SEL_MAX)
    {
      return DJOY_BUTTON_SELECT_BIT;
    }
  else if (data >= JOY_LEFT_MIN && data <= JOY_LEFT_MAX)
    {
      return DJOY_LEFT_BIT;
    }
  else if (data >= JOY_DOWN_MIN && data <= JOY_DOWN_MAX)
    {
      return DJOY_DOWN_BIT;
    }
  else if (data >= JOY_UP_MIN && data <= JOY_UP_MAX)
    {
      return DJOY_UP_BIT;
    }
  else if (data >= JOY_RIGHT_MIN && data <= JOY_RIGHT_MAX)
    {
      return DJOY_RIGHT_BIT;
    }

  return 0;
}

/****************************************************************************
 * Name: djoy_sample_raw
 *
 * Description:
 *   Sample the joystick ADC channel and decode the joystick state.  The
 *   ADC device access is serialized since the joystick poll worker and
 *   readers of other ADC channels can run concurrently.
 *
 ****************************************************************************/

static djoy_buttonset_t djoy_sample_raw(void)
{
  struct adc_msg_s adcmsg[JOY_ADC_NCHANNELS];
  djoy_buttonset_t sample = 0;
  ssize_t nread;
  int i;
  int ret;

  ret = nxmutex_lock(&g_adclock);
  if (ret < 0)
    {
      return 0;
    }

  /* Drain any stale samples so that the channel data is not shifted
   * against this conversion sequence.
   */

  while (file_read(&g_adcfile, adcmsg, sizeof(adcmsg)) > 0);

  /* Trigger the ADC conversion of all configured channels */

  ret = file_ioctl(&g_adcfile, ANIOC_TRIGGER, 0);
  if (ret < 0)
    {
      ierr("ERROR: ANIOC_TRIGGER failed: %d\n", ret);
      goto out;
    }

  /* Collect the fresh samples.  The device is opened in non-blocking
   * mode, so poll for the completed conversions.
   */

  nread = 0;
  for (i = 0; i < 10 && nread <= 0; i++)
    {
      nread = file_read(&g_adcfile, adcmsg, sizeof(adcmsg));
      if (nread <= 0)
        {
          nxsig_usleep(1000);
        }
    }

  if (nread < 0)
    {
      if (nread != -EAGAIN && nread != -EINTR)
        {
          ierr("ERROR: read failed: %d\n", (int)nread);
        }

      goto out;
    }

  /* Find the joystick channel and decode it */

  for (i = 0; i < nread / (ssize_t)sizeof(struct adc_msg_s); i++)
    {
      if (adcmsg[i].am_channel == JOY_ADC_CHANNEL)
        {
          sample = djoy_decode(adcmsg[i].am_data);
          break;
        }
    }

out:
  nxmutex_unlock(&g_adclock);
  return sample;
}

/****************************************************************************
 * Name: djoy_sample
 *
 * Description:
 *   Return the current state of all discrete joystick buttons.  While the
 *   poll worker is running the last polled state is returned so that
 *   driver reads do not compete with the worker for the ADC device.
 *
 ****************************************************************************/

static djoy_buttonset_t djoy_sample(const struct djoy_lowerhalf_s *lower)
{
  if (g_djoyhandler != NULL)
    {
      return g_djoysample;
    }

  return djoy_sample_raw();
}

/****************************************************************************
 * Name: djoy_worker
 *
 * Description:
 *   Periodically sample the joystick state and notify the upper half
 *   driver when a joystick event of interest occurs.
 *
 ****************************************************************************/

static void djoy_worker(void *arg)
{
  djoy_buttonset_t sample;

  if (g_djoyhandler == NULL)
    {
      return;
    }

  sample = djoy_sample_raw();
  if (g_djoyfirst)
    {
      /* Just establish the joystick state baseline after enable */

      g_djoyfirst  = false;
      g_djoysample = sample;
    }
  else if (((g_djoypress & sample & ~g_djoysample) != 0) ||
           ((g_djoyrelease & g_djoysample & ~sample) != 0))
    {
      g_djoysample = sample;
      g_djoyhandler(&g_djoylower, g_djoyarg);
    }
  else
    {
      g_djoysample = sample;
    }

  work_queue(LPWORK, &g_djoywork, djoy_worker, NULL, JOY_POLL_TICKS);
}

/****************************************************************************
 * Name: djoy_enable
 *
 * Description:
 *   Enable joystick event notifications.  There are no interrupt sources
 *   available for the ADC-based joystick, so the joystick state is polled
 *   periodically from the low priority work queue.
 *
 *   NOTE: the upper half driver can call this function with a critical
 *   section held, so it must not block.  The ADC is accessed only from
 *   the poll worker.
 *
 ****************************************************************************/

static void djoy_enable(const struct djoy_lowerhalf_s *lower,
                        djoy_buttonset_t press, djoy_buttonset_t release,
                        djoy_interrupt_t handler, void *arg)
{
  work_cancel(LPWORK, &g_djoywork);

  g_djoypress   = press;
  g_djoyrelease = release;
  g_djoyhandler = handler;
  g_djoyarg     = arg;

  if (handler != NULL)
    {
      g_djoyfirst = true;
      work_queue(LPWORK, &g_djoywork, djoy_worker, NULL, 0);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_djoy_initialize
 *
 * Description:
 *   Initialize and register the discrete joystick driver
 *
 ****************************************************************************/

int stm32_djoy_initialize(void)
{
  int ret;

  /* Open the joystick ADC device.  The ADC driver must be registered
   * before this function is called.
   */

  ret = file_open(&g_adcfile, JOY_ADC_DEVPATH, O_RDONLY | O_NONBLOCK);
  if (ret < 0)
    {
      ierr("ERROR: Failed to open %s: %d\n", JOY_ADC_DEVPATH, ret);
      return ret;
    }

  /* Register the joystick device as /dev/djoy0 */

  ret = djoy_register("/dev/djoy0", &g_djoylower);
  if (ret < 0)
    {
      ierr("ERROR: djoy_register failed: %d\n", ret);
      file_close(&g_adcfile);
    }

  return ret;
}

#endif /* CONFIG_INPUT_DJOYSTICK && CONFIG_ADC */
