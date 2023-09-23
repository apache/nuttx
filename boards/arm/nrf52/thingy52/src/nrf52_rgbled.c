/****************************************************************************
 * boards/arm/nrf52/thingy52/src/nrf52_rgbled.c
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

#include <sys/types.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/timers/pwm.h>
#include <nuttx/leds/rgbled.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <arch/board/board.h>

#include "thingy52.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Lightwell RGB LED on SX1509 LED driver outputs */

#define RGBLED_R_PIN (7)
#define RGBLED_G_PIN (5)
#define RGBLED_B_PIN (6)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int nrf52_rgb_setup(FAR struct pwm_lowerhalf_s *dev);
static int nrf52_rgb_shutdown(FAR struct pwm_lowerhalf_s *dev);
static int nrf52_rgb_start(FAR struct pwm_lowerhalf_s *dev,
                           FAR const struct pwm_info_s *info);
static int nrf52_rgb_stop(FAR struct pwm_lowerhalf_s *dev);
static int nrf52_rgb_ioctl(FAR struct pwm_lowerhalf_s *dev, int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The RGB LED is not driven by a real PWM peripheral - the color intensity
 * is set through the io-expander PWM operation, one channel per lightwell
 * pin.
 */

static const struct pwm_ops_s g_rgb_ops =
{
  .setup    = nrf52_rgb_setup,
  .shutdown = nrf52_rgb_shutdown,
  .start    = nrf52_rgb_start,
  .stop     = nrf52_rgb_stop,
  .ioctl    = nrf52_rgb_ioctl,
};

static struct pwm_lowerhalf_s g_rgb_dev =
{
  .ops = &g_rgb_ops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_rgb_setup
 ****************************************************************************/

static int nrf52_rgb_setup(FAR struct pwm_lowerhalf_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: nrf52_rgb_shutdown
 ****************************************************************************/

static int nrf52_rgb_shutdown(FAR struct pwm_lowerhalf_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: nrf52_rgb_start
 *
 * Description:
 *   Apply the requested duty cycles.  Each channel number is the SX1509 pin
 *   of one lightwell color.
 *
 ****************************************************************************/

static int nrf52_rgb_start(FAR struct pwm_lowerhalf_s *dev,
                           FAR const struct pwm_info_s *info)
{
  FAR struct ioexpander_dev_s *ioe = nrf52_sx1509_get();
  int pin;
  int ret;
  int i;

  if (ioe == NULL)
    {
      return -ENODEV;
    }

  for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
    {
      pin = info->channels[i].channel;
      if (pin <= 0)
        {
          continue;
        }

      ret = IOEXP_SETPWM(ioe, pin, info->channels[i].duty);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: nrf52_rgb_stop
 ****************************************************************************/

static int nrf52_rgb_stop(FAR struct pwm_lowerhalf_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: nrf52_rgb_ioctl
 ****************************************************************************/

static int nrf52_rgb_ioctl(FAR struct pwm_lowerhalf_s *dev, int cmd,
                           unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_rgbled_initialize
 *
 * Description:
 *   Register the lightwell RGB LED as /dev/rgbled0.  Must be called after
 *   the SX1509 io expander has been initialized.
 *
 ****************************************************************************/

int nrf52_rgbled_initialize(void)
{
  int ret;

  ret = rgbled_register("/dev/rgbled0", &g_rgb_dev, &g_rgb_dev, &g_rgb_dev,
                        RGBLED_R_PIN, RGBLED_G_PIN, RGBLED_B_PIN);
  if (ret < 0)
    {
      lederr("ERROR: rgbled_register failed: %d\n", ret);
      return ret;
    }

  return OK;
}
