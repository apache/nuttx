/****************************************************************************
 * boards/arm/nrf91/thingy91/src/nrf91_rgbled.c
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
#include <string.h>
#include <nuttx/debug.h>

#include <nuttx/timers/pwm.h>
#include <nuttx/leds/rgbled.h>
#include <arch/board/board.h>

#include "nrf91_pwm.h"
#include "thingy91.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf91_rgbled_register
 *
 * Description:
 *   Bring up one PWM instance (channels 0-2) and register it as an RGB LED
 *   device at 'path'.
 *
 ****************************************************************************/

static int nrf91_rgbled_register(int pwm, const char *path)
{
  struct pwm_lowerhalf_s *led = NULL;
  struct pwm_info_s       info;
  int                     ret;

  led = nrf91_pwminitialize(pwm);
  if (!led)
    {
      lederr("ERROR: Failed to get PWM%d lower half\n", pwm);
      return -ENODEV;
    }

  /* A single PWM instance drives all three colour channels */

  memset(&info, 0, sizeof(info));
  info.frequency = 100;
  info.channels[0].duty = 0;
  info.channels[0].channel = 1;
  info.channels[1].duty = 0;
  info.channels[1].channel = 2;
  info.channels[2].duty = 0;
  info.channels[2].channel = 3;

  led->ops->setup(led);
  led->ops->start(led, &info);

  ret = rgbled_register(path, led, led, led, 1, 2, 3);
  if (ret < 0)
    {
      lederr("ERROR: rgbled_register(%s) failed: %d\n", path, ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf91_rgbled_initialize
 *
 * Description:
 *   Register the on-board RGB LEDs: the lightwell LED (PWM0, P0.29/30/31) as
 *   /dev/rgbled0 and, when PWM1 is enabled, the sense LED (PWM1, P0.0/1/2)
 *   as /dev/rgbled1.
 *
 ****************************************************************************/

int nrf91_rgbled_initialize(void)
{
  static bool initialized = false;
  int         ret;

  if (initialized)
    {
      return OK;
    }

  /* The Thingy:91 has two RGB LEDs, each driven by one PWM instance using
   * its channels 0-2: PWM0 -> lightwell LED (/dev/rgbled0),
   * PWM1 -> sense LED (/dev/rgbled1).
   */

#if defined(CONFIG_NRF91_PWM0_CH0) && defined(CONFIG_NRF91_PWM0_CH1) && \
    defined(CONFIG_NRF91_PWM0_CH2)
  /* Lightwell RGB LED */

  ret = nrf91_rgbled_register(0, "/dev/rgbled0");
  if (ret < 0)
    {
      return ret;
    }
#endif

#if defined(CONFIG_NRF91_PWM1_CH0) && defined(CONFIG_NRF91_PWM1_CH1) && \
    defined(CONFIG_NRF91_PWM1_CH2)
  /* Sense RGB LED */

  ret = nrf91_rgbled_register(1, "/dev/rgbled1");
  if (ret < 0)
    {
      return ret;
    }
#endif

  initialized = true;
  return OK;
}
