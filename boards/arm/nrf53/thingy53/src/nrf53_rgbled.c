/****************************************************************************
 * boards/arm/nrf53/thingy53/src/nrf53_rgbled.c
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
#include <nuttx/debug.h>

#include <nuttx/timers/pwm.h>
#include <nuttx/leds/rgbled.h>
#include <arch/board/board.h>

#include "nrf53_pwm.h"
#include "thingy53.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PWM0 channels 0, 1 and 2 used */

#  if !defined(CONFIG_NRF53_PWM0_CH0) || !defined(CONFIG_NRF53_PWM0_CH1) || \
      !defined(CONFIG_NRF53_PWM0_CH2)
#    error Invalid configuration for RGBLED driver
#  endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_rgbled_initialize
 *
 * Description:
 *   Configure the RGB LED.
 *
 ****************************************************************************/

int nrf53_rgbled_initialize(void)
{
  static bool             initialized = false;
  struct pwm_lowerhalf_s *ledr        = NULL;
  struct pwm_lowerhalf_s *ledg        = NULL;
  struct pwm_lowerhalf_s *ledb        = NULL;
  struct pwm_info_s       info;
  int                     ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* Call nrf53_pwminitialize() to get an instance of the PWM interface */

      ledr = nrf53_pwminitialize(0);
      if (!ledr)
        {
          lederr("ERROR: Failed to get the NRF53 PWM lower half to LEDR\n");
          return -ENODEV;
        }

      /* Define frequency and duty cycle */

      ledg = ledr;
      ledb = ledr;

      info.frequency = 100;
      info.channels[0].duty = 0;
      info.channels[1].duty = 0;
      info.channels[2].duty = 0;

      /* Initialize LED R */

      ledr->ops->setup(ledr);
      ledr->ops->start(ledr, &info);

      /* Register the RGB LED diver at "/dev/rgbled0" */

      ret = rgbled_register("/dev/rgbled0", ledr, ledg, ledb, 1, 2, 3);
      if (ret < 0)
        {
          lederr("ERROR: rgbled_register failed: %d\n", ret);
          return ret;
        }

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}
