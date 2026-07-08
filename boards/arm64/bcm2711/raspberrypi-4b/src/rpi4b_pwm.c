/****************************************************************************
 * boards/arm64/bcm2711/raspberrypi-4b/src/rpi4b_pwm.c
 *
 * Contributed by: Matteo Golin <linguini@apache.org>
 *
 * SPDX-License-Identifer: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements. See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership. The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
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
#include <nuttx/debug.h>

#include <syslog.h>
#include <errno.h>

#include <nuttx/timers/pwm.h>
#include <nuttx/timers/oneshot.h>

#ifdef CONFIG_RPI4B_AUDIOJACK
#include <nuttx/audio/tone.h>
#endif

#include "bcm2711_pwm.h"
#include "bcm2711_oneshot.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpi4b_pwm_initialize
 *
 * Description:
 *   Initialize PWM interfaces on the RPi4B.
 *   Also initializes PWM-audio driver if that was selected
 *
 ****************************************************************************/

int rpi4b_pwm_initialize(void)
{
  int ret = 0;
  int err = 0;
#ifdef CONFIG_BCM2711_PWM0
  struct pwm_lowerhalf_s *pwm0;
#endif
#ifdef CONFIG_BCM2711_PWM1
  struct pwm_lowerhalf_s *pwm1;
#endif
#ifdef CONFIG_RPI4B_AUDIOJACK
  struct oneshot_lowerhalf_s *oneshot;
#endif

#ifdef CONFIG_BCM2711_PWM0
  pwm0 = bcm2711_pwminitialize(0);
  if (pwm0 == NULL)
    {
      syslog(LOG_ERR, "Couldn't initialize PWM0.");
      err = -EAGAIN;
      ret = err;
    }

  /* Only try to register if initialization was successful */

  if (err == 0)
    {
      err = pwm_register("/dev/pwm0", pwm0);
      if (err < 0)
        {
          syslog(LOG_ERR, "Couldn't register PWM0: %d\n", err);
          ret = err;
        }
    }
#endif /* CONFIG_BCM2711_PWM0 */

#ifdef CONFIG_BCM2711_PWM1
  pwm1 = bcm2711_pwminitialize(1);
  if (pwm1 == NULL)
    {
      syslog(LOG_ERR, "Couldn't initialize PWM1.");
      err = -EAGAIN;
      ret = err;
    }

  /* Only try to register if initialization was successful */

  if (err == 0)
    {
      err = pwm_register("/dev/pwm1", pwm1);
      if (err < 0)
        {
          syslog(LOG_ERR, "Couldn't register PWM1: %d\n", err);
          ret = err;
        }
    }
#endif /* CONFIG_BCM2711_PWM1 */

#ifdef CONFIG_RPI4B_AUDIOJACK

  /* Don't try to register audio device with NULL lower-half */

  if (pwm1 == NULL)
    {
      audwarn("PWM1 not setup, skipping tone registration.");
      return ret;
    }

  /* Get a oneshot timer driver. We allocate timer 0 for this. I pick a
   * random resolution of 10us since I know the BCM2711 timer driver can do
   * 1us of resolution and ignores this parameter otherwise.
   */

  oneshot = oneshot_initialize(0, 10);
  if (oneshot == NULL)
    {
      syslog(LOG_ERR, "Couldn't initialize one-shot timer channel 0.");
      return -EAGAIN;
    }

  /* NOTE: The PWM interface supports two channels. We register the audio
   * device on the first one.
   */

  audinfo("Registering /dev/tone1 using PWM1 channel 1.");
  err = tone_register("/dev/tone1", pwm1, 1, oneshot);
  if (err != 0)
    {
      syslog(LOG_ERR, "Couldn't register PWM audio tone driver: %d\n", err);
      ret = err;
    }
#endif /* CONFIG_RPI4B_AUDIOJACK */

  return ret;
}
