/****************************************************************************
 * boards/arm/rp2040/common/src/rp2040_pwmdev.c
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

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include "rp2040_pwm.h"

#ifdef CONFIG_RP2040_PWM

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_pwmdev_initialize
 *
 * Description:
 *   Initialize and register spi driver for the specified pwm port
 *
 ****************************************************************************/

#if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
int rp2040_pwmdev_initialize(int      slice,
                             int      pin_a,
                             int      pin_b,
                             uint32_t flags)
#else
int rp2040_pwmdev_initialize(int      slice,
                             int      pin,
                             uint32_t flags)
#endif
{
  int ret;
  struct rp2040_pwm_lowerhalf_s *pwm_lowerhalf;

  pwminfo("Initializing /dev/pwm%d a %d b %d f 0x%08lX..\n",
           slice,
           pin_a,
           pin_b,
           flags);

  /* Initialize spi device */

#if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
  pwm_lowerhalf = rp2040_pwm_initialize(slice, pin_a, pin_b, flags);
#else
  pwm_lowerhalf = rp2040_pwm_initialize(slice, pin, flags);
#endif

  if (!pwm_lowerhalf)
    {
      pwmerr("ERROR: Failed to initialize pwm%d.\n", slice);
      return -ENODEV;
    }

  char path[10] = "/dev/pwmN";
  path[8] = '0' + slice; /* replace "N" with slice number. */

  ret = pwm_register(path, (struct pwm_lowerhalf_s *) pwm_lowerhalf);
  if (ret < 0)
    {
      pwmerr("ERROR: Failed to register pwm%d: %d\n", slice, ret);
      return -ENODEV;
    }

  return OK;
}

#endif /* CONFIG_RP2040_PWM */

