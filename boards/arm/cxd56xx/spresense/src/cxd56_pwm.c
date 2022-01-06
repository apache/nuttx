/****************************************************************************
 * boards/arm/cxd56xx/spresense/src/cxd56_pwm.c
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

#include <nuttx/board.h>
#include <nuttx/timers/pwm.h>

#include "cxd56_pwm.h"

#ifdef CONFIG_PWM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_CXD56_PWM0) || defined(CONFIG_CXD56_PWM1) || \
    defined(CONFIG_CXD56_PWM2) || defined(CONFIG_CXD56_PWM3)
static int pwm_initialize(uint32_t channel)
{
  char devname[16];
  struct pwm_lowerhalf_s *pwm = NULL;
  int ret;

  /* Call cxd56_pwminitialize() to get an instance of the PWM interface */

  pwm = cxd56_pwminitialize(channel);
  if (!pwm)
    {
      pwmerr("Failed to get the CXD56 PWM%ld lower half\n", channel);
      return -ENODEV;
    }

  /* Register the PWM driver at "/dev/pwmX" */

  snprintf(devname, sizeof(devname), "/dev/pwm%ld", channel);
  ret = pwm_register(devname, pwm);
  if (ret < 0)
    {
      pwmerr("pwm_register(%s) failed: %d\n", devname, ret);
      return ret;
    }

  return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_pwm_setup
 *
 * Description:
 *   All CXD56 architectures must provide the following interface to work
 *   with examples/pwm.
 *
 ****************************************************************************/

int board_pwm_setup(void)
{
  static bool initialized = false;

  /* Have we already initialized? */

  if (!initialized)
    {
#ifdef CONFIG_CXD56_PWM0
      pwm_initialize(CXD56_PWM_CH0);
#endif

#ifdef CONFIG_CXD56_PWM1
      pwm_initialize(CXD56_PWM_CH1);
#endif

#ifdef CONFIG_CXD56_PWM2
      pwm_initialize(CXD56_PWM_CH2);
#endif

#ifdef CONFIG_CXD56_PWM3
      pwm_initialize(CXD56_PWM_CH3);
#endif

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}

#endif /* CONFIG_PWM */
