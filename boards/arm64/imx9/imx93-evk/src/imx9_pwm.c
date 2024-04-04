/****************************************************************************
 * boards/arm64/imx9/imx93-evk/src/imx9_pwm.c
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
#include <nuttx/timers/pwm.h>
#include <arch/board/board.h>

#include <errno.h>

#include "imx9_flexio_pwm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_pwm_setup
 *
 * Description:
 *
 *   Initialize PWM and register PWM devices
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0 on success, negated error value on error
 *
 ****************************************************************************/

int imx9_pwm_setup(void)
{
  struct pwm_lowerhalf_s *lower_half = NULL;
  int ret = 0;

#ifdef CONFIG_IMX9_FLEXIO1_PWM
  lower_half = imx9_flexio_pwm_init(PWM_FLEXIO1);

  if (lower_half)
    {
      ret = pwm_register("/dev/pwm0", lower_half);
    }
  else
    {
      ret = -ENODEV;
    }

  if (ret < 0)
    {
      return ret;
    }
#endif

#ifdef CONFIG_IMX9_FLEXIO2_PWM
  lower_half = imx9_flexio_pwm_init(PWM_FLEXIO2);

  if (lower_half)
    {
      ret = pwm_register("/dev/pwm1", lower_half);
    }
  else
    {
      ret = -ENODEV;
    }
#endif

  return ret;
}
