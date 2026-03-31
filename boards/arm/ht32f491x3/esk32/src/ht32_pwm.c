/****************************************************************************
 * boards/arm/ht32f491x3/esk32/src/ht32_pwm.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with this
 * work for additional information regarding copyright ownership.  The ASF
 * licenses this file to you under the Apache License, Version 2.0 (the
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

#include <arch/board/board.h>
#include <nuttx/board.h>
#include <nuttx/timers/pwm.h>

#include "chip.h"
#include "arm_internal.h"
#include "ht32f491x3_pwm.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ht32_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 * Return Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ht32_pwm_setup(void)
{
#ifdef CONFIG_PWM
  struct pwm_lowerhalf_s *pwm;
  int ret;

#if defined(CONFIG_HT32F491X3_TMR3_PWM)
  pwm = ht32f491x3_pwminitialize(3);
  if (!pwm)
    {
      pwmerr("ERROR: Failed to get the HT32 PWM lower half\n");
      return -ENODEV;
    }

  ret = pwm_register("/dev/pwm0", pwm);
  if (ret < 0)
    {
      pwmerr("ERROR: pwm_register failed: %d\n", ret);
      return ret;
    }

  return OK;
#else
  return -ENODEV;
#endif
#else
  return -ENODEV;
#endif
}
