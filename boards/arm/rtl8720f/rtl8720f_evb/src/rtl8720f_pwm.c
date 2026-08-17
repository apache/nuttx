/****************************************************************************
 * boards/arm/rtl8720f/rtl8720f_evb/src/rtl8720f_pwm.c
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

#include <sys/param.h>
#include <syslog.h>
#include <errno.h>

#include "ameba_gpio.h"
#include "ameba_pwm.h"
#include "rtl8720f_rtl8720f_evb.h"

#ifdef CONFIG_AMEBA_PWM

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Output pad for each PWM hardware channel (0..AMEBA_PWM_NCHAN-1), all off
 * the one shared time base.  Channels the board does not use carry
 * AMEBA_PWM_PIN_NC.  The pads are examples used by the `pwm` config
 * (examples/pwm); adjust them to match your board's wiring.  Any pad can be
 * routed to any channel through the crossbar, so only this table changes.
 */

static const uint8_t g_pwm_pins[] =
{
  AMEBA_PA(23),      /* channel 1 -> PWM0 */
  AMEBA_PA(24),      /* channel 2 -> PWM1 */
  AMEBA_PWM_PIN_NC,  /* channel 3 unused  */
  AMEBA_PWM_PIN_NC,  /* channel 4 unused  */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtl8720f_pwm_initialize
 *
 * Description:
 *   Register the board's PWM timer at /dev/pwm0.
 *
 ****************************************************************************/

int rtl8720f_pwm_initialize(void)
{
  int ret;

  ret = ameba_pwm_register("/dev/pwm0", g_pwm_pins, nitems(g_pwm_pins));
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: ameba_pwm_register(/dev/pwm0) failed: %d\n", ret);
      return ret;
    }

  return OK;
}

#endif /* CONFIG_AMEBA_PWM */
