/****************************************************************************
 * boards/arm/rtl8730e/rtl8730e_evb/src/rtl8730e_pwm.c
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

#include "ameba_gpio.h"
#include "ameba_pwm.h"
#include "rtl8730e_evb.h"

#ifdef CONFIG_AMEBA_PWM

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Output pad for each PWM hardware channel (0..AMEBA_PWM_NCHAN-1), all off
 * the one shared TIM8 time base.  Channels the board does not use carry
 * AMEBA_PWM_PIN_NC.
 *
 * Despite the single PINMUX_FUNCTION_PWM code, a pad is tied to one specific
 * channel: the chip pin-mux table names each pad's PWM alternate function
 * HS_PWM<n>, which is channel n + 1 here.  An entry may therefore only be
 * cleared or replaced by another pad carrying that same HS_PWM<n>; moving a
 * pad to a different row silently drives the wrong channel.
 *
 * Channels 2-5 use PA9/PA10/PA4/PA5, as the SDK's own raw_pwm example does
 * on amebasmart; channels 1 and 6 use PA7/PA6 rather than the example's
 * PA14/PA13, which are SWD_CLK and SWD_DATA and so stay muxed to the debug
 * port (the example frees them by calling Pinmux_Swdoff() first, but giving
 * up the debug port is a board decision rather than a driver one).
 *
 * All six pads double as I2C, UART or SPI1 pads, so a config registering
 * those too must move one of them elsewhere; PB14-PB16 carry HS_PWM0-HS_PWM2
 * with no other function and are the natural alternative for channels 1-3.
 * Pads the audio codec drives (PA20-PB6) must not be used.
 */

static const uint8_t g_pwm_pins[] =
{
  AMEBA_PA(7),       /* channel 1 -> HS_PWM0 */
  AMEBA_PA(9),       /* channel 2 -> HS_PWM1 */
  AMEBA_PA(10),      /* channel 3 -> HS_PWM2 */
  AMEBA_PA(4),       /* channel 4 -> HS_PWM3 */
  AMEBA_PA(5),       /* channel 5 -> HS_PWM4 */
  AMEBA_PA(6),       /* channel 6 -> HS_PWM5 */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtl8730e_pwm_initialize
 *
 * Description:
 *   Register the board's PWM timer at /dev/pwm0.
 *
 ****************************************************************************/

int rtl8730e_pwm_initialize(void)
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
