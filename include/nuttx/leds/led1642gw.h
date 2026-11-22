/****************************************************************************
 * include/nuttx/leds/led1642gw.h
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

#ifndef __INCLUDE_NUTTX_LEDS_LED1642GW_H
#define __INCLUDE_NUTTX_LEDS_LED1642GW_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/timers/pwm.h>

#include <stdbool.h>
#include <stdint.h>

/* Configuration
 * CONFIG_LEDS_LED1642GW - Enables support for the LED1642GW driver
 */

#ifdef CONFIG_LEDS_LED1642GW

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Number of constant current outputs */

#define LED1642GW_NCHANNELS    16

/* Maximum value of the current gain */

#define LED1642GW_GAIN_MAX     63

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The LED1642GW serial interface is bit-banged with the board logic that
 * drives the CLK, SDI and LE lines.  The board must also provide the PWCLK
 * clock that feeds the brightness counter of the device.
 */

struct led1642gw_config_s
{
  /* Set the state of the CLK, SDI and LE lines */

  CODE void (*clk)(FAR const struct led1642gw_config_s *config, bool state);
  CODE void (*sdi)(FAR const struct led1642gw_config_s *config, bool state);
  CODE void (*le)(FAR const struct led1642gw_config_s *config, bool state);

  uint8_t gain;     /* Current gain: 0 - LED1642GW_GAIN_MAX */
  bool    range;    /* Current range: false = low, true = high */
  bool    pwm16;    /* Brightness resolution: false = 12-bit, true = 16-bit */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: led1642gw_initialize
 *
 * Description:
 *   Initialize one LED1642GW device: write the configuration register and
 *   turn all outputs off.  The returned PWM lower half can be passed to
 *   pwm_register() or rgbled_register().
 *
 *   The PWM channel n is the output n - 1 and the duty is the brightness
 *   of the output.  The PWM frequency is set by the PWCLK clock.
 *
 * Input Parameters:
 *   config - The board configuration.  It must persist as long as the
 *            driver persists.
 *
 * Returned Value:
 *   Valid PWM lower half structure reference on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct pwm_lowerhalf_s *
led1642gw_initialize(FAR const struct led1642gw_config_s *config);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_LEDS_LED1642GW */
#endif /* __INCLUDE_NUTTX_LEDS_LED1642GW_H */
