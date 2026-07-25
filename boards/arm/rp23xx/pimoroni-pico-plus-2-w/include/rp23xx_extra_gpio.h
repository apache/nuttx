/****************************************************************************
 * boards/arm/rp23xx/pimoroni-pico-plus-2-w/include/rp23xx_extra_gpio.h
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

#ifndef __BOARDS_ARM_RP23XX_PIMORONI_PICO_PLUS_2_W_INCLUDE_RP23XX_EXTRA_GPIO_H
#define __BOARDS_ARM_RP23XX_PIMORONI_PICO_PLUS_2_W_INCLUDE_RP23XX_EXTRA_GPIO_H

/****************************************************************************
 *  These functions control the GPIO pins on the CYW43439 wireless chip, not
 *  the GPIO pins of the RP2350.  The pin assignments for the Pimoroni Pico
 *  Plus 2 W are:
 *
 *  GPIO 0 - output - controls the onboard LED
 *  GPIO 1 - output - controls the onboard voltage regulator mode.
 *  GPIO 2 - input  - Reads as non-zero if power supplied by USB or VBUS pin.
 *
 *  These only work once the wireless chip has been activated, which happens
 *  when the wlan0 network interface is brought up.  Until then every access
 *  fails with -EIO.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/wireless/ieee80211/bcmf_gpio.h>
#include <nuttx/wireless/ieee80211/bcmf_gspi.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RP23XX_EXTRA_GPIO_NUM    3     /* Number of extra GPIO pins */

#define RP23XX_EXTRA_GPIO_LED    0     /* Onboard LED               */
#define RP23XX_EXTRA_GPIO_VREG   1     /* Voltage regulator mode    */
#define RP23XX_EXTRA_GPIO_VBUS   2     /* VBUS present              */

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

EXTERN gspi_dev_t *g_cyw43439;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_extra_gpio_put
 *
 * Description:
 *   Change the state of a GPIO pin on the CYW43439.
 *
 * Returned Value:
 *   OK on success, or a negated errno.  -EIO means the wireless chip is not
 *   running yet, which is the case until wlan0 has been brought up.
 *
 ****************************************************************************/

static inline int rp23xx_extra_gpio_put(uint32_t gpio, bool value)
{
  DEBUGASSERT(gpio < RP23XX_EXTRA_GPIO_NUM);

  if (g_cyw43439 == NULL)
    {
      return -EIO;
    }

  return bcmf_set_gpio(g_cyw43439->priv, gpio, value);
}

/****************************************************************************
 * Name: rp23xx_extra_gpio_get
 *
 * Description:
 *   Read the state of a GPIO pin on the CYW43439.
 *
 * Returned Value:
 *   OK on success, or a negated errno.  -EIO means the wireless chip is not
 *   running yet, which is the case until wlan0 has been brought up.
 *
 ****************************************************************************/

static inline int rp23xx_extra_gpio_get(uint32_t gpio, bool *value)
{
  DEBUGASSERT(gpio < RP23XX_EXTRA_GPIO_NUM);

  if (g_cyw43439 == NULL)
    {
      return -EIO;
    }

  return bcmf_get_gpio(g_cyw43439->priv, gpio, value);
}

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_RP23XX_PIMORONI_PICO_PLUS_2_W_INCLUDE_RP23XX_EXTRA_GPIO_H */
