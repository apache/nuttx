/****************************************************************************
 * boards/arm/rp2040/raspberrypi-pico-w/include/rp2040_extra_gpio.h
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

#ifndef __BOARDS_ARM_RP2040_RASPBERRYPI_PICO_W_INCLUDE_RP2040_EXTRA_GPIO_H
#define __BOARDS_ARM_RP2040_RASPBERRYPI_PICO_W_INCLUDE_RP2040_EXTRA_GPIO_H

/****************************************************************************
 *  These functions control the gpio pins on the CYW43439 not the RP2040.
 *  The pin assignments for the Raspberry Pi Pico W are:
 *
 *  GPIO 0 - output - controls the onboard LED
 *  GPIO 1 - output - controls the onboard voltage regulator mode.
 *  GPIO 2 - input  - Reads as non-zero if power supplied by USB or VBUS pin.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <assert.h>
#include <stdbool.h>

#include <nuttx/wireless/ieee80211/bcmf_gpio.h>
#include <nuttx/wireless/ieee80211/bcmf_gspi.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

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

extern gspi_dev_t *g_cyw43439;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RP2040_EXTRA_GPIO_NUM    3     /* Number of extra GPIO pins */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_extra_gpio_put
 *
 * Description:
 *  Change the state of the GPIO pins on the CYW43439
 ****************************************************************************/

static inline void rp2040_extra_gpio_put(uint32_t gpio, int val)
{
  DEBUGASSERT(gpio < RP2040_EXTRA_GPIO_NUM);

  bcmf_set_gpio(g_cyw43439->priv, gpio, val);
}

/****************************************************************************
 * Name: rp2040_extra_gpio_get
 *
 * Description:
 *  Get the state of the GPIO pins on the CYW43439
 ****************************************************************************/

static inline bool rp2040_extra_gpio_get(uint32_t gpio)
{
  bool value;

  DEBUGASSERT(gpio < RP2040_EXTRA_GPIO_NUM);

  bcmf_get_gpio(g_cyw43439->priv, gpio, &value);

  return value;
}

/****************************************************************************
 * Name: rp2040_extra_gpio_setdir
 *
 * Description:
 *  Change the direction of the GPIO pins on the CYW43439
 *
 * Note -- I added this function as a placeholder in case other boards
 *         implement extra GPIO pins where controlling the direction
 *         makes sense.
 ****************************************************************************/

static inline void rp2040_extra_gpio_setdir(uint32_t gpio, int out)
{
  DEBUGASSERT(gpio < RP2040_EXTRA_GPIO_NUM);

  DEBUGPANIC(); /* Function not yet implemented. */
}

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif