/****************************************************************************
 * arch/risc-v/src/espressif/esp_gpio.c
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

#include <assert.h>
#include <debug.h>
#include <stdint.h>
#include <sys/types.h>

#include <arch/irq.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "riscv_internal.h"

#include "esp_gpio.h"

#include "esp_rom_gpio.h"
#include "hal/gpio_hal.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static gpio_hal_context_t g_gpio_hal =
{
  .dev = GPIO_HAL_GET_HW(GPIO_PORT_0)
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on encoded pin attributes.
 *
 * Input Parameters:
 *   pin           - GPIO pin to be configured.
 *   attr          - Attributes to be configured for the selected GPIO pin.
 *                   The following attributes are accepted:
 *                   - Direction (OUTPUT or INPUT)
 *                   - Pull (PULLUP, PULLDOWN or OPENDRAIN)
 *                   - Function (if not provided, assume function GPIO by
 *                     default)
 *                   - Drive strength (if not provided, assume DRIVE_2 by
 *                     default)
 *
 * Returned Value:
 *   Zero (OK) on success, or -1 (ERROR) in case of failure.
 *
 ****************************************************************************/

int esp_configgpio(int pin, gpio_pinattr_t attr)
{
  DEBUGASSERT(pin >= 0 && pin < SOC_GPIO_PIN_COUNT);

  /* Handle input pins */

  if ((attr & INPUT) != 0)
    {
      gpio_hal_input_enable(&g_gpio_hal, pin);
    }
  else
    {
      gpio_hal_input_disable(&g_gpio_hal, pin);
    }

  if ((attr & OPEN_DRAIN) != 0)
    {
      gpio_hal_od_enable(&g_gpio_hal, pin);
    }
  else
    {
      gpio_hal_od_disable(&g_gpio_hal, pin);
    }

  if ((attr & OUTPUT) != 0)
    {
      gpio_hal_output_enable(&g_gpio_hal, pin);
    }
  else
    {
      gpio_hal_output_disable(&g_gpio_hal, pin);
    }

  if ((attr & PULLUP) != 0)
    {
      gpio_hal_pullup_en(&g_gpio_hal, pin);
    }
  else
    {
      gpio_hal_pullup_dis(&g_gpio_hal, pin);
    }

  if ((attr & PULLDOWN) != 0)
    {
      gpio_hal_pulldown_en(&g_gpio_hal, pin);
    }
  else
    {
      gpio_hal_pulldown_dis(&g_gpio_hal, pin);
    }

  if ((attr & DRIVE_MASK) != 0)
    {
      uint32_t val = ((attr & DRIVE_MASK) >> DRIVE_SHIFT) - 1;
      gpio_hal_set_drive_capability(&g_gpio_hal, pin, val);
    }
  else
    {
      gpio_hal_set_drive_capability(&g_gpio_hal, pin,
                                    GPIO_DRIVE_CAP_DEFAULT);
    }

  if ((attr & FUNCTION_MASK) != 0)
    {
      uint32_t val = ((attr & FUNCTION_MASK) >> FUNCTION_SHIFT) - 1;
      gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[pin], val);
    }
  else
    {
      gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[pin], PIN_FUNC_GPIO);
    }

  return OK;
}

/****************************************************************************
 * Name: esp_gpio_matrix_in
 *
 * Description:
 *   Set GPIO input to a signal.
 *   NOTE: one GPIO can receive inputs from several signals.
 *
 * Input Parameters:
 *   pin           - GPIO pin to be configured.
 *                   - If pin == 0x3c, cancel input to the signal, input 0
 *                     to signal.
 *                   - If pin == 0x3a, input nothing to signal.
 *                   - If pin == 0x38, cancel input to the signal, input 1
 *                     to signal.
 *   signal_idx    - Signal index.
 *   inv           - Flag indicating whether the signal is inverted.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_gpio_matrix_in(uint32_t pin, uint32_t signal_idx, bool inv)
{
  esp_rom_gpio_connect_in_signal(pin, signal_idx, inv);
}

/****************************************************************************
 * Name: esp_gpio_matrix_out
 *
 * Description:
 *   Set signal output to GPIO.
 *   NOTE: one signal can output to several GPIOs.
 *
 * Input Parameters:
 *   pin           - GPIO pin to be configured.
 *   signal_idx    - Signal index.
 *                   - If signal_idx == 0x100, cancel output to the GPIO.
 *   out_inv       - Flag indicating whether the signal output is inverted.
 *   oen_inv       - Flag indicating whether the signal output enable is
 *                   inverted.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_gpio_matrix_out(uint32_t pin, uint32_t signal_idx, bool out_inv,
                         bool oen_inv)
{
  esp_rom_gpio_connect_out_signal(pin, signal_idx, out_inv, oen_inv);
}

