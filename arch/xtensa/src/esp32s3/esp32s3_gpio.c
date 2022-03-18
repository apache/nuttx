/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_gpio.c
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

#include <sys/types.h>
#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <arch/irq.h>

#include "xtensa.h"
#include "esp32s3_irq.h"
#include "hardware/esp32s3_iomux.h"
#include "hardware/esp32s3_gpio.h"

#include "esp32s3_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP32S3_NPINS   49

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: is_valid_gpio
 *
 * Description:
 *   Check if the requested pin is a valid GPIO pin.
 *
 * Input Parameters:
 *   pin  - Pin to be checked for validity.
 *
 * Returned Value:
 *   True if the requested pin is a valid GPIO pin, false otherwise.
 *
 ****************************************************************************/

static inline bool is_valid_gpio(uint32_t pin)
{
  /* ESP32-S3 has 45 GPIO pins numbered from 0 to 21 and 26 to 48 */

  return pin <= 21 || (pin >= 26 && pin < ESP32S3_NPINS);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_configgpio
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

int esp32s3_configgpio(uint32_t pin, gpio_pinattr_t attr)
{
  uintptr_t regaddr;
  uint32_t func;
  uint32_t cntrl;
  uint32_t pin2func;

  DEBUGASSERT(is_valid_gpio(pin));

  func  = 0;
  cntrl = 0;

  /* Handle input pins */

  if ((attr & INPUT) != 0)
    {
      if (pin < 32)
        {
          putreg32((UINT32_C(1) << pin), GPIO_ENABLE_W1TC_REG);
        }
      else
        {
          putreg32((UINT32_C(1) << (pin - 32)), GPIO_ENABLE1_W1TC_REG);
        }

      /* Input enable */

      func |= FUN_IE;

      if ((attr & PULLUP) != 0)
        {
          func |= FUN_PU;
        }
      else if ((attr & PULLDOWN) != 0)
        {
          func |= FUN_PD;
        }
    }

  /* Handle output pins */

  if ((attr & OUTPUT) != 0)
    {
      putreg32((UINT32_C(1) << pin), GPIO_ENABLE_W1TS_REG);
    }

  /* Configure the pad's function */

  if ((attr & FUNCTION_MASK) != 0)
    {
      uint32_t val = ((attr & FUNCTION_MASK) >> FUNCTION_SHIFT) - 1;
      func |= val << MCU_SEL_S;
    }
  else
    {
      /* Function not provided, assuming function GPIO by default */

      func |= (uint32_t)(PIN_FUNC_GPIO << MCU_SEL_S);
    }

  /* Configure the pad's drive strength */

  if ((attr & DRIVE_MASK) != 0)
    {
      uint32_t val = ((attr & DRIVE_MASK) >> DRIVE_SHIFT) - 1;
      func |= val << FUN_DRV_S;
    }
  else
    {
      /* Drive strength not provided, assuming strength 2 by default */

      func |= UINT32_C(2) << FUN_DRV_S;
    }

  if ((attr & OPEN_DRAIN) != 0)
    {
      cntrl |= (1 << GPIO_PIN_PAD_DRIVER_S);
    }

  /* Set the pin function to its register */

  pin2func = (pin + 1) * 4;
  regaddr = REG_IO_MUX_BASE + pin2func;
  putreg32(func, regaddr);

  regaddr = GPIO_REG(pin);
  putreg32(cntrl, regaddr);
  return OK;
}

/****************************************************************************
 * Name: esp32s3_gpio_matrix_in
 *
 * Description:
 *   Set GPIO input to a signal.
 *   NOTE: one GPIO can input to several signals.
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

void esp32s3_gpio_matrix_in(uint32_t pin, uint32_t signal_idx, bool inv)
{
  uint32_t regaddr = GPIO_FUNC0_IN_SEL_CFG_REG + (signal_idx * 4);
  uint32_t regval = pin << GPIO_FUNC0_IN_SEL_S;

  if (inv)
    {
      regval |= GPIO_FUNC0_IN_INV_SEL;
    }

  if (pin != 0x3a)
    {
      regval |= GPIO_SIG0_IN_SEL;
    }

  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: esp32s3_gpio_matrix_out
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

void esp32s3_gpio_matrix_out(uint32_t pin, uint32_t signal_idx, bool out_inv,
                             bool oen_inv)
{
  uint32_t regaddr = GPIO_FUNC0_OUT_SEL_CFG_REG + (pin * 4);
  uint32_t regval = signal_idx << GPIO_FUNC0_OUT_SEL_S;

  DEBUGASSERT(is_valid_gpio(pin));

  putreg32(1ul << pin, GPIO_ENABLE_W1TS_REG);

  if (out_inv)
    {
      regval |= GPIO_FUNC0_OUT_INV_SEL;
    }

  if (oen_inv)
    {
      regval |= GPIO_FUNC0_OEN_INV_SEL;
    }

  putreg32(regval, regaddr);
}

