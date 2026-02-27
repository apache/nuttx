/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_gpio.c
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

/* Config */

#include <nuttx/config.h>

/* Libc */

#include <assert.h>
#include <debug.h>
#include <stdint.h>
#include <sys/types.h>

/* NuttX */

#include <arch/irq.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

/* Arch */

#include "riscv_internal.h"

#include "esp_gpio.h"
#include "esp_irq.h"

/* HAL */

#include "esp_err.h"
#include "soc/interrupts.h"
#include "esp_rom_gpio.h"
#include "hal/gpio_hal.h"
#include "driver/gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP_INTR_FLAG_DEFAULT 0

/****************************************************************************
 * Private Data
 ****************************************************************************/

static gpio_hal_context_t g_gpio_hal =
{
  .dev = GPIO_HAL_GET_HW(GPIO_PORT_0)
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_intr_handler_adapter
 *
 * Description:
 *   This function acts as an adapter to bridge interrupt service routines
 *   between NuttX and the Espressif's interrupt service routine. It is
 *   called when a GPIO interrupt occurs, retrieves the function pointer and
 *   associated data from the 'intr_adapter_from_nuttx' structure passed as
 *   an argument, and invokes the original user-provided interrupt handler
 *   with the IRQ number and user argument.
 *
 * Input Parameters:
 *   arg - Pointer to a structure of type 'intr_adapter_from_nuttx' that
 *         holds the handler function, the associated IRQ, the context, and
 *         the user argument.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_GPIO_IRQ
static void esp_intr_handler_adapter(void *arg)
{
  struct intr_adapter_from_nuttx *adapter;

  adapter = (struct intr_adapter_from_nuttx *)arg;

  adapter->func(adapter->irq, adapter->context, adapter->arg);
}
#endif

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

  gpio_hal_set_intr_type(&g_gpio_hal, pin,
                         (attr & INTR_TYPE_MASK) >> INTR_TYPE_SHIFT);

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
      gpio_hal_func_sel(&g_gpio_hal, pin, val);
    }
  else
    {
      gpio_hal_func_sel(&g_gpio_hal, pin, PIN_FUNC_GPIO);
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

/****************************************************************************
 * Name: esp_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 * Input Parameters:
 *   pin           - GPIO pin to be modified.
 *   value         - The value to be written (0 or 1).
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_gpiowrite(int pin, bool value)
{
  DEBUGASSERT(pin >= 0 && pin <= SOC_GPIO_PIN_COUNT);

  gpio_hal_set_level(&g_gpio_hal, pin, value);
}

/****************************************************************************
 * Name: esp_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 * Input Parameters:
 *   pin           - GPIO pin to be read.
 *
 * Returned Value:
 *   The boolean representation of the input value (true/false).
 *
 ****************************************************************************/

bool esp_gpioread(int pin)
{
  DEBUGASSERT(pin >= 0 && pin <= SOC_GPIO_PIN_COUNT);

  return gpio_hal_get_level(&g_gpio_hal, pin) != 0;
}

/****************************************************************************
 * Name: esp_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   GPIO pins.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_GPIO_IRQ
void esp_gpioirqinitialize(void)
{
  /* Setup the GPIO interrupt. */

  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
}
#endif

/****************************************************************************
 * Name: esp_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GPIO
 *
 * Input Parameters:
 *   id           - GPIO to be enabled.
 *
 * Returned Value:
 *   Zero (OK) on success, or -1 (ERROR) in case of failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_GPIO_IRQ
int esp_gpioirqenable(int id)
{
  esp_err_t esp_ret;

  esp_ret = gpio_intr_enable(id);
  if (esp_ret != ESP_OK)
    {
      gpioerr("gpio_intr_enable() failed: %d\n", esp_ret);
      return -ERROR;
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: esp_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO
 *
 * Input Parameters:
 *   id           - GPIO to be disabled.
 *
 * Returned Value:
 *   Zero (OK) on success, or -1 (ERROR) in case of failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_GPIO_IRQ
int esp_gpioirqdisable(int id)
{
  esp_err_t esp_ret;

  esp_ret = gpio_intr_disable(id);
  if (esp_ret != ESP_OK)
    {
      gpioerr("gpio_intr_disable() failed: %d\n", esp_ret);
      return -ERROR;
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: esp_gpio_irq
 *
 * Description:
 *   Register or unregister a button interrupt handler for the specified
 *   button ID. Passing a non-NULL handler attaches and enables the ISR for
 *   the button; passing NULL disables the interrupt and removes any
 *   previously registered handler.
 *
 * Input Parameters:
 *   id           - Identifies the button to be monitored.
 *   irqhandler   - The handler to be called when the interrupt occurs.
 *                  Set to NULL to disable the interrupt.
 *   arg          - Pointer to the argument that will be provided to the
 *                  interrupt handler.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_GPIO_IRQ
int esp_gpio_irq(int id, xcpt_t irqhandler, void *arg)
{
  int ret;
  int irq = ESP_PIN2IRQ(id);

  if (NULL != irqhandler)
    {
      esp_err_t esp_ret;
      struct intr_adapter_from_nuttx *adapter;

      gpioinfo("Attach %p\n", irqhandler);

      adapter = kmm_calloc(1, sizeof(struct intr_adapter_from_nuttx));
      if (adapter == NULL)
        {
          gpioerr("kmm_calloc() failed\n");
          return -ERROR;
        }

      adapter->func = irqhandler;
      adapter->irq = irq;
      adapter->context = NULL;
      adapter->arg = arg;

      esp_ret = gpio_isr_handler_add(id, esp_intr_handler_adapter,
                                     (void *)adapter);
      if (esp_ret != ESP_OK)
        {
          gpioerr("gpio_isr_handler_add() failed: %d\n", ret);
          return -ERROR;
        }
    }
  else
    {
      gpioinfo("Disable the interrupt\n");
      gpio_isr_handler_remove(id);
    }

  return OK;
}
#endif
