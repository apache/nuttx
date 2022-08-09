/****************************************************************************
 * arch/xtensa/src/esp32s2/esp32s2_gpio.c
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
#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "xtensa.h"
#include "esp32s2_gpio.h"
#include "esp32s2_irq.h"
#include "hardware/esp32s2_gpio.h"
#include "hardware/esp32s2_iomux.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP32S2_NPINS   47

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ESP32S2_GPIO_IRQ
static int g_gpio_cpuint;
#endif

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
  /* ESP32-S2 has 43 GPIO pins numbered from 0 to 21 and 26 to 46 */

  return pin <= 21 || (pin >= 26 && pin < ESP32S2_NPINS);
}

/****************************************************************************
 * Name: gpio_dispatch
 *
 * Description:
 *   Second level dispatch for GPIO interrupt handling.
 *
 * Input Parameters:
 *   irq           - GPIO IRQ number.
 *   status        - Value from the GPIO interrupt status clear register.
 *   regs          - Saved CPU context.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S2_GPIO_IRQ
static void gpio_dispatch(int irq, uint32_t status, uint32_t *regs)
{
  uint32_t mask;

  /* Check each bit in the status register */

  for (int i = 0; i < 32 && status != 0; i++)
    {
      /* Check if there is an interrupt pending for this pin */

      mask = UINT32_C(1) << i;
      if ((status & mask) != 0)
        {
          /* Yes... perform the second level dispatch */

          irq_dispatch(irq + i, regs);

          /* Clear the bit in the status so that we might execute this loop
           * sooner.
           */

          status &= ~mask;
        }
    }
}
#endif

/****************************************************************************
 * Name: gpio_interrupt
 *
 * Description:
 *   GPIO interrupt handler.
 *
 * Input Parameters:
 *   irq           - Identifier of the interrupt request.
 *   context       - Context data from the ISR.
 *   arg           - Opaque pointer to the internal driver state structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S2_GPIO_IRQ
static int gpio_interrupt(int irq, void *context, void *arg)
{
  uint32_t status;

  /* Read and clear the lower GPIO interrupt status */

  status = getreg32(GPIO_STATUS_REG);
  putreg32(status, GPIO_STATUS_W1TC_REG);

  /* Dispatch pending interrupts in the lower GPIO status register */

  gpio_dispatch(ESP32S2_FIRST_GPIOIRQ, status, (uint32_t *)context);

  /* Read and clear the upper GPIO interrupt status */

  status = getreg32(GPIO_STATUS1_REG) & GPIO_STATUS1_INTERRUPT_M;
  putreg32(status, GPIO_STATUS1_W1TC_REG);

  /* Dispatch pending interrupts in the lower GPIO status register */

  gpio_dispatch(ESP32S2_FIRST_GPIOIRQ + 32, status, (uint32_t *)context);
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s2_configgpio
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

int esp32s2_configgpio(int pin, gpio_pinattr_t attr)
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
          putreg32(UINT32_C(1) << pin, GPIO_ENABLE_W1TC_REG);
        }
      else
        {
          putreg32(UINT32_C(1) << (pin - 32), GPIO_ENABLE1_W1TC_REG);
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
      if (pin < 32)
        {
          putreg32(UINT32_C(1) << pin, GPIO_ENABLE_W1TS_REG);
        }
      else
        {
          putreg32(UINT32_C(1) << (pin - 32), GPIO_ENABLE1_W1TS_REG);
        }
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
      cntrl |= UINT32_C(1) << GPIO_PIN_PAD_DRIVER_S;
    }

  pin2func = (pin + 1) * 4;
  regaddr = DR_REG_IO_MUX_BASE + pin2func;
  putreg32(func, regaddr);

  regaddr = GPIO_REG(pin);
  putreg32(cntrl, regaddr);
  return OK;
}

/****************************************************************************
 * Name: esp32s2_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin.
 *
 * Input Parameters:
 *   pin           - GPIO pin to be written.
 *   value         - Value to be written to the GPIO pin. True will output
 *                   1 (one) to the GPIO, while false will output 0 (zero).
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s2_gpiowrite(int pin, bool value)
{
  DEBUGASSERT(is_valid_gpio(pin));

  if (value)
    {
      if (pin < 32)
        {
          putreg32(UINT32_C(1) << pin, GPIO_OUT_W1TS_REG);
        }
      else
        {
          putreg32(UINT32_C(1) << (pin - 32), GPIO_OUT1_W1TS_REG);
        }
    }
  else
    {
      if (pin < 32)
        {
          putreg32(UINT32_C(1) << pin, GPIO_OUT_W1TC_REG);
        }
      else
        {
          putreg32(UINT32_C(1) << (pin - 32), GPIO_OUT1_W1TC_REG);
        }
    }
}

/****************************************************************************
 * Name: esp32s2_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin.
 *
 * Input Parameters:
 *   pin           - GPIO pin to be read.
 *
 * Returned Value:
 *   True in case the read value is 1 (one). If 0 (zero), then false will be
 *   returned.
 *
 ****************************************************************************/

bool esp32s2_gpioread(int pin)
{
  uint32_t regval;

  DEBUGASSERT(is_valid_gpio(pin));

  if (pin < 32)
    {
      regval = getreg32(GPIO_IN_REG);
      return ((regval >> pin) & 1) != 0;
    }
  else
    {
      regval = getreg32(GPIO_IN1_REG);
      return ((regval >> (pin - 32)) & 1) != 0;
    }
}

/****************************************************************************
 * Name: esp32s2_gpioirqinitialize
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

#ifdef CONFIG_ESP32S2_GPIO_IRQ
void esp32s2_gpioirqinitialize(void)
{
  /* Setup the GPIO interrupt. */

  g_gpio_cpuint = esp32s2_setup_irq(ESP32S2_PERIPH_GPIO_INT_PRO,
                                    1, ESP32S2_CPUINT_LEVEL);
  DEBUGASSERT(g_gpio_cpuint >= 0);

  /* Attach and enable the interrupt handler */

  DEBUGVERIFY(irq_attach(ESP32S2_IRQ_GPIO_INT_PRO, gpio_interrupt, NULL));
  up_enable_irq(ESP32S2_IRQ_GPIO_INT_PRO);
}
#endif

/****************************************************************************
 * Name: esp32s2_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for the specified GPIO IRQ.
 *
 * Input Parameters:
 *   irq           - Identifier of the interrupt request.
 *   intrtype      - Interrupt type, select from gpio_intrtype_t.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S2_GPIO_IRQ
void esp32s2_gpioirqenable(int irq, gpio_intrtype_t intrtype)
{
  uintptr_t regaddr;
  uint32_t regval;
  int pin;

  DEBUGASSERT(irq >= ESP32S2_FIRST_GPIOIRQ && irq <= ESP32S2_LAST_GPIOIRQ);

  /* Convert the IRQ number to a pin number */

  pin = ESP32S2_IRQ2PIN(irq);

  /* Disable the GPIO interrupt during the configuration. */

  up_disable_irq(ESP32S2_IRQ_GPIO_INT_PRO);

  /* Get the address of the GPIO PIN register for this pin */

  regaddr = GPIO_REG(pin);
  regval  = getreg32(regaddr);
  regval &= ~(GPIO_PIN_INT_ENA_M | GPIO_PIN_INT_TYPE_M);

  /* Set the pin ENA field */

  regval |= GPIO_PIN_INT_ENA_M;
  regval |= (uint32_t)intrtype << GPIO_PIN_INT_TYPE_S;
  putreg32(regval, regaddr);

  /* Configuration done. Re-enable the GPIO interrupt. */

  up_enable_irq(ESP32S2_IRQ_GPIO_INT_PRO);
}
#endif

/****************************************************************************
 * Name: esp32s2_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for the specified GPIO IRQ.
 *
 * Input Parameters:
 *   irq           - Identifier of the interrupt request.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S2_GPIO_IRQ
void esp32s2_gpioirqdisable(int irq)
{
  uintptr_t regaddr;
  uint32_t regval;
  int pin;

  DEBUGASSERT(irq >= ESP32S2_FIRST_GPIOIRQ && irq <= ESP32S2_LAST_GPIOIRQ);

  /* Convert the IRQ number to a pin number */

  pin = ESP32S2_IRQ2PIN(irq);

  /* Disable the GPIO interrupt during the configuration. */

  up_disable_irq(ESP32S2_IRQ_GPIO_INT_PRO);

  /* Reset the pin ENA and TYPE fields */

  regaddr = GPIO_REG(pin);
  regval  = getreg32(regaddr);
  regval &= ~(GPIO_PIN_INT_ENA_M | GPIO_PIN_INT_TYPE_M);
  putreg32(regval, regaddr);

  /* Configuration done. Re-enable the GPIO interrupt. */

  up_enable_irq(ESP32S2_IRQ_GPIO_INT_PRO);
}
#endif

/****************************************************************************
 * Name: esp32s2_gpio_matrix_in
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

void esp32s2_gpio_matrix_in(uint32_t pin, uint32_t signal_idx, bool inv)
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
 * Name: esp32s2_gpio_matrix_out
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

void esp32s2_gpio_matrix_out(uint32_t pin, uint32_t signal_idx,
                             bool out_inv, bool oen_inv)
{
  uint32_t regaddr = GPIO_FUNC0_OUT_SEL_CFG_REG + (pin * 4);
  uint32_t regval = signal_idx << GPIO_FUNC0_OUT_SEL_S;

  DEBUGASSERT(is_valid_gpio(pin));

  if (pin < 32)
    {
      putreg32(UINT32_C(1) << pin, GPIO_ENABLE_W1TS_REG);
    }
  else
    {
      putreg32(UINT32_C(1) << (pin - 32), GPIO_ENABLE1_W1TS_REG);
    }

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
