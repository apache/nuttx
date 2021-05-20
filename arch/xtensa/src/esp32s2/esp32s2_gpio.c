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

#include <sys/types.h>
#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <arch/irq.h>

#include "xtensa.h"
#include "hardware/esp32s2_iomux.h"
#include "hardware/esp32s2_gpio.h"
#include "esp32s2_cpuint.h"
#include "esp32s2_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NGPIO_HPINS  (ESP32S2_NIRQ_GPIO - 32)
#define NGPIO_HMASK  ((1ul << NGPIO_HPINS) - 1)
#define _NA_         0xff

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
 * Name: gpio_dispatch
 *
 * Description:
 *   Second level dispatch for GPIO interrupt handling.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S2_GPIO_IRQ
static void gpio_dispatch(int irq, uint32_t status, uint32_t *regs)
{
  uint32_t mask;
  int i;

  /* Check each bit in the status register */

  for (i = 0; i < 32 && status != 0; i++)
    {
      /* Check if there is an interrupt pending for this pin */

      mask = (1ul << i);
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
 ****************************************************************************/

#ifdef CONFIG_ESP32S2_GPIO_IRQ
static int gpio_interrupt(int irq, FAR void *context, FAR void *arg)
{
  uint32_t status;

  /* Read and clear the lower GPIO interrupt status */

  status = getreg32(GPIO_STATUS_REG);
  putreg32(status, GPIO_STATUS_W1TC_REG);

  /* Dispatch pending interrupts in the lower GPIO status register */

  gpio_dispatch(ESP32S2_FIRST_GPIOIRQ, status, (uint32_t *)context);

  /* Read and clear the upper GPIO interrupt status */

  status = getreg32(GPIO_STATUS1_REG) & NGPIO_HMASK;
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
 ****************************************************************************/

int esp32s2_configgpio(int pin, gpio_pinattr_t attr)
{
  uintptr_t regaddr;
  uint32_t func;
  uint32_t cntrl;
  uint32_t pin2func;

  DEBUGASSERT(pin >= 0 && pin <= ESP32S2_NGPIOS);

  /* Handle input pins */

  func  = 0;
  cntrl = 0;

  if ((attr & INPUT) != 0)
    {
      if (pin < 32)
        {
          putreg32((1ul << pin), GPIO_ENABLE_W1TC_REG);
        }
      else
        {
          putreg32((1ul << (pin - 32)), GPIO_ENABLE1_W1TC_REG);
        }

      /* Input enable */

      func |= FUN_IE;

      if ((attr & PULLUP) != 0)
        {
          func |= FUN_PU;
        }
      else if (attr & PULLDOWN)
        {
          func |= FUN_PD;
        }
    }

  /* Handle output pins */

  if ((attr & OUTPUT) != 0)
    {
      if (pin < 32)
        {
          putreg32((1ul << pin), GPIO_ENABLE_W1TS_REG);
        }
      else
        {
          putreg32((1ul << (pin - 32)), GPIO_ENABLE1_W1TS_REG);
        }
    }

  /* Add drivers */

  func |= (uint32_t)(2ul << FUN_DRV_S);

  /* Select the pad's function.  If no function was given, consider it a
   * normal input or output (i.e. function3).
   */

  if ((attr & FUNCTION_MASK) != 0)
    {
      func |= (uint32_t)(((attr >> FUNCTION_SHIFT) - 1) << MCU_SEL_S);
    }
  else
    {
      func |= (uint32_t)(PIN_FUNC_GPIO << MCU_SEL_S);
    }

  if ((attr & OPEN_DRAIN) != 0)
    {
      cntrl |= (1 << GPIO_PIN_PAD_DRIVER_S);
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
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void esp32s2_gpiowrite(int pin, bool value)
{
  DEBUGASSERT(pin >= 0 && pin <= ESP32S2_NGPIOS);

  if (value)
    {
      if (pin < 32)
        {
          putreg32((uint32_t)(1ul << pin), GPIO_OUT_W1TS_REG);
        }
      else
        {
          putreg32((uint32_t)(1ul << (pin - 32)), GPIO_OUT1_W1TS_REG);
        }
    }
  else
    {
      if (pin < 32)
        {
          putreg32((uint32_t)(1ul << pin), GPIO_OUT_W1TC_REG);
        }
      else
        {
          putreg32((uint32_t)(1ul << (pin - 32)), GPIO_OUT1_W1TC_REG);
        }
    }
}

/****************************************************************************
 * Name: esp32s2_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool esp32s2_gpioread(int pin)
{
  uint32_t regval;

  DEBUGASSERT(pin >= 0 && pin <= ESP32S2_NGPIOS);

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
 ****************************************************************************/

#ifdef CONFIG_ESP32S2_GPIO_IRQ
void esp32s2_gpioirqinitialize(void)
{
  /* Allocate a level-sensitive, priority 1 CPU interrupt */

  g_gpio_cpuint = esp32s2_alloc_levelint(1);
  DEBUGASSERT(g_gpio_cpuint >= 0);

  /* Attach the GPIO peripheral to the allocated CPU interrupt */

  up_disable_irq(g_gpio_cpuint);
  esp32s2_attach_peripheral(ESP32S2_PERI_GPIO_INT_PRO, g_gpio_cpuint);

  /* Attach and enable the interrupt handler */

  DEBUGVERIFY(irq_attach(ESP32S2_PERI_GPIO_INT_PRO, gpio_interrupt, NULL));
  up_enable_irq(g_gpio_cpuint);
}
#endif

/****************************************************************************
 * Name: esp32s2_gpioirqenable
 *
 * Description:
 *   Enable the COPY interrupt for specified GPIO IRQ
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

  /* Get the address of the GPIO PIN register for this pin */

  up_disable_irq(g_gpio_cpuint);

  regaddr = GPIO_REG(pin);
  regval  = getreg32(regaddr);
  regval &= ~(GPIO_PIN_INT_ENA_M | GPIO_PIN_INT_TYPE_M);

  /* Set the pin ENA field:
   *
   *   Bit 0: APP CPU interrupt enable
   *   Bit 1: APP CPU non-maskable interrupt enable
   *   Bit 3: PRO CPU interrupt enable
   *   Bit 4: PRO CPU non-maskable interrupt enable
   *   Bit 5: SDIO's extent interrupt enable.
   */

  /* PRO_CPU */

  regval |= ((1 << 2) << GPIO_PIN_INT_ENA_S);

  regval |= (intrtype << GPIO_PIN_INT_TYPE_S);
  putreg32(regval, regaddr);

  up_enable_irq(g_gpio_cpuint);
}
#endif

/****************************************************************************
 * Name: esp32s2_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
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

  /* Get the address of the GPIO PIN register for this pin */

  up_disable_irq(g_gpio_cpuint);

  regaddr = GPIO_REG(pin);
  regval  = getreg32(regaddr);
  regval &= ~(GPIO_PIN_INT_ENA_M | GPIO_PIN_INT_TYPE_M);
  putreg32(regval, regaddr);

  up_enable_irq(g_gpio_cpuint);
}
#endif

/****************************************************************************
 * Name: esp32s2_gpio_matrix_in
 *
 * Description:
 *   Set gpio input to a signal
 *   NOTE: one gpio can input to several signals
 *   If gpio == 0x30, cancel input to the signal, input 0 to signal
 *   If gpio == 0x38, cancel input to the signal, input 1 to signal,
 *   for I2C pad
 *
 ****************************************************************************/

void esp32s2_gpio_matrix_in(uint32_t gpio, uint32_t signal_idx, bool inv)
{
  uint32_t regaddr = GPIO_FUNC0_IN_SEL_CFG_REG + (signal_idx * 4);
  uint32_t regval = (gpio << GPIO_FUNC0_IN_SEL_S);

  if (inv)
    {
      regval |= GPIO_FUNC0_IN_INV_SEL;
    }

  if (gpio != 0x34)
    {
      regval |= GPIO_SIG0_IN_SEL;
    }

  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: esp32s2_gpio_matrix_out
 *
 * Description:
 *   Set signal output to gpio
 *   NOTE: one signal can output to several gpios
 *   If signal_idx == 0x100, cancel output put to the gpio
 *
 ****************************************************************************/

void esp32s2_gpio_matrix_out(uint32_t gpio, uint32_t signal_idx,
                             bool out_inv, bool oen_inv)
{
  uint32_t regaddr = GPIO_FUNC0_OUT_SEL_CFG_REG + (gpio * 4);
  uint32_t regval = signal_idx << GPIO_FUNC0_OUT_SEL_S;

  if (gpio >= GPIO_PIN_COUNT)
    {
      return;
    }

  if (gpio < 32)
    {
      putreg32((1ul << gpio), GPIO_ENABLE_W1TS_REG);
    }
  else
    {
      putreg32((1ul << (gpio - 32)), GPIO_ENABLE1_W1TS_REG);
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
