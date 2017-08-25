/****************************************************************************
 * arch/xtensa/src/esp32/esp32_gpio.c
 *
 * Developed for NuttX by:
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derivies from sample code provided by Expressif Systems:
 *
 *   Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <arch/irq.h>

#include "xtensa.h"
#include "chip/esp32_iomux.h"
#include "chip/esp32_gpio.h"
#include "esp32_cpuint.h"
#include "esp32_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NGPIO_HPINS  (ESP32_NIRQ_GPIO - 32)
#define NGPIO_HMASK  ((1ul << NGPIO_HPINS) - 1)
#define _NA_         0xff

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ESP32_GPIO_IRQ
static uint8_t g_gpio_cpuint;
#endif

static const uint8_t g_pin2func[40] =
{
  0x44, 0x88, 0x40, 0x84, 0x48, 0x6c, 0x60, 0x64,  /* 0-7 */
  0x68, 0x54, 0x58, 0x5c, 0x34, 0x38, 0x30, 0x3c,  /* 8-15 */
  0x4c, 0x50, 0x70, 0x74, 0x78, 0x7c, 0x80, 0x8c,  /* 16-23 */
  _NA_, 0x24, 0x28, 0x2c, _NA_, _NA_, _NA_, _NA_,  /* N/A, 25-27, N/A, N/A, N/A, N/A */
  0x1c, 0x20, 0x14, 0x18, 0x04, 0x08, 0x0c, 0x10   /* 32-39 */
};

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

#ifdef CONFIG_ESP32_GPIO_IRQ
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

#ifdef CONFIG_ESP32_GPIO_IRQ
static int gpio_interrupt(int irq, FAR void *context, FAR void *arg)
{
  uint32_t status;

  /* Read and clear the lower GPIO interrupt status */

  status = getreg32(GPIO_STATUS_REG);
  putreg32(status, GPIO_STATUS_W1TC_REG);

  /* Dispatch pending interrupts in the lower GPIO status register */

  gpio_dispatch(ESP32_FIRST_GPIOIRQ, status, (uint32_t *)context);

  /* Read and clear the upper GPIO interrupt status */

  status = getreg32(GPIO_STATUS1_REG) & NGPIO_HMASK;
  putreg32(status, GPIO_STATUS1_W1TC_REG);

  /* Dispatch pending interrupts in the lower GPIO status register */

  gpio_dispatch(ESP32_FIRST_GPIOIRQ + 32, status, (uint32_t *)context);
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on encoded pin attributes.
 *
 ****************************************************************************/

int esp32_configgpio(int pin, gpio_pinattr_t attr)
{
  uintptr_t regaddr;
  uint32_t func;
  uint32_t cntrl;
  unsigned int pinmode;

  DEBUGASSERT(pin >=0 && pin <= ESP32_NIRQ_GPIO);

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

  else if ((attr & OUTPUT) != 0)
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

  /* Input enable... Required for output as well? */

  func |= FUN_IE;

  pinmode = (attr & PINMODE_MASK);
  if (pinmode == INPUT || pinmode == OUTPUT)
    {
      func |= (uint32_t)(2 << MCU_SEL_S);
    }
  else if ((attr & FUNCTION_MASK) == SPECIAL)
    {
      func |= (uint32_t)((((pin) == 1 || (pin) == 3) ? 0 : 1) << MCU_SEL_S);
    }
  else /* if ((attr & FUNCTION) != 0) */
    {
      func |= (uint32_t)((attr >> FUNCTION_SHIFT) << MCU_SEL_S);
    }

  if ((attr & OPEN_DRAIN) != 0)
    {
      cntrl = (1 << GPIO_PIN_PAD_DRIVER_S);
    }

  regaddr = DR_REG_IO_MUX_BASE + g_pin2func[pin];
  putreg32(func, regaddr);

  regaddr = GPIO_REG(pin);
  putreg32(cntrl, regaddr);
  return OK;
}

/****************************************************************************
 * Name: esp32_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void esp32_gpiowrite(int pin, bool value)
{
  DEBUGASSERT(pin >=0 && pin <= ESP32_NIRQ_GPIO);

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
 * Name: esp32_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool esp32_gpioread(int pin)
{
  uint32_t regval;

  DEBUGASSERT(pin >=0 && pin <= ESP32_NIRQ_GPIO);

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
 * Name: esp32_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   GPIO pins.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32_GPIO_IRQ
void esp32_gpioirqinitialize(void)
{
  int cpu;

  /* Allocate a level-sensitive, priority 1 CPU interrupt */

  g_gpio_cpuint = esp32_alloc_levelint(1);
  DEBUGASSERT(g_gpio_cpuint >= 0);

  /* Set up to receive peripheral interrupts on the current CPU */

#ifdef CONFIG_SMP
  cpu = up_cpu_index();
#else
  cpu = 0;
#endif

  /* Attach the GPIO peripheral to the allocated CPU interrupt */

  up_disable_irq(g_gpio_cpuint);
  esp32_attach_peripheral(cpu, ESP32_PERIPH_CPU_GPIO, g_gpio_cpuint);

  /* Attach and enable the interrupt handler */

  DEBUGVERIFY(irq_attach(ESP32_IRQ_CPU_GPIO, gpio_interrupt, NULL));
  up_enable_irq(g_gpio_cpuint);
}
#endif

/****************************************************************************
 * Name: esp32_gpioirqenable
 *
 * Description:
 *   Enable the COPY interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32_GPIO_IRQ
void esp32_gpioirqenable(int irq, gpio_intrtype_t intrtype)
{
  uintptr_t regaddr;
  uint32_t regval;
  int cpu;
  int pin;

  DEBUGASSERT(irq <= ESP32_FIRST_GPIOIRQ && irq <= ESP32_LAST_GPIOIRQ);

  /* Convert the IRQ number to a pin number */

  pin = ESP32_IRQ2PIN(irq);

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

  cpu = up_cpu_index();
  if (cpu == 0)
    {
      /* PRO_CPU */

      regval |= ((1 << 2) << GPIO_PIN_INT_ENA_S);
    }
  else
    {
      /* APP_CPU */

      regval |= ((1 << 0) << GPIO_PIN_INT_ENA_S);
    }

  regval |= (intrtype << GPIO_PIN_INT_TYPE_S);
  putreg32(regval, regaddr);

  up_enable_irq(g_gpio_cpuint);
}
#endif

/****************************************************************************
 * Name: esp32_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32_GPIO_IRQ
void esp32_gpioirqdisable(int irq)
{
  uintptr_t regaddr;
  uint32_t regval;
  int pin;

  DEBUGASSERT(irq <= ESP32_FIRST_GPIOIRQ && irq <= ESP32_LAST_GPIOIRQ);

  /* Convert the IRQ number to a pin number */

  pin = ESP32_IRQ2PIN(irq);

  /* Get the address of the GPIO PIN register for this pin */

  up_disable_irq(g_gpio_cpuint);

  regaddr = GPIO_REG(pin);
  regval  = getreg32(regaddr);
  regval &= ~(GPIO_PIN_INT_ENA_M | GPIO_PIN_INT_TYPE_M);
  putreg32(regval, regaddr);

  up_enable_irq(g_gpio_cpuint);
}
#endif
