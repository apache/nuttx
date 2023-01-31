/****************************************************************************
 * arch/risc-v/src/litex/litex_gpio.c
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
#include <string.h> /* To access ffs() */
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <arch/irq.h>

#include "riscv_internal.h"
#include "litex_gpio.h"
#include "litex_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO_REG_OE_OFFSET          0x00
#define GPIO_REG_IN_OFFSET          0x04
#define GPIO_REG_OUT_OFFSET         0x08
#define GPIO_REG_MODE_OFFSET        0x0C
#define GPIO_REG_EDGE_OFFSET        0x10
#define GPIO_REG_EV_STATUS_OFFSET   0x14
#define GPIO_REG_EV_PEND_OFFSET     0x18
#define GPIO_REG_EV_ENABLE_OFFSET   0x1C

#ifdef CONFIG_LITEX_GPIO_IRQ
/* Helper to calculate the GPIO port index from an IRQ number. */
#define irq_to_gpio_index(_irqno) (( _irqno - LITEX_IRQ_GPIO) % 32)

/* Helper to calculate the extended IRQ number from GPIO  port index. */
#define gpio_index_to_irq(_i, _p) ((_i * 32) + _p + LITEX_FIRST_GPIOIRQ)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

static uint32_t gpiobase_at(uint32_t index);

#ifdef CONFIG_LITEX_GPIO_IRQ
static void gpio_dispatch(int irq, uint32_t gpiobase, uint32_t * regs);
static int litex_gpio_interrupt(int irq, void *context, void * arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A lookup table to keep track of all the base addresses for all gateware
 * defined GPIO peripherals. This saves calculating the actual base address
 * in the ISR routines, as there is only an IRQ number available at that
 * point.
 *
 * The actual memory addresses are populated the first time it is used.
 */

static uint32_t g_gpio_base[LITEX_GPIO_MAX] =
{
  0
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t gpiobase_at(uint32_t index)
{
  DEBUGASSERT(index < LITEX_GPIO_MAX);
  if (g_gpio_base[0] == 0)
    {
      for (int i = 0; i < LITEX_GPIO_MAX; i++)
        {
          g_gpio_base[i] = LITEX_GPIO_BASE +
                          (LITEX_GPIO_OFFSET * i);
        }
    }

  return g_gpio_base[index];
}

#ifdef CONFIG_LITEX_GPIO_IRQ
static void gpio_dispatch(int irq, uint32_t gpiobase, uint32_t * regs)
{
  int i;
  int ndx = 0;
  uint32_t bitmask;

  /* Query which pin interrupts are enabled */

  uint32_t enabled = getreg32(gpiobase + GPIO_REG_EV_ENABLE_OFFSET);

  /* Query which interrupts are pending */

  uint32_t pending = getreg32(gpiobase + GPIO_REG_EV_PEND_OFFSET);

  while ((i = ffs(pending)))
    {
      ndx += i;
      bitmask = 0x1 << (ndx - 1);

      /* Clear the pending interrupt */

      modifyreg32(gpiobase + GPIO_REG_EV_PEND_OFFSET, bitmask, bitmask);

      /* Dispatch the appropriate interrupt handler if the ISR is enabled. */

      if (enabled & bitmask)
        {
          irq_dispatch(irq + ndx - 1, regs);
        }

      pending >>= i;
    }
}
#endif

#ifdef CONFIG_LITEX_GPIO_IRQ
static int litex_gpio_interrupt(int irq, void * context, void * arg)
{
  uint32_t gpiobase = 0;
  uint32_t gpioindex = irq_to_gpio_index(irq);

  DEBUGASSERT(gpioindex < LITEX_GPIO_MAX);

  gpiobase = gpiobase_at(gpioindex);

  gpio_dispatch(LITEX_FIRST_GPIOIRQ, gpiobase, (uint32_t *)context);
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: litex_gpio_config
 *
 * Description:
 *   Configure a GPIO pin based on encoded pin attributes. GPIO's currently
 *   only support a single attribute.
 *
 * Input Parameters:
 *   port      - The GPIO port number to read from.
 *   pin       - The pin, or bit offset inside the GPIO port.
 *   attr      - The pin attribute to apply to the pin.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int litex_gpio_config(uint32_t port, uint32_t pin, gpio_pinattr_t attr)
{
  int ret = OK;
  int bitmask = 0x1 << pin;
  switch (attr)
  {
     case GPIO_PINMODE_INPUT:
      modifyreg32(gpiobase_at(port) + GPIO_REG_OE_OFFSET, bitmask, 0);
      break;
    case GPIO_PINMODE_OUTPUT:
      modifyreg32(gpiobase_at(port) + GPIO_REG_OE_OFFSET, bitmask, bitmask);
      break;
    default:
      ret = -ENODEV;
      break;
  }

  return ret;
}

/****************************************************************************
 * Name: litex_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin.
 *
 * Input Parameters:
 *   port      - The GPIO port number to read from.
 *   pin       - The pin, or bit offset inside the GPIO port.
 *
 ****************************************************************************/

void litex_gpio_write(uint32_t port, uint32_t pin, bool value)
{
    modifyreg32(gpiobase_at(port) + GPIO_REG_OUT_OFFSET, 0x1 << pin,
                                              (uint32_t)value << pin);
}

/****************************************************************************
 * Name: litex_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin.
 *
 * Input Parameters:
 *   port      - The GPIO port number to read from.
 *   pin       - The pin, or bit offset inside the GPIO port.
 *
 * Returned Value:
 *   True if the pin is logic high. False if the pin is logic low.
 *
 ****************************************************************************/

bool litex_gpio_read(uint32_t port, uint32_t pin)
{
  return (getreg32(gpiobase_at(port) + GPIO_REG_IN_OFFSET) >> pin) & 0x1;
}

/****************************************************************************
 * Name: litex_gpio_irq_enable
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   GPIO pins on the GPIO port.
 *
 * Input Parameters:
 *   irq       - The IRQ number to enable. Must be a hardware interrupt
 *               number as defined in irq.h.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_LITEX_GPIO_IRQ
int litex_gpio_irq_enable(int irq)
{
  int ret;
  up_disable_irq(irq);
  ret = irq_attach(irq, litex_gpio_interrupt, NULL);
  if (ret == OK)
    {
      up_enable_irq(irq);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: litex_gpio_irq_configure
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ.
 *
 * Input Parameters:
 *   config      - The interrupt configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 ****************************************************************************/

#ifdef CONFIG_LITEX_GPIO_IRQ
int litex_gpio_irq_config(struct gpio_isr_config_s * config)
{
  int bitmask = 0x1 << config->pin;
  int irq = gpio_index_to_irq(config->port, config->pin);
  int ret = OK;
  uint32_t gpiobase = gpiobase_at(config->port);

  if (config->type & GPIO_ISR_NONE)
    {
      modifyreg32(gpiobase + GPIO_REG_EV_PEND_OFFSET, bitmask, bitmask);
      modifyreg32(gpiobase + GPIO_REG_EV_ENABLE_OFFSET, bitmask, 0);
      return OK;
    }

  if (config->handler)
    {
      ret = irq_attach(irq, config->handler, config->userdata);
    }

  if (ret != OK)
    {
      return ret;
    }

/* There are two registers which control the event type.
 * - MODE: 0 sets single edge, 1 sets both edges.
 * - EDGE: (only when  mode = 0) 0 sets rising edge, 0 sets falling edge.
 */

  if (config->type & GPIO_ISR_BOTH_EDGES)
    {
      modifyreg32(gpiobase + GPIO_REG_MODE_OFFSET, bitmask, bitmask);
    }
  else
    {
      modifyreg32(gpiobase + GPIO_REG_MODE_OFFSET, bitmask, 0);
      modifyreg32(gpiobase + GPIO_REG_EDGE_OFFSET, bitmask,
                    (config->type & GPIO_ISR_FALLING_EDGE) ? bitmask : 0);
    }

  modifyreg32(gpiobase + GPIO_REG_EV_PEND_OFFSET, bitmask, bitmask);
  modifyreg32(gpiobase + GPIO_REG_EV_ENABLE_OFFSET, bitmask, bitmask);
  return OK;
}
#endif

/****************************************************************************
 * Name: litex_gpio_irq_disable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ port.
 *
 * Input Parameters:
 *   irq       - The IRQ number to disable. Must be a hardware interrupt
 *               number as defined in irq.h.
 *
 ****************************************************************************/

#ifdef CONFIG_LITEX_GPIO_IRQ
void litex_gpio_irq_disable(int irq)
{
  up_disable_irq(irq);
}
#endif
