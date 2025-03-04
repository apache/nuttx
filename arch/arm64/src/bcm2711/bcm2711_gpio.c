/****************************************************************************
 * arch/arm64/src/bcm2711/bcm2711_gpio.c
 *
 * Author: Matteo Golin <matteo.golin@gmail.com>
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

#include <nuttx/arch.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include "arm64_arch.h"
#include "arm64_gic.h"
#include "arm64_internal.h"
#include "bcm2711_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Number of IRQs for GPIO interrupts */

#define NUM_GPIO_IRQS (sizeof(g_gpio_irqs) / sizeof(g_gpio_irqs[0]))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* True if the primary interrupt handler (which call pin-specific handlers)
 * has already been attached to GPIO 0-3 IRQs. False otherwise.
 */

static bool g_gpio_irqs_init = false;

/* GPIO pin interrupt handler functions. */

static xcpt_t g_gpio_pin_isrs[BCM_GPIO_NUM] =
{
  0
};

/* Arguments to GPIO pin interrupt handlers. */

static void *g_gpio_pin_isr_args[BCM_GPIO_NUM] =
{
  0
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* TODO: Which GPIO pins are associated with VC IRQ 49 (GPIO 0), 50, 51 and
 * 52?
 */

/* TODO: is it necessary to encode the alternate function possibilities for
 * each pin in a lookup table?
 */

/* Mapping from function selection enum to BCM2711 function selection
 * representation.
 */

static const uint8_t g_fsel_map[] =
{
  [BCM_GPIO_FUNC0] = BCM_GPIO_FS_ALT0, [BCM_GPIO_FUNC1] = BCM_GPIO_FS_ALT1,
  [BCM_GPIO_FUNC2] = BCM_GPIO_FS_ALT2, [BCM_GPIO_FUNC3] = BCM_GPIO_FS_ALT3,
  [BCM_GPIO_FUNC4] = BCM_GPIO_FS_ALT4, [BCM_GPIO_FUNC5] = BCM_GPIO_FS_ALT5,
  [BCM_GPIO_INPUT] = BCM_GPIO_FS_IN,   [BCM_GPIO_OUTPUT] = BCM_GPIO_FS_OUT,
};

/* IRQs for GPIO interrupts */

static const uint32_t g_gpio_irqs[] =
{
  BCM_IRQ_VC_GPIO0,
  BCM_IRQ_VC_GPIO1,
  BCM_IRQ_VC_GPIO2,
  BCM_IRQ_VC_GPIO3,
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline void bcm2711_gpio_help_set(uint32_t gpio, uint32_t reg1,
                                         uint32_t reg2, bool val);
static inline bool bcm2711_gpio_help_get(uint32_t gpio, uint32_t reg1,
                                         uint32_t reg2);

static int bcm2711_gpio_interrupt_handler(int irq, void *context, void *arg);
static int bcm2711_gpio_irqs_init(void);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcm2711_gpio_help_set
 *
 * Description:
 *   Helper function for setting a GPIO pin value on the BCM2711 GPIO
 *   register where one register is for pins 0-31 and the second is pins
 *   32-57.
 *
 * Input parameters:
 *   gpio - The GPIO pin number to set the value of.
 *   reg1 - The GPIO register for pins 0-31.
 *   reg2 - The GPIO register for pins 0-33.
 *   val - The value to set (true for 1, false for 0).
 *
 ****************************************************************************/

static inline void bcm2711_gpio_help_set(uint32_t gpio, uint32_t reg1,
                                         uint32_t reg2, bool val)
{
  /* Choose appropriate bit and register */

  uint32_t bitmask;
  uint32_t reg;

  if (gpio <= 31)
    {
      bitmask = (1 << gpio);
      reg = reg1;
    }
  else
    {
      bitmask = (1 << (gpio - 32));
      reg = reg2;
    }

  /* Set or clear bit */

  if (val)
    {
      modreg32(bitmask, bitmask, reg);
    }
  else
    {
      modreg32(0, bitmask, reg);
    }
}

/****************************************************************************
 * Name: bcm2711_gpio_help_get
 *
 * Description:
 *   Helper function for reading a GPIO pin value on the BCM2711 GPIO
 *   register where one register is for pins 0-31 and the second is pins
 *   32-57.
 *
 * Input parameters:
 *   gpio - The GPIO pin number to get the value of.
 *   reg1 - The GPIO register for pins 0-31.
 *   reg2 - The GPIO register for pins 0-33.
 *
 ****************************************************************************/

static inline bool bcm2711_gpio_help_get(uint32_t gpio, uint32_t reg1,
                                         uint32_t reg2)
{
  /* Choose appropriate bit and register */

  uint32_t bitmask;
  uint32_t reg;

  if (gpio <= 31)
    {
      bitmask = (1 << gpio);
      reg = reg1;
    }
  else
    {
      bitmask = (1 << (gpio - 32));
      reg = reg2;
    }

  return getreg32(reg) & bitmask;
}

/****************************************************************************
 * Name: bcm2711_gpio_interrupt_handler
 *
 * Description:
 *   Interrupt handler for GPIO 1, GPIO 2 and GPIO 3 IRQs.
 *
 * Input parameters:
 *   irq - The IRQ number
 *   context - The interrupt context.
 *   arg - The argument passed to the interrupt handler.
 *
 ****************************************************************************/

static int bcm2711_gpio_interrupt_handler(int irq, void *context, void *arg)
{
  /* TODO: depending on irq number, decide which GPIO handlers to search
   * through and call
   */

  /* TODO: test interrupt handling */

  xcpt_t isr;

  /* Since I don't know which IRQ corresponds to which GPIO pins, for now
   * I'll search all 58 GPIOs on an interrupt
   */

  for (uint32_t gpio = 0; gpio < BCM_GPIO_NUM; gpio++)
    {
      isr = g_gpio_pin_isrs[gpio];
      if (bcm2711_gpio_event_get(gpio) && isr != NULL)
        {
          isr(gpio, context, g_gpio_pin_isr_args[gpio]);
        }
    }

  return 0;
}

/****************************************************************************
 * Name: bcm2711_gpio_irqs_init
 *
 * Description:
 *   Attach the primary GPIO interrupt handler to GPIO 0, 1, 2 and 3 IRQs.
 *
 * Returns:
 *    0 if successful, negated errno otherwise.
 *
 ****************************************************************************/

static int bcm2711_gpio_irqs_init(void)
{
  int err;

  /* Attach all interrupt handlers for primary IRQs. */

  for (int i = 0; i < NUM_GPIO_IRQS; i++)
    {
      err = irq_attach(g_gpio_irqs[i], bcm2711_gpio_interrupt_handler, NULL);
      if (err) return err;
    }

  /* Enable all GPIO IRQs. */

  for (int i = 0; i < NUM_GPIO_IRQS; i++)
    {
      up_enable_irq(g_gpio_irqs[i]);
      arm64_gic_irq_set_priority(g_gpio_irqs[i], 0, IRQ_TYPE_LEVEL);
    }

  /* Mark as initialized. */

  g_gpio_irqs_init = true;
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcm2711_gpio_set_pulls
 *
 * Description:
 *   Set the specified GPIO pin to have pull up, pull down or no
 *   resistor. With both `up` and `down` as false, the resistor will be set
 *   to none. It is not possible to set both pull-up and pull-down.
 *
 * Input parameters:
 *   gpio - The GPIO pin number to set the resistors on.
 *   up - True to set pull-up resistor, false otherwise.
 *   down - True to set pull-down resistor, false otherwise.
 *
 ****************************************************************************/

void bcm2711_gpio_set_pulls(uint32_t gpio, bool up, bool down)
{
  DEBUGASSERT(gpio < BCM_GPIO_NUM);
  DEBUGASSERT(!(up && down)); /* Not valid to set pull-up and pull-down */

  /* Pick direction. */

  uint32_t direction = 0;
  if (up)
    {
      direction = BCM_GPIO_PULLUP;
    }
  else if (down)
    {
      direction = BCM_GPIO_PULLDOWN;
    }
  else
    {
      direction = BCM_GPIO_NORES;
    }

  /* Set GPIO pin resistor. */

  uint32_t value = 0;
  if (gpio <= 15)
    {
      value = (direction << (gpio * 2));
      modreg32(value, value, BCM_GPIO_PUP_PDN_CNTRL_REG0);
    }
  else if (gpio <= 31 && gpio > 15)
    {
      value = (direction << ((gpio - 16) * 2));
      modreg32(value, value, BCM_GPIO_PUP_PDN_CNTRL_REG1);
    }
  else if (gpio <= 47 && gpio > 31)
    {
      value = (direction << ((gpio - 32) * 2));
      modreg32(value, value, BCM_GPIO_PUP_PDN_CNTRL_REG2);
    }
  else if (gpio <= 57 && gpio > 47)
    {
      value = (direction << ((gpio - 48) * 2));
      modreg32(value, value, BCM_GPIO_PUP_PDN_CNTRL_REG3);
    }
}

/****************************************************************************
 * Name: bcm2711_gpio_set_func
 *
 * Description:
 *   Set the specified GPIO pin to be input, output or use one of its
 *   alternative functions.
 *
 * Input parameters:
 *   gpio - The GPIO pin number to set the function of.
 *   func - The function to set the GPIO pin to use.
 *
 ****************************************************************************/

void bcm2711_gpio_set_func(uint32_t gpio, enum bcm2711_gpio_func_e func)
{
  DEBUGASSERT(gpio < BCM_GPIO_NUM);

  uint32_t value = 0;
  if (gpio <= 9)
    {
      value = (g_fsel_map[func] << (gpio * 3));
      modreg32(value, value, BCM_GPIO_GPFSEL0);
    }
  else if (gpio <= 19 && gpio > 9)
    {
      value = (g_fsel_map[func] << ((gpio - 10) * 3));
      modreg32(value, value, BCM_GPIO_GPFSEL1);
    }
  else if (gpio <= 29 && gpio > 20)
    {
      value = (g_fsel_map[func] << ((gpio - 20) * 3));
      modreg32(value, value, BCM_GPIO_GPFSEL2);
    }
  else if (gpio <= 39 && gpio > 30)
    {
      value = (g_fsel_map[func] << ((gpio - 30) * 3));
      modreg32(value, value, BCM_GPIO_GPFSEL3);
    }
  else if (gpio <= 49 && gpio > 40)
    {
      value = (g_fsel_map[func] << ((gpio - 40) * 3));
      modreg32(value, value, BCM_GPIO_GPFSEL4);
    }
  else if (gpio <= 57 && gpio > 50)
    {
      value = (g_fsel_map[func] << ((gpio - 50) * 3));
      modreg32(value, value, BCM_GPIO_GPFSEL5);
    }
}

/****************************************************************************
 * Name: bcm2711_gpio_pin_set
 *
 * Description:
 *   Set the output of a GPIO output pin to high or low.
 *   Calling this function on a GPIO pin set as an input does nothing.
 *
 * Input parameters:
 *   gpio - The GPIO pin number to set high or low.
 *   high  - True to set the pin high, false to set the pin low.
 *
 ****************************************************************************/

void bcm2711_gpio_pin_set(uint32_t gpio, bool set)
{
  DEBUGASSERT(gpio < BCM_GPIO_NUM);

  if (set)
    {
      bcm2711_gpio_help_set(gpio, BCM_GPIO_GPSET0, BCM_GPIO_GPSET0, true);
    }
  else
    {
      bcm2711_gpio_help_set(gpio, BCM_GPIO_GPCLR0, BCM_GPIO_GPCLR0, true);
    }
}

/****************************************************************************
 * Name: bcm2711_gpio_pin_get
 *
 * Description:
 *   Get the current value of the GPIO.
 *
 * Input parameters:
 *   gpio - The GPIO pin number to set high or low.
 *
 * Return:
 *    True for high, false for low.
 *
 ****************************************************************************/

bool bcm2711_gpio_pin_get(uint32_t gpio)
{
  DEBUGASSERT(gpio < BCM_GPIO_NUM);

  if (gpio <= 31)
    {
      return getreg32(BCM_GPIO_GPLEV0) & (1 << gpio);
    }
  else
    {
      return getreg32(BCM_GPIO_GPLEV1) & (1 << (gpio - 32));
    }
}

/****************************************************************************
 * Name: bcm2711_gpio_event_get
 *
 * Description:
 *   Check if an event was detected for the given GPIO pin.
 *   The event bit will be set if an event has happened that matches the
 *   event detection configuration for the given pin (rising edge, falling
 *   edge, level).
 *
 * Input parameters:
 *   gpio - The GPIO pin number to check for an event.
 *
 * Return:
 *    True if an event was detected, false otherwise.
 *
 ****************************************************************************/

bool bcm2711_gpio_event_get(uint32_t gpio)
{
  DEBUGASSERT(gpio < BCM_GPIO_NUM);
  return bcm2711_gpio_help_get(gpio, BCM_GPIO_GPEDS0, BCM_GPIO_GPEDS1);
}

/****************************************************************************
 * Name: bcm2711_gpio_event_clear
 *
 * Description:
 *   Clear the event detect status for the given GPIO pin.
 *
 * Input parameters:
 *   gpio - The GPIO pin number to clear the event status of.
 *
 ****************************************************************************/

void bcm2711_gpio_event_clear(uint32_t gpio)
{
  DEBUGASSERT(gpio < BCM_GPIO_NUM);
  bcm2711_gpio_help_set(gpio, BCM_GPIO_GPEDS0, BCM_GPIO_GPEDS1, false);
}

/****************************************************************************
 * Name: bcm2711_gpio_rising_edge
 *
 * Description:
 *   Set/clear rising edge event detection for the given GPIO pin.
 *
 * Input parameters:
 *   gpio - The GPIO pin number to set the event detection of.
 *   set - True to set, false to clear.
 *
 ****************************************************************************/

void bcm2711_gpio_rising_edge(uint32_t gpio, bool set)
{
  DEBUGASSERT(gpio < BCM_GPIO_NUM);
  bcm2711_gpio_help_set(gpio, BCM_GPIO_GPREN0, BCM_GPIO_GPREN1, set);
}

/****************************************************************************
 * Name: bcm2711_gpio_falling_edge
 *
 * Description:
 *   Set/clear falling edge event detection for the given GPIO pin.
 *
 * Input parameters:
 *   gpio - The GPIO pin number to set the event detection of.
 *   set - True to set, false to clear.
 *
 ****************************************************************************/

void bcm2711_gpio_falling_edge(uint32_t gpio, bool set)
{
  DEBUGASSERT(gpio < BCM_GPIO_NUM);
  bcm2711_gpio_help_set(gpio, BCM_GPIO_GPFEN0, BCM_GPIO_GPFEN1, set);
}

/****************************************************************************
 * Name: bcm2711_gpio_high_level
 *
 * Description:
 *   Set/clear high level event detection for the given GPIO pin.
 *
 * Input parameters:
 *   gpio - The GPIO pin number to set the event detection of.
 *   set - True to set, false to clear.
 *
 ****************************************************************************/

void bcm2711_gpio_high_level(uint32_t gpio, bool set)
{
  DEBUGASSERT(gpio < BCM_GPIO_NUM);
  bcm2711_gpio_help_set(gpio, BCM_GPIO_GPHEN0, BCM_GPIO_GPHEN1, set);
}

/****************************************************************************
 * Name: bcm2711_gpio_low_level
 *
 * Description:
 *   Set/clear low level event detection for the given GPIO pin.
 *
 * Input parameters:
 *   gpio - The GPIO pin number to set the event detection of.
 *   set - True to set, false to clear.
 *
 ****************************************************************************/

void bcm2711_gpio_low_level(uint32_t gpio, bool set)
{
  DEBUGASSERT(gpio < BCM_GPIO_NUM);
  bcm2711_gpio_help_set(gpio, BCM_GPIO_GPLEN0, BCM_GPIO_GPLEN1, set);
}

/****************************************************************************
 * Name: bcm2711_gpio_rising_edge_async
 *
 * Description:
 *   Set/clear async rising edge event detection for the given GPIO pin.
 *
 * Input parameters:
 *   gpio - The GPIO pin number to set the event detection of.
 *   set - True to set, false to clear.
 *
 ****************************************************************************/

void bcm2711_gpio_rising_edge_async(uint32_t gpio, bool set)
{
  DEBUGASSERT(gpio < BCM_GPIO_NUM);
  bcm2711_gpio_help_set(gpio, BCM_GPIO_GPAREN0, BCM_GPIO_GPAREN1, set);
}

/****************************************************************************
 * Name: bcm2711_gpio_falling_edge_async
 *
 * Description:
 *   Set/clear async falling edge event detection for the given GPIO pin.
 *
 * Input parameters:
 *   gpio - The GPIO pin number to set the event detection of.
 *   set - True to set, false to clear.
 *
 ****************************************************************************/

void bcm2711_gpio_falling_edge_async(uint32_t gpio, bool set)
{
  DEBUGASSERT(gpio < BCM_GPIO_NUM);
  bcm2711_gpio_help_set(gpio, BCM_GPIO_GPAFEN0, BCM_GPIO_GPAFEN1, set);
}

/****************************************************************************
 * Name: bcm2711_gpio_irq_attach
 *
 * Description:
 *   Attach an interrupt handler for the specified GPIO pin.
 *   NOTE: Interrupt mode (rising edge, falling edge, etc.) is configured
 *   separately. Once configured, the IRQ is enabled for that event type.
 *
 * Input parameters:
 *   gpio - The GPIO pin number to attach the handler for.
 *   isr - The interrupt handler function.
 *   arg - The argument to be passed to the interrupt handler.
 *
 ****************************************************************************/

int bcm2711_gpio_irq_attach(uint32_t gpio, xcpt_t isr, void *arg)
{
  DEBUGASSERT(gpio < BCM_GPIO_NUM);
  int ret = 0;

  /* If primary interrupt handler has not been attached to IRQs yet, do that
   * first.
   */

  if (!g_gpio_irqs_init)
    {
      ret = bcm2711_gpio_irqs_init();
      if (ret < 0) return ret; /* Early return if primary attach failed. */
    }

  /* Save handler information for this pin. */

  g_gpio_pin_isrs[gpio] = isr;
  g_gpio_pin_isr_args[gpio] = arg;

  /* Clear pending interrupts for this pin. */

  bcm2711_gpio_event_clear(gpio);

  return ret;
}

/****************************************************************************
 * Name: bcm2711_gpio_irq_detach
 *
 * Description:
 *   Detach an interrupt handler for a GPIO pin. NOTE: this does not disable
 *   interrupts for that particular pin; this must be done by disabling event
 *   detection for that pin separately.
 *   This function just detaches the pin's ISR, ensuring it won't be called
 *   when an interrupt is triggered.
 *
 * Input parameters:
 *   gpio - The GPIO pin number to detach the handler of.
 *
 ****************************************************************************/

void bcm2711_gpio_irq_detach(uint32_t gpio)
{
  DEBUGASSERT(gpio < BCM_GPIO_NUM);
  g_gpio_pin_isrs[gpio] = NULL;
  g_gpio_pin_isr_args[gpio] = NULL;
}
