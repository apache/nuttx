/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_gpio.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "lpc43_gpio.h"

#ifdef CONFIG_LPC43_GPIO_IRQ
#include "lpc43_gpioint.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_configinput
 *
 * Description:
 *   Configure a GPIO pin as an input (or pre-configured the pin for an
 *   interrupt).
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Interrupts are disabled so that read-modify-write operations are safe.
 *
 ****************************************************************************/

static inline void lpc43_configinput(uint16_t gpiocfg,
                                     unsigned int port, unsigned int pin)
{
  uintptr_t regaddr;
  uint32_t regval;

  /* Then configure the pin as a normal input by clearing the corresponding
   * bit in the GPIO DIR register for the port.
   */

  regaddr = LPC43_GPIO_DIR(port);
  regval  = getreg32(regaddr);
  regval &= ~GPIO_DIR(pin);
  putreg32(regval, regaddr);

  /* To be able to read the signal on the GPIO input, the input
   * buffer must be enabled in the syscon block for the corresponding pin.
   * This should have been done when the pin was configured as a GPIO.
   */
}

/****************************************************************************
 * Name: lpc43_configoutput
 *
 * Description:
 *   Configure a GPIO pin as an output.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Interrupts are disabled so that read-modify-write operations are safe.
 *
 ****************************************************************************/

static inline void lpc43_configoutput(uint16_t gpiocfg,
                                      unsigned int port, unsigned int pin)
{
  uintptr_t regaddr;
  uint32_t regval;

  /* Then configure the pin as an output by setting the corresponding
   * bit in the GPIO DIR register for the port.
   */

  regaddr = LPC43_GPIO_DIR(port);
  regval  = getreg32(regaddr);
  regval |= GPIO_DIR(pin);
  putreg32(regval, regaddr);

  /* Set the initial value of the output */

  lpc43_gpio_write(gpiocfg, GPIO_IS_ONE(gpiocfg));

  /* To be able to read the signal on the GPIO input, the input
   * buffer must be enabled in the syscon block for the corresponding pin.
   * This should have been done when the pin was configured as a GPIO.
   */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_gpio_config
 *
 * Description:
 *   Configure a GPIO based on bit-encoded description of the pin.  NOTE:
 *   The pin *must* have first been configured for GPIO usage with a
 *   corresponding call to lpc43_pin_config().
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

int lpc43_gpio_config(uint16_t gpiocfg)
{
  unsigned int port  = ((gpiocfg & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT);
  unsigned int pin   = ((gpiocfg & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT);
  irqstate_t   flags;
  int          ret   = OK;

  DEBUGASSERT(port < NUM_GPIO_PORTS && pin < NUM_GPIO_PINS);

  /* Handle the GPIO configuration by the basic mode of the pin */

  flags = enter_critical_section();
  switch (gpiocfg & GPIO_MODE_MASK)
    {
      case GPIO_MODE_INPUT:     /* GPIO input pin */
        lpc43_configinput(gpiocfg, port, pin);
        break;

      case GPIO_MODE_OUTPUT:     /* GPIO output pin */
        lpc43_configoutput(gpiocfg, port, pin);
        break;

      case GPIO_MODE_PININTR:    /* GPIO pin interrupt */
        lpc43_configinput(gpiocfg, port, pin);
#ifdef CONFIG_LPC43_GPIO_IRQ
        ret = lpc43_gpioint_pinconfig(gpiocfg);
#endif
        break;

      case GPIO_MODE_GRPINTR:    /* GPIO group interrupt */
        lpc43_configinput(gpiocfg, port, pin);
#ifdef CONFIG_LPC43_GPIO_IRQ
        ret = lpc43_gpioint_grpconfig(gpiocfg);
#endif
        break;

      default :
        serr("ERROR: Unrecognized pin mode: %04x\n", gpiocfg);
        ret = -EINVAL;
        break;
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: lpc43_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void lpc43_gpio_write(uint16_t gpiocfg, bool value)
{
  unsigned int port = ((gpiocfg & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT);
  unsigned int pin  = ((gpiocfg & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT);

  DEBUGASSERT(port < NUM_GPIO_PORTS && pin < NUM_GPIO_PINS);

  /* Write the value (0 or 1).  To the pin byte register */

  putreg8((uint8_t)value, LPC43_GPIO_B(port, pin));
}

/****************************************************************************
 * Name: lpc43_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 * Returned Value:
 *   The boolean state of the input pin
 *
 ****************************************************************************/

bool lpc43_gpio_read(uint16_t gpiocfg)
{
  unsigned int port = ((gpiocfg & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT);
  unsigned int pin  = ((gpiocfg & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT);

  DEBUGASSERT(port < NUM_GPIO_PORTS && pin < NUM_GPIO_PINS);

  /* Get the value of the pin from the pin byte register */

  return (getreg8(LPC43_GPIO_B(port, pin)) & GPIO_B) != 0;
}
