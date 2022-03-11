/****************************************************************************
 * arch/arm/src/nrf52/nrf52_gpio.c
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
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spinlock.h>

#include "arm_internal.h"
#include "hardware/nrf52_gpio.h"
#include "nrf52_gpio.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_gpio_regget
 *
 * Description:
 *   Get a register address for given GPIO port and register offset
 *
 ****************************************************************************/

static inline uint32_t nrf52_gpio_regget(int port, uint32_t offset)
{
  uint32_t base = 0;

  /* Get base address for port */

  if (port == 0)
    {
      base = NRF52_GPIO_P0_BASE;
    }
#ifdef CONFIG_NRF52_HAVE_PORT1
  else if (port == 1)
    {
      base = NRF52_GPIO_P1_BASE;
    }
#endif

  return (base + offset);
}

/****************************************************************************
 * Name: nrf52_gpio_input
 *
 * Description:
 *   Configure a GPIO input pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

static inline void nrf52_gpio_input(unsigned int port, unsigned int pin)
{
  uint32_t offset;

  offset = nrf52_gpio_regget(port, NRF52_GPIO_DIRCLR_OFFSET);

  /* Configure the pin as an input */

  putreg32(1U << pin, offset);

  /* Enable input buffer */

  offset = nrf52_gpio_regget(port, NRF52_GPIO_PIN_CNF_OFFSET(pin));
  modifyreg32(offset, GPIO_CNF_INPUT, 0);
}

/****************************************************************************
 * Name: nrf52_gpio_output
 *
 * Description:
 *   Configure a GPIO output pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

static inline void nrf52_gpio_output(nrf52_pinset_t cfgset,
                                     unsigned int port, unsigned int pin)
{
  uint32_t offset;

  /* Disable input buffer */

  offset = nrf52_gpio_regget(port, NRF52_GPIO_PIN_CNF_OFFSET(pin));
  modifyreg32(offset, 0, GPIO_CNF_INPUT);

  offset = nrf52_gpio_regget(port, NRF52_GPIO_DIRSET_OFFSET);

  nrf52_gpio_write(cfgset, ((cfgset & GPIO_VALUE) != GPIO_VALUE_ZERO));

  /* Configure the pin as an output */

  putreg32(1U << pin, offset);
}

/****************************************************************************
 * Name: nrf52_gpio_mode
 *
 * Description:
 *   Configure a GPIO mode based on bit-encoded description of the pin.
 *
 ****************************************************************************/

static inline void nrf52_gpio_mode(nrf52_pinset_t cfgset,
                                   unsigned int port, unsigned int pin)
{
  uint32_t mode;
  uint32_t regval;
  uint32_t offset;

  offset = nrf52_gpio_regget(port, NRF52_GPIO_PIN_CNF_OFFSET(pin));

  mode = cfgset & GPIO_MODE_MASK;

  regval = getreg32(offset);
  regval &= ~GPIO_CNF_PULL_MASK;

  if (mode == GPIO_PULLUP)
    {
      regval |= GPIO_CNF_PULL_UP;
    }
  else if (mode == GPIO_PULLDOWN)
    {
      regval |= GPIO_CNF_PULL_DOWN;
    }

  putreg32(regval, offset);
}

/****************************************************************************
 * Name: nrf52_gpio_sense
 *
 * Description:
 *   Set SENSE configuration for an input pin
 *
 ****************************************************************************/

static inline void nrf52_gpio_sense(nrf52_pinset_t cfgset,
                                    unsigned int port, unsigned int pin)
{
  uint32_t mode;
  uint32_t regval;
  uint32_t offset;

  mode = cfgset & GPIO_SENSE_MASK;

  offset = nrf52_gpio_regget(port, NRF52_GPIO_PIN_CNF_OFFSET(pin));
  regval = getreg32(offset);

  regval &= ~GPIO_CNF_SENSE_MASK;

  if (mode == GPIO_SENSE_HIGH)
    {
      regval |= GPIO_CNF_SENSE_HIGH;
    }
  else if (mode == GPIO_SENSE_LOW)
    {
      regval |= GPIO_CNF_SENSE_LOW;
    }

  putreg32(regval, offset);
}

/****************************************************************************
 * Name: nrf52_gpio_drive
 *
 * Description:
 *   Set DRIVE configuration for a pin
 *
 ****************************************************************************/

static inline void nrf52_gpio_drive(nrf52_pinset_t cfgset,
                                    unsigned int port, unsigned int pin)
{
  uint32_t drive;
  uint32_t regval;
  uint32_t offset;

  drive = cfgset & GPIO_DRIVE_MASK;

  offset = nrf52_gpio_regget(port, NRF52_GPIO_PIN_CNF_OFFSET(pin));
  regval = getreg32(offset);

  regval &= ~GPIO_CNF_DRIVE_MASK;

  switch (drive)
    {
      case GPIO_DRIVE_S0S1:
        regval |= GPIO_CNF_DRIVE_S0S1;
        break;
      case GPIO_DRIVE_S0H1:
        regval |= GPIO_CNF_DRIVE_S0H1;
        break;
      case GPIO_DRIVE_S0D1:
        regval |= GPIO_CNF_DRIVE_S0D1;
        break;
      case GPIO_DRIVE_H0D1:
        regval |= GPIO_CNF_DRIVE_H0D1;
        break;
      case GPIO_DRIVE_H0H1:
        regval |= GPIO_CNF_DRIVE_H0H1;
        break;
      case GPIO_DRIVE_H0S1:
        regval |= GPIO_CNF_DRIVE_H0S1;
        break;
      case GPIO_DRIVE_D0H1:
        regval |= GPIO_CNF_DRIVE_D0H1;
        break;
      case GPIO_DRIVE_D0S1:
        regval |= GPIO_CNF_DRIVE_D0S1;
        break;
    }

  putreg32(regval, offset);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_gpio_config
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int nrf52_gpio_config(nrf52_pinset_t cfgset)
{
  unsigned int port = 0;
  unsigned int pin;
  irqstate_t flags;
  int ret = OK;

  /* Verify that this hardware supports the select GPIO port */

#ifdef CONFIG_NRF52_HAVE_PORT1
  port = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
#endif

  if (port < NRF52_GPIO_NPORTS)
    {
      /* Get the pin number and select the port configuration register for
       * that pin.
       */

      pin = GPIO_PIN_DECODE(cfgset);

      flags = spin_lock_irqsave(NULL);

      /* First, configure the port as a generic input so that we have a
       * known starting point and consistent behavior during the re-
       * configuration.
       */

      nrf52_gpio_input(port, pin);

      /* Set the mode bits */

      nrf52_gpio_mode(cfgset, port, pin);

      /* Set the drive bits (needed also for input pins
       * for some peripherals).
       */

      nrf52_gpio_drive(cfgset, port, pin);

      /* Handle according to pin function */

      switch (cfgset & GPIO_FUNC_MASK)
        {
        case GPIO_INPUT:   /* GPIO input pin */
          nrf52_gpio_sense(cfgset, port, pin);
          break;

        case GPIO_OUTPUT:  /* GPIO outpout pin */
          nrf52_gpio_output(cfgset, port, pin);
          break;

        default:
          ret = -EINVAL;
        }

      spin_unlock_irqrestore(NULL, flags);
    }
  else
    {
      ret = -EINVAL;
    }

  return ret;
}

/****************************************************************************
 * Name: nrf52_gpio_unconfig
 *
 * Description:
 *   Unconfigure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int nrf52_gpio_unconfig(nrf52_pinset_t cfgset)
{
  unsigned int pin;
  unsigned int port = 0;
  uint32_t offset;

  /* Get port and pin number */

  pin  = GPIO_PIN_DECODE(cfgset);
#ifdef CONFIG_NRF52_HAVE_PORT1
  port = GPIO_PORT_DECODE(cfgset);
#endif

  /* Get address offset */

  offset = nrf52_gpio_regget(port, NRF52_GPIO_PIN_CNF_OFFSET(pin));

  /* Configure as input and disconnect input buffer */

  putreg32(GPIO_CNF_INPUT, offset);

  return OK;
}

/****************************************************************************
 * Name: nrf52_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void nrf52_gpio_write(nrf52_pinset_t pinset, bool value)
{
  unsigned int pin;
  unsigned int port = 0;
  uint32_t offset;

  /* Get port and pin number */

  pin  = GPIO_PIN_DECODE(pinset);
#ifdef CONFIG_NRF52_HAVE_PORT1
  port = GPIO_PORT_DECODE(pinset);
#endif

  /* Get register address */

  if (value)
    {
      offset = nrf52_gpio_regget(port, NRF52_GPIO_OUTSET_OFFSET);
    }
  else
    {
      offset = nrf52_gpio_regget(port, NRF52_GPIO_OUTCLR_OFFSET);
    }

  /* Put register value */

  putreg32(1 << pin, offset);
}

/****************************************************************************
 * Name: nrf52_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool nrf52_gpio_read(nrf52_pinset_t pinset)
{
  unsigned int port;
  unsigned int pin;
  uint32_t regval;
  uint32_t offset;

  /* Get port and pin number */

  pin  = GPIO_PIN_DECODE(pinset);
  port = GPIO_PORT_DECODE(pinset);

  /* Get register address */

  offset = nrf52_gpio_regget(port, NRF52_GPIO_IN_OFFSET);

  /* Get register value */

  regval = getreg32(offset);

  return (regval >> pin) & 1UL;
}

/****************************************************************************
 * Name: nrf52_gpio_detectmode
 *
 * Description:
 *  Set DETECTMODE to either default or latched
 *
 ****************************************************************************/

void nrf52_gpio_detectmode(int port, enum nrf52_gpio_detectmode_e mode)
{
  uint32_t offset = nrf52_gpio_regget(port, NRF52_GPIO_DETECTMODE_OFFSET);

  putreg32(mode == NRF52_GPIO_DETECTMODE_DETECT ?
           GPIO_DETECTMODE_DEFAULT :
           GPIO_DETECTMODE_LDETECT, offset);
}
