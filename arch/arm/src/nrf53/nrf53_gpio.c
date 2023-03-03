/****************************************************************************
 * arch/arm/src/nrf53/nrf53_gpio.c
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
#include "hardware/nrf53_gpio.h"
#include "nrf53_gpio.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_gpio_regget
 *
 * Description:
 *   Get a register address for given GPIO port and register offset
 *
 ****************************************************************************/

static inline uint32_t nrf53_gpio_regget(int port, uint32_t offset)
{
  uint32_t base = 0;

  /* Get base address for port */

  if (port == 0)
    {
      base = NRF53_GPIO_P0_BASE;
    }
  else if (port == 1)
    {
      base = NRF53_GPIO_P1_BASE;
    }

  return (base + offset);
}

/****************************************************************************
 * Name: nrf53_gpio_input
 *
 * Description:
 *   Configure a GPIO input pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

static inline void nrf53_gpio_input(unsigned int port, unsigned int pin)
{
  uint32_t offset;

  offset = nrf53_gpio_regget(port, NRF53_GPIO_DIRCLR_OFFSET);

  /* Configure the pin as an input */

  putreg32(1U << pin, offset);

  /* Enable input buffer */

  offset = nrf53_gpio_regget(port, NRF53_GPIO_PIN_CNF_OFFSET(pin));
  modifyreg32(offset, GPIO_CNF_INPUT, 0);
}

/****************************************************************************
 * Name: nrf53_gpio_output
 *
 * Description:
 *   Configure a GPIO output pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

static inline void nrf53_gpio_output(nrf53_pinset_t cfgset,
                                     unsigned int port, unsigned int pin)
{
  uint32_t offset;

  /* Disable input buffer */

  offset = nrf53_gpio_regget(port, NRF53_GPIO_PIN_CNF_OFFSET(pin));
  modifyreg32(offset, 0, GPIO_CNF_INPUT);

  offset = nrf53_gpio_regget(port, NRF53_GPIO_DIRSET_OFFSET);

  nrf53_gpio_write(cfgset, ((cfgset & GPIO_VALUE) != GPIO_VALUE_ZERO));

  /* Configure the pin as an output */

  putreg32(1U << pin, offset);
}

/****************************************************************************
 * Name: nrf53_gpio_mode
 *
 * Description:
 *   Configure a GPIO mode based on bit-encoded description of the pin.
 *
 ****************************************************************************/

static inline void nrf53_gpio_mode(nrf53_pinset_t cfgset,
                                   unsigned int port, unsigned int pin)
{
  uint32_t mode;
  uint32_t regval;
  uint32_t offset;

  offset = nrf53_gpio_regget(port, NRF53_GPIO_PIN_CNF_OFFSET(pin));

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
 * Name: nrf53_gpio_sense
 *
 * Description:
 *   Set SENSE configuration for an input pin
 *
 ****************************************************************************/

static inline void nrf53_gpio_sense(nrf53_pinset_t cfgset,
                                    unsigned int port, unsigned int pin)
{
  uint32_t mode;
  uint32_t regval;
  uint32_t offset;

  mode = cfgset & GPIO_SENSE_MASK;

  offset = nrf53_gpio_regget(port, NRF53_GPIO_PIN_CNF_OFFSET(pin));
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
 * Name: nrf53_gpio_drive
 *
 * Description:
 *   Set DRIVE configuration for a pin
 *
 ****************************************************************************/

static inline void nrf53_gpio_drive(nrf53_pinset_t cfgset,
                                    unsigned int port, unsigned int pin)
{
  uint32_t drive;
  uint32_t regval;
  uint32_t offset;

  drive = cfgset & GPIO_DRIVE_MASK;

  offset = nrf53_gpio_regget(port, NRF53_GPIO_PIN_CNF_OFFSET(pin));
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
      case GPIO_DRIVE_EOS1:
        regval |= GPIO_CNF_DRIVE_EOS1;
        break;
      case GPIO_DRIVE_SOE1:
        regval |= GPIO_CNF_DRIVE_SOE1;
        break;
      case GPIO_DRIVE_EOE1:
        regval |= GPIO_CNF_DRIVE_EOE1;
        break;
      case GPIO_DRIVE_DOE1:
        regval |= GPIO_CNF_DRIVE_DOE1;
        break;
      case GPIO_DRIVE_EOD1:
        regval |= GPIO_CNF_DRIVE_EOD1;
        break;
    }

  putreg32(regval, offset);
}

#ifdef CONFIG_NRF53_APPCORE

/****************************************************************************
 * Name: nrf53_gpio_mcusel
 *
 * Description:
 *   Set MCUSEL configuration for a pin
 *
 ****************************************************************************/

static inline void nrf53_gpio_mcusel(nrf53_pinset_t cfgset,
                                     unsigned int port, unsigned int pin)
{
  uint32_t mcusel;
  uint32_t regval;
  uint32_t offset;

  mcusel = cfgset & GPIO_MCUSEL_MASK;

  offset = nrf53_gpio_regget(port, NRF53_GPIO_PIN_CNF_OFFSET(pin));
  regval = getreg32(offset);

  regval &= ~GPIO_CNF_MCUSEL_MASK;

  switch (mcusel)
    {
      case GPIO_MCUSEL_APP:
        regval |= GPIO_CNF_MCUSEL_CPUAPP;
        break;
      case GPIO_MCUSEL_NET:
        regval |= GPIO_CNF_MCUSEL_CPUNET;
       break;
      case GPIO_MCUSEL_PERIP:
        regval |= GPIO_CNF_MCUSEL_PERIP;
        break;
      case GPIO_MCUSEL_TND:
        regval |= GPIO_CNF_MCUSEL_TND;
        break;
    }

  putreg32(regval, offset);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_gpio_config
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int nrf53_gpio_config(nrf53_pinset_t cfgset)
{
  unsigned int port = 0;
  unsigned int pin;
  irqstate_t flags;
  int ret = OK;

  /* Verify that this hardware supports the select GPIO port */

  port = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;

  if (port < NRF53_GPIO_NPORTS)
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

      nrf53_gpio_input(port, pin);

      /* Set the mode bits */

      nrf53_gpio_mode(cfgset, port, pin);

      /* Set the drive bits (needed also for input pins
       * for some peripherals).
       */

      nrf53_gpio_drive(cfgset, port, pin);

#ifdef CONFIG_NRF53_APPCORE
      /* Select which MCU/Subsystem controls this pin.
       * NOTE: accessible only from secure code and from the app core.
       */

      nrf53_gpio_mcusel(cfgset, port, pin);
#endif

      /* Handle according to pin function */

      switch (cfgset & GPIO_FUNC_MASK)
        {
        case GPIO_INPUT:   /* GPIO input pin */
          nrf53_gpio_sense(cfgset, port, pin);
          break;

        case GPIO_OUTPUT:  /* GPIO outpout pin */
          nrf53_gpio_output(cfgset, port, pin);
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
 * Name: nrf53_gpio_unconfig
 *
 * Description:
 *   Unconfigure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int nrf53_gpio_unconfig(nrf53_pinset_t cfgset)
{
  unsigned int pin;
  unsigned int port = 0;
  uint32_t offset;

  /* Get port and pin number */

  pin  = GPIO_PIN_DECODE(cfgset);
  port = GPIO_PORT_DECODE(cfgset);

  /* Get address offset */

  offset = nrf53_gpio_regget(port, NRF53_GPIO_PIN_CNF_OFFSET(pin));

  /* Configure as input and disconnect input buffer */

  putreg32(GPIO_CNF_INPUT, offset);

  return OK;
}

/****************************************************************************
 * Name: nrf53_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void nrf53_gpio_write(nrf53_pinset_t pinset, bool value)
{
  unsigned int pin;
  unsigned int port = 0;
  uint32_t offset;

  /* Get port and pin number */

  pin  = GPIO_PIN_DECODE(pinset);
  port = GPIO_PORT_DECODE(pinset);

  /* Get register address */

  if (value)
    {
      offset = nrf53_gpio_regget(port, NRF53_GPIO_OUTSET_OFFSET);
    }
  else
    {
      offset = nrf53_gpio_regget(port, NRF53_GPIO_OUTCLR_OFFSET);
    }

  /* Put register value */

  putreg32(1 << pin, offset);
}

/****************************************************************************
 * Name: nrf53_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool nrf53_gpio_read(nrf53_pinset_t pinset)
{
  unsigned int port;
  unsigned int pin;
  uint32_t regval;
  uint32_t offset;

  /* Get port and pin number */

  pin  = GPIO_PIN_DECODE(pinset);
  port = GPIO_PORT_DECODE(pinset);

  /* Get register address */

  offset = nrf53_gpio_regget(port, NRF53_GPIO_IN_OFFSET);

  /* Get register value */

  regval = getreg32(offset);

  return (regval >> pin) & 1UL;
}

/****************************************************************************
 * Name: nrf53_gpio_detectmode
 *
 * Description:
 *  Set DETECTMODE to either default or latched
 *
 ****************************************************************************/

void nrf53_gpio_detectmode(int port, enum nrf53_gpio_detectmode_e mode)
{
  uint32_t offset = nrf53_gpio_regget(port, NRF53_GPIO_DETECTMODE_OFFSET);

  putreg32(mode == NRF53_GPIO_DETECTMODE_DETECT ?
           GPIO_DETECTMODE_DEFAULT :
           GPIO_DETECTMODE_LDETECT, offset);
}
