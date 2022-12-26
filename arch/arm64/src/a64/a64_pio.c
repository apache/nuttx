/****************************************************************************
 * arch/arm64/src/a64/a64_pio.c
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

#include <stdint.h>
#include <time.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "arm64_internal.h"
#include "chip.h"
#include "a64_pio.h"
#include "hardware/a64_pio.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: a64_pio_pin
 *
 * Description:
 *   Return the Port Number for the bit-encoded description of the pin.
 *
 * Input Parameters:
 *   cfgset - Bit-encoded description of a pin
 *
 * Returned Value:
 *   Port Number (PIO_REG_PORTB to PIO_REG_PORTL)
 *
 ****************************************************************************/

static inline int a64_pio_port(pio_pinset_t cfgset)
{
  int port = (cfgset & PIO_PORT_MASK) >> PIO_PORT_SHIFT;

  DEBUGASSERT(port >= PIO_REG_PORTB && port <= PIO_REG_PORTL);
  return port;
}

/****************************************************************************
 * Name: a64_pio_pin
 *
 * Description:
 *   Return the Pin Number for the bit-encoded description of the pin.
 *
 * Input Parameters:
 *   cfgset - Bit-encoded description of a pin
 *
 * Returned Value:
 *   Pin Number (0 to 24)
 *
 ****************************************************************************/

static inline int a64_pio_pin(pio_pinset_t cfgset)
{
  int pin = (cfgset & PIO_PIN_MASK) >> PIO_PIN_SHIFT;

  DEBUGASSERT(pin >= 0 && pin <= 24);
  return pin;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: a64_pio_config
 *
 * Description:
 *   Configure a PIO pin based on bit-encoded description of the pin.
 *
 * Input Parameters:
 *   cfgset - Bit-encoded description of a pin
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

int a64_pio_config(pio_pinset_t cfgset)
{
  unsigned int port = a64_pio_port(cfgset);
  unsigned int pin  = a64_pio_pin(cfgset);
  unsigned int shift;
  unsigned int value;
  uintptr_t cfgaddr;
  uintptr_t puaddr;
  uintptr_t drvaddr;
  uintptr_t intaddr;
  uintptr_t dataddr;
  uint32_t regval;
  irqstate_t flags;

  /* Disable interrupts to prohibit re-entrance. */

  flags = enter_critical_section();

  /* Set the peripheral ID (0=input, 1=output) and interrupt mode */

  switch (pin >> 3)
    {
      case 0: /* PIO 0-7 */
        cfgaddr = (port == PIO_REG_PORTL) ?
                  A64_RPIO_CFG0 :
                  A64_PIO_CFG0(port);
        intaddr = (port == PIO_REG_PORTL) ?
                  A64_RPIO_INT_CFG0 :
                  A64_PIO_INT_CFG0;
        break;

      case 1: /* PIO 8-15 */
        cfgaddr = (port == PIO_REG_PORTL) ?
                  A64_RPIO_CFG1 :
                  A64_PIO_CFG1(port);
        intaddr = (port == PIO_REG_PORTL) ?
                  A64_RPIO_INT_CFG1 :
                  A64_PIO_INT_CFG1;
        break;

      case 2: /* PIO 16-23 */
        cfgaddr = (port == PIO_REG_PORTL) ?
                  A64_RPIO_CFG2 :
                  A64_PIO_CFG2(port);
        intaddr = (port == PIO_REG_PORTL) ?
                  A64_RPIO_INT_CFG2 :
                  A64_PIO_INT_CFG2;
        break;

      case 3: /* PIO 24-31 */
        cfgaddr = (port == PIO_REG_PORTL) ?
                  A64_RPIO_CFG3 :
                  A64_PIO_CFG3(port);
        intaddr = (port == PIO_REG_PORTL) ?
                  A64_RPIO_INT_CFG3 :
                  A64_PIO_INT_CFG3;
        break;

      default:
        leave_critical_section(flags);
        return -EINVAL;
    }

  value = (cfgset & PIO_MODE_MASK) >> PIO_MODE_SHIFT;
  shift = (pin & 7) << 2;

  regval = getreg32(cfgaddr);
  regval &= ~(7 << shift);
  regval |= (value << shift);
  putreg32(regval, cfgaddr);

  /* Do not modify the INT MASK unless this pin is configured
   * as an external PIO interrupt.
   */

  if ((cfgset & PIO_EINT_MASK) == PIO_EINT)
    {
      value = (cfgset & PIO_INT_MASK) >> PIO_INT_SHIFT;

      regval = getreg32(intaddr);
      regval &= ~(7 << shift);
      regval |= (value << shift);
      putreg32(regval, intaddr);
    }

  /* Set the pull-up/down and drive strength */

  switch (pin >> 4)
    {
      case 0: /* PIO 0-15 */
        puaddr  = (port == PIO_REG_PORTL) ?
                  A64_RPIO_PUL0 :
                  A64_PIO_PUL0(port);
        drvaddr = (port == PIO_REG_PORTL) ?
                  A64_RPIO_DRV0 :
                  A64_PIO_DRV0(port);
        break;

      case 1: /* PIO 16-31 */
        puaddr  = (port == PIO_REG_PORTL) ?
                  A64_RPIO_PUL1 :
                  A64_PIO_PUL1(port);
        drvaddr = (port == PIO_REG_PORTL) ?
                  A64_RPIO_DRV1 :
                  A64_PIO_DRV1(port);
        break;

      default:
        leave_critical_section(flags);
        return -EINVAL;
    }

  value = (cfgset & PIO_PULL_MASK) >> PIO_PULL_SHIFT;
  shift = (pin & 15) << 1;

  regval = getreg32(puaddr);
  regval &= ~(3 << shift);
  regval |= (value << shift);
  putreg32(regval, puaddr);

  value = (cfgset & PIO_DRIVE_MASK) >> PIO_DRIVE_SHIFT;

  regval = getreg32(drvaddr);
  regval &= ~(3 << shift);
  regval |= (value << shift);
  putreg32(regval, drvaddr);

  /* Set the output value (will have no effect on inputs) */

  dataddr = (port == PIO_REG_PORTL) ?
            A64_RPIO_DAT :
            A64_PIO_DAT(port);
  regval = getreg32(dataddr);

  if ((cfgset & PIO_OUTPUT_SET) != 0)
    {
      regval |= PIO_DAT(pin);
    }
  else
    {
      regval &= ~PIO_DAT(pin);
    }

  putreg32(regval, dataddr);

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: a64_piowrite
 *
 * Description:
 *   Write one or zero to the selected PIO pin.
 *
 * Input Parameters:
 *   pinset - PIO pin
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void a64_pio_write(pio_pinset_t pinset, bool value)
{
  unsigned int port = a64_pio_port(pinset);
  unsigned int pin  = a64_pio_pin(pinset);
  irqstate_t flags;
  uintptr_t regaddr;
  uint32_t regval;

  /* Disable interrupts to prohibit re-entrance. */

  flags = enter_critical_section();

  /* Set the output value (will have no effect on inputs */

  regaddr = (port == PIO_REG_PORTL) ?
            A64_RPIO_DAT :
            A64_PIO_DAT(port);
  regval  = getreg32(regaddr);

  if (value)
    {
      regval |= PIO_DAT(pin);
    }
  else
    {
      regval &= ~PIO_DAT(pin);
    }

  putreg32(regval, regaddr);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: a64_pio_read
 *
 * Description:
 *   Read one or zero from the selected PIO pin
 *
 * Input Parameters:
 *   pinset - PIO pin
 *
 * Returned Value:
 *   Input value of PIO pin
 *
 ****************************************************************************/

bool a64_pio_read(pio_pinset_t pinset)
{
  unsigned int port = a64_pio_port(pinset);
  unsigned int pin  = a64_pio_pin(pinset);
  uintptr_t regaddr;
  uint32_t regval;

  /* Get the input value */

  regaddr = (port == PIO_REG_PORTL) ?
            A64_RPIO_DAT :
            A64_PIO_DAT(port);
  regval  = getreg32(regaddr);
  return ((regval & PIO_DAT(pin)) != 0);
}
