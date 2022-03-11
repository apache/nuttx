/****************************************************************************
 * arch/arm/src/tiva/cc13xx/cc13xx_gpio.c
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
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/spinlock.h>

#include "arm_internal.h"
#include "tiva_gpio.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *   NOTE: Power and clocking provided in __start().
 *
 ****************************************************************************/

int tiva_configgpio(pinconfig_t pinconfig)
{
  irqstate_t flags;
  uintptr_t regaddr;
  uint32_t regval;
  unsigned int dio;
  unsigned int portid;

  /* The following requires exclusive access to the GPIO registers */

  flags = spin_lock_irqsave(NULL);

#ifdef CONFIG_TIVA_GPIO_IRQS
  /* Mask and clear any pending GPIO interrupt */

  tiva_gpioirqdisable(pinconfig);
  tiva_gpioirqclear(pinconfig);
#endif

  /* Configure the GPIO as an input.  We don't even know if the pin is a
   * a GPIO yet, but may prevent glitches when configure GPIO output pins.
   */

  dio = (pinconfig->gpio & GPIO_DIO_MASK) >> GPIO_DIO_SHIFT;

  regval  = getreg32(TIVA_GPIO_DOE);
  regval &= ~(1 << dio);
  putreg32(regval, TIVA_GPIO_DOE);

  /* Get the address of the IOC configuration register associated with
   * this DIO and write the user-privided IOC configuration image.
   *
   * NOTE that we make no checks so it is possible that a bad IOC value
   * could cause problems.
   */

  regaddr = TIVA_IOC_IOCFG(dio);
  putreg32(pinconfig->ioc, regaddr);

  /* If the pin was an output, then set the initial value of the output
   * and enable the output.
   */

  portid = (pinconfig->ioc & IOC_IOCFG_PORTID_MASK) >>
           IOC_IOCFG_PORTID_SHIFT;

  if (portid == IOC_IOCFG_PORTID_GPIO &&
      (pinconfig->gpio & GPIO_OUTPUT) != 0)
    {
      /* Set the initial output value */

      if ((pinconfig->gpio & GPIO_VALUE_MASK) == GPIO_VALUE_ZERO)
        {
          regaddr = TIVA_GPIO_DOUTCLR;
        }
      else /* if ((pinconfig->gpio & GPIO_VALUE_MASK) == GPIO_VALUE_ONE) */
        {
          regaddr = TIVA_GPIO_DOUTSET;
        }

      putreg32(1 << dio, regaddr);

      /* Configure the GPIO as an output */

      regval  = getreg32(TIVA_GPIO_DOE);
      regval |= (1 << dio);
      putreg32(regval, TIVA_GPIO_DOE);
    }

  spin_unlock_irqrestore(NULL, flags);
  return OK;
}

/****************************************************************************
 * Name: tiva_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void tiva_gpiowrite(pinconfig_t pinconfig, bool value)
{
  uintptr_t regaddr;
  unsigned int dio;

  /* Are we setting the output to one or zero? */

  if (value)
    {
      regaddr = TIVA_GPIO_DOUTSET;
    }
  else /* if ((pinconfig & GPIO_VALUE_MASK) == GPIO_VALUE_ONE) */
    {
      regaddr = TIVA_GPIO_DOUTCLR;
    }

  dio = (pinconfig->gpio & GPIO_DIO_MASK) >> GPIO_DIO_SHIFT;
  putreg32(1 << dio, regaddr);
}

/****************************************************************************
 * Name: tiva_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool tiva_gpioread(pinconfig_t pinconfig)
{
  unsigned int dio;

  /* Return the input value from the specified DIO */

  dio = (pinconfig->gpio & GPIO_DIO_MASK) >> GPIO_DIO_SHIFT;
  return (getreg32(TIVA_GPIO_DIN) & (1 << dio)) != 0;
}
