/****************************************************************************
 * arch/arm/src/max326xx/max32690/max32690_gpio.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <nuttx/spinlock.h>

#include "arm_internal.h"
#include "max326_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

spinlock_t g_max32690_gpio_lock = SP_UNLOCKED;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max32690_default
 *
 * Description:
 *   Configure a pin as a default GPIO input as it was on reset
 *
 ****************************************************************************/

static void max32690_default(max32690_gpio_regs_t *gpio,
                             max32690_pinconfig_t *conf)
{
    uint32_t pinmask = 1 << conf->pin;

      /* Disable interrupts and wake-up events */

    gpio->inten_clr =  pinmask;  /* Pin interrupt disabled triggered */
    gpio->intmode  &= ~pinmask;  /* Level triggered */
    gpio->intpol   &= ~pinmask;  /* Input low triggers */
    gpio->dualedge &= ~pinmask;  /* Disable dual edge */
    gpio->wken_clr  =  pinmask;  /* Disable wakeup */

      /* Make the pins an input an clear output value */

    gpio->outen_clr =  pinmask;  /* Disable output drivers */
    gpio->out_clr   =  pinmask;  /* Set the output value to zero */

      /* Set alternate functions to I/O */

    gpio->en0_set   =  pinmask;
    gpio->en1_clr   =  pinmask;
    gpio->en2_clr   =  pinmask;

      /* enable input connection to pin */

    gpio->inen     |= pinmask;

      /* Reset drive strength */

    gpio->ds0      &= ~pinmask;
    gpio->ds1      &= ~pinmask;

      /* Disable pull up and pull down */

    gpio->padctrl0 &= ~pinmask;
    gpio->ps       &= ~pinmask;

      /* Set VVoltage to V_DDIOH */

    gpio->vssel |= pinmask;

      /* Disable slew and hysteresis */

    gpio->hysen    &= ~pinmask;
    gpio->srsel    &= ~pinmask;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max32690_get_gpio_bank_regptr
 *
 * Description:
 *   Get the GPIO access struct address based in the GPIO Bank
 *
 ****************************************************************************/

max32690_gpio_regs_t *max32690_get_gpio_bank_regptr(int bank)
{
    max32690_gpio_regs_t *gpio_regs = 0;

    switch (bank)
    {
        case 0:
            gpio_regs = (max32690_gpio_regs_t *)MAX326_GPIO0_BASE;
            break;
        case 1:
            gpio_regs = (max32690_gpio_regs_t *)MAX326_GPIO1_BASE;
            break;
        case 2:
            gpio_regs = (max32690_gpio_regs_t *)MAX326_GPIO2_BASE;
            break;
        case 3:
            gpio_regs = (max32690_gpio_regs_t *)MAX326_GPIO3_BASE;
            break;
    }

    DEBUGASSERT(gpio_regs != 0);

    return gpio_regs;
}

static
void set_pin_electrical_characteristics(max32690_gpio_regs_t *gpio,
                                        uint16_t conf,
                                        uint32_t pinmask)
{
      /* Enable slew and hysteresis */

      if ((conf & GPIO_SLEW) != 0)
    {
        gpio->srsel |= pinmask;
    }

      if ((conf & GPIO_HYSTERESIS) != 0)
    {
        gpio->hysen |= pinmask;
    }

    switch (conf & GPIO_DRIVE_MASK)
        {
            case GPIO_DRIVE_0_V_DIO:
                gpio->ds1   &= ~pinmask;
                gpio->ds0   &= ~pinmask;
                gpio->vssel &= ~pinmask;
                break;

            case GPIO_DRIVE_1_V_DIO:
                gpio->ds1   &= ~pinmask;
                gpio->ds0   |=  pinmask;
                gpio->vssel &= ~pinmask;
                break;

            case GPIO_DRIVE_2_V_DIO:
                gpio->ds1   |=  pinmask;
                gpio->ds0   &= ~pinmask;
                gpio->vssel &= ~pinmask;
                break;

            case GPIO_DRIVE_3_V_DIO:
                gpio->ds1   |= pinmask;
                gpio->ds0   |= pinmask;
                gpio->vssel &= ~pinmask;
                break;

            case GPIO_DRIVE_0_V_DIO_H:
                gpio->ds1   &= ~pinmask;
                gpio->ds0   &= ~pinmask;
                gpio->vssel |=  pinmask;
                break;

            case GPIO_DRIVE_1_V_DIO_H:
                gpio->ds1   &= ~pinmask;
                gpio->ds0   |=  pinmask;
                gpio->vssel |=  pinmask;
                break;

            case GPIO_DRIVE_2_V_DIO_H:
                gpio->ds1   |=  pinmask;
                gpio->ds0   &= ~pinmask;
                gpio->vssel |=  pinmask;
                break;

            case GPIO_DRIVE_3_V_DIO_H:
                gpio->ds1   |= pinmask;
                gpio->ds0   |= pinmask;
                gpio->vssel |=  pinmask;
                break;

            default:
                DEBUGPANIC();
                return;
        }

    switch (conf & GPIO_IN_MODE_MASK)
        {
            case GPIO_FLOAT:
                gpio->padctrl1 &= ~pinmask;
                gpio->padctrl0 &= ~pinmask;
                break;

            case GPIO_PULLUP_WEAK:
                gpio->padctrl1 &= ~pinmask;
                gpio->padctrl0 |=  pinmask;
                gpio->ps       &= ~pinmask;
                gpio->vssel    &= ~pinmask;
                break;

            case GPIO_PULLUP_STRONG:
                gpio->padctrl1 &= ~pinmask;
                gpio->padctrl0 |=  pinmask;
                gpio->ps       |=  pinmask;
                gpio->vssel    &= ~pinmask;
                break;

            case GPIO_PULL_DOWN_WEAK:
                gpio->padctrl1 |=  pinmask;
                gpio->padctrl0 &= ~pinmask;
                gpio->ps       &= ~pinmask;
                gpio->vssel    |=  pinmask;
                break;

            case GPIO_PULL_DOWN_STRONG:
                gpio->padctrl1 |=  pinmask;
                gpio->padctrl0 &= ~pinmask;
                gpio->ps       |=  pinmask;
                gpio->vssel    |= ~pinmask;
                break;

            default:
                DEBUGPANIC();
                return;
        }
}

/****************************************************************************
 * Name: max32690_gpio_config
 *
 * Description:
 *   Configure a GPIO pin. Config based on bit-encoded descriptions.
 *
 ****************************************************************************/

int  max32690_gpio_config(max32690_pinconfig_t pinconf)
{
    irqstate_t flags;
    uint32_t pinmask;

    DEBUGASSERT(pinconf.pin <= GPIO_PINMAX);
    pinmask = 1 << pinconf.pin;

      /* Modification of all registers must be atomic */

    flags = spin_lock_irqsave(&g_max32690_gpio_lock);

    max32690_gpio_regs_t *gpio;
    gpio = max32690_get_gpio_bank_regptr(pinconf.gpio_bank);

      /* First, force the pin configuration to the default generic
       * input state.
       * So that we know we are starting from a known state.
       */

    max32690_default(gpio, &pinconf);

      /* Then perform the actual pin configuration.  We need only
       * to set values that are not in the default, reset state.
       */

      /* Handle the pin function */

    switch (pinconf.config & GPIO_FUNC_MASK)
    {
        case GPIO_IO:

            /* first change to I/O */

            gpio->en0_set = pinmask;
            gpio->en1_clr = pinmask;
            gpio->en2_clr = pinmask;

            set_pin_electrical_characteristics(gpio, pinconf.config,
                                               pinmask);

            if ((pinconf.config & GPIO_OUTPUT_MODE) == GPIO_OUTPUT_MODE)
            {
                gpio->outen_set  = pinmask;   /* enable pin output mode */
            }

            if ((pinconf.config & GPIO_VALUE_ONE) == GPIO_VALUE_ONE)
            {
                gpio->out_set = pinmask;      /* Set output high */
            }

            break;

      case GPIO_ALT1:

            /* first change to I/O (transition to AF1) */

            gpio->en0_set = pinmask;

            gpio->en1_clr = pinmask;
            gpio->en2_clr = pinmask;

            set_pin_electrical_characteristics(gpio, pinconf.config,
                                               pinmask);

            /* now disable en0 to switch to alternate mode */

            gpio->en0_clr = pinmask;

            if ((pinconf.config & GPIO_OUTPUT_MODE) == GPIO_OUTPUT_MODE)
            {
                gpio->outen_set  = pinmask;   /* enable pin output mode */
            }

            break;

      case GPIO_ALT2:

            /* first change to I/O (transition to AF2) */

            gpio->en0_set = pinmask;

            gpio->en1_set = pinmask;
            gpio->en2_clr = pinmask;

            set_pin_electrical_characteristics(gpio, pinconf.config,
                                               pinmask);

            /* now disable en0 to switch to alternate mode */

            gpio->en0_clr = pinmask;

            if ((pinconf.config & GPIO_OUTPUT_MODE) == GPIO_OUTPUT_MODE)
            {
                gpio->outen_set  = pinmask;   /* enable pin output mode */
            }

            break;

      case GPIO_ALT3:

            /* first change to I/O (transition to AF3) */

            gpio->en0_set = pinmask;

            gpio->en1_clr = pinmask;
            gpio->en2_set = pinmask;

            set_pin_electrical_characteristics(gpio, pinconf.config,
                                               pinmask);

            /* now disable en0 to switch to alternate mode */

            gpio->en0_clr = pinmask;

            if ((pinconf.config & GPIO_OUTPUT_MODE) == GPIO_OUTPUT_MODE)
            {
                gpio->outen_set  = pinmask;   /* enable pin output mode */
            }

            break;

        default:
            DEBUGPANIC();
            return ERROR;
    }

    spin_unlock_irqrestore(&g_max32690_gpio_lock, flags);
    return OK;
}

/****************************************************************************
 * Name: max32690_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void max32690_gpio_write(max32690_pinconfig_t pinconfig, bool value)
{
    DEBUGASSERT(pinconfig.gpio_bank <= 3);
    DEBUGASSERT(pinconfig.pin <= GPIO_PINMAX);

      /* Modification of registers areatomic */

    max32690_gpio_regs_t *gpio;
    gpio = max32690_get_gpio_bank_regptr(pinconfig.gpio_bank);

      if (value)
    {
        gpio->out_set = 1 << pinconfig.pin;
    }
    else
    {
        gpio->out_clr = 1 << pinconfig.pin;
    }
}

/****************************************************************************
 * Name: max32690_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool max32690_gpio_read(max32690_pinconfig_t pinconfig)
{
    DEBUGASSERT(pinconfig.gpio_bank <= 3);
    DEBUGASSERT(pinconfig.pin <= GPIO_PINMAX);

    max32690_gpio_regs_t *gpio;
    gpio = max32690_get_gpio_bank_regptr(pinconfig.gpio_bank);

      return (gpio->in & (1 << pinconfig.pin)) != 0;
}

