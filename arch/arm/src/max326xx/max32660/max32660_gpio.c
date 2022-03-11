/****************************************************************************
 * arch/arm/src/max326xx/max32660/max32660_gpio.c
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

/*   MODE                  AF1 AF0
 *   I/O                    0   1
 *   Alternate Function 1   0   0
 *   Alternate Function 2   1   0
 *   Alternate Function 3   1   1
 */

#define IO_MODE          1,0
#define AF_FUNC1         0,0
#define AF_FUNC2         0,1
#define AF_FUNC3         1,1

/* Pins 0-1, 4-7, and 10-13 support 4 levels of drive strength */

#define DS1_SET          0x00003cf3

/* Pins 0-1, 4-7, and 10-13 support pull-up resistors */

#define PULLUP_SET       0x00003cf3

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
static const char *g_afmode[4] =
{
  "Alt Function 1"
  "GPIO"
  "Alt Function 2"
  "Alt Function 3"
};

static const char *g_pullmode[3] =
{
  "None"
  "Pull-Down"
  "Pull-Up"
};

static const char *g_dsmode[4] =
{
  "Low"
  "Medium Low"
  "Medium High"
  "High"
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max326_altfunc
 *
 * Description:
 *   Set the alternate function pins:
 *
 *   MODE                  AF1 AF0
 *   I/O                    0   1
 *   Alternate Function 1   0   0
 *   Alternate Function 2   1   0
 *   Alternate Function 3   1   1
 *
 ****************************************************************************/

void max326_altfunc(uint32_t pinmask, bool af0, bool af1)
{
  uint32_t regval;

  regval  = getreg32(MAX326_GPIO0_AF0SEL);
  if (af0)
    {
      regval |= pinmask;
    }
  else
    {
      regval &= ~pinmask;
    }

  putreg32(regval, MAX326_GPIO0_AF0SEL);

  regval  = getreg32(MAX326_GPIO0_AF1SEL);
  if (af1)
    {
      regval |= pinmask;
    }
  else
    {
      regval &= ~pinmask;
    }

  putreg32(regval, MAX326_GPIO0_AF1SEL);
}

/****************************************************************************
 * Name: max326_default
 *
 * Description:
 *   Configure a pin as a default GPIO input as it was on reset (unless this
 *   is an SWD pin)
 *
 ****************************************************************************/

void max326_default(uint32_t pinmask)
{
  uint32_t regval;

  /* Disable interrupts and wake-up events */

  regval  = getreg32(MAX326_GPIO0_INTEN);  /* Pin interrupt disabled triggered */
  regval &= ~pinmask;
  putreg32(regval, MAX326_GPIO0_INTEN);

  regval  = getreg32(MAX326_GPIO0_INTMODE);  /* Level triggered */
  regval &= ~pinmask;
  putreg32(regval, MAX326_GPIO0_INTMODE);

  regval  = getreg32(MAX326_GPIO0_INTPOL);  /* Input low triggers */
  regval &= ~pinmask;
  putreg32(regval, MAX326_GPIO0_INTPOL);

  regval  = getreg32(MAX326_GPIO0_INTDUALEDGE);  /* Disable dual edge */
  regval &= ~pinmask;
  putreg32(regval, MAX326_GPIO0_INTDUALEDGE);

  regval  = getreg32(MAX326_GPIO0_WAKEEN);  /* Disable wakeup */
  regval &= ~pinmask;
  putreg32(regval, MAX326_GPIO0_WAKEEN);

  /* Make the pin an input */

  regval  = getreg32(MAX326_GPIO0_OUTEN);  /* Disable output drivers */
  regval &= ~pinmask;
  putreg32(regval, MAX326_GPIO0_OUTEN);

  regval  = getreg32(MAX326_GPIO0_OUT);    /* Set the output value to zero */
  regval &= ~pinmask;
  putreg32(regval, MAX326_GPIO0_OUT);

  /* Set alternate functions to I/O */

  max326_altfunc(pinmask, IO_MODE);

  /* Reset drive strength */

  regval  = getreg32(MAX326_GPIO0_DS0SEL);
  regval &= ~pinmask;
  putreg32(regval, MAX326_GPIO0_DS0SEL);

  regval  = getreg32(MAX326_GPIO0_DS1SEL);
  regval &= ~pinmask;
  putreg32(regval, MAX326_GPIO0_DS1SEL);

  /* Disable pull up and pull down */

  regval  = getreg32(MAX326_GPIO0_PULLEN);
  regval &= ~pinmask;
  putreg32(regval, MAX326_GPIO0_PULLEN);

  regval  = getreg32(MAX326_GPIO0_PULLSEL);
  regval &= ~pinmask;
  putreg32(regval, MAX326_GPIO0_PULLSEL);

  /* Disable slew and hysteresis */

  regval  = getreg32(MAX326_GPIO0_SRSEL);
  regval &= ~pinmask;
  putreg32(regval, MAX326_GPIO0_SRSEL);

  regval  = getreg32(MAX326_GPIO0_INHYSEN);
  regval &= ~pinmask;
  putreg32(regval, MAX326_GPIO0_INHYSEN);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max326_gpio_config
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int max326_gpio_config(max326_pinset_t pinset)
{
  irqstate_t flags;
  max326_pinset_t subset;
  unsigned int pin;
  uint32_t pinmask;
  uint32_t regval;

  pin     = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  DEBUGASSERT(pin <= GPIO_PINMAX);
  pinmask = 1 << pin;

  /* Modification of all registers must be atomic */

  flags = spin_lock_irqsave(NULL);

  /* First, force the pin configuration to the default generic input state.
   * So that we know we are starting from a known state.
   */

  max326_default(pinmask);

  /* Then perform the actual pin configuration.  We need only to set values
   * that are not in the default, reset state.
   */

  /* Enable slew and hysteresis */

  if ((pinset & GPIO_SLEW) != 0)
    {
      regval  = getreg32(MAX326_GPIO0_SRSEL);
      regval |= pinmask;
      putreg32(regval, MAX326_GPIO0_SRSEL);
    }

  if ((pinset & GPIO_HYSTERESIS) != 0)
    {
      regval  = getreg32(MAX326_GPIO0_INHYSEN);
      regval |= pinmask;
      putreg32(regval, MAX326_GPIO0_INHYSEN);
    }

  /* Enable pull up and pull down */

  subset = pinset & GPIO_MODE_MASK;
  if (subset != GPIO_FLOAT)
    {
      bool upcap;               /* True: GPIO_PULLUP supported */

      /* Enable pull-down resistor.  All pins support pull down resisters,
       * but it works differently:
       *
       * - For pins that support only pull-down, the PULLEN register
       *   selects the pull down resister.
       * - For pins that support both pull-up and pull down, the PULLEN
       *   enables the feature but the PULLSEL registers selects the pull
       *   up or down direction.
       */

      upcap = ((PULLUP_SET & pinmask) != 0);

      if (upcap || subset == GPIO_PULLDOWN)
        {
          regval  = getreg32(MAX326_GPIO0_PULLEN);
          regval |= pinmask;
          putreg32(regval, MAX326_GPIO0_PULLEN);
        }

      /* Enable pull-ups (only certain pins support pull up resistors) */

      if (upcap && subset == GPIO_PULLUP)
        {
          /* Enable pull-up resistor */

          regval  = getreg32(MAX326_GPIO0_PULLSEL);
          regval |= pinmask;
          putreg32(regval, MAX326_GPIO0_PULLSEL);
        }
    }

  /* Set drive strength */

  subset = pinset & GPIO_DRIVE_MASK;
  if ((DS1_SET & pinmask) == 0)
    {
      /* Only two levels of drive strength */

      if (subset == GPIO_DRIVE_MEDHI || subset == GPIO_DRIVE_HI)
        {
          regval  = getreg32(MAX326_GPIO0_DS0SEL);
          regval |= pinmask;
          putreg32(regval, MAX326_GPIO0_DS0SEL);
        }
    }
  else
    {
      /* Four levels of drive strength:  Order by drive strength:
       *
       * LO    DS0=0 DS1=0
       * MEDLO DS0=1 DS1=0
       * MEDHI DS0=0 DS1=1
       * HI    DS0=1 DS1=1
       */

      if (subset == GPIO_DRIVE_MEDLO || subset == GPIO_DRIVE_HI)
        {
          regval  = getreg32(MAX326_GPIO0_DS0SEL);
          regval |= pinmask;
          putreg32(regval, MAX326_GPIO0_DS0SEL);
        }

      if (subset == GPIO_DRIVE_MEDHI || subset == GPIO_DRIVE_HI)
        {
          regval  = getreg32(MAX326_GPIO0_DS1SEL);
          regval |= pinmask;
          putreg32(regval, MAX326_GPIO0_DS1SEL);
        }
    }

  /* Handle the pin function */

  switch (pinset & GPIO_FUNC_MASK)
    {
      case GPIO_OUTPUT:
        if ((pinset & GPIO_VALUE) == GPIO_VALUE_ONE)
          {
            regval  = getreg32(MAX326_GPIO0_OUT);  /* Set output high */
            regval |= pinmask;
            putreg32(regval, MAX326_GPIO0_OUT);
          }

        regval  = getreg32(MAX326_GPIO0_OUTEN);  /* Enable output drivers */
        regval |= pinmask;
        putreg32(regval, MAX326_GPIO0_OUTEN);
        break;

      case GPIO_ALT1:
        max326_altfunc(pinmask, AF_FUNC1);
        break;

      case GPIO_ALT2:
        max326_altfunc(pinmask, AF_FUNC2);
        break;

      case GPIO_ALT3:
        max326_altfunc(pinmask, AF_FUNC3);
        break;

      case GPIO_INPUT:   /* Already done */
      case GPIO_INTFE:   /* Treat interrupts as inputs for now */
      case GPIO_INTRE:
      case GPIO_INTBOTH:
      case GPIO_INTLOW:
      case GPIO_INTHIGH:
        break;
    }

#ifdef CONFIG_MAX326XX_GPIOIRQ
    /* Configure the interrupt */

  if (GPIO_IS_INTR(pinset))
    {
      max326_gpio_irqconfig(pinset);
    }
#endif

  /* Enable the wakeup event */

  if ((pinset & GPIO_WAKEUP) != 0)
    {
      regval  = getreg32(MAX326_GPIO0_WAKEEN);  /* Disable wakeup */
      regval |= pinmask;
      putreg32(regval, MAX326_GPIO0_WAKEEN);
    }

  spin_unlock_irqrestore(NULL, flags);
  return OK;
}

/****************************************************************************
 * Name: max326_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void max326_gpio_write(max326_pinset_t pinset, bool value)
{
  irqstate_t flags;
  unsigned int pin;
  uint32_t regval;

  pin     = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  DEBUGASSERT(pin <= GPIO_PINMAX);

  /* Modification of registers must be atomic */

  flags  = spin_lock_irqsave(NULL);
  regval = getreg32(MAX326_GPIO0_OUT);
  if (value)
    {
      regval |= (1 << pin);   /* Set output high */
    }
  else
    {
      regval &= ~(1 << pin);  /* Set output low */
    }

  putreg32(regval, MAX326_GPIO0_OUT);
  spin_unlock_irqrestore(NULL, flags);
}

/****************************************************************************
 * Name: max326_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool max326_gpio_read(max326_pinset_t pinset)
{
  unsigned int pin;
  uint32_t regval;

  pin    = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  DEBUGASSERT(pin <= GPIO_PINMAX);

  regval = getreg32(MAX326_GPIO0_IN);  /* Set output high */
  return (regval & (1 << pin)) != 0;
}

/****************************************************************************
 * Function:  max326_gpio_dump
 *
 * Description:
 *   Decode and dump all GPIO registers associated with the port and pin
 *   numbers in the provided pinset.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
int max326_gpio_dump(max326_pinset_t pinset, const char *msg)
{
  unsigned int pin;
  uint32_t pinmask;
  uint32_t regval;
  unsigned int afmode;
  unsigned int pullmode;
  unsigned int dsmode;
  bool edge;

  pin     = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  DEBUGASSERT(pin <= GPIO_PINMAX);
  pinmask = 1 << pin;

  gpioinfo("P0.%u:\n", pin);

/*   MODE                  AF1 AF0
 *   I/O                    0   1
 *   Alternate Function 1   0   0
 *   Alternate Function 2   1   0
 *   Alternate Function 3   1   1
 */

  regval  = getreg32(MAX326_GPIO0_AF0SEL);
  afmode  = (regval & pinmask) != 0 ? 1 : 0;
  regval  = getreg32(MAX326_GPIO0_AF1SEL);
  afmode |= (regval & pinmask) != 0 ? 2 : 0;
  gpioinfo("           Mode:  %d\n", g_afmode[afmode]);

  regval = getreg32(MAX326_GPIO0_OUTEN);
  gpioinfo("  Output Enable:  %s\n",
           (regval & pinmask) != 0 ? "Yes" : "No");

  regval = getreg32(MAX326_GPIO0_OUT);
  gpioinfo("   Output Value:  %s\n",
           (regval & pinmask) != 0 ? "High" : "Low");

  regval = getreg32(MAX326_GPIO0_IN);
  gpioinfo("    Input Value:  %s\n",
           (regval & pinmask) != 0 ? "High" : "Low");

  regval = getreg32(MAX326_GPIO0_INTMODE);
  edge   = (regval & pinmask) != 0;
  gpioinfo("      Intr Mode:  %s\n", edge ? "Yes" : "No");

  regval = getreg32(MAX326_GPIO0_INTPOL);
  if (edge)
    {
      uint32_t dualedge = getreg32(MAX326_GPIO0_INTDUALEDGE);
      if ((dualedge & pinmask) != 0)
        {
          gpioinfo("      Intr Edge:  Both edges\n");
        }
      else
        {
          gpioinfo("      Intr Edge:  %s\n",
                   (regval & pinmask) != 0 ? "Rising" : "Falling");
        }
    }
  else
    {
      gpioinfo("     Intr Level:  %s\n",
               (regval & pinmask) != 0 ? "High" : "Low");
    }

  regval = getreg32(MAX326_GPIO0_INTEN);
  gpioinfo("   Intr Enabled:  %s\n",
           (regval & pinmask) != 0 ? "Yes" : "No");

  regval = getreg32(MAX326_GPIO0_INTFL);
  gpioinfo("   Intr Pending:  %s\n",
           (regval & pinmask) != 0 ? "Yes" : "No");

  regval = getreg32(MAX326_GPIO0_WAKEEN);
  gpioinfo(" Wakeup Enabled:  %s\n",
           (regval & pinmask) != 0 ? "Yes" : "No");

  pullmode = 0;
  regval   = getreg32(MAX326_GPIO0_PULLEN);
  if ((regval & pinmask) != 0)
    {
      if ((PULLUP_SET & pinmask) == 0)
        {
          pullmode = 1;
        }
      else
        {
          regval = getreg32(MAX326_GPIO0_PULLSEL);
          pullmode = (regval & pinmask) != 0 ? 2 : 1;
        }
    }

  gpioinfo("  Resister Mode:  %s\n", g_pullmode[pullmode]);

  dsmode = (PULLUP_SET & pinmask) == 0 ? 1 : 0;
  regval = getreg32(MAX326_GPIO0_DS0SEL);
  if ((regval & pinmask) != 0)
    {
      if ((PULLUP_SET & pinmask) == 0)
        {
          dsmode = 3;
        }
      else
        {
          regval = getreg32(MAX326_GPIO0_DS1SEL);
          dsmode = (regval & pinmask) != 0 ? 3 : 1;
        }
    }
  else if ((PULLUP_SET & pinmask) != 0)
    {
      regval = getreg32(MAX326_GPIO0_DS1SEL);
      if ((regval & pinmask) != 0)
        {
          dsmode = 2;
        }
    }

  gpioinfo(" Drive Strength:  %s\n", g_dsmode[dsmode]);

  regval = getreg32(MAX326_GPIO0_INHYSEN);
  gpioinfo("     Hysteresis:  %s\n",
           (regval & pinmask) != 0 ? "Yes" : "No");

  regval = getreg32(MAX326_GPIO0_SRSEL);
  gpioinfo("   Slew Enabled:  %s\n",
           (regval & pinmask) != 0 ? "Yes" : "No");

  return 0;
}
#endif
