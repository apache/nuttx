/****************************************************************************
 * arch/arm/src/bcm2708/bcm_gpio.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <nuttx/arch.h>

#include "up_arch.h"
#include "bcm_config.h"
#include "chip/bcm2708_gpio.h"
#include "bcm_gpio.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Function:  bcm_gpio_fsel
 *
 * Description:
 *   Set the FSEL field for the given pin.
 *
 ************************************************************************************/

static void bcm_gpio_fsel(unsigned int pin, uint8_t mode)
{
  uintptr_t regaddr = BCM_GPIO_GPFSEL(pin);
  uint32_t  clrbits = BCM_GPIO_GPFSEL_FSEL_MASK(pin);
  uint32_t  setbits = BCM_GPIO_GPFSEL_FSEL(pin, mode);

  modifyreg32(regaddr, clrbits, setbits);
}

/************************************************************************************
 * Function:  bcm_gpio_pudclk
 *
 * Description:
 *   Clocks the value written into the pud to the GPIO pin.
 *
 ************************************************************************************/

static void bcm_gpio_pudclk(unsigned int pin, bool enable)
{
  uintptr_t regaddr = BCM_GPIO_GPPUDCLK(pin);
  uint32_t mask     = BCM_GPIO_GPPUD_PUDCLK_MASK(pin);

  if (enable)
    {
      modifyreg32(regaddr, 0, mask);
    }
  else
    {
      modifyreg32(regaddr, mask, 0);
    }
}

/************************************************************************************
 * Function:  bcm_gpio_configpud
 *
 * Description:
 *   "The GPIO Pull-up/down Clock Registers control the actuation of internal
 *    pull-downs on the respective GPIO pins. These registers must be used in
 *    conjunction with the GPPUD register to effect GPIO Pull-up/down changes. The
 *    following sequence of events is required:
 *
 *   "1. Write to GPPUD to set the required control signal (i.e. Pull-up or Pull-
 *       Down or neither to remove the current Pull-up/down)
 *   "2. Wait 150 cycles – this provides the required set-up time for the control
 *       signal
 *   "3. Write to GPPUDCLK0/1 to clock the control signal into the GPIO pads you
 *       wish to modify – NOTE only the pads which receive a clock will be modified,
 *       all others will retain their previous state.
 *   "4. Wait 150 cycles – this provides the required hold time for the control
 *       signal
 *   "5. Write to GPPUD to remove the control signal
 *   "6. Write to GPPUDCLK0/1 to remove the clock"
 *
 ************************************************************************************/

static void bcm_gpio_configpud(unsigned int pin, gpio_pinset_t pinset)
{
  uint32_t pud = (pinset & GPIO_PUD_MASK) >> GPIO_PUD_SHIFT;

  /* "1. Write to GPPUD to set the required control signal (i.e. Pull-up or Pull-
   *     Down or neither to remove the current Pull-up/down)
   */

  putreg32(pud, BCM_GPIO_GPPUD);

  /* "2. Wait 150 cycles – this provides the required set-up time for the control
   *     signal
   */

  up_udelay(10);

  /* "3. Write to GPPUDCLK0/1 to clock the control signal into the GPIO pads you
   *     wish to modify – NOTE only the pads which receive a clock will be modified,
   *     all others will retain their previous state.
   */

  bcm_gpio_pudclk(pin, true);

  /* "4. Wait 150 cycles – this provides the required hold time for the control
   *     signal
   */

  up_udelay(10);

  /* "5. Write to GPPUD to remove the control signal */

  putreg32(BCM_GPIO_GPPUD_PUD_OFF, BCM_GPIO_GPPUD);

  /* "6. Write to GPPUDCLK0/1 to remove the clock" */

  bcm_gpio_pudclk(pin, false);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Function:  bcm_gpio_initialize
 *
 * Description:
 *   Based on configuration within the .config file, it does:
 *    - Remaps positions of alternative functions for GPIO.
 *
 *   Typically called from bcm_start().
 *
 ************************************************************************************/

void bcm_gpio_initialize(void)
{
  /* Nothing to be done */
}

/************************************************************************************
 * Name: bcm_gpio_config
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ************************************************************************************/

int bcm_gpio_config(gpio_pinset_t pinset)
{
  unsigned int pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  uint8_t mode;

  /* First, force the pin into an innocuous input state */

  bcm_gpio_fsel(pin, BCM_GPIO_GPFSEL_FSEL_INPUT);

#ifdef CONFIG_BCM2708_GPIO_IRQ
  /* Make sure that interrupts are disabled */

  bcm_gpio_irqconfig(pinset & ~GPIO_INT_MASK);
#endif

  /* Configure the pin pull-ups or pull-downs */

  bcm_gpio_configpud(pin, pinset);

  /* If the the pin is an output, set the correct output state */

  mode = (pinset & GPIO_MODE_MASK) >> GPIO_MODE_SHIFT;
  if (mode == BCM_GPIO_GPFSEL_FSEL_OUTPUT)
    {
      bool value = (pinset & GPIO_OUTPUT_MASK) != GPIO_OUTPUT_CLEAR;
      bcm_gpio_write(pinset, value);
    }

  /* Finally configure the correct mode */

  bcm_gpio_fsel(pin, mode);
  return OK;
}

/************************************************************************************
 * Name: bcm_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ************************************************************************************/

void bcm_gpio_write(gpio_pinset_t pinset, bool value)
{
  unsigned int pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  uintptr_t regaddr;

  if (value)
    {
      regaddr = BCM_GPIO_GPSET(pin);
    }
  else
    {
      regaddr = BCM_GPIO_GPCLR(pin);
    }

  putreg32(BCM_GPIO_GPSET_SET(pin), regaddr);
}

/************************************************************************************
 * Name: bcm_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ************************************************************************************/

bool bcm_gpio_read(gpio_pinset_t pinset)
{
  unsigned int pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  uintptr_t regaddr = BCM_GPIO_GPLEV(pin);
  uint32_t mask     = BCM_GPIO_GPLEV_LEV(pin);

  return (getreg32(regaddr) & mask) != 0;
}
