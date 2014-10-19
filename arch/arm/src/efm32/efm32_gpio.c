/************************************************************************************
 * arch/arm/src/efm32/efm32_gpio.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include "efm32_gpio.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define __GPIO_INPUT  (1 << 0) /* Bit 0: GPIO is an input */
#define __GPIO_OUTPUT (1 << 1) /* Bit 1: GPIO is an output */
#define __GPIO_DOUT   (1 << 2) /* Bit 2: An input modified with DOUT setting */
#define __GPIO_DRIVE  (1 << 3) /* Bit 3: An output with drive selection */

 /************************************************************************************
 * Private Data
 ************************************************************************************/

static const uint8_t g_gpiomode[16] =
{
  (__GPIO_DOUT),                  /* 0 DISABLED Input disabled. Pullup if DOUT is
                                   *    set. */
  (__GPIO_INPUT | __GPIO_DOUT),   /*  1 INPUT Input enabled. Filter if DOUT is set. */
  (__GPIO_INPUT | __GPIO_DOUT),   /*  2 INPUTPULL Input enabled. DOUT determines pull
                                   *    direction */
  (__GPIO_INPUT | __GPIO_DOUT),   /*  3 INPUTPULLFILTER Input enabled with filter.
                                   *    COUT determines pull direction */
  (__GPIO_OUTPUT),                /*  4 PUSHPULL Push-pull output */
  (__GPIO_OUTPUT | __GPIO_DRIVE), /*  5 PUSHPULLDRIVE Push-pull output with drive-
                                   *    strength set by DRIVEMODE */
  (__GPIO_OUTPUT),                /*  6 WIREDOR Wired-or output */
  (__GPIO_OUTPUT),                /*  7 WIREDORPULLDOWN Wired-or output with pull-
                                   *    down */
  (__GPIO_OUTPUT),                /*  8 WIREDAND Open-drain output */
  (__GPIO_OUTPUT),                /*  9 WIREDANDFILTER Open-drain output with filter */
  (__GPIO_OUTPUT),                /* 10 WIREDANDPULLUP Open-drain output with pullup */
  (__GPIO_OUTPUT),                /* 11 WIREDANDPULLUPFILTER Open-drain output with
                                   *    filter and pullup */
  (__GPIO_OUTPUT | __GPIO_DRIVE), /* 12 WIREDANDDRIVE Open-drain output with
                                   *    drive-strength set by DRIVEMODE */
  (__GPIO_OUTPUT | __GPIO_DRIVE), /* 13 WIREDANDDRIVEFILTER Open-drain output with
                                   *    filter and drive-strength set by DRIVEMODE */
  (__GPIO_OUTPUT | __GPIO_DRIVE), /* 14 WIREDANDDRIVEPULLUP Open-drain output with
                                   *    pullup and drive-strength set by DRIVEMODE */
  (__GPIO_OUTPUT | __GPIO_DRIVE)  /* 15 WIREDANDDRIVEPULLUPFILTER Open-drain output
                                   *    with filter, pullup and drive-strength set
                                   *    by DRIVEMODE */
};

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: efm32_port
 *
 * Description:
 *   Extract the encoded port number
 *
 ************************************************************************************/

static inline uint8_t efm32_port(gpio_pinset_t cfgset)
{
  return (uint8_t)((cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT);
}

/************************************************************************************
 * Name: efm32_pin
 *
 * Description:
 *   Extract the encoded pin number
 *
 ************************************************************************************/

static inline uint8_t efm32_pin(gpio_pinset_t cfgset)
{
  return (uint8_t)((cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT);
}

/************************************************************************************
 * Name: efm32_mode
 *
 * Description:
 *   Extract the encoded pin mode
 *
 ************************************************************************************/

static inline uint8_t efm32_pin(gpio_pinset_t cfgset)
{
  return (uint8_t)((cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT);
}

/************************************************************************************
 * Name: efm32_drivestrength
 *
 * Description:
 *   Extract the output drive strength
 *
 ************************************************************************************/

static uint8_t efm32_drivestrength(uint8_t mode, gpio_pinset_t cfgset)
{
  if ((g_gpiomode[mode] & __GPIO_DRIVE) != 0)
    {
      return (uint8_t)((cfgset & GPIO_DRIVE_MASK) >> GPIO_DRIVE_SHIFT);
    }
  else
    {
      return _GPIO_DRIVE_STANDARD;
    }
}

/************************************************************************************
 * Name: efm32_dout
 *
 * Description:
 *   Extract the encoded port number
 *
 ************************************************************************************/

static uint8_t efm32_dout(uint8_t mode, gpio_pinset_t cfgset)
{
  if ((g_gpiomode[mode] & __GPIO_DOUT) != 0)
    {
      return (uint8_t)((cfgset >> GPIO_MODE_DOUT_SHIFT) & 1);
    }
  else if ((g_gpiomode[mode] & __GPIO_OUTPUT) != 0)
    {
      return (uint8_t)((cfgset >> GPIO_OUTPUT_SHIFT) & 1);
    }
  else
    {
      return 0;
    }
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: efm32_configgpio
 *
 * Description:
 *   Configure a PIO pin based on bit-encoded description of the pin.
 *
 ************************************************************************************/

int efm32_configgpio(gpio_pinset_t cfgset)
{
  uint8_t port  = efm32_pin(cfgset);
  uint8_t pin   = efm32_port(cfgset);
  uint8_t mode  = efm32_mode(cfgset);
  uint8_t drive = efm32_drivestrength(mode, cfgset);
  uint8_t dout  = efm32_dout(mode, cfgset);

#warning Missing logic
  return -ENOSYS;
}

/************************************************************************************
 * Name: efm32_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected PIO pin
 *
 ************************************************************************************/

void efm32_gpiowrite(gpio_pinset_t pinset, bool value)
{
#warning Missing logic
}

/************************************************************************************
 * Name: efm32_gpioread
 *
 * Description:
 *   Read one or zero from the selected PIO pin
 *
 ************************************************************************************/

bool efm32_gpioread(gpio_pinset_t pinset)
{
#warning Missing logic
  return false;
}

/************************************************************************************
 * Function:  efm32_dumpgpio
 *
 * Description:
 *   Dump all PIO registers associated with the base address of the provided pinset.
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_GPIO
int efm32_dumpgpio(uint32_t pinset, const char *msg)
{
#warning Missing logic
  return -ENOSYS;
}
#endif
