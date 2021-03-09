/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_pingpio.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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

#include <assert.h>
#include <errno.h>

#include "arm_arch.h"
#include "hardware/s32k1xx_gpio.h"
#include "s32k1xx_pin.h"

#include <arch/board/board.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void s32k1xx_gpiowrite(uint32_t pinset, bool value)
{
  uintptr_t    base;
  unsigned int port;
  unsigned int pin;

  DEBUGASSERT((pinset & _PIN_MODE_MASK) == _PIN_MODE_GPIO);
  DEBUGASSERT((pinset & _PIN_IO_MASK) == _PIN_OUTPUT);

  /* Get the port number and pin number */

  port = (pinset & _PIN_PORT_MASK) >> _PIN_PORT_SHIFT;
  pin  = (pinset & _PIN_MASK)      >> _PIN_SHIFT;

  DEBUGASSERT(port < S32K1XX_NPORTS);
  if (port < S32K1XX_NPORTS)
    {
      /* Get the base address of GPIO block for this port */

      base = S32K1XX_GPIO_BASE(port);

      /* Set or clear the output */

      if (value)
        {
          putreg32((1 << pin), base + S32K1XX_GPIO_PSOR_OFFSET);
        }
      else
        {
          putreg32((1 << pin), base + S32K1XX_GPIO_PCOR_OFFSET);
        }
    }
}

/****************************************************************************
 * Name: s32k1xx_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool s32k1xx_gpioread(uint32_t pinset)
{
  uintptr_t    base;
  uint32_t     regval;
  unsigned int port;
  unsigned int pin;
  bool         ret = false;

  DEBUGASSERT((pinset & _PIN_MODE_MASK) == _PIN_MODE_GPIO);
  DEBUGASSERT((pinset & _PIN_IO_MASK) == _PIN_INPUT);

  /* Get the port number and pin number */

  port = (pinset & _PIN_PORT_MASK) >> _PIN_PORT_SHIFT;
  pin  = (pinset & _PIN_MASK)      >> _PIN_SHIFT;

  DEBUGASSERT(port < S32K1XX_NPORTS);
  if (port < S32K1XX_NPORTS)
    {
      /* Get the base address of GPIO block for this port */

      base = S32K1XX_GPIO_BASE(port);

      /* return the state of the pin */

      regval = getreg32(base + S32K1XX_GPIO_PDIR_OFFSET);
      ret    = ((regval & (1 << pin)) != 0);
    }
  return ret;
}
