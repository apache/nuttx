/****************************************************************************
 * arch/arm/src/imx/imx_gpio.c
 * arch/arm/src/chip/imx_gpio.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include "up_arch.h"
#include "imx_gpio.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Funtions
 ****************************************************************************/

/****************************************************************************
 * Name: imxgpio_configoutput
 ****************************************************************************/

void imxgpio_configoutput(int port, int bit, int value)
{
  imxgpio_configinput(port, bit);         /* Same as input except: */
  imxgpio_dirout(GPIOA, 2);                 /* Output */

  if (value)
    {
      imxgpio_setoutput(GPIOA, 2);          /* Set output = 1 */
    }
  else
    {
      imxgpio_clroutput(GPIOA, 2);          /* Set output = 0 */
    }
}

/****************************************************************************
 * Name: imxgpio_configinput
 ****************************************************************************/

void imxgpio_configinput(int port, int bit)
{
  imxgpio_pullupdisable(GPIOA, 2);          /* No pullup */
  imxgpio_dirin(GPIOA, 2);                  /* Input */
  imxgpio_gpiofunc(GPIOA, 2);               /* Use as GPIO */
  imxgpio_primaryperipheralfunc(GPIOA, 2);  /* Not necessary */
  imxgpio_ocrain(GPIOA, 2);                 /* Output AIN */
  imxgpio_aoutgpio(GPIOA, 2);               /* AOUT input is GPIO */
  imxgpio_boutgpio(GPIOA, 2);               /* BOUT input is GPIO */
}

/****************************************************************************
 * Name: imxgpio_configprimary
 ****************************************************************************/

void imxgpio_configprimary(int port, int bit)
{
  imxgpio_configinput(port, bit);           /* Same as input except: */
  imxgpio_peripheralfunc(GPIOA, 2);         /* Use as peripheral */
  imxgpio_primaryperipheralfunc(GPIOA, 2);  /* Primary function*/
}
