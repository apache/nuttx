/****************************************************************************
 * boards/z16/z16f/z16f2800100zcog/src/z16f_boot.c
 *
 *   Copyright (C) 2008, 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Based upon sample code included with the Zilog ZDS-II toolchain.
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

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z16f_gpioinit
 *
 * Description:
 *   Configure board-specific GPIO usage here.  Driver pin configurations
 *   are set in the associated device drivers (such as UART, SPI, I2C,
 *   etc.) and must be preserved.
 *
 ****************************************************************************/

static void z16f_gpioinit(void)
{
  /* Configure LEDs and Run/Stop switch port */

  putreg8(getreg8(Z16F_GPIOA_DD) | 0x87, Z16F_GPIOA_DD);
  putreg8(getreg8(Z16F_GPIOA_OUT) | 0x07, Z16F_GPIOA_OUT);
  putreg8(getreg8(Z16F_GPIOA_DD) & 0xf8, Z16F_GPIOA_DD);

  /* Configure rate switch port */

  putreg8(getreg8(Z16F_GPIOB_DD) | 0x20, Z16F_GPIOB_DD);
  putreg8(getreg8(Z16F_GPIOB_AFL) | 0x20, Z16F_GPIOB_AFL);

#if 0 /* Not yet */
  putreg8(0x05, Z16F_ADC0_MAX);
  putreg8(0xf5, Z16F_ADC0_CTL);
#endif

  /* Configure Direction switch port */

  putreg8(getreg8(Z16F_GPIOC_DD) | 0x01, Z16F_GPIOC_DD);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void z16f_board_initialize(void)
{
  z16f_gpioinit();
}
