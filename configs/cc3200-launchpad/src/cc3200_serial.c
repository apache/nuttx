/************************************************************************************
 * configs/cc3200/src/cc3200_serial.c
 *
 *   Copyright (C) 2013 Droidifi LLC. All rights reserved.
 *   Author: Jim Ewing <jim@droidifi.com>
 *
 *   Adapted for the cc3200 from code:
 *
 *   Copyright (C) Gregory Nutt.
 *   Gregory Nutt <gnutt@nuttx.org>
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

#include <nuttx/arch.h>

#include <arch/board/cc3200_utils.h>

#include "chip/cc3200_memorymap.h"
#include "tiva_start.h"
#include "up_internal.h"

#include "cc3200_launchpad.h"

#if !defined(HAVE_SERIALCONSOLE)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#ifndef CONFIG_TIVA_BOARD_EARLYINIT
#  error CONFIG_TIVA_BOARD_EARLYINIT is required
#endif

/************************************************************************************
 * Private Data
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static void cc3200_uart0_init(void)
{
  HWREG(0x44025048) |= 0x01;

  cc3200_pin_type_uart(PIN_55, PIN_MODE_3);
  cc3200_pin_type_uart(PIN_57, PIN_MODE_3);

  while(HWREG(0x4000C018) & 0x08)
    {
    }

  HWREG(0x4000C02C) &= ~(0x00000010);
  HWREG(0x4000C030) &= ~(0x01 | 0x100 | 0x200);
  HWREG(0x4000C030) &= ~(0x20);

  HWREG(0x4000C024)  = ((((80000000 * 8) / 115200) + 1) / 2) / 64;
  HWREG(0x4000C028)  = ((((80000000 * 8) / 115200) + 1) / 2) % 64;

  HWREG(0x4000C02C)  = (0x60 | 0x82 | 0x10);
  HWREG(0x4000C030) |= (0x01 | 0x100 | 0x200);
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: board_earlyinit
 *
 * Description:
 *   Performs the low level UART initialization early in  debug so that the serial
 *   console will be available during bootup.  This must be called before
 *   up_consoleinit.
 *
 ************************************************************************************/

void board_earlyinit(void)
{
  cc3200_init();
  cc3200_uart0_init();
}

/************************************************************************************
 * Name: up_consoleinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes that
 *   board_earlyinit was called previously.
 *
 ************************************************************************************/

#if USE_SERIALDRIVER
void up_consoleinit(void)
{
  /* There is probably a problem if we are here */

  lowconsole_init();
}
#endif

/************************************************************************************
 * Name: cc3200_uart_init
 ************************************************************************************/

void cc3200_uart_init(void)
{
  cc3200_uart0_init();
}

#endif /* !HAVE_SERIALCONSOLE && CONFIG_ARCH_LCD */
