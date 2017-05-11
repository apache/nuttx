/****************************************************************************
 * configs/cc3200-launchpad/src/cc3200_boot.c
 *
 *   Copyright (C) 2014 Droidifi LLC. All rights reserved.
 *   Copyright (C) 2014, 2016 Gregory Nutt.
 *   Author: Jim Ewing <jim@droidifi.com>
 *           Gregory Nutt <gnutt@nuttx.org>
 *
 * Adapted for the cc3200 from code:
 *
 *   Copyright (C) 2014 Gregory Nutt.
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
#include <nuttx/kmalloc.h>

#include <debug.h>
#include <stdio.h>

#include <nuttx/board.h>
#include <arch/board/board.h>
#include <arch/board/cc3200_utils.h>

#include "cc3200_launchpad.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CC3200_SRAM1_BASE 0x20000000
#define CC3200_SRAM1_SIZE 0x4000

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 *   CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 *   CONFIG_BOARD_INITIALIZE=y, CONFIG_NSH_LIBRARY=y, &&
 *   CONFIG_LIB_BOARDCTL=n :
 *     Called from board_initialize().
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initalization logic and the
 *         matching application logic.  The value cold be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{
  return OK;
}

/****************************************************************************
 * Name: tiva_boardinitialize
 *
 * Description:
 *   All Tiva architectures must provide the following entry point.  This entry
 *   point is called early in the initialization -- after all memory has been
 *   configured and mapped but before any devices have been initialized.
 *
 ****************************************************************************/

void tiva_boardinitialize(void)
{
  cc3200_init();
  cc3200_uart_init();

  cc3200_print("\r\nCC3200 init\r\n");

  cc3200_led_initialize();
}

/****************************************************************************
 * Name: up_addregion
 *
 * Description:
 *   Memory may be added in non-contiguous chunks.  Additional chunks are
 *   added by calling this function.
 *
 ****************************************************************************/

#if CONFIG_MM_REGIONS > 1
void up_addregion(void)
{
  kumm_addregion((FAR void*)CC3200_SRAM1_BASE, CC3200_SRAM1_SIZE);
}
#endif
