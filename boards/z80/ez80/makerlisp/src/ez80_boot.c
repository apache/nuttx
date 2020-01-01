/****************************************************************************
 * boards/z80/ez80/makerlisp/src/ez80_boot.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Parts of this file are based on MakerLisp sample code by Luther Johnson
 * which has a compatible MIT license
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

#include <stdbool.h>

#include <nuttx/arch.h>
#include <arch/chip/io.h>

#include "chip.h"
#include "z80_internal.h"
#include "makerlisp.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

bool g_ebpresent = false;  /* True:  I/O Expansion board is present */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VGA_MAX_DELAY 2000000

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ez80_vga_initialize
 *
 * Description:
 *   If CONFIG_MAKERLISP_VGA is defined and the I/O controller is attached,
 *   then initialize the VGA interface.
 *
 ****************************************************************************/

#ifdef CONFIG_MAKERLISP_VGA
static void ez80_vga_initialize(void)
{
  /* I/O Expansion board attached? */

  if (g_ebpresent)
    {
      bool vgapresent = false;
      int delay;

      /* Wait for VGA ready */

      for (delay = 0; delay < VGA_MAX_DELAY; delay++)
        {
          if ((inp(EZ80_PB_DR) & EZ80_GPIOD1) != 0)
            {
              vgapresent = true;
              break;
            }
        }

      /* Is VGA ready (and, hence, present)? */

      if (vgapresent)
        {
          /* Yes.. set newline mode, graphic attributes:
           *
           * \e = ESCAPE character
           * Assumption:  VGA is on the console UART.
           */

          up_puts("\e[20h\e[0m");

          /* Clear, home cursor, beep */

          up_puts("\e[2J\e[H\a");

          /* The VGA display controller and keyboard controller come up by
           * default emulating a terminal with "newline mode" on.  The
           * following turns off that mode.
           */

          up_puts("\e[20l");
        }
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ez80_board_initialize
 *
 * Description:
 *   All eZ80 architectures must provide the following entry point.  This
 *   entry point is called early in the initialization -- after basic CPU
 *   configuration is complete but before any devices have been initialized.
 *
 ****************************************************************************/

void ez80_board_initialize(void)
{
  register uint8_t regval;

  /* Port B pin 5 is set if the I/O expansion board is present */

  regval = inp(EZ80_PB_DR);
  g_ebpresent = (regval & EZ80_GPIOD5 != 0);

  /* Set Port B pin 5 as output, assert /sysreset, SD card power off */

  regval &= ~EZ80_GPIOD5;
  outp(EZ80_PB_DR, regval);

  regval  = inp(EZ80_PB_ALT1);
  regval &= ~EZ80_GPIOD5;
  outp(EZ80_PB_ALT1, regval);

  regval  = inp(EZ80_PB_ALT2);
  regval &= ~EZ80_GPIOD5;
  outp(EZ80_PB_ALT2, regval);

  regval  = inp(EZ80_PB_DDR);
  regval &= ~EZ80_GPIOD5;
  outp(EZ80_PB_DDR, regval);

#ifdef CONFIG_EZ80_SPI
  /* Initialize SPI chip selects */

  ez80_spidev_initialize();
#endif

  /* Leave /sysreset asserted for awhile longer */

  up_udelay(150);

  /* Take the system out of reset and and turn on SD card power */

  regval  = inp(EZ80_PB_DR);
  regval |= EZ80_GPIOD5;
  outp(EZ80_PB_DR, regval);

  /* Wait for the SD card to power up */

  up_udelay(750);

#ifdef CONFIG_MAKERLISP_VGA
  /* Initialize the VGA interface.  We want to do this as early as possible
   * in the boot-up sequence.  Debug output prior initializing VGA will be
   * lost.
   */

  ez80_vga_initialize();
#endif
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize().  board_late_initialize() will be
 *   called immediately after up_initialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  /* Perform board-specific initialization */

  ez80_bringup();
}
#endif
