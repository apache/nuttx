/****************************************************************************
 * arch/arm/src/am335x/am335x_clockconfig.c
 *
 *   Copyright (C) 2018 Petro Karashchenko. All rights reserved.
 *   Author: Petro Karashchenko <petro.karashchenko@gmail.com>
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

#include "up_arch.h"
#include "hardware/am335x_prcm.h"
#include "am335x_config.h"
#include "am335x_clockconfig.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am335x_dmtimer1ms_clockconfig
 ****************************************************************************/

static inline void am335x_dmtimer1ms_clockconfig(void)
{
  putreg32(CM_DPLL_DMTIMER1_CLKSEL_CLK_M_OSC,
           AM335X_CM_DPLL_CLKSEL_TIMER1MS_CLK);

  while ((getreg32(AM335X_CM_DPLL_CLKSEL_TIMER1MS_CLK) &
          CM_DPLL_DMTIMER1MS_CLKSEL_MASK)
          != CM_DPLL_DMTIMER1_CLKSEL_CLK_M_OSC)
    {
    }

  modifyreg32(AM335X_CM_WKUP_TIMER1_CLKCTRL, CM_WKUP_CLKCTRL_MODULEMODE_MASK,
              CM_WKUP_CLKCTRL_MODULEMODE_ENABLE);

  while ((getreg32(AM335X_CM_WKUP_TIMER1_CLKCTRL) &
          (CM_WKUP_CLKCTRL_MODULEMODE_MASK | CM_WKUP_CLKCTRL_IDLEST_MASK))
         != (CM_WKUP_CLKCTRL_MODULEMODE_ENABLE | CM_WKUP_CLKCTRL_IDLEST_FUNC))
    {
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am335x_clockconfig
 *
 * Description:
 *   Called to initialize the AM335X.  This does whatever setup is needed to
 *   put the SoC in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.
 *
 ****************************************************************************/

void am335x_clockconfig(void)
{
  /* Don't change the current basic clock configuration if we are running
   * from SDRAM.  In this case, some bootloader logic has already configured
   * clocking and SDRAM.  We are pretty much committed to using things the
   * way that the bootloader has left them.
   *
   * Clocking will be configured at 792 MHz initially when started via
   * U-Boot.  The Linux kernel will uses the CPU frequency scaling code
   * which will switch the processor frequency between 400 MHz and 1GHz based
   * on load and temperature.  For now, NuttX simply leaves the clocking at
   * 792MHz.
   */

  am335x_dmtimer1ms_clockconfig();

#ifndef CONFIG_AM335X_BOOT_SDRAM
#  warning Missing logic
#endif
}
