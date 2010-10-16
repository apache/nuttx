/**************************************************************************
 * arch/avr/src/at91uc3/at91uc3_clkinit.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/config.h>

#include <arch/board/board.h>

#include "up_arch.h"

#include "at91uc3_config.h"
#include "up_internal.h"
#include "at91uc3_internal.h"
#include "at91uc3_pm.h"

/**************************************************************************
 * Private Definitions
 **************************************************************************/

/**************************************************************************
 * Private Types
 **************************************************************************/

/**************************************************************************
 * Private Function Prototypes
 **************************************************************************/

/**************************************************************************
 * Global Variables
 **************************************************************************/

/**************************************************************************
 * Private Variables
 **************************************************************************/

/**************************************************************************
 * Private Functions
 **************************************************************************/

/**************************************************************************
 * Name: up_enableosc32
 *
 * Description:
 *   Initialiaze the 32KHz oscillaor.  This oscillaor is used by the RTC
 *   logic to provide the sysem timer.
 *
 **************************************************************************/

static inline void up_enableosc32(void)
{
  uint32_t regval;
 
  /* Select the 32KHz oscillator crystal */

  regval = getreg32(AVR32_PM_OSCCTRL32);
  regval &= ~PM_OSCCTRL32_MODE_MASK;
  regval |= PM_OSCCTRL32_MODE_XTAL;
  putreg32(regval, AVR32_PM_OSCCTRL32);

  /* Enable the 32-kHz clock */

  regval = getreg32(AVR32_PM_OSCCTRL32);
  regval &= ~PM_OSCCTRL_STARTUP_MASK;
  regval |= PM_OSCCTRL32_EN|(AVR32_OSC32STARTUP << PM_OSCCTRL_STARTUP_SHIFT);
  putreg32(regval, AVR32_PM_OSCCTRL32);
}

/**************************************************************************
 * Name: up_enableosc0
 *
 * Description:
 *   Initialiaze clock/PLL settings per the definitions in the board.h
 *   file.
 *
 **************************************************************************/

static inline void up_enableosc0(void)
{
  uint32_t regval;

  /* Enable OSC0 in the correct crystal mode by setting the mode value in OSCCTRL0 */

  regval  = getreg32(AVR32_PM_OSCCTRL0);
  regval &= ~PM_OSCCTRL_MODE_MASK;
#if AVR32_FOSC0 < 900000
  regval |= PM_OSCCTRL_MODE_XTALp9;  /* Crystal XIN 0.4-0.9MHz */
#elif AVR32_FOSC0 < 3000000
  regval |= PM_OSCCTRL_MODE_XTAL3;   /* Crystal XIN 0.9-3.0MHz */
#elif AVR32_FOSC0 < 8000000
  regval |= PM_OSCCTRL_MODE_XTAL8;   /* Crystal XIN 3.0-8.0MHz */
#else
  regval |= PM_OSCCTRL_MODE_XTALHI;  /* Crystal XIN above 8.0MHz */
#endif
  putreg32(regval, AVR32_PM_OSCCTRL0);

  /* Enable CLK 0 using the startup time provided in board.h.  This startup time
   * is critical and depends on the characteristics of the crystal.
   */

  regval  = getreg32(AVR32_PM_OSCCTRL0);
  regval &= ~PM_OSCCTRL_STARTUP_MASK;
  regval |= (AVR32_OSC0STARTUP << PM_OSCCTRL_STARTUP_SHIFT);
  putreg32(regval, AVR32_PM_OSCCTRL0);

  /* Enable OSC0 */

  regval = getreg32(AVR32_PM_MCCTRL);
  regval |= PM_MCCTRL_OSC0EN;
  putreg32(regval, AVR32_PM_MCCTRL);

  /* Wait for CLK0 to be ready */

  while ((getreg32(AVR32_PM_POSCSR) & PM_POSCSR_OSC0RDY) == 0);
}

/**************************************************************************
 * Name: up_mainclk
 *
 * Description:
 *   Initialiaze clock/PLL settings per the definitions in the board.h
 *   file.
 *
 **************************************************************************/

static inline void up_mainclk(uint32_t mcsel)
{
  uint32_t regval;
 
  regval = getreg32(AVR32_PM_MCCTRL);
  regval &= ~PM_MCCTRL_MCSEL_MASK;
  regval |= mcsel;
  putreg32(regval, AVR32_PM_MCCTRL);
}

/**************************************************************************
 * Public Functions
 **************************************************************************/

/**************************************************************************
 * Name: up_clkinit
 *
 * Description:
 *   Initialiaze clock/PLL settings per the definitions in the board.h
 *   file.
 *
 **************************************************************************/

void up_clkinitialize(void)
{
  /* Enable the 32KHz oscillator (need by the RTC module) */

  up_enableosc32();

#if defined(AVR32_CLOCK_OSC0)
  /* Enable OSC0 using the settings in board.h */

  up_enableosc0();
 
  /* Then switch the main clock to OSC0 */

  up_mainclk(PM_MCCTRL_MCSEL_OSC0);

#elif defined(AVR32_CLOCK_PLL0)
#  warning "Missing Logic"
#else
#  error "No main clock"
#endif
}



