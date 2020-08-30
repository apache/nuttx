/****************************************************************************
 * arch/arm/src/nrf52/nrf52_clockconfig.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author:  Janne Rosberg <janne@offcode.fi>
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

#include <stdint.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "arm_arch.h"
#include "arm_internal.h"

#include "nrf52_clockconfig.h"
#include "hardware/nrf52_clock.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_clockconfig
 *
 * Description:
 *   Called to initialize the NRF52xxx.  This does whatever setup is needed
 *   to put the MCU in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.  This function also performs
 *   other low-level chip as necessary.
 *
 ****************************************************************************/

void nrf52_clockconfig(void)
{
#ifdef CONFIG_NRF52_HFCLK_XTAL
  /* Initilize HFCLK crystal oscillator */

  putreg32(0x0, NRF52_CLOCK_EVENTS_HFCLKSTARTED);
  putreg32(0x1, NRF52_CLOCK_TASKS_HFCLKSTART);

  while (!getreg32(NRF52_CLOCK_EVENTS_HFCLKSTARTED))
    {
      /* wait for external oscillator to start */
    }
#endif

#ifdef CONFIG_NRF52_USE_LFCLK
  /* Initialize LFCLK */

#if defined(CONFIG_NRF52_LFCLK_XTAL)
  putreg32(CLOCK_LFCLKSRC_SRC_XTAL, NRF52_CLOCK_LFCLKSRC);
#elif defined(CONFIG_NRF52_LFCLK_SYNTH)
  putreg32(CLOCK_LFCLKSRC_SRC_SYNTH, NRF52_CLOCK_LFCLKSRC);
#else
  putreg32(CLOCK_LFCLKSRC_SRC_RC, NRF52_CLOCK_LFCLKSRC);
#endif

  /* Trigger LFCLK start */

  putreg32(0x0, NRF52_CLOCK_EVENTS_LFCLKSTARTED);
  putreg32(0x1, NRF52_CLOCK_TASKS_LFCLKSTART);

  while (!getreg32(NRF52_CLOCK_EVENTS_LFCLKSTARTED))
    {
      /* Wait for LFCLK to be running */
    }

#if defined(CONFIG_NRF52_LFCLK_RC)
  /* TODO: calibrate LFCLK RC oscillator */
#endif
#endif
}
