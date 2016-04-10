/****************************************************************************
 * arch/arm/src/sama5/sam_sckc.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "sam_sckc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_sckc_enable
 *
 * Description:
 *   Enable or disable the slow clock oscillator driver by an external
 *   crystal.
 *
 * Input Parameters:
 *   enable - True: enable the slow clock, False: disable the slow clock
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_sckc_enable(bool enable)
{
  uint32_t regval;

#ifdef ATSAMA5D3
  /* REVISIT: Missing the logic that disables the external OSC32 */
  /* Enable external OSC 32 kHz */

  regval  = getreg32(SAM_SCKC_CR);
  regval |= SCKC_CR_OSC32EN;
  putreg32(regval, SAM_SCKC_CR);

  /* Wait for 32,768 XTAL start-up time */

  up_udelay(5 * USEC_PER_SEC / BOARD_SLOWCLK_FREQUENCY);

  /* Disable OSC 32 kHz bypass */

  regval &= ~SCKC_CR_OSC32BYP;
  putreg32(regval, SAM_SCKC_CR);

  /* Switch slow clock source to external OSC 32 kHz */

  regval |= SCKC_CR_OSCSEL;
  putreg32(regval, SAM_SCKC_CR);

  /* Wait 5 slow clock cycles for internal resynchronization */

  up_udelay(5 * USEC_PER_SEC / BOARD_SLOWCLK_FREQUENCY);

  /* Disable internal RC 32 kHz */

  regval &= ~SCKC_CR_RCEN;
  putreg32(regval, SAM_SCKC_CR);

#else
  /* Switch slow clock source to external OSC 32 kHz */

  regval = enable ? SCKC_CR_OSCSEL : 0;
  putreg32(regval, SAM_SCKC_CR);

  /* Wait 5 slow clock cycles for internal resynchronization */

  up_udelay(5 * USEC_PER_SEC / BOARD_SLOWCLK_FREQUENCY);
#endif
}
