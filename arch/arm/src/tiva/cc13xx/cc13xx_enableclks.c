/****************************************************************************
 * arch/arm/src/tiva/cc13xx/cc13xx_enableclks.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>

#include "hardware/tiva_prcm.h"
#include "tiva_enableclks.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  cc13xx_periph_enableclks
 *
 * Description:
 *   Enable clocking in the selected modes for this peripheral.
 *
 ****************************************************************************/

void cc13xx_periph_enableclk(uint32_t peripheral, uint32_t modeset)
{
  DEBUGASSERT(modeset != 0);

  if ((modeset & CC13XX_RUNMODE_CLOCK) != 0)
    {
      prcm_periph_runenable(peripheral);
    }

  if ((modeset & CC13XX_SLEEPMODE_CLOCK) != 0)
    {
      prcm_periph_sleepenable(peripheral);
    }

  if ((modeset & CC13XX_DEEPSLEEPMODE_CLOCK) != 0)
    {
      prcm_periph_deepsleepenable(peripheral);
    }

  prcm_load_set();
  while (!prcm_load_get())
    {
    }
}

/****************************************************************************
 * Name:  cc13xx_periph_disableclk
 *
 * Description:
 *   Disable clocking in the selected modes for this peripheral.
 *
 ****************************************************************************/

void cc13xx_periph_disableclk(uint32_t peripheral, uint32_t modeset)
{
  DEBUGASSERT(modeset != 0);

  if ((modeset & CC13XX_RUNMODE_CLOCK) != 0)
    {
      prcm_periph_rundisable(peripheral);
    }

  if ((modeset & CC13XX_SLEEPMODE_CLOCK) != 0)
    {
      prcm_periph_sleepdisable(peripheral);
    }

  if ((modeset & CC13XX_DEEPSLEEPMODE_CLOCK) != 0)
    {
      prcm_periph_deepsleepdisable(peripheral);
    }

  prcm_load_set();
}
