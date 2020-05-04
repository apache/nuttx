/****************************************************************************
 * arch/arm/src/max326xx/common/max326_icc.c
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
#include <stdbool.h>
#include <assert.h>

#include "arm_arch.h"
#include "hardware/max326_icc.h"
#include "max326_periphclks.h"
#include "max326_icc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max326_icc_enable
 *
 * Description:
 *   Enable/disable the instruction cache
 *
 ****************************************************************************/

void max326_icc_enable(bool enable)
{
  uint32_t regval;

  if (enable)
    {
      /* Enable ICC peripheral clocking */

      max326_icc_enableclk();

      /* Enable the cache and wait for it to become ready */

      do
        {
          putreg32(ICC_CTRLSTAT_ENABLE, MAX326_ICC_CTRLSTAT);
          regval = getreg32(MAX326_ICC_CTRLSTAT);
        }
      while ((regval & ICC_CTRLSTAT_READY) == 0);
    }
   else
     {
       /* Disable the cache */

       putreg32(0, MAX326_ICC_CTRLSTAT);

       /* Disable clocking to the ICC peripheral */

       max326_icc_disableclk();
     }
}

/****************************************************************************
 * Name: max326_icc_invalidate
 *
 * Description:
 *   Invalidate the instruction cache
 *
 ****************************************************************************/

void max326_icc_invalidate(void)
{
  /* Any write to the INVDTALL register will invalidate the entire cache. */

  putreg32(1, MAX326_ICC_INVDTALL);

  /* Wait for the cache to become ready again */

  while ((getreg32(MAX326_ICC_CTRLSTAT) & ICC_CTRLSTAT_READY) == 0)
    {
    }
}

/****************************************************************************
 * Name: up_addrenv_coherent
 *
 * Description:
 *   Flush D-Cache and invalidate I-Cache in preparation for a change in
 *   address environments.  This should immediately precede a call to
 *   up_addrenv_select();
 *
 * Input Parameters:
 *   addrenv - Describes the address environment to be made coherent.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_ADDRENV
int up_addrenv_coherent(FAR const group_addrenv_t *addrenv)
{
  max326_icc_invalidate();
}
#endif
