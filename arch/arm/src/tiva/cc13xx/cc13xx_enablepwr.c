/****************************************************************************
 * arch/arm/src/tiva/cc13xx/cc13xx_enablepwr.c
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
#include "tiva_enablepwr.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint16_t g_domain_usage[2];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  cc13xx_periph_enablepwr
 *
 * Description:
 *   Enable the power domain associated with the peripheral.
 *
 ****************************************************************************/

void cc13xx_periph_enablepwr(uint32_t peripheral)
{
  irqstate_t flags;
  uint32_t domain;
  unsigned int dndx;
  unsigned int pndx;

  dndx   = PRCM_DOMAIN_INDEX(peripheral);
  pndx   = PRCM_PERIPH_ID(peripheral);
  domain = (dndx == 0 ? PRCM_DOMAIN_SERIAL : PRCM_DOMAIN_PERIPH);

  /* Remember that this peripheral needs power in this domain */

  flags = spin_lock_irqsave();
  g_domain_usage[dndx] |= (1 << pndx);

  /* Make sure that power is enabled in that domain */

  prcm_powerdomain_on(domain);
  spin_unlock_irqrestore(flags);

  /* Wait for the power domain to be ready.  REVISIT:  This really should be
   * in the critical section but this could take too long.
   */

  while (!prcm_powerdomain_status(domain))
    {
    }
}

/****************************************************************************
 * Name:  cc13xx_periph_disablepwr
 *
 * Description:
 *   Disable the power domain associated with the peripheral if and only if
 *   all peripherals using that power domain no longer need power.
 *
 ****************************************************************************/

void cc13xx_periph_disablepwr(uint32_t peripheral)
{
  int dndx = PRCM_DOMAIN_INDEX(peripheral);
  int pndx = PRCM_PERIPH_ID(peripheral);
  irqstate_t flags;

  /* This peripheral no longer needs power in this domain */

  flags = spin_lock_irqsave();
  g_domain_usage[dndx] &= ~(1 << pndx);

  /* If there are no peripherals needing power in this domain, then turn off
   * the power domain.
   */

  if (g_domain_usage[dndx] == 0)
    {
      prcm_powerdomain_off(pndx == 0 ?
                           PRCM_DOMAIN_SERIAL : PRCM_DOMAIN_PERIPH);
    }

  spin_unlock_irqrestore(flags);
}
