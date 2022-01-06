/****************************************************************************
 * arch/arm/src/tiva/cc13xx/cc13xx_enablepwr.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/spinlock.h>

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

  flags = spin_lock_irqsave(NULL);
  g_domain_usage[dndx] |= (1 << pndx);

  /* Make sure that power is enabled in that domain */

  prcm_powerdomain_on(domain);
  spin_unlock_irqrestore(NULL, flags);

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

  flags = spin_lock_irqsave(NULL);
  g_domain_usage[dndx] &= ~(1 << pndx);

  /* If there are no peripherals needing power in this domain, then turn off
   * the power domain.
   */

  if (g_domain_usage[dndx] == 0)
    {
      prcm_powerdomain_off(pndx == 0 ?
                           PRCM_DOMAIN_SERIAL : PRCM_DOMAIN_PERIPH);
    }

  spin_unlock_irqrestore(NULL, flags);
}
