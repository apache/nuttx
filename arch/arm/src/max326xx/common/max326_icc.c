/****************************************************************************
 * arch/arm/src/max326xx/common/max326_icc.c
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
#include <stdbool.h>
#include <assert.h>

#include "arm_internal.h"
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
int up_addrenv_coherent(const arch_addrenv_t *addrenv)
{
  max326_icc_invalidate();
}
#endif
