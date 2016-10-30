/****************************************************************************
 * arch/xtensa/src/common/xtensa_cpenable.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#include <nuttx/irq.h>
#include <nuttx/sched.h>

#include <arch/xtensa/xtensa_coproc.h>
#include <arch/chip/core-isa.h>

#include "xtensa.h"

#if XCHAL_CP_NUM > 0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_coproc_enable
 *
 * Description:
 *   Enable a set of co-processors.
 *
 * Input Parameters:
 *   cpstate - A pointer to the Co-processor state save structure.
 *   cpset   - A bit set of co-processors to be enabled.  Matches bit layout
 *             of the CPENABLE register.  Bit 0-XCHAL_CP_NUM:  0 = no change
 *             1 = enable
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void xtensa_coproc_enable(struct xtensa_cpstate_s *cpstate, int cpset)
{
  irqstate_t flags;
  uint32_t cpenable;

  /* These operations must be atomic */

  flags = enter_critical_section();

  /* Don't enable co-processors that may already be enabled
   *
   *          cpenable
   *            0   1
   *          --- ---
   *  cpset 0 | 0   0
   *        1 | 1   0
   */

  cpset ^= (cpset & cpstate->cpenable);
  if (cpset != 0)
    {
      /* Enable the co-processors */

      cpenable = xtensa_get_cpenable();
      cpenable |= cpset;
      xtensa_set_cpenable(cpenable);

      cpstate->cpenable  = cpenable;
      cpstate->cpstored &= ~cpset;
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: xtensa_coproc_disable
 *
 * Description:
 *   Enable a set of co-processors.
 *
 * Input Parameters:
 *   cpstate - A pointer to the Co-processor state save structure.
 *   cpset   - A bit set of co-processors to be enabled.  Matches bit layout
 *             of the CPENABLE register.  Bit 0-XCHAL_CP_NUM:  0 = no change
 *             1 = disable
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void xtensa_coproc_disable(struct xtensa_cpstate_s *cpstate, int cpset)
{
  irqstate_t flags;
  uint32_t cpenable;

  /* These operations must be atomic */

  flags = enter_critical_section();

  /* Don't disable co-processors that are already be disabled.
   *
   *          cpenable
   *            0   1
   *          --- ---
   *  cpset 0 | 0   0
   *        1 | 0   1
   */

  cpset &= cpstate->cpenable;
  if (cpset != 0)
    {
      /* Disable the co-processors */

      cpenable = xtensa_get_cpenable();
      cpenable &= ~cpset;
      xtensa_set_cpenable(cpenable);

      cpstate->cpenable  = cpenable;
      cpstate->cpstored &= ~cpset;
    }

  leave_critical_section(flags);
}

#endif /* XCHAL_CP_NUM */
