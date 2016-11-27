/****************************************************************************
 * arch/arm/src/armv7-a/arm_scu.c
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

#include <stdint.h>

#include <nuttx/irq.h>

#include "up_arch.h"
#include "sctlr.h"
#include "scu.h"

#ifdef CONFIG_SMP

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_get_actlr
 *
 * Description:
 *   Get the contents of the ACTLR register
 *
 ****************************************************************************/

static inline uint32_t arm_get_actlr(void)
{
  uint32_t actlr;

  __asm__ __volatile__
  (
    "\tmrc  p15, 0, %0, c1, c0, 1\n"  /* Read ACTLR */
    : "=r"(actlr)
    :
    :
  );

  return actlr;
}

/****************************************************************************
 * Name: arm_set_actlr
 *
 * Description:
 *   Set the contents of the ACTLR register
 *
 ****************************************************************************/

static inline void arm_set_actlr(uint32_t actlr)
{
  __asm__ __volatile__
  (
    "\tmcr p15, 0, %0, c1, c0, 1\n" /* Write ACTLR */
    :
    : "r"(actlr)
    :
  );
}

/****************************************************************************
 * Name: arm_modify_actlr
 *
 * Description:
 *   Set the bits in the ACTLR register
 *
 ****************************************************************************/

static inline void arm_modify_actlr(uint32_t setbits)
{
  irqstate_t flags = enter_critical_section();
  uint32_t actlr = arm_get_actlr();
  arm_set_actlr(actlr | setbits);
  leave_critical_section(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_enable_smp
 *
 * Description:
 *   Enable the SCU and make certain that current CPU is participating in
 *   the SMP cache coherency.
 *
 ****************************************************************************/

void arm_enable_smp(int cpu)
{
  /* Invalidate the data cache -- Missing logic. */

  /* Handle actions unique to CPU0 */

  if (cpu == 0)
    {
      /* Invalidate the SCU duplicate tags for all processors */

      putreg32((SCU_INVALIDATE_ALL_WAYS << SCU_INVALIDATE_CPU0_SHIFT) |
               (SCU_INVALIDATE_ALL_WAYS << SCU_INVALIDATE_CPU1_SHIFT) |
               (SCU_INVALIDATE_ALL_WAYS << SCU_INVALIDATE_CPU2_SHIFT) |
               (SCU_INVALIDATE_ALL_WAYS << SCU_INVALIDATE_CPU3_SHIFT),
               SCU_INVALIDATE);

      /* Invalidate the L2C-310 -- Missing logic. */

      /* Enable the SCU */

      modifyreg32(SCU_CTRL, 0, SCU_CTRL_ENABLE); /* CPU0 only */
    }

  /* Enable the data cache -- Missing logic. */

  /* This CPU now participates the SMP cache coherency */

  arm_modify_actlr(ACTLR_SMP);
}

#endif
