/****************************************************************************
 * arch/arm/src/armv7-a/arm_scu.c
 *
 *   Copyright (C) 2016, 2019 Gregory Nutt. All rights reserved.
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

#include "arm_arch.h"
#include "cp15_cacheops.h"
#include "barriers.h"
#include "sctlr.h"
#include "scu.h"

#ifdef CONFIG_SMP

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_get_sctlr
 *
 * Description:
 *   Get the contents of the SCTLR register
 *
 ****************************************************************************/

static inline uint32_t arm_get_sctlr(void)
{
  uint32_t sctlr;

  __asm__ __volatile__
  (
    "\tmrc   p15, 0, %0, c1, c0, 0\n"  /* Read SCTLR */
    : "=r"(sctlr)
    :
    :
  );

  return sctlr;
}

/****************************************************************************
 * Name: arm_set_sctlr
 *
 * Description:
 *   Set the contents of the SCTLR register
 *
 ****************************************************************************/

static inline void arm_set_sctlr(uint32_t sctlr)
{
  __asm__ __volatile__
  (
    "\tmcr  p15, 0, %0, c1, c0, 0\n" /* Write SCTLR */
    :
    : "r"(sctlr)
    :
  );
}

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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_enable_smp
 *
 * Description:
 *   Enable the SCU and make certain that current CPU is participating in
 *   the SMP cache coherency.
 *
 * Assumption:
 *   Called early in the CPU start-up.  No special critical sections are
 *   needed if only CPU-private registers are modified.
 *
 ****************************************************************************/

void arm_enable_smp(int cpu)
{
  uint32_t regval;

  /* Handle actions unique to CPU0 which comes up first */

  if (cpu == 0)
    {
      /* Invalidate the SCU duplicate tags for all processors */

      putreg32((SCU_INVALIDATE_ALL_WAYS << SCU_INVALIDATE_CPU0_SHIFT) |
               (SCU_INVALIDATE_ALL_WAYS << SCU_INVALIDATE_CPU1_SHIFT) |
               (SCU_INVALIDATE_ALL_WAYS << SCU_INVALIDATE_CPU2_SHIFT) |
               (SCU_INVALIDATE_ALL_WAYS << SCU_INVALIDATE_CPU3_SHIFT),
               SCU_INVALIDATE);

      /* Invalidate CPUn L1 data cache so that is will we be reloaded from
       * coherent L2.
       */

      cp15_invalidate_dcache_all();
      ARM_DSB();

      /* Invalidate the L2C-310 -- Missing logic. */

      /* Enable the SCU */

      regval  = getreg32(SCU_CTRL);
      regval |= SCU_CTRL_ENABLE;
      putreg32(regval, SCU_CTRL);
    }

  /* Actions for other CPUs */

  else
    {
      /* Invalidate CPUn L1 data cache so that is will we be reloaded from
       * coherent L2.
       */

      cp15_invalidate_dcache_all();
      ARM_DSB();

      /* Wait for the SCU to be enabled by the primary processor -- should
       * not be necessary.
       */
    }

  /* Enable the data cache, set the SMP mode with ACTLR.SMP=1.
   *
   *   SMP - Sgnals if the Cortex-A9 processor is taking part in coherency
   *         or not.
   *
   * Cortex-A9 also needs  ACTLR.FW=1
   *
   *   FW  - Cache and TLB maintenance broadcast.
   */

  regval  = arm_get_actlr();
  regval |= ACTLR_SMP;
#ifdef CONFIG_ARCH_CORTEXA9
  regval |= ACTLR_FW;
#endif
  arm_set_actlr(regval);

  regval  = arm_get_sctlr();
  regval |= SCTLR_C;
  arm_set_sctlr(regval);
}

#endif
