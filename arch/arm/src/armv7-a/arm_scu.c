/****************************************************************************
 * arch/arm/src/armv7-a/arm_scu.c
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

#include "arm_internal.h"
#include "cp15_cacheops.h"
#include "barriers.h"
#include "sctlr.h"
#include "scu.h"

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
  regval |= SCTLR_C | SCTLR_I | SCTLR_M;
  arm_set_sctlr(regval);
}
