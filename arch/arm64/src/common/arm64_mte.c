/****************************************************************************
 * arch/arm64/src/common/arm64_mte.c
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

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "arm64_arch.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GCR_EL1_VAL     0x10001

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int arm64_mte_is_support(void)
{
  int supported;
  __asm__ volatile (
    "mrs %0, ID_AA64PFR1_EL1\n"
    "ubfx %0, %0, #8, #3\n"
    : "=r" (supported)
    :
    : "memory"
  );
  return supported != 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void arm64_enable_mte(void)
{
  uint64_t val;

  if (!arm64_mte_is_support())
    {
      return;
    }

  /* CnP must be enabled only after the MAIR_EL1 register has been set
   * up. Inconsistent MAIR_EL1 between CPUs sharing the same TLB may
   * lead to the wrong memory type being used for a brief window during
   * CPU power-up.
   *
   * CnP is not a boot feature so MTE gets enabled before CnP, but let's
   * make sure that is the case.
   */

  assert(!(read_sysreg(ttbr0_el1) & TTBR_CNP_BIT));
  assert(!(read_sysreg(ttbr1_el1) & TTBR_CNP_BIT));

  val = read_sysreg(sctlr_el1);
  val |= SCTLR_ATA_BIT | SCTLR_TCF1_BIT;
  write_sysreg(val, sctlr_el1);

  write_sysreg(GCR_EL1_VAL, gcr_el1);

  /* If GCR_EL1.RRND=1 is implemented the same way as RRND=0, then
   * RGSR_EL1.SEED must be non-zero for IRG to produce
   * pseudorandom numbers. As RGSR_EL1 is UNKNOWN out of reset, we
   * must initialize it.
   */

  val = (read_sysreg(CNTVCT_EL0) & RGSR_EL1_SEED_MASK) <<
         RGSR_EL1_SEED_SHIFT;

  if (0 == val)
    {
      val = 1 << RGSR_EL1_SEED_SHIFT;
    }

  write_sysreg(val, rgsr_el1);

  /* clear any pending tag check faults in TFSR*_EL1 */

  write_sysreg(0, tfsr_el1);
  write_sysreg(0, tfsre0_el1);
}
