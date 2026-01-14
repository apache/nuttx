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
#include "arm64_mmu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GCR_EL1_VAL     0x10001
#define MTE_TAG_SHIFT   56

/* The alignment length of the MTE must be a multiple of sixteen */

#define MTE_MM_AILGN    16

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int mte_is_support(void)
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

uint8_t up_memtag_get_tag(const void *addr)
{
  return 0xf0 | (uint8_t)(((uint64_t)addr) >> MTE_TAG_SHIFT);
}

uint8_t up_memtag_get_random_tag(const void *addr)
{
  asm("irg %0, %0" : "=r" (addr));

  return up_memtag_get_tag(addr);
}

void *up_memtag_set_tag(const void *addr, uint8_t tag)
{
  return (FAR void *)
         ((((uint64_t)addr) & ~((uint64_t)0xff << MTE_TAG_SHIFT)) |
          ((uint64_t)tag << MTE_TAG_SHIFT));
}

/* Set MTE state */

bool up_memtag_bypass(bool bypass)
{
  uint64_t val = read_sysreg(sctlr_el1);
  bool state = !(val & SCTLR_TCF1_BIT);

  if (bypass)
    {
      val &= ~SCTLR_TCF1_BIT;
    }
  else
    {
      val |= SCTLR_TCF1_BIT;
    }

  write_sysreg(val, sctlr_el1);
  return state;
}

/* Set memory tags for a given memory range */

void up_memtag_tag_mem(const void *addr, size_t size)
{
  size_t i;

  DEBUGASSERT((uintptr_t)addr % MTE_MM_AILGN == 0);
  DEBUGASSERT(size % MTE_MM_AILGN == 0);

  for (i = 0; i < size; i += MTE_MM_AILGN)
    {
      asm("stg %0, [%0]" : : "r"(addr + i));
    }
}

/* Initialize MTE settings and enable memory tagging */

void arm64_mte_init(void)
{
  uint64_t val;

  if (!mte_is_support())
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

  /* Controls the default value for skipping high bytes */

  val = read_sysreg(tcr_el1);
  val |= TCR_TCMA1;
  write_sysreg(val, tcr_el1);

  /* Enable the MTE function */

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
