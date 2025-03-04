/****************************************************************************
 * arch/arm64/src/common/arm64_mpu.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <debug.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <arch/barriers.h>
#include <arch/irq.h>
#include <arch/chip/chip.h>

#include "arm64_arch.h"
#include "arm64_internal.h"
#include "arm64_fatal.h"
#include "arm64_mpu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define __MPU_ASSERT(__cond, fmt, ...) \
  do                                   \
    {                                  \
      if (!(__cond))                   \
        {                              \
          sinfo(fmt, ## __VA_ARGS__);  \
          PANIC();                     \
        }                              \
    }                                  \
  while (false)

/* AArch64 Memory Model Feature Register 0
 * Provides information about the implemented memory model and memory
 * management support in AArch64 state.
 * See Arm Architecture Reference Manual Supplement
 *  Armv8, for Armv8-R AArch64 architecture profile, G1.3.7
 *
 * ID_AA64MMFR0_MSA_FRAC, bits[55:52]
 * ID_AA64MMFR0_MSA, bits [51:48]
 */

#define ID_AA64MMFR0_MSA_MSK        (0xFFUL << 48U)
#define ID_AA64MMFR0_PMSA_EN        (0x1FUL << 48U)
#define ID_AA64MMFR0_PMSA_VMSA_EN   (0x2FUL << 48U)

/* Global status variable holding the number of HW MPU region indices, which
 * have been reserved by the MPU driver to program the static (fixed) memory
 * regions.
 */

static unsigned int g_mpu_region;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: get_num_regions
 *
 * Description:
 *   Get the number of supported MPU regions.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Numbers of the region.
 *
 ****************************************************************************/

static inline unsigned int get_num_regions(void)
{
  uint64_t type;

  type = read_sysreg(mpuir_el1);
  type = type & MPU_IR_REGION_MSK;

  return (unsigned int)type;
}

/****************************************************************************
 * Name: mpu_init
 *
 * Description:
 *   Configure the cache-ability attributes for all the different types
 *   of memory regions.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpu_init(void)
{
  /* Device region(s): Attribute-0
   * Flash region(s): Attribute-1
   * SRAM region(s): Attribute-2
   * SRAM no cache-able regions(s): Attribute-3
   */

  uint64_t mair = MPU_MAIR_ATTRS;

  write_sysreg(mair, mair_el1);
  UP_MB();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpu_allocregion
 *
 * Description:
 *   Allocate the next region
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The index of the allocated region.
 *
 ****************************************************************************/

unsigned int mpu_allocregion(void)
{
  unsigned int num_regions = get_num_regions();
  unsigned int i = ffs(~g_mpu_region) - 1;

  /* There are not enough regions to apply */

  DEBUGASSERT(i < num_regions);
  g_mpu_region |= 1 << i;
  return i;
}

/****************************************************************************
 * Name: mpu_freeregion
 *
 * Description:
 *   Free target region.
 *
 * Input Parameters:
 *  region - The index of the region to be freed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void mpu_freeregion(unsigned int region)
{
  unsigned int num_regions = get_num_regions();

  /* Check region vaild */

  DEBUGASSERT(region < num_regions);

  write_sysreg(region, prselr_el1);
  UP_DSB();

  /* Set the region base, limit and attribute */

  write_sysreg(0, prbar_el1);
  write_sysreg(0, prlar_el1);
  g_mpu_region &= ~(1 << region);
  UP_MB();
}

/****************************************************************************
 * Name: arm64_mpu_enable
 *
 * Description:
 *   Enable the MPU
 *
 * Input Parameters:
 *   None
 *
 * Return Value:
 *   None
 *
 ****************************************************************************/

void arm64_mpu_enable(void)
{
  uint64_t val;

  val = read_sysreg(sctlr_el1);
  val |= (SCTLR_M_BIT
#ifndef CONFIG_ARM64_DCACHE_DISABLE
          | SCTLR_C_BIT
#endif
         );
  write_sysreg(val, sctlr_el1);
  UP_MB();
}

/****************************************************************************
 * Name: arm64_mpu_disable
 *
 * Description:
 *   Disable the MPU
 *
 * Input Parameters:
 *   None
 *
 * Return Value:
 *   None
 *
 ****************************************************************************/

void arm64_mpu_disable(void)
{
  uint64_t val;

  /* Force any outstanding transfers to complete before disabling MPU */

  UP_DMB();

  val = read_sysreg(sctlr_el1);
  val &= ~(SCTLR_M_BIT | SCTLR_C_BIT);
  write_sysreg(val, sctlr_el1);
  UP_MB();
}

/****************************************************************************
 * Name: mpu_modify_region
 *
 * Description:
 *   Modify a region for privileged, strongly ordered memory
 *
 * Input Parameters:
 *   region - The index of the MPU region to modify.
 *   table  - Pointer to a struct containing the configuration
 *            parameters for the region.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void mpu_modify_region(unsigned int region,
                       const struct arm64_mpu_region *table)
{
  uint64_t rbar = table->base & MPU_RBAR_BASE_MSK;
  uint64_t rlar = (table->limit - 1) & MPU_RLAR_LIMIT_MSK;

  /* Check that the region is valid */

  DEBUGASSERT(g_mpu_region & (1 << region));

  rbar |= table->attr.rbar &
          (MPU_RBAR_XN_MSK | MPU_RBAR_AP_MSK | MPU_RBAR_SH_MSK);
  rlar |=
     (table->attr.mair_idx <<
      MPU_RLAR_ATTRINDX_POS) & MPU_RLAR_ATTRINDX_MSK;
  rlar |= MPU_RLAR_EN_MSK;

  /* Select the region */

  write_sysreg(region, prselr_el1);
  UP_DSB();

  /* Set the region base, limit and attribute */

  write_sysreg(rbar, prbar_el1);
  write_sysreg(rlar, prlar_el1);
  UP_MB();
}

/****************************************************************************
 * Name: mpu_configure_region
 *
 * Description:
 *   Configure a region for privileged, strongly ordered memory
 *
 * Input Parameters:
 *   table - Pointer to a struct containing the configuration
 *           parameters for the region.
 *
 * Returned Value:
 *   The region number allocated for the configured region.
 *
 ****************************************************************************/

unsigned int mpu_configure_region(const struct arm64_mpu_region *
                                  table)
{
  unsigned int region = mpu_allocregion();
  mpu_modify_region(region, table);
  return region;
}

/****************************************************************************
 * Name: mpu_dump_region
 *
 * Description:
 *   Dump the region that has been used.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 ****************************************************************************/

void mpu_dump_region(void)
{
  uint64_t sctlr_el1;
  uint64_t prlar;
  uint64_t prbar;
  unsigned int num_regions;
  int i;
  int count = 0;

  num_regions = get_num_regions();
  sctlr_el1 = read_sysreg(sctlr_el1);
  _info("MPU-SCTLR_EL1 Enable:%" PRIu64 ", Cacheable: %" PRIu64 "\n",
        sctlr_el1 & SCTLR_M_BIT, sctlr_el1 & SCTLR_C_BIT);
  for (i = 0; i < num_regions; i++)
    {
      write_sysreg(i, prselr_el1);
      prlar = read_sysreg(prlar_el1);
      prbar = read_sysreg(prbar_el1);
      _info("MPU-%d, 0x%08llX-0x%08llX SH=%llX AP=%llX XN=%llX\n", i,
            prbar & MPU_RBAR_BASE_MSK, prlar & MPU_RLAR_LIMIT_MSK,
            prbar & MPU_RBAR_SH_MSK, prbar & MPU_RBAR_AP_MSK,
            prbar & MPU_RBAR_XN_MSK);
      if (prlar & MPU_RLAR_EN_MSK)
        {
          count++;
        }
    }

  _info("Total Use Region:%d, Remaining Available:%d\n", count,
        num_regions - count);
}

/****************************************************************************
 * Name: arm64_mpu_init
 *
 * Description:
 *   This function here provides the default configuration mechanism
 *   for the Memory Protection Unit (MPU).
 *
 * Input Parameters:
 *   is_primary_core: A boolean indicating whether the current core is the
 *   primary core.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arm64_mpu_init(bool is_primary_core)
{
  uint64_t  val;
  uint32_t  r_index;

#ifdef CONFIG_MM_KASAN_SW_TAGS
  val  = read_sysreg(tcr_el1);
  val |= (TCR_TBI0 | TCR_TBI1 | TCR_ASID_8);
  write_sysreg(val, tcr_el1);
#endif

  /* Current MPU code supports only EL1 */

  __asm__ volatile ("mrs %0, CurrentEL" : "=r" (val));

  __MPU_ASSERT(GET_EL(
                 val) == MODE_EL1,
               "Exception level not EL1, MPU not enabled!\n");

  /* Check whether the processor supports MPU */

  val = read_sysreg(id_aa64mmfr0_el1) & ID_AA64MMFR0_MSA_MSK;
  if ((val != ID_AA64MMFR0_PMSA_EN) && (val != ID_AA64MMFR0_PMSA_VMSA_EN))
    {
      __MPU_ASSERT(0, "MPU not supported!\n");
      return;
    }

  arm64_mpu_disable();

  /* Architecture-specific configuration */

  mpu_init();

  /* Program fixed regions configured at SOC definition. */

  for (r_index = 0U; r_index < g_mpu_config.num_regions; r_index++)
    {
      mpu_configure_region(&g_mpu_config.mpu_regions[r_index]);
    }

  arm64_mpu_enable();
}
