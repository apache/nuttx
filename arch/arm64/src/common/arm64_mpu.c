/***************************************************************************
 * arch/arm64/src/common/arm64_mpu.c
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
 ***************************************************************************/

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <debug.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <arch/irq.h>
#include <arch/chip/chip.h>

#include "arm64_arch.h"
#include "arm64_internal.h"
#include "arm64_fatal.h"
#include "arm64_mpu.h"

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

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

static uint8_t static_regions_num;

/***************************************************************************
 * Private Functions
 ***************************************************************************/

/* Get the number of supported MPU regions. */

static inline uint8_t get_num_regions(void)
{
  uint64_t type;

  type  = read_sysreg(mpuir_el1);
  type  = type & MPU_IR_REGION_MSK;

  return (uint8_t)type;
}

/* ARM Core MPU Driver API Implementation for ARM MPU */

/**
 * @brief enable the MPU
 */

void arm64_core_mpu_enable(void)
{
  uint64_t val;

  val   = read_sysreg(sctlr_el1);
  val   |= SCTLR_M_BIT;
  write_sysreg(val, sctlr_el1);
  ARM64_DSB();
  ARM64_ISB();
}

/**
 * @brief disable the MPU
 */

void arm64_core_mpu_disable(void)
{
  uint64_t val;

  /* Force any outstanding transfers to complete before disabling MPU */

  ARM64_DMB();

  val   = read_sysreg(sctlr_el1);
  val   &= ~SCTLR_M_BIT;
  write_sysreg(val, sctlr_el1);
  ARM64_DSB();
  ARM64_ISB();
}

/* ARM MPU Driver Initial Setup
 *
 * Configure the cache-ability attributes for all the
 * different types of memory regions.
 */

static void mpu_init(void)
{
  /* Device region(s): Attribute-0
   * Flash region(s): Attribute-1
   * SRAM region(s): Attribute-2
   * SRAM no cache-able regions(s): Attribute-3
   */

  uint64_t mair = MPU_MAIR_ATTRS;

  write_sysreg(mair, mair_el1);
  ARM64_DSB();
  ARM64_ISB();
}

static inline void mpu_set_region(uint32_t rnr, uint64_t rbar,
                                  uint64_t rlar)
{
  write_sysreg(rnr, prselr_el1);
  ARM64_DSB();
  write_sysreg(rbar, prbar_el1);
  write_sysreg(rlar, prlar_el1);
  ARM64_DSB();
  ARM64_ISB();
}

/* This internal functions performs MPU region initialization. */

static void region_init(const uint32_t index,
                        const struct arm64_mpu_region *region_conf)
{
  uint64_t  rbar    = region_conf->base & MPU_RBAR_BASE_MSK;
  uint64_t  rlar    = (region_conf->limit - 1) & MPU_RLAR_LIMIT_MSK;

  rbar |= region_conf->attr.rbar &
          (MPU_RBAR_XN_MSK | MPU_RBAR_AP_MSK | MPU_RBAR_SH_MSK);
  rlar |=
    (region_conf->attr.mair_idx <<
      MPU_RLAR_ATTRINDX_POS) & MPU_RLAR_ATTRINDX_MSK;
  rlar |= MPU_RLAR_EN_MSK;

  mpu_set_region(index, rbar, rlar);
}

/***************************************************************************
 * Public Functions
 ***************************************************************************/

/* @brief MPU default configuration
 *
 * This function here provides the default configuration mechanism
 * for the Memory Protection Unit (MPU).
 */

void arm64_mpu_init(bool is_primary_core)
{
  uint64_t  val;
  uint32_t  r_index;

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

  if (g_mpu_config.num_regions > get_num_regions())
    {
      /* Attempt to configure more MPU regions than
       * what is supported by hardware. As this operation
       * is executed during system (pre-kernel) initialization,
       * we want to ensure we can detect an attempt to
       * perform invalid configuration.
       */

      __MPU_ASSERT(0, "Request to configure: %u regions (supported: %u)\n",
                   g_mpu_config.num_regions, get_num_regions());
      return;
    }

  arm64_core_mpu_disable();

  /* Architecture-specific configuration */

  mpu_init();

  /* Program fixed regions configured at SOC definition. */

  for (r_index = 0U; r_index < g_mpu_config.num_regions; r_index++)
    {
      region_init(r_index, &g_mpu_config.mpu_regions[r_index]);
    }

  /* Update the number of programmed MPU regions. */

  static_regions_num = g_mpu_config.num_regions;

  arm64_core_mpu_enable();
}
