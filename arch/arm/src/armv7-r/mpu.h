/****************************************************************************
 * arch/arm/src/armv7-r/mpu.h
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

#ifndef __ARCH_ARM_SRC_ARMV7_R_MPU_H
#define __ARCH_ARM_SRC_ARMV7_R_MPU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <sys/types.h>
#  include <stdint.h>
#  include <stdbool.h>
#  include <debug.h>

#  include "sctlr.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MPU Type Register Bit Definitions */

#define MPUIR_SEPARATE           (1 << 0) /* Bit 0: 0:unified or 1:separate memory maps */
#define MPUIR_DREGION_SHIFT      (8)      /* Bits 8-15: Number MPU data regions */
#define MPUIR_DREGION_MASK       (0xff << MPUIR_DREGION_SHIFT)
#define MPUIR_IREGION_SHIFT      (16)     /* Bits 16-23: Number MPU instruction regions */
#define MPUIR_IREGION_MASK       (0xff << MPUIR_IREGION_SHIFT)

/* Region Base Address Register Definitions
 * [31:5] Physical base address. Defines the base address of a region.
 */
#define MPU_RBAR_ADDR_MASK       0xffffffe0

/* Region Size and Enable Register */

#define MPU_RASR_ENABLE          (1 << 0)  /* Bit 0: Region enable */
#define MPU_RASR_RSIZE_SHIFT     (1)       /* Bits 1-5: Size of the MPU protection region */
#define MPU_RASR_RSIZE_MASK      (31 << MPU_RASR_RSIZE_SHIFT)
#  define MPU_RASR_RSIZE_LOG2(n) ((n-1) << MPU_RASR_RSIZE_SHIFT)

#define MPU_RASR_SRD_SHIFT       (8)       /* Bits 8-15: Subregion disable */
#define MPU_RASR_SRD_MASK        (0xff << MPU_RASR_SRD_SHIFT)
#  define MPU_RASR_SRD_0         (0x01 << MPU_RASR_SRD_SHIFT)
#  define MPU_RASR_SRD_1         (0x02 << MPU_RASR_SRD_SHIFT)
#  define MPU_RASR_SRD_2         (0x04 << MPU_RASR_SRD_SHIFT)
#  define MPU_RASR_SRD_3         (0x08 << MPU_RASR_SRD_SHIFT)
#  define MPU_RASR_SRD_4         (0x10 << MPU_RASR_SRD_SHIFT)
#  define MPU_RASR_SRD_5         (0x20 << MPU_RASR_SRD_SHIFT)
#  define MPU_RASR_SRD_6         (0x40 << MPU_RASR_SRD_SHIFT)
#  define MPU_RASR_SRD_7         (0x80 << MPU_RASR_SRD_SHIFT)

/* Region Access Control Register */

#define MPU_RACR_B               (1 << 0)  /* Bit 0: Bufferable */
#define MPU_RACR_C               (1 << 1)  /* Bit 1: Cacheable */
#define MPU_RACR_S               (1 << 2)  /* Bit 2: Shareable */
#define MPU_RACR_TEX_SHIFT       (3)       /* Bits 3-5: Type extension */
#define MPU_RACR_TEX_MASK        (7 << MPU_RACR_TEX_SHIFT)
#  define MPU_RACR_TEX(n)        ((uint32_t)(n) << MPU_RACR_TEX_SHIFT)
#define MPU_RACR_AP_SHIFT        (8)       /* Bits 8-10: Access permission */
#define MPU_RACR_AP_MASK         (7 << MPU_RACR_AP_SHIFT)
#  define MPU_RACR_AP_NONO       (0 << MPU_RACR_AP_SHIFT) /* PL0:None PL1:None */
#  define MPU_RACR_AP_RWNO       (1 << MPU_RACR_AP_SHIFT) /* PL0:RW   PL1:None */
#  define MPU_RACR_AP_RWRO       (2 << MPU_RACR_AP_SHIFT) /* PL0:RW   PL1:RO   */
#  define MPU_RACR_AP_RWRW       (3 << MPU_RACR_AP_SHIFT) /* PL0:RW   PL1:RW   */
#  define MPU_RACR_AP_RONO       (5 << MPU_RACR_AP_SHIFT) /* PL0:RO   PL1:None */
#  define MPU_RACR_AP_RORO       (6 << MPU_RACR_AP_SHIFT) /* PL0:RO   PL1:RO   */

#define MPU_RACR_XN              (1 << 12) /* Bit 12: Instruction access disable */

/* MPU Region Number Register */

#if defined(CONFIG_ARM_MPU_NREGIONS)
#  if CONFIG_ARM_MPU_NREGIONS <= 8
#    define MPU_RGNR_MASK        (0x00000007)
#  elif CONFIG_ARM_MPU_NREGIONS <= 16
#    define MPU_RGNR_MASK        (0x0000000f)
#  elif CONFIG_ARM_MPU_NREGIONS <= 32
#    define MPU_RGNR_MASK        (0x0000001f)
#  else
#  error "FIXME: Unsupported number of MPU regions"
#  endif
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: mpu_reset
 *
 * Description:
 *   Conditional public interface that resets the MPU to disabled during
 *   MPU initialization.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#if defined(CONFIG_ARM_MPU_RESET)
void mpu_reset(void);
#else
#  define mpu_reset() do { } while (0)
#endif

/****************************************************************************
 * Name: mpu_early_reset
 *
 * Description:
 *   Conditional public interface that resets the MPU to disabled immediately
 *   after reset.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#if defined(CONFIG_ARM_MPU_EARLY_RESET)
void mpu_early_reset(void);
#else
#  define mpu_early_reset() do { } while (0)
#endif

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

unsigned int mpu_allocregion(void);

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

void mpu_freeregion(unsigned int region);

/****************************************************************************
 * Name: mpu_log2regionceil
 *
 * Description:
 *   Determine the smallest value of l2size (log base 2 size) such that the
 *   following is true:
 *
 *   size <= (1 << l2size)
 *
 * Input Parameters:
 *   size - The size of the region.
 *
 * Returned Value:
 *   The logarithm base 2 of the ceiling value for the MPU region size.
 *
 ****************************************************************************/

uint8_t mpu_log2regionceil(size_t size);

/****************************************************************************
 * Name: mpu_log2regionfloor
 *
 * Description:
 *   Determine the largest value of l2size (log base 2 size) such that the
 *   following is true:
 *
 *   size >= (1 << l2size)
 *
 * Input Parameters:
 *   size - The size of the region.
 *
 * Returned Value:
 *   The logarithm base 2 of the floor value for the MPU region size.
 *
 ****************************************************************************/

uint8_t mpu_log2regionfloor(size_t size);

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
 *
 ****************************************************************************/

void mpu_dump_region(void);

/****************************************************************************
 * Name: mpu_subregion
 *
 * Description:
 *   Given the size of the (1) memory to be mapped and (2) the log2 size
 *   of the mapping to use, determine the minimal sub-region set to span
 *   that memory region.
 *
 * Assumption:
 *   l2size has the same properties as the return value from
 *   mpu_log2regionceil()
 *
 * Input Parameters:
 *   base   - The base address of the region.
 *   size   - The size of the region.
 *   l2size - Log2 of the actual region size is <= (1 << l2size).
 *
 * Returned Value:
 *   The subregion settings as a 32-bit value.
 *
 ****************************************************************************/

uint32_t mpu_subregion(uintptr_t base, size_t size, uint8_t l2size);

/****************************************************************************
 * Name: mpu_modify_region
 *
 * Description:
 *   Modify a region for privileged, strongly ordered memory
 *
 * Input Parameters:
 *   region - Region number to modify.
 *   base   - Base address of the region.
 *   size   - Size of the region.
 *   flags  - Flags to configure the region.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void mpu_modify_region(unsigned int region, uintptr_t base, size_t size,
                       uint32_t flags);

/****************************************************************************
 * Name: mpu_configure_region
 *
 * Description:
 *   Configure a region for privileged, strongly ordered memory
 *
 * Input Parameters:
 *   base - Base address of the region.
 *   size - Size of the region.
 *   flags - Flags to configure the region.
 *
 * Returned Value:
 *   The region number allocated for the configured region.
 *
 ****************************************************************************/

unsigned int mpu_configure_region(uintptr_t base, size_t size,
                                  uint32_t flags);

/****************************************************************************
 * Name: mpu_initialize
 *
 * Description:
 *   Configure a region for privileged, strongly ordered memory
 *
 * Input Parameters:
 *   table - MPU Initiaze table.
 *   count - Initialize the number of entries in the region table.
 *
 * Returned Value:
 *   NULL.
 *
 ****************************************************************************/

void mpu_initialize(const struct mpu_region_s *table, size_t count);

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpu_get_mpuir
 *
 * Description:
 *   Read the MPUIR register the characteristics of the MPU
 *
 ****************************************************************************/

static inline unsigned int mpu_get_mpuir(void)
{
  return CP15_GET(MPUIR);
}

/****************************************************************************
 * Name: mpu_get_drbar
 *
 * Description:
 *   Get the DRBAR register
 *
 ****************************************************************************/

static inline unsigned int mpu_get_drbar(void)
{
  return CP15_GET(DRBAR);
}

/****************************************************************************
 * Name: mpu_get_drsr
 *
 * Description:
 *   Get the DRSR register
 *
 ****************************************************************************/

static inline unsigned int mpu_get_drsr(void)
{
  return CP15_GET(DRSR);
}

/****************************************************************************
 * Name: mpu_get_dracr
 *
 * Description:
 *   Get the DRACR register
 *
 ****************************************************************************/

static inline unsigned int mpu_get_dracr(void)
{
  return CP15_GET(DRACR);
}

/****************************************************************************
 * Name: mpu_set_drbar
 *
 * Description:
 *   Write to the DRBAR register
 *
 ****************************************************************************/

static inline void mpu_set_drbar(unsigned int drbar)
{
  CP15_SET(DRBAR, drbar);
}

/****************************************************************************
 * Name: mpu_set_drsr
 *
 * Description:
 *   Write to the DRSR register
 *
 ****************************************************************************/

static inline void mpu_set_drsr(unsigned int drsr)
{
  CP15_SET(DRSR, drsr);
}

/****************************************************************************
 * Name: mpu_set_dracr
 *
 * Description:
 *   Write to the DRACR register
 *
 ****************************************************************************/

static inline void mpu_set_dracr(unsigned int dracr)
{
  CP15_SET(DRACR, dracr);
}

/****************************************************************************
 * Name: mpu_set_irbar
 *
 * Description:
 *   Write to the IRBAR register
 *
 ****************************************************************************/

#ifndef CONFIG_ARM_HAVE_MPU_UNIFIED
static inline void mpu_set_irbar(unsigned int irbar)
{
  CP15_SET(IRBAR, irbar);
}
#endif

/****************************************************************************
 * Name: mpu_set_irsr
 *
 * Description:
 *   Write to the IRSR register
 *
 ****************************************************************************/

#ifndef CONFIG_ARM_HAVE_MPU_UNIFIED
static inline void mpu_set_irsr(unsigned int irsr)
{
  CP15_SET(IRSR, irsr);
}
#endif

/****************************************************************************
 * Name: mpu_set_iracr
 *
 * Description:
 *   Write to the IRACR register
 *
 ****************************************************************************/

#ifndef CONFIG_ARM_HAVE_MPU_UNIFIED
static inline void mpu_set_iracr(unsigned int iracr)
{
  CP15_SET(IRACR, iracr);
}
#endif

/****************************************************************************
 * Name: mpu_set_rgnr
 *
 * Description:
 *   Write to the RGNR register
 *
 ****************************************************************************/

static inline void mpu_set_rgnr(unsigned int rgnr)
{
  CP15_SET(RGNR, rgnr);
}

/****************************************************************************
 * Name: mpu_showtype
 *
 * Description:
 *   Show the characteristics of the MPU
 *
 ****************************************************************************/

static inline void mpu_showtype(void)
{
#ifdef CONFIG_DEBUG_SCHED_INFO
  uint32_t regval = mpu_get_mpuir();
  sinfo("%s MPU Regions: data=%d instr=%d\n",
        (regval & MPUIR_SEPARATE) != 0 ? "Separate" : "Unified",
        (regval & MPUIR_DREGION_MASK) >> MPUIR_DREGION_SHIFT,
        (regval & MPUIR_IREGION_MASK) >> MPUIR_IREGION_SHIFT);
#endif
}

/****************************************************************************
 * Name: mpu_control
 *
 * Description:
 *   Configure and enable (or disable) the MPU
 *
 * Input Parameters:
 *   enable     - Flag indicating whether to enable the MPU.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void mpu_control(bool enable)
{
  uint32_t regval;

  /* Set/clear the following bits in the SCTLR:
   *
   * SCTLR_M   Bit 0:  MPU enable bit
   * SCTLR_BR  Bit 17: Background Region bit (not cleared)
   */

  regval = cp15_rdsctlr();
  if (enable)
    {
      regval |= (SCTLR_M | SCTLR_BR);
    }
  else
    {
      regval &= ~SCTLR_M;
    }

  cp15_wrsctlr(regval);
}

/****************************************************************************
 * Name: mpu_priv_stronglyordered
 *
 * Description:
 *   Configure a region for privileged, strongly ordered memory
 *
 ****************************************************************************/

#define mpu_priv_stronglyordered(base, size) \
  /* Not Cacheable  */ \
  /* Not Bufferable */ \
  /* Shareable      */ \
  /* P:RW   U:None  */ \
  mpu_configure_region(base, size, MPU_RACR_S | MPU_RACR_AP_RWNO)

/****************************************************************************
 * Name: mpu_user_flash
 *
 * Description:
 *   Configure a region for user program flash
 *
 ****************************************************************************/

#define mpu_user_flash(base, size) \
  /* Cacheable   */ \
  /* P:RO   U:RO */ \
  mpu_configure_region(base, size, MPU_RACR_C | MPU_RACR_AP_RORO)

/****************************************************************************
 * Name: mpu_priv_noncache
 *
 * Description:
 *   Configure a non-cacheable region as privileged internal memory
 *
 ****************************************************************************/

#define mpu_priv_noncache(base, size) \
  /* inner/outer non-cache : TEX(4), C(0), B(0), S(1) */ \
  /* Not Cacheable                                    */ \
  /* Not Bufferable                                   */ \
  /* Shareable                                        */ \
  /* TEX                                              */ \
  /* P:RO   U:None    `                               */ \
  /* Instruction access disable                       */ \
  mpu_configure_region(base, size, MPU_RACR_S       | \
                                   MPU_RACR_TEX(4)  | \
                                   MPU_RACR_AP_RWRW | \
                                   MPU_RACR_XN)

/****************************************************************************
 * Name: mpu_priv_flash
 *
 * Description:
 *   Configure a region for privileged program flash
 *
 ****************************************************************************/

#define mpu_priv_flash(base, size) \
  /* Cacheable     */ \
  /* P:RO   U:None */ \
  mpu_configure_region(base, size, MPU_RACR_C | MPU_RACR_AP_RONO)

/****************************************************************************
 * Name: mpu_user_intsram
 *
 * Description:
 *   Configure a region as user internal SRAM
 *
 ****************************************************************************/

#define mpu_user_intsram(base, size) \
  /* Shareable   */ \
  /* Cacheable   */ \
  /* P:RW   U:RW */ \
  mpu_configure_region(base, size, MPU_RACR_S | \
                                   MPU_RACR_C | \
                                   MPU_RACR_AP_RWRW)

/****************************************************************************
 * Name: mpu_priv_intsram
 *
 * Description:
 *   Configure a region as privileged internal SRAM
 *
 ****************************************************************************/

#define mpu_priv_intsram(base, size) \
  /* Shareable     */ \
  /* Cacheable     */ \
  /* P:RW   U:None */ \
  mpu_configure_region(base, size, MPU_RACR_S | \
                                   MPU_RACR_C | \
                                   MPU_RACR_AP_RWNO)

/****************************************************************************
 * Name: mpu_user_extsram
 *
 * Description:
 *   Configure a region as user external SRAM
 *
 ****************************************************************************/

#define mpu_user_extsram(base, size) \
  /* Shareable   */ \
  /* Cacheable   */ \
  /* Bufferable  */ \
  /* P:RW   U:RW */ \
  mpu_configure_region(base, size, MPU_RACR_S | \
                                   MPU_RACR_C | \
                                   MPU_RACR_B | \
                                   MPU_RACR_AP_RWRW)

/****************************************************************************
 * Name: mpu_priv_extsram
 *
 * Description:
 *   Configure a region as privileged external SRAM
 *
 ****************************************************************************/

#define mpu_priv_extsram(base, size) \
  /* Shareable     */ \
  /* Cacheable     */ \
  /* Bufferable    */ \
  /* P:RW   U:None */ \
  mpu_configure_region(base, size, MPU_RACR_S | \
                                   MPU_RACR_C | \
                                   MPU_RACR_B | \
                                   MPU_RACR_AP_RWNO)

/****************************************************************************
 * Name: mpu_peripheral
 *
 * Description:
 *   Configure a region as privileged periperal address space
 *
 ****************************************************************************/

#define mpu_peripheral(base, size) \
  /* Shareable                  */ \
  /* Bufferable                 */ \
  /* P:RW   U:None              */ \
  /* Instruction access disable */ \
  mpu_configure_region(base, size, MPU_RACR_S       | \
                                   MPU_RACR_B       | \
                                   MPU_RACR_AP_RWNO | \
                                   MPU_RACR_XN)

/****************************************************************************
 * Name: mpu_user_intsram_wb
 *
 * Description:
 *   Configure a region as user internal SRAM
 *   Unlike a mpu_user_intsram, this regions includes WB/WA cache policy
 *
 ****************************************************************************/

#define mpu_user_intsram_wb(base, size) \
  /* Not Cacheable  */ \
  /* Not Bufferable */ \
  /* TEX            */ \
  /* P:RW   U:RW    */ \
  mpu_configure_region(base, size, MPU_RACR_B      | \
                                   MPU_RACR_TEX(5) | \
                                   MPU_RACR_AP_RWRW)

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_ARMV7_R_MPU_H */
