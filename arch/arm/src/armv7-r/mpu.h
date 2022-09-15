/****************************************************************************
 * arch/arm/src/armv7-r/mpu.h
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
#  include "cp15.h"
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
 ****************************************************************************/

#if defined(CONFIG_MPU_RESET)
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
 *  Allocate the next region
 *
 ****************************************************************************/

unsigned int mpu_allocregion(void);

/****************************************************************************
 * Name: mpu_log2regionceil
 *
 * Description:
 *   Determine the smallest value of l2size (log base 2 size) such that the
 *   following is true:
 *
 *   size <= (1 << l2size)
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
 ****************************************************************************/

uint8_t mpu_log2regionfloor(size_t size);

/****************************************************************************
 * Name: mpu_subregion
 *
 * Description:
 *   Given (1) the offset to the beginning of valid data, (2) the size of the
 *   memory to be mapped and (2) the log2 size of the mapping to use,
 *   determine the minimal sub-region set to span that memory region.
 *
 * Assumption:
 *   l2size has the same properties as the return value from
 *   mpu_log2regionceil()
 *
 ****************************************************************************/

uint32_t mpu_subregion(uintptr_t base, size_t size, uint8_t l2size);

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
 *   Enable (or disable) the MPU
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

static inline void mpu_priv_stronglyordered(uintptr_t base, size_t size)
{
  unsigned int region = mpu_allocregion();
  uint32_t     regval;
  uint8_t      l2size;
  uint8_t      subregions;

  /* Select the region */

  mpu_set_rgnr(region);

  /* Select the region base address */

  mpu_set_drbar(base & MPU_RBAR_ADDR_MASK);

  /* Select the region size and the sub-region map */

  l2size     = mpu_log2regionceil(size);
  subregions = mpu_subregion(base, size, l2size);

  /* The configure the region */

  regval =                                                /* Not Cacheable  */
                                                          /* Not Bufferable */
           MPU_RACR_S                                   | /* Shareable      */
           MPU_RACR_AP_RWNO;                              /* P:RW   U:None  */
  mpu_set_dracr(regval);

  regval = MPU_RASR_ENABLE                              | /* Enable region  */
           MPU_RASR_RSIZE_LOG2((uint32_t)l2size)        | /* Region size    */
           ((uint32_t)subregions << MPU_RASR_SRD_SHIFT);  /* Sub-regions    */
  mpu_set_drsr(regval);
}

/****************************************************************************
 * Name: mpu_user_flash
 *
 * Description:
 *   Configure a region for user program flash
 *
 ****************************************************************************/

static inline void mpu_user_flash(uintptr_t base, size_t size)
{
  unsigned int region = mpu_allocregion();
  uint32_t     regval;
  uint8_t      l2size;
  uint8_t      subregions;

  /* Select the region */

  mpu_set_rgnr(region);

  /* Select the region base address */

  mpu_set_drbar(base & MPU_RBAR_ADDR_MASK);

  /* Select the region size and the sub-region map */

  l2size     = mpu_log2regionceil(size);
  subregions = mpu_subregion(base, size, l2size);

  /* The configure the region */

  regval =                                                /* Not Cacheable  */
           MPU_RACR_C                                   | /* Cacheable     */
           MPU_RACR_AP_RORO;                              /* P:RO   U:RO   */
  mpu_set_dracr(regval);

  regval = MPU_RASR_ENABLE                              | /* Enable region */
           MPU_RASR_RSIZE_LOG2((uint32_t)l2size)        | /* Region size   */
           ((uint32_t)subregions << MPU_RASR_SRD_SHIFT);  /* Sub-regions   */
  mpu_set_drsr(regval);
}

/****************************************************************************
 * Name: mpu_priv_noncache
 *
 * Description:
 *   Configure a non-cacheable region as privileged internal memory
 *
 ****************************************************************************/

static inline void mpu_priv_noncache(uintptr_t base, size_t size)
{
  unsigned int region = mpu_allocregion();
  uint32_t regval;
  uint8_t l2size;
  uint8_t subregions;

  /* Select the region */

  mpu_set_rgnr(region);

  /* Select the region base address */

  mpu_set_drbar(base & MPU_RBAR_ADDR_MASK);

  /* Select the region size and the sub-region map */

  l2size = mpu_log2regionceil(size);
  subregions = mpu_subregion(base, size, l2size);

  /* The configure the region
   * inner/outer non-cache : TEX(4), C(0), B(0), S(1)
   */

  regval =                                         /* Not Cacheable  */
                                                   /* Not Bufferable */
           MPU_RACR_S                            | /* Shareable      */
           MPU_RACR_TEX(4)                       | /* TEX */
           MPU_RACR_AP_RWRW                      | /* P:RO   U:None  */
           MPU_RACR_XN;                            /* Instruction access disable */
  mpu_set_dracr(regval);

  regval = MPU_RASR_ENABLE                       |     /* Enable region */
           MPU_RASR_RSIZE_LOG2((uint32_t)l2size) |     /* Region size   */
         ((uint32_t)subregions << MPU_RASR_SRD_SHIFT); /* Sub-regions */
    mpu_set_drsr(regval);
}

/****************************************************************************
 * Name: mpu_priv_flash
 *
 * Description:
 *   Configure a region for privileged program flash
 *
 ****************************************************************************/

static inline void mpu_priv_flash(uintptr_t base, size_t size)
{
  unsigned int region = mpu_allocregion();
  uint32_t     regval;
  uint8_t      l2size;
  uint8_t      subregions;

  /* Select the region */

  mpu_set_rgnr(region);

  /* Select the region base address */

  mpu_set_drbar(base & MPU_RBAR_ADDR_MASK);

  /* Select the region size and the sub-region map */

  l2size     = mpu_log2regionceil(size);
  subregions = mpu_subregion(base, size, l2size);

  /* The configure the region */

  regval = MPU_RACR_C                                   | /* Cacheable     */
           MPU_RACR_AP_RONO;                              /* P:RO   U:None */
  mpu_set_dracr(regval);

  regval = MPU_RASR_ENABLE                              | /* Enable region */
           MPU_RASR_RSIZE_LOG2((uint32_t)l2size)        | /* Region size   */
           ((uint32_t)subregions << MPU_RASR_SRD_SHIFT);  /* Sub-regions   */
  mpu_set_drsr(regval);
}

/****************************************************************************
 * Name: mpu_user_intsram
 *
 * Description:
 *   Configure a region as user internal SRAM
 *
 ****************************************************************************/

static inline void mpu_user_intsram(uintptr_t base, size_t size)
{
  unsigned int region = mpu_allocregion();
  uint32_t     regval;
  uint8_t      l2size;
  uint8_t      subregions;

  /* Select the region */

  mpu_set_rgnr(region);

  /* Select the region base address */

  mpu_set_drbar(base & MPU_RBAR_ADDR_MASK);

  /* Select the region size and the sub-region map */

  l2size     = mpu_log2regionceil(size);
  subregions = mpu_subregion(base, size, l2size);

  /* The configure the region */

  regval = MPU_RACR_S                                   | /* Shareable     */
           MPU_RACR_C                                   | /* Cacheable     */
           MPU_RACR_AP_RWRW;                              /* P:RW   U:RW   */
  mpu_set_dracr(regval);

  regval = MPU_RASR_ENABLE                              | /* Enable region */
           MPU_RASR_RSIZE_LOG2((uint32_t)l2size)        | /* Region size   */
           ((uint32_t)subregions << MPU_RASR_SRD_SHIFT);  /* Sub-regions   */
  mpu_set_drsr(regval);
}

/****************************************************************************
 * Name: mpu_priv_intsram
 *
 * Description:
 *   Configure a region as privileged internal SRAM
 *
 ****************************************************************************/

static inline void mpu_priv_intsram(uintptr_t base, size_t size)
{
  unsigned int region = mpu_allocregion();
  uint32_t     regval;
  uint8_t      l2size;
  uint8_t      subregions;

  /* Select the region */

  mpu_set_rgnr(region);

  /* Select the region base address */

  mpu_set_drbar(base & MPU_RBAR_ADDR_MASK);

  /* Select the region size and the sub-region map */

  l2size     = mpu_log2regionceil(size);
  subregions = mpu_subregion(base, size, l2size);

  /* The configure the region */

  regval = MPU_RACR_S                                   | /* Shareable     */
           MPU_RACR_C                                   | /* Cacheable     */
           MPU_RACR_AP_RWNO;                              /* P:RW   U:None */
  mpu_set_dracr(regval);

  regval = MPU_RASR_ENABLE                              | /* Enable region */
           MPU_RASR_RSIZE_LOG2((uint32_t)l2size)        | /* Region size   */
           ((uint32_t)subregions << MPU_RASR_SRD_SHIFT);  /* Sub-regions   */
  mpu_set_drsr(regval);
}

/****************************************************************************
 * Name: mpu_user_extsram
 *
 * Description:
 *   Configure a region as user external SRAM
 *
 ****************************************************************************/

static inline void mpu_user_extsram(uintptr_t base, size_t size)
{
  unsigned int region = mpu_allocregion();
  uint32_t     regval;
  uint8_t      l2size;
  uint8_t      subregions;

  /* Select the region */

  mpu_set_rgnr(region);

  /* Select the region base address */

  mpu_set_drbar(base & MPU_RBAR_ADDR_MASK);

  /* Select the region size and the sub-region map */

  l2size     = mpu_log2regionceil(size);
  subregions = mpu_subregion(base, size, l2size);

  /* The configure the region */

  regval = MPU_RACR_S                                   | /* Shareable     */
           MPU_RACR_C                                   | /* Cacheable     */
           MPU_RACR_B                                   | /* Bufferable    */
           MPU_RACR_AP_RWRW;                              /* P:RW   U:RW   */
  mpu_set_dracr(regval);

  regval = MPU_RASR_ENABLE                              | /* Enable region */
           MPU_RASR_RSIZE_LOG2((uint32_t)l2size)        | /* Region size   */
           ((uint32_t)subregions << MPU_RASR_SRD_SHIFT);  /* Sub-regions   */
  mpu_set_drsr(regval);
}

/****************************************************************************
 * Name: mpu_priv_extsram
 *
 * Description:
 *   Configure a region as privileged external SRAM
 *
 ****************************************************************************/

static inline void mpu_priv_extsram(uintptr_t base, size_t size)
{
  unsigned int region = mpu_allocregion();
  uint32_t     regval;
  uint8_t      l2size;
  uint8_t      subregions;

  /* Select the region */

  mpu_set_rgnr(region);

  /* Select the region base address */

  mpu_set_drbar(base & MPU_RBAR_ADDR_MASK);

  /* Select the region size and the sub-region map */

  l2size     = mpu_log2regionceil(size);
  subregions = mpu_subregion(base, size, l2size);

  /* The configure the region */

  regval = MPU_RACR_S                                   | /* Shareable     */
           MPU_RACR_C                                   | /* Cacheable     */
           MPU_RACR_B                                   | /* Bufferable    */
           MPU_RACR_AP_RWNO;                              /* P:RW   U:None */
  mpu_set_dracr(regval);

  regval = MPU_RASR_ENABLE                              | /* Enable region */
           MPU_RASR_RSIZE_LOG2((uint32_t)l2size)        | /* Region size   */
           ((uint32_t)subregions << MPU_RASR_SRD_SHIFT);  /* Sub-regions   */
  mpu_set_drsr(regval);
}

/****************************************************************************
 * Name: mpu_peripheral
 *
 * Description:
 *   Configure a region as privileged periperal address space
 *
 ****************************************************************************/

static inline void mpu_peripheral(uintptr_t base, size_t size)
{
  unsigned int region = mpu_allocregion();
  uint32_t     regval;
  uint8_t      l2size;
  uint8_t      subregions;

  /* Select the region */

  mpu_set_rgnr(region);

  /* Select the region base address */

  mpu_set_drbar(base & MPU_RBAR_ADDR_MASK);

  /* Select the region size and the sub-region map */

  l2size     = mpu_log2regionceil(size);
  subregions = mpu_subregion(base, size, l2size);

  /* Then configure the region */

  regval = MPU_RACR_S                                   | /* Shareable     */
           MPU_RACR_B                                   | /* Bufferable    */
           MPU_RACR_AP_RWNO                             | /* P:RW   U:None */
           MPU_RACR_XN;                                   /* Instruction access disable */
  mpu_set_dracr(regval);

  regval = MPU_RASR_ENABLE                              | /* Enable region */
           MPU_RASR_RSIZE_LOG2((uint32_t)l2size)        | /* Region size   */
           ((uint32_t)subregions << MPU_RASR_SRD_SHIFT);  /* Sub-regions   */
  mpu_set_drsr(regval);
}

/****************************************************************************
 * Name: mpu_user_intsram_wb
 *
 * Description:
 *   Configure a region as user internal SRAM
 *   Unlike a mpu_user_intsram, this regions includes WB/WA cache policy
 *
 ****************************************************************************/

static inline void mpu_user_intsram_wb(uintptr_t base, size_t size)
{
  unsigned int region = mpu_allocregion();
  uint32_t regval;
  uint8_t l2size;
  uint8_t subregions;

  /* Select the region */

  mpu_set_rgnr(region);

  /* Select the region base address */

  mpu_set_drbar(base & MPU_RBAR_ADDR_MASK);

  /* Select the region size and the sub-region map */

  l2size = mpu_log2regionceil(size);
  subregions = mpu_subregion(base, size, l2size);

  /* The configure the region
   * WB/Write Allocate: TEX(5), C(0), B(1), S(1)
   */

  regval =                                               /* Not Cacheable  */
           MPU_RACR_B                                  | /* Not Bufferable */
           MPU_RACR_TEX(5)                             | /* TEX            */
           MPU_RACR_AP_RWRW;                             /* P:RW   U:RW    */
  mpu_set_dracr(regval);

  regval = MPU_RASR_ENABLE                             | /* Enable region */
           MPU_RASR_RSIZE_LOG2((uint32_t)l2size)       | /* Region size   */
           ((uint32_t)subregions << MPU_RASR_SRD_SHIFT); /* Sub-regions */
  mpu_set_drsr(regval);
}

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_ARMV7_R_MPU_H */
