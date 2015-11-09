/****************************************************************************
 * arch/arm/src/armv7-m/cache.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Some logic in this header file derives from the ARM CMSIS core_cm7.h
 * header file which has a compatible 3-clause BSD license:
 *
 *   Copyright (c) 2009 - 2014 ARM LIMITED.  All rights reserved.
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
 * 3. Neither the name ARM, NuttX nor the names of its contributors may be
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

#ifndef __ARCH_ARM_SRC_ARMV7_M_CACHE_H
#define __ARCH_ARM_SRC_ARMV7_M_CACHE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "up_arch.h"
#include "nvic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Cache Size ID (CCSIDR) register macros used by inline functions
 * Given the value of the CCSIDR reginer (n):
 *
 *   CCSIDR_WAYS    - Returns the (number of ways) - 1
 *   CCSIDR_SETS    - Returns the (number of sets) - 1
 *   CCSIDR_LSSHIFT - Returns log2(cache line size in words) - 2
 *                    Eg. 0 -> 4 words
 *                        1 -> 8 words
 *                        ...
 */

#define CCSIDR_WAYS(n) \
  (((n) & NVIC_CCSIDR_ASSOCIATIVITY_MASK) >> NVIC_CCSIDR_ASSOCIATIVITY_SHIFT)
#define CCSIDR_SETS(n) \
  (((n) & NVIC_CCSIDR_NUMSETS_MASK) >> NVIC_CCSIDR_NUMSETS_SHIFT)
#define CCSIDR_LSSHIFT(n) \
  (((n) & NVIC_CCSIDR_LINESIZE_MASK) >> NVIC_CCSIDR_LINESIZE_SHIFT)

/* intrinsics are used in these inline functions */

#define arm_isb(n) __asm__ __volatile__ ("isb " #n : : : "memory")
#define arm_dsb(n) __asm__ __volatile__ ("dsb " #n : : : "memory")
#define arm_dmb(n) __asm__ __volatile__ ("dmb " #n : : : "memory")

#define ARM_DSB()  arm_dsb(15)
#define ARM_ISB()  arm_isb(15)
#define ARM_DMB()  arm_dmb(15)

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: arm_clz
 *
 * Description:
 *   Access to CLZ instructions
 *
 * Input Parameters:
 *   value - The value to perform the CLZ operation on
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline uint32_t arm_clz(unsigned int value)
{
  uint32_t ret;

  __asm__ __volatile__ ("clz %0, %1" : "=r"(ret) : "r"(value));
  return ret;
}

/****************************************************************************
 * Name: arch_enable_icache
 *
 * Description:
 *   Enable the I-Cache
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void arch_enable_icache(void)
{
#ifdef CONFIG_ARMV7M_ICACHE
  uint32_t regval;

  ARM_DSB();
  ARM_ISB();

  /* Invalidate the entire I-Cache */

  putreg32(0, NVIC_ICIALLU);

  /* Enable the I-Cache */

  regval  = getreg32(NVIC_CFGCON);
  regval |= NVIC_CFGCON_IC;
  putreg32(regval, NVIC_CFGCON);

  ARM_DSB();
  ARM_ISB();
#endif
}

/****************************************************************************
 * Name: arch_disable_icache
 *
 * Description:
 *   Disable the I-Cache
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void arch_disable_icache(void)
{
#ifdef CONFIG_ARMV7M_ICACHE
  uint32_t regval;

  ARM_DSB();
  ARM_ISB();

  /* Disable the I-Cache */

  regval  = getreg32(NVIC_CFGCON);
  regval &= ~NVIC_CFGCON_IC;
  putreg32(regval, NVIC_CFGCON);

  /* Invalidate the entire I-Cache */

  putreg32(0, NVIC_ICIALLU);

  ARM_DSB();
  ARM_ISB();
#endif
}

/****************************************************************************
 * Name: arch_invalidate_icache_all
 *
 * Description:
 *   Invalidate the entire contents of I cache.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void arch_invalidate_icache_all(void)
{
#ifdef CONFIG_ARMV7M_ICACHE
  ARM_DSB();
  ARM_ISB();

  /* Invalidate the entire I-Cache */

  putreg32(0, NVIC_ICIALLU);

  ARM_DSB();
  ARM_ISB();
#endif
}

/****************************************************************************
 * Name: arch_dcache_writethrough
 *
 * Description:
 *   Configure the D-Cache for Write-Through operation.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_ARMV7M_DCACHE) && defined(CONFIG_ARMV7M_DCACHE_WRITETHROUGH)
static inline void arch_dcache_writethrough(void)
{
  uint32_t regval = getreg32(NVIC_CACR);
  regval |= NVIC_CACR_FORCEWT;
  putreg32(regval, NVIC_CACR);
}
#else
#  define arch_dcache_writethrough()
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

 /****************************************************************************
 * Name: arch_enable_dcache
 *
 * Description:
 *   Enable the D-Cache
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ARMV7M_DCACHE
void arch_enable_dcache(void);
#else
#  define arch_enable_dcache()
#endif

/****************************************************************************
 * Name: arch_disable_dcache
 *
 * Description:
 *   Disable the D-Cache
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ARMV7M_DCACHE
void arch_disable_dcache(void);
#else
#  define arch_disable_dcache()
#endif

/****************************************************************************
 * Name: arch_invalidate_dcache
 *
 * Description:
 *   Invalidate the data cache within the specified region; we will be
 *   performing a DMA operation in this region and we want to purge old data
 *   in the cache.
 *
 * Input Parameters:
 *   start - virtual start address of region
 *   end   - virtual end address of region + 1
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   This operation is not atomic.  This function assumes that the caller
 *   has exclusive access to the address range so that no harm is done if
 *   the operation is pre-empted.
 *
 ****************************************************************************/

#ifdef CONFIG_ARMV7M_DCACHE
void arch_invalidate_dcache(uintptr_t start, uintptr_t end);
#else
#  define arch_invalidate_dcache(s,e)
#endif

/****************************************************************************
 * Name: arch_invalidate_dcache_all
 *
 * Description:
 *   Invalidate the entire contents of D cache.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ARMV7M_DCACHE
void arch_invalidate_dcache_all(void);
#else
#  define arch_invalidate_dcache_all()
#endif

/****************************************************************************
 * Name: arch_clean_dcache
 *
 * Description:
 *   Clean the data cache within the specified region by flushing the
 *   contents of the data cache to memory.
 *
 *   NOTE: This operation is un-necessary if the DCACHE is configured in
 *   write-through mode.
 *
 * Input Parameters:
 *   start - virtual start address of region
 *   end   - virtual end address of region + 1
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   This operation is not atomic.  This function assumes that the caller
 *   has exclusive access to the address range so that no harm is done if
 *   the operation is pre-empted.
 *
 ****************************************************************************/

#if defined(CONFIG_ARMV7M_DCACHE) && !defined(CONFIG_ARMV7M_DCACHE_WRITETHROUGH)
void arch_clean_dcache(uintptr_t start, uintptr_t end);
#else
#  define arch_clean_dcache(s,e)
#endif

/****************************************************************************
 * Name: arch_clean_dcache_all
 *
 * Description:
 *   Clean the entire data cache within the specified region by flushing the
 *   contents of the data cache to memory.
 *
 *   NOTE: This operation is un-necessary if the DCACHE is configured in
 *   write-through mode.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   This operation is not atomic.  This function assumes that the caller
 *   has exclusive access to the address range so that no harm is done if
 *   the operation is pre-empted.
 *
 ****************************************************************************/

#if defined(CONFIG_ARMV7M_DCACHE) && !defined(CONFIG_ARMV7M_DCACHE_WRITETHROUGH)
void arch_clean_dcache_all(void);
#else
#  define arch_clean_dcache_all()
#endif

/****************************************************************************
 * Name: arch_flush_dcache
 *
 * Description:
 *   Flush the data cache within the specified region by cleaning and
 *   invalidating the D cache.
 *
 *   NOTE: If DCACHE write-through is configured, then this operation is the
 *   same as arch_invalidate_cache().
 *
 * Input Parameters:
 *   start - virtual start address of region
 *   end   - virtual end address of region + 1
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   This operation is not atomic.  This function assumes that the caller
 *   has exclusive access to the address range so that no harm is done if
 *   the operation is pre-empted.
 *
 ****************************************************************************/

#ifdef CONFIG_ARMV7M_DCACHE
#ifdef CONFIG_ARMV7M_DCACHE_WRITETHROUGH
#  define arch_flush_dcache(s,e) arch_invalidate_dcache(s,e)
#else
void arch_flush_dcache(uintptr_t start, uintptr_t end);
#endif
#else
#  define arch_flush_dcache(s,e)
#endif

/****************************************************************************
 * Name: arch_flush_dcache_all
 *
 * Description:
 *   Flush the entire data cache by cleaning and invalidating the D cache.
 *
 *   NOTE: If DCACHE write-through is configured, then this operation is the
 *   same as arch_invalidate_cache_all().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   This operation is not atomic.  This function assumes that the caller
 *   has exclusive access to the address range so that no harm is done if
 *   the operation is pre-empted.
 *
 ****************************************************************************/

#ifdef CONFIG_ARMV7M_DCACHE
#ifdef CONFIG_ARMV7M_DCACHE_WRITETHROUGH
#  define arch_flush_dcache_all() arch_invalidate_dcache_all()
#else
void arch_flush_dcache_all(void);
#endif
#else
#  define arch_flush_dcache_all()
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif  /* __ARCH_ARM_SRC_ARMV7_M_CACHE_H */
