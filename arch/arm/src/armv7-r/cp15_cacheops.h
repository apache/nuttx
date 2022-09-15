/****************************************************************************
 * arch/arm/src/armv7-r/cp15_cacheops.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Portions of this file derive from Atmel sample code for the SAMA5D3
 * Cortex-A5 which also has a modified BSD-style license:
 *
 *   Copyright (c) 2012, Atmel Corporation
 *   All rights reserved.
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
 * 3. Neither the name NuttX nor Atmel nor the names of the contributors may
 *    be used to endorse or promote products derived from this software
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

/* References:
 *
 *  "ARM Architecture Reference Manual, ARMv7-A and ARMv7-R edition",
 *   Copyright 1996-1998, 2000, 2004-2012 ARM. All rights reserved.
 *   ARM DDI 0406C.c (ID051414)
 */

#ifndef __ARCH_ARM_SRC_ARMV7_R_CP15_CACHEOPS_H
#define __ARCH_ARM_SRC_ARMV7_R_CP15_CACHEOPS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "sctlr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Cache definitions ********************************************************/

/* L1 Memory */

#define CP15_L1_LINESIZE 32

/* CP15 Registers ***********************************************************/

/* Terms:
 * 1) Point of coherency (PoC)
 *    The PoC is the point at which all agents that can access memory are
 *    guaranteed to see the same copy of a memory location
 * 2) Point of unification (PoU)
 *    The PoU is the point by which the instruction and data caches and the
 *    translation table walks of the processor are guaranteed to see the same
 *    copy of a memory location.
 *
 * Cache Operations:
 *
 * CP15 Register:     ICIALLUIS
 *   Description:     Invalidate entire instruction cache Inner Shareable.
 *   Register Format: Should be zero (SBZ)
 *   Instruction:     MCR p15, 0, <Rd>, c7, c1, 0
 * CP15 Register:     BPIALLIS
 *   Description:     Invalidate entire branch predictor array Inner
 *                    Shareable.
 *   Register Format: Should be zero (SBZ)
 *   Instruction:     MCR p15, 0, <Rd>, c7, c1, 6
 * CP15 Register:     ICIALLU
 *   Description:     Invalidate all instruction caches to PoU. Also flushes
 *                    branch target cache.
 *   Register Format: Should be zero (SBZ)
 *   Instruction:     MCR p15, 0, <Rd>, c7, c5, 0
 * CP15 Register:     ICIMVAU
 *   Description:     Invalidate instruction cache by VA to PoU.
 *   Register Format: VA
 *   Instruction:     MCR p15, 0, <Rd>, c7, c5, 1
 * CP15 Register:     CP15ISB
 *   Description:     Instruction Synchronization Barrier operation
 *   NOTE:            Deprecated and no longer documented
 *   Instruction:     MCR p15, 0, <Rd>, c7, c5, 4
 * CP15 Register:     BPIALL
 *   Description:     Invalidate entire branch predictor array.
 *   Register Format: Should be zero (SBZ)
 *   Instruction:     MCR p15, 0, <Rd>, c7, c5, 6
 * CP15 Register:     BPIMVA
 *   Description:     Invalidate VA from branch predictor array.
 *   Register Format: Should be zero (SBZ)
 *   Instruction:     MCR p15, 0, <Rd>, c7, c5, 7
 * CP15 Register:     DCIMVAC
 *   Description:     Invalidate data cache line by VA to PoC.
 *   Register Format: VA
 *   Instruction:     MCR p15, 0, <Rd>, c7, c6, 1
 * CP15 Register:     DCISW
 *   Description:     Invalidate data cache line by Set/Way.
 *   Register Format: Set/Way
 *   Instruction:     MCR p15, 0, <Rd>, c7, c6, 2
 * CP15 Register:     DCCMVAC
 *   Description:     Clean data cache line to PoC by VA.
 *   Register Format: VA
 *   Instruction:     MCR p15, 0, <Rd>, c7, c10, 1
 * CP15 Register:     DCCSW
 *   Description:     Clean data cache line by Set/Way.
 *   Register Format: Set/Way
 *   Instruction:     MCR p15, 0, <Rd>, c7, c10, 2
 * CP15 Register:     CP15DSB
 *   Description:     Data Synchronization Barrier operation
 *   NOTE:            Deprecated and no longer documented
 *   Instruction:     MCR p15, 0, <Rd>, c7, c10, 4
 * CP15 Register:     CP15DMB
 *   Description:     Data Memory Barrier operation
 *   NOTE:            Deprecated and no longer documented
 *   Instruction:     MCR p15, 0, <Rd>, c7, c10, 5
 * CP15 Register:     DCCMVAU
 *   Description:     Clean data or unified cache line by VA to PoU.
 *   Register Format: VA
 *   Instruction:     MCR p15, 0, <Rd>, c7, c11, 1
 * CP15 Register:     DCCIMVAC
 *   Description:     Clean and invalidate data cache line by VA to PoC.
 *   Register Format: VA
 *   Instruction:     MCR p15, 0, <Rd>, c7, c14, 1
 * CP15 Register:     DCCISW
 *   Description:     Clean and invalidate data cache line by Set/Way.
 *   Register Format: Set/Way
 *   Instruction:     MCR p15, 0, <Rd>, c7, c14, 2
 */

/* Set/way format */

#define CACHE_WAY_SHIFT (3)     /* Bits 30-31: Way in set being accessed */
#define CACHE_WAY_MASK  (3 << CACHE_WAY_SHIFT)
#define CACHE_SET_SHIFT (5)     /* Bits 5-(S+4): Way in set being accessed */
                                /* For 4KB cache size: S=5 */
#define CACHE_SET4KB_MASK  (0x1f << CACHE_SET_SHIFT)
                                /* Bits 10-29: Reserved */
                                /* For 8KB cache size: S=6 */
#define CACHE_SET8KB_MASK  (0x3f << CACHE_SET_SHIFT)
                                /* Bits 11-29: Reserved */
                                /* For 16KB cache size: S=7 */
#define CACHE_SET16KB_MASK  (0x7f << CACHE_SET_SHIFT)
                                /* Bits 12-29: Reserved */
                                /* For 32KB cache size: S=8 */
#define CACHE_SET32KB_MASK  (0xff << CACHE_SET_SHIFT)
                                /* Bits 13-29: Reserved */
                                /* For 64KB cache size: S=9 */
#define CACHE_SET64KB_MASK  (0x1fff << CACHE_SET_SHIFT)
                                /* Bits 14-29: Reserved */

/* VA and SBZ format */

#define CACHE_SBZ_SHIFT     (4)       /* Bits 0-4:  Should be zero (SBZ) */
#define CACHE_SBZ_MASK      (31 << TLB_SBZ_SHIFT)
#define CACHE_VA_MASK       (0xfffffffe0) /* Bits 5-31: Virtual address */

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

/* cp15_cache Cache Operations
 *
 * Usage
 *
 * They are performed as MCR instructions and only operate on a level 1 cache
 * associated with  ARM v7 processor.
 *
 * The supported operations are:
 *
 *   1. Any of these operations can be applied to any data cache or any
 *      unified cache.
 *   2. Invalidate by MVA. Performs an invalidate of a data or unified cache
 *      line
 *      based on the address it contains.
 *   3. Invalidate by set/way. Performs an invalidate of a data or unified
 *      cache line based on its location in the cache hierarchy.
 *   4. Clean by MVA.  Performs a clean of a data or unified cache line based
 *      on the address it contains.
 *   5. Clean by set/way. Performs a clean of a data or unified cache line
 *      based on its location in the cache hierarchy.
 *   6. Clean and Invalidate by MVA. Performs a clean and invalidate of a
 *      data or unified cache line based on the address it contains.
 *   7. Clean and Invalidate by set/way. Performs a clean and invalidate of
 *      a data or unified cache line based on its location in the cache
 *      hierarchy.
 *
 * NOTE: Many of these operations are implemented as assembly language
 * macros or as C inline functions in the file cache.h.  The larger functions
 * are implemented here as C-callable functions.
 */

#ifdef __ASSEMBLY__

/****************************************************************************
 * Name: cp15_enable_dcache
 *
 * Description:
 *   Enable L1 D Cache
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

.macro cp15_enable_dcache, tmp
  mrc p15, 0, \tmp, c1, c0, 0 /* Read SCTLR */
  orr \tmp, \tmp, #(0x1 << 2) /* Enable D cache */
  mcr p15, 0, \tmp, c1, c0, 0 /* Update the SCTLR */
.endm

/****************************************************************************
 * Name: cp15_disable_dcache
 *
 * Description:
 *   Disable L1 D Cache
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

.macro cp15_disable_dcache, tmp
  mrc p15, 0, \tmp, c1, c0, 0 /* Read SCTLR */
  bic \tmp, \tmp, #(0x1 << 2) /* Disable D cache */
  mcr p15, 0, \tmp, c1, c0, 0 /* Update the SCTLR */
.endm

/****************************************************************************
 * Name: cp15_enable_icache
 *
 * Description:
 *   Enable L1 I Cache
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

.macro cp15_enable_icache, tmp
  mrc p15, 0, \tmp, c1, c0, 0  /* Read SCTLR */
  orr \tmp, \tmp, #(0x1 << 12) /* Enable I cache */
  mcr p15, 0, \tmp, c1, c0, 0  /* Update the SCTLR */
.endm

/****************************************************************************
 * Name: cp15_disable_icache
 *
 * Description:
 *   Disable L1 I Cache
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

.macro cp15_disable_icache, tmp
  mrc p15, 0, \tmp, c1, c0, 0  /* Read SCTLR */
  bic \tmp, \tmp, #(0x1 << 12) /* Disable I cache */
  mcr p15, 0, \tmp, c1, c0, 0  /* Update the SCTLR */
.endm

/****************************************************************************
 * Name: cp15_invalidate_icache_inner_sharable
 *
 * Description:
 *   Invalidate I cache predictor array inner shareable
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

.macro cp15_invalidate_icache_inner_sharable, tmp
  mov \tmp, #0
  mrc p15, 0, \tmp, c7, c1, 0 /* ICIALLUIS */
.endm

/****************************************************************************
 * Name: cp15_invalidate_btb_inner_sharable
 *
 * Description:
 *   Invalidate entire branch predictor array inner shareable
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

.macro cp15_invalidate_btb_inner_sharable, tmp
  mov \tmp, #0
  mrc p15, 0, \tmp, c7, c1, 6 /* BPIALLIS */
.endm

/****************************************************************************
 * Name: cp15_invalidate_icache
 *
 * Description:
 *   Invalidate all instruction caches to PoU, also flushes branch target
 *   cache
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

.macro cp15_invalidate_icache, tmp
  mov \tmp, #0
  mrc p15, 0, \tmp, c7, c5, 0 /* ICIALLU */
.endm

/****************************************************************************
 * Name: cp15_invalidate_icache_bymva
 *
 * Description:
 *   Invalidate instruction caches by VA to PoU
 *
 * Input Parameters:
 *   va - Register with VA format
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

.macro cp15_invalidate_icache_bymva, va
  mrc p15, 0, \va, c7, c5, 1 /* ICIMVAU */
.endm

/****************************************************************************
 * Name: cp15_flush_btb
 *
 * Description:
 *   Invalidate entire branch predictor array
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

.macro cp15_flush_btb, tmp
  mov \tmp, #0
  mrc p15, 0, \tmp, c7, c5, 6 /* BPIALL */
.endm

/****************************************************************************
 * Name: cp15_flush_btb_bymva
 *
 * Description:
 *   Invalidate branch predictor array entry by MVA
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

.macro cp15_flush_btb_bymva, tmp
  mov \tmp, #0
  mrc p15, 0, \tmp, c7, c5, 7 /* BPIMVA */
.endm

/****************************************************************************
 * Name: cp15_invalidate_dcacheline_bymva
 *
 * Description:
 *   Invalidate data cache line by VA to PoC
 *
 * Input Parameters:
 *   va - Register with VA format
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

.macro cp15_invalidate_dcacheline_bymva, va
  mrc p15, 0, \va, c7, c6, 1 /* DCIMVAC */
.endm

/****************************************************************************
 * Name: cp15_invalidate_dcacheline_bysetway
 *
 * Description:
 *   Invalidate data cache line by set/way
 *
 * Input Parameters:
 *   setway - Register with Set/Way format
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

.macro cp15_invalidate_dcacheline_bysetway, setway
  mrc p15, 0, \setway, c7, c6, 2 /* DCISW */
.endm

/****************************************************************************
 * Name: cp15_clean_dcache_bymva
 *
 * Description:
 *   Clean data cache line by MVA
 *
 * Input Parameters:
 *   va - Register with VA format
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

.macro cp15_clean_dcache_bymva, va
  mrc p15, 0, \va, c7, c10, 1 /* DCCMVAC */
.endm

/****************************************************************************
 * Name: cp15_clean_dcache_bysetway
 *
 * Description:
 *   Clean data cache line by Set/way
 *
 * Input Parameters:
 *   setway - Register with Set/Way format
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

.macro cp15_clean_dcache_bysetway, setway
  mrc p15, 0, \setway, c7, c10, 2 /* DCCSW */
.endm

/****************************************************************************
 * Name: cp15_clean_ucache_bymva
 *
 * Description:
 *   Clean unified cache line by MVA
 *
 * Input Parameters:
 *   setway - Register with VA format
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

.macro cp15_clean_ucache_bymva, setway
  mrc p15, 0, \setway, c7, c11, 1 /* DCCMVAU */
.endm

/****************************************************************************
 * Name: cp15_cleaninvalidate_dcacheline_bymva
 *
 * Description:
 *   Clean and invalidate data cache line by VA to PoC
 *
 * Input Parameters:
 *   va - Register with VA format
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

.macro cp15_cleaninvalidate_dcacheline_bymva, va
  mrc p15, 0, \va, c7, c14, 1 /* DCCIMVAC */
.endm

/****************************************************************************
 * Name: cp15_cleaninvalidate_dcacheline
 *
 * Description:
 *   Clean and Incalidate data cache line by Set/Way
 *
 * Input Parameters:
 *   setway - Register with Set/Way format
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

.macro cp15_cleaninvalidate_dcacheline, setway
  mrc p15, 0, \setway, c7, c14, 2 /* DCCISW */
.endm

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: cp15_enable_dcache
 *
 * Description:
 *   Enable L1 D Cache
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void cp15_enable_dcache(void)
{
  uint32_t sctlr;

  sctlr = CP15_GET(SCTLR);
  sctlr |= SCTLR_C;
  CP15_SET(SCTLR, sctlr);
}

/****************************************************************************
 * Name: cp15_disable_dcache
 *
 * Description:
 *   Disable L1 D Cache
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void cp15_disable_dcache(void)
{
  uint32_t sctlr;

  sctlr = CP15_GET(SCTLR);
  sctlr &= ~SCTLR_C;
  CP15_SET(SCTLR, sctlr);
}

/****************************************************************************
 * Name: cp15_enable_icache
 *
 * Description:
 *   Enable L1 I Cache
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void cp15_enable_icache(void)
{
  uint32_t sctlr;

  sctlr = CP15_GET(SCTLR);
  sctlr |= SCTLR_I;
  CP15_SET(SCTLR, sctlr);
}

/****************************************************************************
 * Name: cp15_disable_icache
 *
 * Description:
 *   Disable L1 I Cache
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void cp15_disable_icache(void)
{
  uint32_t sctlr;

  sctlr = CP15_GET(SCTLR);
  sctlr &= ~SCTLR_I;
  CP15_SET(SCTLR, sctlr);
}

/****************************************************************************
 * Name: cp15_invalidate_icache_inner_sharable
 *
 * Description:
 *   Invalidate I cache predictor array inner shareable
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void cp15_invalidate_icache_inner_sharable(void)
{
  CP15_SET(ICIALLUIS, 0);
}

/****************************************************************************
 * Name: cp15_invalidate_btb_inner_sharable
 *
 * Description:
 *   Invalidate entire branch predictor array inner shareable
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void cp15_invalidate_btb_inner_sharable(void)
{
  CP15_SET(BPIALLIS, 0);
}

/****************************************************************************
 * Name: cp15_invalidate_icache
 *
 * Description:
 *   Invalidate all instruction caches to PoU, also flushes branch target
 *   cache
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void cp15_invalidate_icache(void)
{
  CP15_SET(ICIALLU, 0);
}

/****************************************************************************
 * Name: cp15_invalidate_icache_bymva
 *
 * Description:
 *   Invalidate instruction caches by VA to PoU
 *
 * Input Parameters:
 *   va - 32-bit value with VA format
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void cp15_invalidate_icache_bymva(unsigned int va)
{
  CP15_SET(ICIMVAU, va);
}

/****************************************************************************
 * Name: cp15_flush_btb
 *
 * Description:
 *   Invalidate entire branch predictor array
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void cp15_flush_btb(void)
{
  CP15_SET(BPIALL, 0);
}

/****************************************************************************
 * Name: cp15_flush_btb_bymva
 *
 * Description:
 *   Invalidate branch predictor array entry by MVA
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void cp15_flush_btb_bymva(void)
{
  CP15_SET(BPIMVA, 0);
}

/****************************************************************************
 * Name: cp15_invalidate_dcacheline_bymva
 *
 * Description:
 *   Invalidate data cache line by VA to PoC
 *
 * Input Parameters:
 *   va - 32-bit value with VA format
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

/* Invalidate data cache line by VA to PoC */

static inline void cp15_invalidate_dcacheline_bymva(unsigned int va)
{
  CP15_SET(DCIMVAC, va);
}

/****************************************************************************
 * Name: cp15_invalidate_dcacheline_bysetway
 *
 * Description:
 *   Invalidate data cache line by set/way
 *
 * Input Parameters:
 *   setway - 32-bit value with Set/Way format
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

/* Invalidate data cache line by set/way */

static inline void cp15_invalidate_dcacheline_bysetway(unsigned int setway)
{
  CP15_SET(DCISW, setway);
}

/****************************************************************************
 * Name: cp15_clean_dcache_bymva
 *
 * Description:
 *   Clean data cache line by MVA
 *
 * Input Parameters:
 *   va - 32-bit value with VA format
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

/* Clean data cache line by MVA */

static inline void cp15_clean_dcache_bymva(unsigned int va)
{
  CP15_SET(DCCMVAC, va);
}

/****************************************************************************
 * Name: cp15_clean_dcache_bysetway
 *
 * Description:
 *   Clean data cache line by Set/way
 *
 * Input Parameters:
 *   setway - 32-bit value with Set/Way format
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void cp15_clean_dcache_bysetway(unsigned int setway)
{
  CP15_SET(DCCSW, setway);
}

/****************************************************************************
 * Name: cp15_clean_ucache_bymva
 *
 * Description:
 *   Clean unified cache line by MVA
 *
 * Input Parameters:
 *   setway - 32-bit value with VA format
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void cp15_clean_ucache_bymva(unsigned int setway)
{
  CP15_SET(DCCMVAU, setway);
}

/****************************************************************************
 * Name: cp15_cleaninvalidate_dcacheline_bymva
 *
 * Description:
 *   Clean and invalidate data cache line by VA to PoC
 *
 * Input Parameters:
 *   va - 32-bit value with VA format
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void cp15_cleaninvalidate_dcacheline_bymva(unsigned int va)
{
  CP15_SET(DCCIMVAC, va);
}

/****************************************************************************
 * Name: cp15_cleaninvalidate_dcacheline
 *
 * Description:
 *   Clean and Incalidate data cache line by Set/Way
 *
 * Input Parameters:
 *   setway - 32-bit value with Set/Way format
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void cp15_cleaninvalidate_dcacheline(unsigned int setway)
{
  CP15_SET(DCCISW, setway);
}

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
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
 * Name: cp15_coherent_dcache
 *
 * Description:
 *   Ensure that the I and D caches are coherent within specified region
 *   by cleaning the D cache (i.e., flushing the D cache contents to memory
 *   and invalidating the I cache). This is typically used when code has been
 *   written to a memory region, and will be executed.
 *
 * Input Parameters:
 *   start - virtual start address of region
 *   end   - virtual end address of region + 1
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void cp15_coherent_dcache(uintptr_t start, uintptr_t end);

/****************************************************************************
 * Name: cp15_invalidate_dcache
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
 ****************************************************************************/

void cp15_invalidate_dcache(uintptr_t start, uintptr_t end);

/****************************************************************************
 * Name: cp15_invalidate_dcache_all
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

void cp15_invalidate_dcache_all(void);

/****************************************************************************
 * Name: cp15_clean_dcache
 *
 * Description:
 *   Clean the data cache within the specified region by flushing the
 *   contents of the data cache to memory.
 *
 * Input Parameters:
 *   start - virtual start address of region
 *   end   - virtual end address of region + 1
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void cp15_clean_dcache(uintptr_t start, uintptr_t end);

/****************************************************************************
 * Name: cp15_clean_dcache_all
 *
 * Description:
 *   Clean the entire contents of D cache.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void cp15_clean_dcache_all(void);

/****************************************************************************
 * Name: cp15_flush_dcache
 *
 * Description:
 *   Flush the data cache within the specified region by cleaning and
 *   invalidating the D cache.
 *
 * Input Parameters:
 *   start - virtual start address of region
 *   end   - virtual end address of region + 1
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void cp15_flush_dcache(uintptr_t start, uintptr_t end);

/****************************************************************************
 * Name: cp15_flush_dcache_all
 *
 * Description:
 *   Flush the entire contents of D cache.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void cp15_flush_dcache_all(void);

/****************************************************************************
 * Name: cp15_cache_size
 *
 * Description:
 *   Get cp15 cache size in byte
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Cache size in byte
 *
 ****************************************************************************/

uint32_t cp15_cache_size(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_ARMV7_R_CP15_CACHEOPS_H */
