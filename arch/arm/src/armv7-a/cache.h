/************************************************************************************
 * arch/arm/src/armv7-a/cache.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *
 *  "Cortex-A5™ MPCore, Technical Reference Manual", Revision: r0p1, Copyright © 2010
 *   ARM. All rights reserved. ARM DDI 0434B (ID101810)
 *  "ARM® Architecture Reference Manual, ARMv7-A and ARMv7-R edition", Copyright ©
 *   1996-1998, 2000, 2004-2012 ARM. All rights reserved. ARM DDI 0406C.b (ID072512)
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_ARMV7_A_CPSR_H
#define __ARCH_ARM_SRC_ARMV7_A_CPSR_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Reference: Cortex-A5™ MPCore Paragraph 4.1.5, "Cache Operations Registers."
 *
 * Terms:
 * 1) Point of coherency (PoC)
 *    The PoC is the point at which all agents that can access memory are guaranteed
 *    to see the same copy of a memory location
 * 2) Point of unification (PoU)
 *    The PoU is the point by which the instruction and data caches and the
 *    translation table walks of the processor are guaranteed to see the same copy
 *    of a memory location.
 *
 * Cache Operations:
 *
 * CP15 Register:     ICIALLUIS
 *   Description:     Invalidate entire instruction cache Inner Shareable.
 *   Register Format: SBZ
 *   Instruction:     MCR p15, 0, <Rd>, c7, c1, 0
 * CP15 Register:     BPIALLIS
 *   Description:     Invalidate entire branch predictor array Inner Shareable.
 *   Register Format: SBZ
 *   Instruction:     MCR p15, 0, <Rd>, c7, c1, 6
 * CP15 Register:     ICIALLU
 *   Description:     Invalidate all instruction caches to PoU. Also flushes branch
 *                    target cache.
 *   Register Format: SBZ
 *   Instruction:     MCR p15, 0, <Rd>, c7, c5, 0
 * CP15 Register:     ICIMVAU
 *   Description:     Invalidate instruction cache by VA to PoU.
 *   Register Format: VA
 *   Instruction:     MCR p15, 0, <Rd>, c7, c5, 1
 * CP15 Register:     BPIALL
 *   Description:     Invalidate entire branch predictor array.
 *   Register Format: SBZ
 *   Instruction:     MCR p15, 0, <Rd>, c7, c5, 6
 * CP15 Register:     BPIMVA
 *   Description:     Invalidate VA from branch predictor array.
 *   Register Format: SBZ
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

#define CACHE_SBZ_SHIFT     (4)       /* Bits 0-4:  SBZ */
#define CACHE_SBZ_MASK      (31 << TLB_SBZ_SHIFT)
#define CACHE_VA_MASK       (0xfffffffe0) /* Bits 5-31: Virtual address */

/************************************************************************************
 * Assemby Macros
 ************************************************************************************/

#ifdef __ASSEMBLY__

/* Invalidate I cache predictor array inner sharable */

	.macro	cp15_invalidate_icache_inner_sharable, scratch
	mov		\scratch, #0
	mrc		p15, 0, \scratch, c7, c1, 0
	.endm

/* Invalidate entire branch predictor array inner sharable */

	.macro	cp15_invalidate_btb_inner_sharable, scratch
	mov		\scratch, #0
	mrc		p15, 0, \scratch, c7, c1, 6
	.endm

/* Invalidate all instruction caches to PoU, also flushes branch target cache */

	.macro	cp15_invalidate_icache, scratch
	mov		\scratch, #0
	mrc		p15, 0, \scratch, c7, c5, 0
	.endm

/* Invalidate instruction caches by VA to PoU */

	.macro	cp15_invalidate_icache_bymva, scratch
	mov		\scratch, #0
	mrc		p15, 0, \scratch, c7, c5, 1
	.endm

/* Flush entire branch predictor array */

	.macro	cp15_flush_btb, scratch
	mov		\scratch, #0
	mrc		p15, 0, \scratch, c7, c5, 6
	.endm

/* Flush branch predictor array entry by MVA */

	.macro	cp15_flush_btb_bymva, scratch
	mov		\scratch, #0
	mrc		p15, 0, \scratch, c7, c5, 7
	.endm

/* Invalidate data cache line by VA to PoC */

	.macro	cp15_invalidate_dcacheline_bymva, scratch
	mov		\scratch, #0
	mrc		p15, 0, \scratch, c7, c6, 1
	.endm

/* Invalidate data cache line by set/way */

	.macro	cp15_invalidate_dcacheline_bysetway, scratch
	mov		\scratch, #0
	mrc		p15, 0, \scratch, c7, c6, 2
	.endm

/* Clean data cache line by MVA */

	.macro	cp15_clean_dcache_bymva, scratch
	mov		\scratch, #0
	mrc		p15, 0, \scratch, c7, c10, 1
	.endm

/* Clean data cache line by Set/way */

	.macro	cp15_clean_dcache_bysetway, scratch
	mov		\scratch, #0
	mrc		p15, 0, \scratch, c7, c10, 2
	.endm

/* Clean unified cache line by MVA */

	.macro	cp15_clean_dcache_bymva, scratch
	mov		\scratch, #0
	mrc		p15, 0, \scratch, c7, c11, 1
	.endm

/* Clean and invalidate data cache line by VA to PoC */

	.macro	cp15_cleaninvalidate_dcacheline_bymva, scratch
	mov		\scratch, #0
	mrc		p15, 0, \scratch, c7, c14, 1
	.endm

/* Clean and Incalidate data cache line by Set/Way */

	.macro	cp15_cleaninvalidate_dcacheline, scratch
	mov		\scratch, #0
	mrc		p15, 0, \scratch, c7, c14, 2
	.endm

#endif /* __ASSEMBLY__ */

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

/* Invalidate I cache predictor array inner sharable */

static inline void cp15_invalidate_icache_inner_sharable(void)
{
  __asm__ __volatile__
    (
      "\tmov r0, #0\n"
      "\tmcr p15, 0, r0, c7, c1, 0\n"
      :
      : "r" (ttb)
      : "r0", "memory"
    );

/* Invalidate entire branch predictor array inner sharable */

static inline void cp15_invalidate_btb_inner_sharable(void)
{
  __asm__ __volatile__
    (
      "\tmov r0, #0\n"
      "\tmcr p15, 0, r0, c7, c1, 6\n"
      :
      : "r" (ttb)
      : "r0", "memory"
    );

/* Invalidate all instruction caches to PoU, also flushes branch target cache */

static inline void cp15_invalidate_icache(void)
{
  __asm__ __volatile__
    (
      "\tmov r0, #0\n"
      "\tmcr p15, 0, r0, c7, c5, 0\n"
      :
      : "r" (ttb)
      : "r0", "memory"
    );

/* Invalidate instruction caches by VA to PoU */

static inline void cp15_invalidate_icache_bymva(void)
{
  __asm__ __volatile__
    (
      "\tmov r0, #0\n"
      "\tmcr p15, 0, r0, c7, c5, 1\n"
      :
      : "r" (ttb)
      : "r0", "memory"
    );

/* Flush entire branch predictor array */

static inline void cp15_flush_btb(void)
{
  __asm__ __volatile__
    (
      "\tmov r0, #0\n"
      "\tmcr p15, 0, r0, c7, c5, 6\n"
      :
      : "r" (ttb)
      : "r0", "memory"
    );

/* Flush branch predictor array entry by MVA */

static inline void cp15_flush_btb_bymva(void)
{
  __asm__ __volatile__
    (
      "\tmov r0, #0\n"
      "\tmcr p15, 0, r0, c7, c5, 7\n"
      :
      : "r" (ttb)
      : "r0", "memory"
    );

/* Invalidate data cache line by VA to PoC */

static inline void cp15_invalidate_dcacheline_bymva(void)
{
  __asm__ __volatile__
    (
      "\tmov r0, #0\n"
      "\tmcr p15, 0, r0, c7, c6, 1\n"
      :
      : "r" (ttb)
      : "r0", "memory"
    );

/* Invalidate data cache line by set/way */

static inline void cp15_invalidate_dcacheline_bysetway(void)
{
  __asm__ __volatile__
    (
      "\tmov r0, #0\n"
      "\tmcr p15, 0, r0, c7, c6, 2\n"
      :
      : "r" (ttb)
      : "r0", "memory"
    );

/* Clean data cache line by MVA */

static inline void cp15_clean_dcache_bymva(void)
{
  __asm__ __volatile__
    (
      "\tmov r0, #0\n"
      "\tmcr p15, 0, r0, c7, c10, 1\n"
      :
      : "r" (ttb)
      : "r0", "memory"
    );

/* Clean data cache line by Set/way */

static inline void cp15_clean_dcache_bysetway(void)
{
  __asm__ __volatile__
    (
      "\tmov r0, #0\n"
      "\tmcr p15, 0, r0, c7, c10, 2\n"
      :
      : "r" (ttb)
      : "r0", "memory"
    );

/* Clean unified cache line by MVA */

static inline void cp15_clean_dcache_bymva(void)
{
  __asm__ __volatile__
    (
      "\tmov r0, #0\n"
      "\tmcr p15, 0, r0, c7, c11, 1\n"
      :
      : "r" (ttb)
      : "r0", "memory"
    );

/* Clean and invalidate data cache line by VA to PoC */

static inline void cp15_cleaninvalidate_dcacheline_bymva(void)
{
  __asm__ __volatile__
    (
      "\tmov r0, #0\n"
      "\tmcr p15, 0, r0, c7, c14, 1\n"
      :
      : "r" (ttb)
      : "r0", "memory"
    );

/* Clean and Incalidate data cache line by Set/Way */

static inline void cp15_cleaninvalidate_dcacheline(void)
{
  __asm__ __volatile__
    (
      "\tmov r0, #0\n"
      "\tmcr p15, 0, r0, c7, c14, 2\n"
      :
      : "r" (ttb)
      : "r0", "memory"
    );

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif  /* __ARCH_ARM_SRC_ARMV7_A_CPSR_H */
