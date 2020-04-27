/****************************************************************************
 * arch/mips/src/mips32/mips_cache.S
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Abdelatif Guettouche <abdelatif.guettouche@gmail.com>
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/cache.h>

#include <arch/mips32/registers.h>
#include <arch/mips32/cp0.h>

#include "mips_arch.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Cache algorithm.
 * When the cache is enabled the write-back write-allocate algorithm is used.
 */

#define CACHE_ENABLE  CP0_CONFIG_K0_CACHEABLE
#define CACHE_DISABLE CP0_CONFIG_K0_UNCACHED

/* Cache Operations  ********************************************************/

#define INDEX_INVALIDATE_I 0x00
#define INDEX_INVALIDATE_D 0x01
#define INDEX_STORE_TAG_I  0x08
#define INDEX_STORE_TAG_D  0x09
#define HIT_INVALIDATE_I   0x10
#define HIT_INVALIDATE_D   0x11
#define HIT_WRITEBACK_D    0x15

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

  .file  "mips_cache.S"

/****************************************************************************
 * Assembly Language Macros
 ****************************************************************************/

/****************************************************************************
 * Name: CACHE_OP

 * Description:
 *   Performs cache operation on an address range.
 *
 ****************************************************************************/

  .macro CACHE_OP  op, start, end, lsize
1:
  cache    \op, 0(\start)
  addu     \start, \start, \lsize
  bne      \start, \end, 1b
  .endm

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icache_linesize
 *
 * Description:
 *   Get I-Cache line size.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   V0 = Size of I-Cache line
 *
 ****************************************************************************/

  .text
  .set nomips16
  .globl icache_linesize
  .ent icache_linesize

icache_linesize:

#ifndef CONFIG_MIPS32_CACHE_AUTOINFO
  li v0, CONFIG_MIPS32_ILINE_SIZE
#else
  li v0, 0

  /* Is a cache implemented? */

  mfc0 t0, MIPS32_CP0_CONFIG

  and t1, t0, CP0_CONFIG_M_MASK
  beqz t1, 1f  /* No cache implemented, leave. */

  /* Get the I-Cache line size */

  mfc0 t0, MIPS32_CP0_CONFIG1

  and t1, t0, CP0_CONFIG1_IL_MASK
  beqz t1, 1f /* No I-Cache implemented, leave. */

  srl t1, t1, CP0_CONFIG1_IL_SHIFT
  li t2, 2
  srl v0, t2, t1 /* Cache line size = 2 << IL_VALUE */
#endif

1:
  jr  ra
  nop

  .size icache_linesize,.-icache_linesize
  .end icache_linesize

/****************************************************************************
 * Name: icache_associativity
 *
 * Description:
 *   Get I-Cache associativity
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   V0 = I-Cache associativity.
 *
 ****************************************************************************/

  .text
  .set nomips16
  .globl icache_associativity
  .ent icache_associativity

icache_associativity:

  /* Is a cache implemented? */

  mfc0 t0, MIPS32_CP0_CONFIG

  and t1, t0, CP0_CONFIG_M_MASK
  beqz t1, 1f  /* No cache implemented, leave. */

  /* Get I-Cache associativity. */

  mfc0 t0, MIPS32_CP0_CONFIG1

  and t1, t0, CP0_CONFIG1_IA_MASK
  srl t1, t1, CP0_CONFIG1_IA_SHIFT
  addu v0, t1, 1 /* Cache associativity = IA_VALUE + 1 */

1:
  jr  ra
  nop

  .size icache_associativity,.-icache_associativity
  .end icache_associativity

/****************************************************************************
 * Name: icache_sets
 *
 * Description:
 *   Get I-Cache sets
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   V0 = I-Cache sets.
 *
 ****************************************************************************/

  .text
  .set nomips16
  .globl icache_sets
  .ent icache_sets

icache_sets:

  /* Is a cache implemented? */

  mfc0 t0, MIPS32_CP0_CONFIG

  and t1, t0, CP0_CONFIG_M_MASK
  beqz t1, 1f  /* No cache implemented, leave. */

  /* Get I-Cache sets. */

  mfc0 t0, MIPS32_CP0_CONFIG1

  and t1, t0, CP0_CONFIG1_IS_MASK
  srl t1, t1, CP0_CONFIG1_IS_SHIFT
  addu t1, 6 /* Cache sets = 1 << IS_VALUE + 6 */
  li t2, 1
  sll v0, t2, t1

1:
  jr  ra
  nop

  .size icache_sets,.-icache_sets
  .end icache_sets

/****************************************************************************
 * Name: icache_size
 *
 * Description:
 *   Get I-Cache size.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   V0 = Size of I-Cache.
 *
 ****************************************************************************/

  .text
  .set nomips16
  .globl icache_size
  .ent icache_size

icache_size:

#ifndef CONFIG_MIPS32_CACHE_AUTOINFO
  li v0, CONFIG_MIPS32_ICACHE_SIZE
#else
  addiu sp, sp, -16
  sw    s0, 0(sp)
  sw    s1, 4(sp)
  sw    s2, 8(sp)
  sw    ra, 12(sp)

  /* Get I-Cache line size */

  jal icache_linesize
  nop
  move s0, v0

  /* Get I-Cache associativity */

  jal icache_associativity
  nop
  move s1, v0

  /* Get I-Cache sets per way */

  jal icache_sets
  nop
  move s2, v0

  /* Cache size = line size * associativity * sets per way */

  multu s0, s1
  mflo t0
  multu s2, t0
  mflo v0

  lw s0, 0(sp)
  lw s1, 4(sp)
  lw s2, 8(sp)
  lw ra, 12(sp)
  addiu sp, sp, 16
#endif

1:
  jr  ra
  nop

  .size icache_size,.-icache_size
  .end icache_size

/****************************************************************************
 * Name: dcache_linesize
 *
 * Description:
 *   Get D-Cache line size.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   V0 = Size of D-Cache line
 *
 ****************************************************************************/

  .text
  .set nomips16
  .globl dcache_linesize
  .ent dcache_linesize

dcache_linesize:

#ifndef CONFIG_MIPS32_CACHE_AUTOINFO
  li v0, CONFIG_MIPS32_DLINE_SIZE
#else
  li v0, 0

  /* Is a cache implemented? */

  mfc0 t0, MIPS32_CP0_CONFIG

  and t1, t0, CP0_CONFIG_M_MASK
  beqz t1, 1f  /* No cache implemented, leave. */

  /* Get D-Cache line size */

  mfc0 t0, MIPS32_CP0_CONFIG1

  and t1, t0, CP0_CONFIG1_DL_MASK
  beqz t1, 1f /* No D-Cache implemented, leave. */

  srl t1, t1, CP0_CONFIG1_DL_SHIFT
  li t2, 2
  srl v0, t2, t1 /* Cache line size = 2 << DL_VALUE */
#endif

1:
  jr  ra
  nop

  .size dcache_linesize,.-dcache_linesize
  .end dcache_linesize

/****************************************************************************
 * Name: dcache_associativity
 *
 * Description:
 *   Get D-Cache associativity
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   V0 = D-Cache associativity.
 *
 ****************************************************************************/

  .text
  .set nomips16
  .globl dcache_associativity
  .ent dcache_associativity

dcache_associativity:

  /* Is a cache implemented? */

  mfc0 t0, MIPS32_CP0_CONFIG

  and t1, t0, CP0_CONFIG_M_MASK
  beqz t1, 1f  /* No cache implemented, leave. */

  /* Get D-Cache associativity. */

  mfc0 t0, MIPS32_CP0_CONFIG1

  and t1, t0, CP0_CONFIG1_DA_MASK
  srl t1, t1, CP0_CONFIG1_DA_SHIFT
  addu v0, t1, 1 /* Cache associativity = DA_VALUE + 1 */

1:
  jr  ra
  nop

  .size dcache_associativity,.-dcache_associativity
  .end dcache_associativity

/****************************************************************************
 * Name: dcache_sets
 *
 * Description:
 *   Get D-Cache sets
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   V0 = D-Cache sets.
 *
 ****************************************************************************/

  .text
  .set nomips16
  .globl dcache_sets
  .ent dcache_sets

dcache_sets:

  /* Is a cache implemented? */

  mfc0 t0, MIPS32_CP0_CONFIG

  and t1, t0, CP0_CONFIG_M_MASK
  beqz t1, 1f  /* No cache implemented, leave. */

  /* Get D-Cache sets. */

  mfc0 t0, MIPS32_CP0_CONFIG1

  and t1, t0, CP0_CONFIG1_DS_MASK
  srl t1, t1, CP0_CONFIG1_DS_SHIFT
  addu t1, 6 /* Cache sets = 1 << DS_VALUE + 6 */
  li t2, 1
  sll v0, t2, t1

1:
  jr  ra
  nop

  .size dcache_sets,.-dcache_sets
  .end dcache_sets

/****************************************************************************
 * Name: dcache_size
 *
 * Description:
 *   Get D-Cache size.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   V0 = Size of D-Cache.
 *
 ****************************************************************************/

  .text
  .set nomips16
  .globl dcache_size
  .ent dcache_size

dcache_size:

#ifndef CONFIG_MIPS32_CACHE_AUTOINFO
  li v0, CONFIG_MIPS32_DCACHE_SIZE
#else
  addiu sp, sp, -16
  sw    s0, 0(sp)
  sw    s1, 4(sp)
  sw    s2, 8(sp)
  sw    ra, 12(sp)

  /* Get D-Cache line size */

  jal dcache_linesize
  nop
  move s0, v0

  /* Get D-Cache associativity */

  jal dcache_associativity
  nop
  move s1, v0

  /* Get D-Cache sets per way */

  jal dcache_sets
  nop
  move s2, v0

  /* Cache size = line size * associativity * sets per way */

  multu s0, s1
  mflo t0
  multu s2, t0
  mflo v0

  lw s0, 0(sp)
  lw s1, 4(sp)
  lw s2, 8(sp)
  lw ra, 12(sp)
  addiu sp, sp, 16
#endif

1:
  jr  ra
  nop

  .size dcache_size,.-dcache_size
  .end dcache_size

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_enable_icache
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
 * Caution:
 *   The writable global variables aren't initialized yet.
 *
 ****************************************************************************/

#ifdef CONFIG_MIPS32_ICACHE
  .text
  .set nomips16
  .globl up_enable_icache
  .ent up_enable_icache

up_enable_icache:

  /* Disable interrupts */

  di

  /* Enable both I & D Caches */

  mfc0 t1, MIPS32_CP0_CONFIG
  ori  t1, CP0_CONFIG_K0_MASK
  xori t1, CP0_CONFIG_K0_MASK
  ori  t1, CACHE_ENABLE
  mtc0 t1, MIPS32_CP0_CONFIG

  /* Restore interrupts */

  ei

  jr ra
  nop

  .size up_enable_icache,.-up_enable_icache
  .end up_enable_icache
#endif

/****************************************************************************
 * Name: up_disable_icache
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

#ifdef CONFIG_MIPS32_ICACHE
  .text
  .set nomips16
  .globl up_disable_icache
  .ent up_disable_icache

up_disable_icache:

  /* Disable interrupts */

  di

  /* Disable both I & D Caches */

  mfc0 t1, MIPS32_CP0_CONFIG
  ori  t1, CP0_CONFIG_K0_MASK
  xori t1, CP0_CONFIG_K0_MASK
  ori  t1, CACHE_DISABLE
  mtc0 t1, MIPS32_CP0_CONFIG

  /* Restore interrupts */

  ei

  jr ra
  nop

  .size up_disable_icache,.-up_disable_icache
  .end up_disable_icache
#endif

/****************************************************************************
 * Name: up_invalidate_icache
 *
 * Description:
 *   Invalidate the instruction cache within the specified region.
 *
 * Input Parameters:
 *   start - virtual start address of region
 *   end   - virtual end address of region + 1
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_MIPS32_ICACHE
  .text
  .set nomips16
  .globl up_invalidate_icache
  .ent up_invalidate_icache

up_invalidate_icache:

  addiu sp, sp, -8
  sw    ra, 0(sp)

  /* Get I-Cache line size */

  jal icache_linesize
  nop
  move t2, v0
  beqz t2, 1f  /* No I-Cache, nothing to do here. */

  /* Align start and end addresses to cache line size */

  .set noreorder

  subu t3, t2, 1
  not  t3

  and  t0, a0, t3

  subu t1, a1, 1
  and  t1, t1, t3

  /* Perform the cache operation */

  CACHE_OP HIT_INVALIDATE_I, t0, t1, t2

  /* Ensure all pending cache operations are completed. */

  sync

1:
  .set reorder

  lw    ra, 0(sp)
  addiu sp, sp, 8

  jr ra
  nop

  .size up_invalidate_icache,.-up_invalidate_icache
  .end up_invalidate_icache
#endif

/****************************************************************************
 * Name: up_invalidate_icache_all
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

#ifdef CONFIG_MIPS32_ICACHE
  .text
  .set nomips16
  .globl up_invalidate_icache_all
  .ent up_invalidate_icache_all

up_invalidate_icache_all:

  addiu sp, sp, -8
  sw    ra, 0(sp)

  /* Get I-Cache size */

  jal icache_size
  nop
  move t0, v0
  beqz t0, 1f /* No I-Cache, nothing to do here. */

  /* Get I-Cache line size */

  jal icache_linesize
  nop
  move t1, v0
  beqz t1, 1f /* No I-Cache, nothing to do here. */

  /* Start address is IBASE, end address is IBASE + Cache Size */

  .set noreorder

  li t3, CONFIG_MIPS32_KSEG0_IBASE
  addu t4, t3, t0

  /* Perform the cache operation */

  CACHE_OP HIT_INVALIDATE_I, t3, t4, t1

  /* Ensure all pending cache operations are completed. */

  sync

1:
  .set reorder

  lw    ra, 0(sp)
  addiu sp, sp, 8

  jr ra
  nop

  .size up_invalidate_icache_all,.-up_invalidate_icache_all
  .end up_invalidate_icache_all
#endif

 /****************************************************************************
 * Name: up_enable_dcache
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
 * Caution:
 *   The writable global variables aren't initialized yet.
 *
 ****************************************************************************/

#ifdef CONFIG_MIPS32_DCACHE
  .text
  .set nomips16
  .globl up_enable_dcache
  .ent up_enable_dcache

up_enable_dcache:

  /* Disable interrupts */

  di
  .set noreorder

  /* Enable both I & D Caches */

  mfc0 t1, MIPS32_CP0_CONFIG
  ori  t1, CP0_CONFIG_K0_MASK
  xori t1, CP0_CONFIG_K0_MASK
  ori  t1, CACHE_ENABLE
  mtc0 t1, MIPS32_CP0_CONFIG

  /* Restore interrupts */

  .set reorder
  ei

  jr ra
  nop

  .size up_enable_dcache,.-up_enable_dcache
  .end up_enable_dcache
#endif

/****************************************************************************
 * Name: up_disable_dcache
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

#ifdef CONFIG_MIPS32_DCACHE
  .text
  .set nomips16
  .globl up_disable_dcache
  .ent up_disable_dcache

up_disable_dcache:

  /* Disable interrupts */

  di
  .set noreorder

  /* Disable both I & D Caches */

  mfc0 t1, MIPS32_CP0_CONFIG
  ori  t1, CP0_CONFIG_K0_MASK
  xori t1, CP0_CONFIG_K0_MASK
  ori  t1, CACHE_DISABLE
  mtc0 t1, MIPS32_CP0_CONFIG

  /* Restore interrupts */

  .set reorder
  ei

  jr ra
  nop

  .size up_disable_dcache,.-up_disable_dcache
  .end up_disable_dcache
#endif

/****************************************************************************
 * Name: up_invalidate_dcache
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

#ifdef CONFIG_MIPS32_DCACHE
  .text
  .set nomips16
  .globl up_invalidate_dcache
  .ent up_invalidate_dcache

up_invalidate_dcache:

  addiu sp, sp, -8
  sw    ra, 0(sp)

  /* Get D-Cache line size */

  jal dcache_linesize
  nop
  move t2, v0
  beqz t2, 1f  /* No D-Cache, nothing to do here. */

  /* Align start and end addresses to cache line size */

  .set noreorder

  subu t3, t2, 1
  not  t3

  and  t0, a0, t3

  subu t1, a1, 1
  and  t1, t1, t3

  /* Perform the cache operation */

  CACHE_OP HIT_INVALIDATE_D, t0, t1, t2

  /* Ensure all pending cache operations are completed. */

  sync

1:
  .set reorder

  lw    ra, 0( sp)
  addiu sp, sp, 8

  jr ra
  nop

  .size up_invalidate_dcache,.-up_invalidate_dcache
  .end up_invalidate_dcache
#endif

/****************************************************************************
 * Name: up_invalidate_dcache_all
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

#ifdef CONFIG_MIPS32_DCACHE
  .text
  .set nomips16
  .globl up_invalidate_dcache_all
  .ent up_invalidate_dcache_all

up_invalidate_dcache_all:

  addiu sp, sp, -8
  sw    ra, 0(sp)

  /* Get D-Cache size */

  jal dcache_size
  nop
  move t0, v0
  beqz t0, 1f /* No D-Cache, nothing to do here. */

  /* Get D-Cache line size */

  jal dcache_linesize
  nop
  move t1, v0
  beqz t1, 1f /* No D-Cache, nothing to do here. */

  /* Start address is DBASE, end address is DBASE + Cache Size */

  .set noreorder

  li t3, CONFIG_MIPS32_KSEG0_DBASE
  addu t4, t3, t0

  /* Perform the cache operation */

  CACHE_OP HIT_INVALIDATE_D, t3, t4, t1

  /* Ensure all pending cache operations are completed. */

  sync

1:
  .set reorder

  lw    ra, 0(sp)
  addiu sp, sp, 8

  jr ra
  nop

  .size up_invalidate_dcache_all,.-up_invalidate_dcache_all
  .end up_invalidate_dcache_all
#endif

/****************************************************************************
 * Name: up_clean_dcache
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

#ifdef CONFIG_MIPS32_DCACHE
  .text
  .set nomips16
  .globl up_clean_dcache
  .ent up_clean_dcache

up_clean_dcache:

  addiu sp, sp, -8
  sw    ra, 0(sp)

  /* Get D-Cache line size */

  jal dcache_linesize
  nop
  move t2, v0
  beqz t2, 1f  /* No D-Cache, nothing to do here. */

  /* Align start and end addresses to cache size */

  .set noreorder

  subu t3, t2, 1
  not  t3

  and  t0, a0, t3

  subu t1, a1, 1
  and  t1, t1, t3

  /* Perform the cache operation */

  CACHE_OP HIT_WRITEBACK_D, t0, t1, t2

  /* Ensure all pending cache operations are completed. */

  sync

1:
  .set reorder

  lw    ra, 0( sp)
  addiu sp, sp, 8

  jr ra
  nop

  .size up_clean_dcache,.-up_clean_dcache
  .end up_clean_dcache
#endif

/****************************************************************************
 * Name: up_clean_dcache_all
 *
 * Description:
 *   Clean the entire data cache within the specified region by flushing the
 *   contents of the data cache to memory.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_MIPS32_DCACHE
  .text
  .set nomips16
  .globl up_clean_dcache_all
  .ent up_clean_dcache_all

up_clean_dcache_all:

  addiu sp, sp, -8
  sw    ra, 0(sp)

  /* Get D-Cache size */

  jal dcache_size
  nop
  move t0, v0
  beqz t0, 1f /* No D-Cache, nothing to do here. */

  /* Get D-Cache line size */

  jal dcache_linesize
  nop
  move t1, v0
  beqz t1, 1f /* No D-Cache, nothing to do here. */

  /* Start address is DBASE, end address is DBASE + Cache Size */

  .set noreorder

  li t3, CONFIG_MIPS32_KSEG0_DBASE
  addu t4, t3, t0

  /* Perform the cache operation */

  CACHE_OP HIT_WRITEBACK_D, t3, t4, t1

  /* Ensure all pending cache operations are completed. */

  sync

1:
  .set reorder

  lw    ra, 0(sp)
  addiu sp, sp, 8

  jr ra
  nop

  .size up_clean_dcache_all,.-up_clean_dcache_all
  .end up_clean_dcache_all
#endif

/****************************************************************************
 * Name: up_flush_dcache
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

#ifdef CONFIG_MIPS32_DCACHE
  .text
  .set nomips16
  .globl up_flush_dcache
  .ent up_flush_dcache

up_flush_dcache:

  addiu sp, sp, -8
  sw    ra, 0(sp)

  /* Get D-Cache line size */

  jal dcache_linesize
  nop
  move t2, v0
  beqz t2, 1f  /* No D-Cache, nothing to do here. */

  /* Align start and end addresses to cache size */

  .set noreorder

  subu t3, t2, 1
  not  t3

  and  t0, a0, t3

  subu t1, a1, 1
  and  t1, t1, t3

  /* Perform the cache operation */

  CACHE_OP HIT_WRITEBACK_D, t0, t1, t2

  /* Ensure all pending cache operations are completed. */

  sync

1:
  .set reorder

  lw    ra, 0(sp)
  addiu sp, sp, 8

  jr ra
  nop

  .size up_flush_dcache,.-up_flush_dcache
  .end up_flush_dcache
#endif

/****************************************************************************
 * Name: up_flush_dcache_all
 *
 * Description:
 *   Flush the entire data cache by cleaning and invalidating the D cache.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_MIPS32_DCACHE
  .text
  .set nomips16
  .globl up_flush_dcache_all
  .ent up_flush_dcache_all

up_flush_dcache_all:

  addiu sp, sp, -8
  sw    ra, 0(sp)

  /* Get D-Cache size */

  jal dcache_size
  nop
  move t0, v0
  beqz t0, 1f /* No D-Cache, nothing to do here. */

  /* Get D-Cache line size */

  jal dcache_linesize
  nop
  move t1, v0
  beqz t1, 1f /* No D-Cache, nothing to do here. */

  /* Start address is DBASE, end address is DBASE + Cache Size */

  .set noreorder

  li t3, CONFIG_MIPS32_KSEG0_DBASE
  addu t4, t3, t0

  /* Perform the cache operation */

  CACHE_OP HIT_WRITEBACK_D, t3, t4, t1

  /* Ensure all pending cache operations are completed. */

  sync

1:
  .set reorder

  lw    ra, 0(sp)
  addiu sp, sp, 8

  jr ra
  nop

  .size up_flush_dcache_all,.-up_flush_dcache_all
  .end up_flush_dcache_all
#endif

/****************************************************************************
 * Name: up_coherent_dcache
 *
 * Description:
 *   Ensure that the I and D caches are coherent within specified region
 *   by cleaning the D cache (i.e., flushing the D cache contents to memory
 *   and invalidating the I cache. This is typically used when code has been
 *   written to a memory region, and will be executed.
 *
 * Input Parameters:
 *   addr - virtual start address of region
 *   len  - Size of the address region in bytes
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_MIPS32_DCACHE
  .text
  .set nomips16
  .globl up_coherent_dcache
  .ent up_coherent_dcache

up_coherent_dcache:

  addiu sp, sp, -8
  sw    ra, 0(sp)

  /* If size is null, nothing to do here. */

  beqz a1, 1f

  /* End address = addr + len*/

  addu a1, a0

  /* Flush any dirty D-Cache lines */

  jal up_flush_dcache
  nop

  /* Invalidate the entire I-Cache */

  jal up_invalidate_icache_all
  nop

1:
  lw ra, 0(sp)
  addiu sp, sp, 8

  jr ra
  nop

  .size up_coherent_dcache,.-up_coherent_dcache
  .end up_coherent_dcache
#endif
