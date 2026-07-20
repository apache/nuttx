/****************************************************************************
 * arch/mips/src/jz4780/mti_cache.S
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright 2014-2015, Imagination Technologies Limited and/or its
 *                      affiliated group companies.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "mips32-memorymap.h"

#ifdef CONFIG_ALLOW_BSD_COMPONENTS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FUNCTION(x) .global x; .type x, STT_FUNC; x:
#define DATA(x) .global x; .type x, STT_OBJECT; x:

/* Cache operations */

#define Index_Invalidate_I               0x00        /* 0       0 */
#define Index_Writeback_Inv_D            0x01        /* 0       1 */
#define Index_Writeback_Inv_T            0x02        /* 0       2 */
#define Index_Writeback_Inv_S            0x03        /* 0       3 */
#define Index_Load_Tag_I                 0x04        /* 1       0 */
#define Index_Load_Tag_D                 0x05        /* 1       1 */
#define Index_Load_Tag_T                 0x06        /* 1       2 */
#define Index_Load_Tag_S                 0x07        /* 1       3 */
#define Index_Store_Tag_I                0x08        /* 2       0 */
#define Index_Store_Tag_D                0x09        /* 2       1 */
#define Index_Store_Tag_T                0x0A        /* 2       2 */
#define Index_Store_Tag_S                0x0B        /* 2       3 */
#define Hit_Invalidate_I                 0x10        /* 4       0 */
#define Hit_Invalidate_D                 0x11        /* 4       1 */
#define Hit_Invalidate_T                 0x12        /* 4       2 */
#define Hit_Invalidate_S                 0x13        /* 4       3 */
#define Fill_I                           0x14        /* 5       0 */
#define Hit_Writeback_Inv_D              0x15        /* 5       1 */
#define Hit_Writeback_Inv_T              0x16        /* 5       2 */
#define Hit_Writeback_Inv_S              0x17        /* 5       3 */
#define Hit_Writeback_D                  0x19        /* 6       1 */
#define Hit_Writeback_T                  0x1A        /* 6       2 */
#define Hit_Writeback_S                  0x1B        /* 6       3 */
#define Fetch_Lock_I                     0x1C        /* 7       0 */
#define Fetch_Lock_D                     0x1D        /* 7       1 */
#define Fetch_Lock_S                     0x1F        /* 7       3 */

#define C0_CONFIG                        $16
#define C0_CONFIG0                       $16, 0
#define C0_CONFIG1                       $16, 1
#define C0_CONFIG2                       $16, 2
#define C0_CONFIG3                       $16, 3
#define C0_CONFIG4                       $16, 4
#define C0_CONFIG5                       $16, 5

#define  CFG1_IL_SHIFT                   19
#define  CFG1_IL_BITS                    3
#define  CFG1_IS_SHIFT                   22  /* Unless n==7, then 32 */
#define  CFG1_IS_BITS                    3
#define  CFG1_IA_SHIFT                   16
#define  CFG1_IA_BITS                    3
#define  CFG1_DS_SHIFT                   13
#define  CFG1_DS_BITS                    3
#define  CFG1_DL_SHIFT                   10
#define  CFG1_DL_BITS                    3
#define  CFG1_DA_SHIFT                   7
#define  CFG1_DA_BITS                    3

#define  CFG2_SS_SHIFT                   8
#define  CFG2_SS_BITS                    4
#define  CFG2_SL_SHIFT                   4
#define  CFG2_SL_BITS                    4
#define  CFG2_SA_SHIFT                   0
#define  CFG2_SA_BITS                    4

#define  CFG3_M_SHIFT                    31
#define  CFG4_M_SHIFT                    31
#define  CFG5_L2C_SHIFT                  10

/* Global data declaration with size. */

#define EXPORTS(name, sz)     \
  .globl name;                \
  .type name, @object;        \
  .size name, sz;             \
name:

#define DECL(x, val)          \
  EXPORTS(x, 4);              \
  .word  val

#define PTR_SUBU  subu  /* decrement pointer (SGI uses sub) */
#define PTR_ADDU  addu  /* increment pointer (SGI uses add) */
#define PTR_MFC0  mfc0  /* access CP0 pointer width register */
#define PTR_MTC0  mtc0  /* access CP0 pointer width register */

/* Macros to automate cache operations */

#define addr     $t0
#define maxaddr  $t1
#define mask     $t2

#define cacheop(kva, n, linesize, op)          \
  /* check for bad size */                     \
  blez  n, 11f ;                               \
  PTR_ADDU maxaddr, kva, n ;                   \
  /* align to line boundaries */               \
  PTR_SUBU mask, linesize, 1 ;                 \
  not  mask ;                                  \
  and  addr, kva, mask ;                       \
  PTR_SUBU addr, linesize ;                    \
  PTR_ADDU maxaddr, -1 ;                       \
  and  maxaddr, mask ;                         \
  /* the cacheop loop */                       \
10:  PTR_ADDU addr, linesize ;                 \
  cache  op, 0(addr) ;                         \
  bne  addr, maxaddr, 10b ;                    \
11:

/* virtual cache op: no limit on size of region */

#define vcacheop(kva, n, linesize, op)         \
  cacheop(kva, n, linesize, op)

/* indexed cache op: region limited to cache size */

#define icacheop(kva, n, linesize, size, op)   \
    move     $t3, n;                           \
    bltu     n, size, 12f;                     \
    move     $t3, size;                        \
12:  cacheop(kva, $t3, linesize, op)


/* caches may not have been sized yet */

#define SIZE_CACHE(reg, which)                 \
    lw       reg, which;                       \
9:  blez     reg, 9f;                          \
    sync

#define tmp         $t0
#define cfg         $t1
#define icachesize  $t2
#define ilinesize   $t3
#define iways       $ta0
#define dcachesize  $ta1
#define dlinesize   $ta2
#define dways       $ta3
#define scachesize  $t8
#define slinesize   $t9
#define sways       $v0
#define tmp1        $v1
#define tmp2        $a0
#define tmp3        $a1
#define tmp4        $a2
#define tmp5        $a3

.set nomips16

DECL(mips_icache_size, -1)
DECL(mips_icache_linesize, -1)
DECL(mips_icache_ways, 1)

DECL(mips_dcache_size, -1)
DECL(mips_dcache_linesize, -1)
DECL(mips_dcache_ways, 1)

DECL(mips_scache_size, -1)
DECL(mips_scache_linesize, -1)
DECL(mips_scache_ways, 1)

/****************************************************************************
 *
 * void m32_size_cache()
 *
 * Work out size of I, D & S caches (assume already initialised)
 *
 ****************************************************************************/

FUNCTION(m32_size_cache)
    lw       $t0, mips_icache_size
    move     tmp3, $ra
    bgtz     $t0, 8f        # already known?

    bal      _size_cache
    move     $ra, tmp3

8:  # Return
    jr       $ra


/****************************************************************************
 *
 * void m32_clean_icache (unsigned kva, size_t n)
 *
 * Writeback and invalidate address range in instruction caches
 *
 ****************************************************************************/

FUNCTION(m32_clean_icache)
    SIZE_CACHE($a2, mips_icache_linesize)
    vcacheop($a0, $a1, $a2, Hit_Invalidate_I)

    lw       $a2, mips_scache_linesize
    blez     $a2, 9f
    vcacheop($a0, $a1, $a2, Hit_Writeback_Inv_S)
    sync

9:  jr.hb    $ra


/****************************************************************************
 *
 * static void _size_cache()
 *
 * Internal routine to determine cache sizes by looking at config
 * registers.  Sizes are returned in registers, as follows:
 *
 * Do not use tmp3 (reg a1) and tmp1 (reg v1) in this function.
 *
 ****************************************************************************/

FUNCTION(_size_cache)
    # Read $config, 0 to check presence of $config, 1
    mfc0     cfg, C0_CONFIG

    # Read Configuration register, select 1
    mfc0     cfg, C0_CONFIG1

    # Get I-cache line size
    ext      tmp, cfg, CFG1_IL_SHIFT, CFG1_IL_BITS
    beqz     tmp, 8f    # No I-cache

    # Get number of I-cache ways
    ext      iways, cfg, CFG1_IA_SHIFT, CFG1_IA_BITS
    addiu    iways, iways, 1
    move     icachesize, iways

    # Total icache size = lines/way * linesize * ways
    li       ilinesize, 1
    addiu    tmp, tmp, 1
    sllv     ilinesize, ilinesize, tmp
    sllv     icachesize, icachesize, tmp

    # Get I-cache lines per way
    ext      tmp, cfg, CFG1_IS_SHIFT, CFG1_IS_BITS
    addiu    tmp, tmp, 1
    andi     tmp, tmp, 7
    addiu    tmp, tmp, 5
    sllv     icachesize, icachesize, tmp

    # Store icache config
    sw       icachesize, mips_icache_size
    sw       ilinesize, mips_icache_linesize
    sw       iways, mips_icache_ways

8:  # No I-cache, check for D-cache
    ext      tmp, cfg, CFG1_DL_SHIFT, CFG1_DL_BITS
    beqz     tmp, 9f          # No D-cache

    # Get number of dcache ways
    ext      dways, cfg, CFG1_DA_SHIFT, CFG1_DA_BITS
    addiu    dways, dways, 1
    move     dcachesize, dways

    # Total dcache size = lines/way * linesize * ways
    li       dlinesize, 1
    addiu    tmp, tmp, 1
    sllv     dlinesize, dlinesize, tmp
    sllv     dcachesize, dcachesize, tmp

    # Get dcache lines per way
    ext      tmp, cfg, CFG1_DS_SHIFT, CFG1_DS_BITS
    addiu    tmp, tmp, 1
    andi     tmp, tmp, 7
    addiu    tmp, tmp, 5
    sllv     dcachesize, dcachesize, tmp

    # Store dcache config
    sw       dcachesize, mips_dcache_size
    sw       dlinesize, mips_dcache_linesize
    sw       dways, mips_dcache_ways
9:
    LA       tmp, __cache_size_hook
    move     tmp4, $ra
    jal      tmp
    move     $ra, tmp4

    # Return
    jr       $ra

/****************************************************************************
 *
 * static void __cache_size_hook()
 *
 * Internal routine to determine cache sizes by looking at config
 * registers.  Sizing information is stored directly to memory.
 *
 * Do not use tmp3 (reg a1), tmp1 (reg v1) or tmp4 (a2) in this function.
 *
 ****************************************************************************/

FUNCTION(__cache_size_hook)

    # If we are operating with a coherency manager, abort.
    # Check if we have config 5 register present
    mfc0     tmp, C0_CONFIG3
    ext      tmp, tmp, CFG3_M_SHIFT, 1
    beqz     tmp, 2f

    mfc0     tmp, C0_CONFIG4
    ext      tmp, tmp, CFG4_M_SHIFT, 1
    beqz     tmp, 2f

    # Do we have a memory mapped L2 cache config?
    mfc0     tmp, C0_CONFIG5
    ext      tmp, tmp, CFG5_L2C_SHIFT, 1
    beqz     tmp, 2f

    # No CM3 code supplied but we have a memory mapped L2 config
    # Report a Boot failure through UHI
    li       $t9, 23
    # Reason - L2 cache config
    li       $a0, 1
    # Syscall number
    li       $v0, 1
    # Trigger the UHI operation
    syscall  1
    # Should never return
1:
    b        1b

2:  mfc0     cfg, C0_CONFIG2

    # Get scache line size (log2)
    ext      tmp, cfg, CFG2_SL_SHIFT, CFG2_SL_BITS
    beqz     tmp, 3f    # no s-cache
    addiu    tmp, tmp, 1

    # Get number of scache ways
    ext      sways, cfg, CFG2_SA_SHIFT, CFG2_SA_BITS
    addiu    sways, sways, 1
    move     scachesize, sways

    # Total scache size = lines/way * linesize * ways
    li       slinesize, 1
    sllv     slinesize, slinesize, tmp
    sllv     scachesize, scachesize, tmp

    # Get scache lines per way
    ext      tmp, cfg, CFG2_SS_SHIFT, CFG2_SS_BITS
    addiu    tmp, tmp, 6
    sllv     scachesize, scachesize, tmp

    sw       scachesize, mips_scache_size
    sw       slinesize, mips_scache_linesize
    sw       sways, mips_scache_ways
3:
    # Return
    jr       $ra


/****************************************************************************
 *
 * void m32_flush_cache (void)
 *
 * Writeback and invalidate all caches
 *
 ****************************************************************************/

FUNCTION(m32_flush_cache)
    SIZE_CACHE($a1, mips_dcache_size)

    /* writeback and invalidate primary caches individually */
    lw       $a2, mips_dcache_linesize
    li       $a0, KSEG0_BASE
    cacheop($a0, $a1, $a2, Index_Writeback_Inv_D)

9:  lw       $a1, mips_icache_size
    lw       $a2, mips_icache_linesize
    blez     $a1, 9f
    li       $a0, KSEG0_BASE
    cacheop($a0, $a1, $a2, Index_Invalidate_I)

9:  lw       $a1, mips_scache_size
    lw       $a2, mips_scache_linesize
    blez     $a1, 9f
    sync
    li       $a0, KSEG0_BASE
    cacheop($a0, $a1, $a2, Index_Writeback_Inv_S)

9:  sync
    jr.hb    $ra


/****************************************************************************
 *
 * void m32_flush_dcache (void)
 *
 * Writeback and invalidate data caches only
 *
 ****************************************************************************/

FUNCTION(m32_flush_dcache)
    SIZE_CACHE($a1, mips_dcache_size)

    /* writeback and invalidate primary data cache */
    lw       $a2, mips_dcache_linesize
    li       $a0, KSEG0_BASE
    cacheop($a0, $a1, $a2, Index_Writeback_Inv_D)

9:  lw       $a1, mips_scache_size
    lw       $a2, mips_scache_linesize
    blez     $a1, 9f
    sync
    li       $a0, KSEG0_BASE
    cacheop($a0, $a1, $a2, Index_Writeback_Inv_S)

9:  sync
    jr.hb    $ra


/****************************************************************************
 *
 * void m32_flush_icache (void)
 *
 * Writeback and invalidate instruction cache only
 *
 ****************************************************************************/

FUNCTION(m32_flush_icache)
  SIZE_CACHE($a1, mips_icache_size)

    /* writeback and invalidate primary instruction cache */
    lw       $a2, mips_icache_linesize
    li       $a0, KSEG0_BASE
    cacheop($a0, $a1, $a2, Index_Invalidate_I)

9:  lw       $a1, mips_scache_size
    blez     $a1, 9f
    lw       $a2, mips_scache_linesize
    li       $a0, KSEG0_BASE
    cacheop($a0, $a1, $a2, Index_Writeback_Inv_S)

9:  sync
    jr.hb    $ra


/****************************************************************************
 *
 * void m32_clean_cache (unsigned kva, size_t n)
 *
 * Writeback and invalidate address range in all caches
 *
 ****************************************************************************/

FUNCTION(m32_clean_cache)
    SIZE_CACHE($a2, mips_dcache_linesize)
    vcacheop($a0, $a1, $a2, Hit_Writeback_Inv_D)

9:  lw       $a2, mips_icache_linesize
    blez     $a2, 9f
    vcacheop($a0, $a1, $a2, Hit_Invalidate_I)

9:  lw       $a2, mips_scache_linesize
    blez     $a2, 9f

    sync
    vcacheop($a0, $a1, $a2, Hit_Writeback_Inv_S)

9:  sync
    jr.hb    $ra


/****************************************************************************
 *
 * void m32_sync_icache (unsigned kva, size_t n)
 *
 * Synchronise icache and dcache for virtual address range
 *
 ****************************************************************************/

FUNCTION(m32_sync_icache)
    /* check for bad size */
    PTR_ADDU maxaddr, $a0, $a1
    blez     $a1, 9f

    /* get synci step and skip if not required */
    rdhwr    $a2, $1
    PTR_ADDU maxaddr, -1
    beqz     $a2, 9f

    /* ensure stores complete */
    sync

    /* align to line boundaries */
    PTR_SUBU mask, $a2, 1
    not      mask
    and      addr, $a0, mask
    PTR_SUBU addr, $a2
    and      maxaddr, mask

  /* the cacheop loop */
10: PTR_ADDU addr, $a2
    synci    0(addr)
    bne      addr, maxaddr, 10b

9:  sync
    jr.hb    $ra


/****************************************************************************
 *
 * void m32_dcache_clean_invalidate (unsigned kva, size_t n)
 *
 * Writeback and invalidate address range in data caches
 *
 ****************************************************************************/

FUNCTION(m32_dcache_clean_invalidate)
    SIZE_CACHE($a2, mips_dcache_linesize)
    vcacheop($a0, $a1, $a2, Hit_Writeback_Inv_D)

9:  lw       $a2, mips_scache_linesize
    blez     $a2, 9f
    sync
    vcacheop($a0, $a1, $a2, Hit_Writeback_Inv_S)

9:  sync
    jr.hb    $ra


/****************************************************************************
 *
 * void m32_dcache_invalidate (unsigned kva, size_t n)
 *
 * Invalidate (but don't writeback) address range in data caches
 * XXX Only safe if region is totally cache-line aligned.
 *
 ****************************************************************************/

FUNCTION(m32_dcache_invalidate)
    SIZE_CACHE($a2, mips_dcache_linesize)
    vcacheop($a0, $a1, $a2, Hit_Invalidate_D)

9:  lw       $a2, mips_scache_linesize
    blez     $a2, 9f
    vcacheop($a0, $a1, $a2, Hit_Invalidate_S)

9:  sync
    jr.hb    $ra


/*
 * Cache locking
 *
 * The MIPS32 cache architecture does support per-line cache locking.
 *
 * WARNING: if you lock any cache lines, then don't call the
 * mips_flush_xcache routines, because these will flush the
 * locked data out of the cache too; use only mips_clean_xcache.
 */

/****************************************************************************
 *
 * void m32_lock_dcache (void *data, size_t n)
 *
 * Load and lock a block of data into the d-cache
 *
 ****************************************************************************/

FUNCTION(m32_lock_dcache)
    SIZE_CACHE($a2, mips_dcache_linesize)
    vcacheop($a0, $a1, $a2, Fetch_Lock_D)
    sync
9:  jr.hb    $ra


/****************************************************************************
 *
 * void m32_lock_icache (void *code, size_t n)
 *
 * Load and lock a block of instructions into the i-cache
 *
 ****************************************************************************/

FUNCTION(m32_lock_icache)
    SIZE_CACHE($a2, mips_icache_linesize)
    vcacheop($a0, $a1, $a2, Fetch_Lock_I)
    sync
9:  jr.hb    $ra


/****************************************************************************
 *
 * void m32_lock_scache (void * data, size_t n)
 *
 * Load and lock a block of data into the s-cache
 *
 ****************************************************************************/

FUNCTION(m32_lock_scache)
    SIZE_CACHE($a2, mips_scache_linesize)
    vcacheop($a0, $a1, $a2, Fetch_Lock_S)
    sync
9:  jr.hb    $ra


/****************************************************************************
 *
 * void m32_dcache_clean (unsigned kva, size_t n)
 *
 * Writeback address range in data caches
 *
 ****************************************************************************/

FUNCTION(m32_dcache_clean)
    SIZE_CACHE($a2, mips_dcache_linesize)
    vcacheop($a0, $a1, $a2, Hit_Writeback_D)

9:  lw       $a2, mips_scache_linesize
    blez     $a2, 9f
    vcacheop($a0, $a1, $a2, Hit_Writeback_S)

9:  sync
    jr.hb    $ra

#endif /* CONFIG_ALLOW_BSD_COMPONENTS */


