/************************************************************************************
 * arch/arm/src/armv7-a/chip/arm-l2cc_pl310.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Reference: "CoreLink™ Level 2 Cache Controller L2C-310", Revision r3p2,
 *   Technical Reference Manual, ARM DDI 0246F (ID011711), ARM
 *
 * NOTE: This logic is incompatible with older versions of the PL310!
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

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <arch/irq.h>

#include "up_arch.h"
#include "l2cc.h"
#include "l2cc_pl310.h"

#ifdef CONFIG_ARMV7A_L2CC_PL310

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/
/* Configuration ***********************************************************/
/* Number of ways depends on ARM configuration */

#if defined(CONFIG_ARMV7A_ASSOCIATIVITY_8WAY)
#  define PL310_NWAYS      8
#  define PL310_WAY_MASK   0x000000ff
#elif defined(CONFIG_ARMV7A_ASSOCIATIVITY_8WAY)
#  define PL310_NWAYS 16
#  define PL310_WAY_MASK   0x0000ffff
#else
#  error "Number of ways not selected"
#endif

/* The size of one depends on ARM configuration */

#if defined(CONFIG_ARMV7A_WAYSIZE_16KB)
#  define PL310_WAYSIZE (16*1024)
#elif defined(CONFIG_ARMV7A_WAYSIZE_32KB)
#  define PL310_WAYSIZE (32*1024)
#elif defined(CONFIG_ARMV7A_WAYSIZE_64KB)
#  define PL310_WAYSIZE (64*1024)
#elif defined(CONFIG_ARMV7A_WAYSIZE_128KB)
#  define PL310_WAYSIZE (128*1024)
#elif defined(CONFIG_ARMV7A_WAYSIZE_256KB)
#  define PL310_WAYSIZE (256*1024)
#elif defined(CONFIG_ARMV7A_WAYSIZE_512KB)
#  define PL310_WAYSIZE (512*1024)
#else
#  error "Way size not selected"
#endif

/* The size of the cache is then the product of the number of ways times
 * the size of each way.
 */

#define PL310_CACHE_SIZE           (PL310_NWAYS * PL310_WAYSIZE)

/* Use for aligning addresses to a cache line boundary */

#define PL310_CACHE_LINE_MASK      (PL310_CACHE_LINE_SIZE - 1)

/* Configurable options
 *
 * REVISIT: Currently there are not configuration options.  All values
 * are just set to the default.
 */

/* Bit 0:  Full line zero enable
 *
 * Default:  0=Full line of write zero behavior disabled
 */

#define L2CC_ACR_FLZE_CONFIG       (0)  /* 0=Full line of write zero behavior disabled */

/* Bit 10: High Priority for SO and Dev Reads Enable
 *
 * Default: 0=Strongly Ordered and Device reads have lower priority than
 *          cacheable accesses
 */

#define L2CC_ACR_HPSO_CONFIG       (0) /* 0=Have lower priority than cache */

/* Bit 11: Store Buffer Device Limitation Enable
 *
 * Default: 0=Store buffer device limitation disabled
 */

#define L2CC_ACR_SBDLE_CONFIG     (0) /* 0=Store buffer device limitation disabled */

/* Bit 12: Exclusive Cache Configuration
 *
 * Default: 0=Disabled
 */

#define L2CC_ACR_EXCC_CONFIG       (0) /* 0=Disabled */

/* Bit 13: Shared Attribute Invalidate Enable
 *
 * Default: 0=Shared invalidate behavior disabled
 */

#define L2CC_ACR_SAIE_CONFIG       (0) /* 0=Shared invalidate behavior disabled */

/* Bit 20: Event Monitor Bus Enable
 *
 * Default: 0=Disabled
 */

#define L2CC_ACR_EMBEN_CONFIG      (0) /* 0=Disabled */

/* Bit 21: Parity Enable
 *
 * Default: 0=Disabled
 */

#define L2CC_ACR_PEN_CONFIG        (0) /* 0=Disabled */

/* Bit 22: Shared Attribute Override Enable
 *
 * Default: 0=Treats shared accesses as specified in the TRM
 */

#define L2CC_ACR_SAOEN_CONFIG      (0) /* 0=As specified in the TRM */

/* Bits 23-24:  Force Write Allocate
 *
 * Default: 0=Use AWCACHE attributes for WA
 */

#define L2CC_ACR_FWA_CONFIG        L2CC_ACR_FWA_AWCACHE /* Use AWCACHE attributes for WA */

/* Bit 25: Cache Replacement Policy
 *
 * Default: 1=Round robin replacement policy
 */

#define L2CC_ACR_CRPOL_CONFIG      L2CC_ACR_CRPOL /* 1=Round robin replacement policy */

/* Bit 26: Non-Secure Lockdown Enable
 *
 * Default: 0=Lockdown registers cannot be modified using non-secure acceses
 */

#define L2CC_ACR_NSLEN_CONFIG      (0) /* 0=Secure access only */

/* Bit 27: Non-Secure Interrupt Access Control
 *
 * Default: 0=Interrupt Clear and Mask can only be modified or read with
 *          secure accesses
 */

#define L2CC_ACR_NSIAC_CONFIG      (0) /* 0=Secure access only */

/* Bit 28: Data Prefetch Enable
 *
 * Default: 0=Data prefetching disabled
 */

#define L2CC_ACR_DPEN_CONFIG       (0) /* 0=Data prefetching disabled */

/* Bit 29: Instruction Prefetch Enable
 *
 * Default: 0=Instruction prefetching disabled
 */

#define L2CC_ACR_IPEN_CONFIG       (0) /* 0=Instruction prefetching disabled */

/* Bit 30: Early BRESP enable
 *
 * Default: 0=Early BRESP disabled
 */

#define L2CC_ACR_EBRESP_CONFIG     (0) /* 0=Early BRESP disabled */

#define L2CC_ACR_CONFIG \
  (L2CC_ACR_FLZE_CONFIG  | L2CC_ACR_HPSO_CONFIG  | L2CC_ACR_SBDLE_CONFIG | \
   L2CC_ACR_EXCC_CONFIG  | L2CC_ACR_SAIE_CONFIG  | L2CC_ACR_EMBEN_CONFIG | \
   L2CC_ACR_PEN_CONFIG   | L2CC_ACR_SAOEN_CONFIG | L2CC_ACR_FWA_CONFIG   | \
   L2CC_ACR_CRPOL_CONFIG | L2CC_ACR_NSLEN_CONFIG | L2CC_ACR_NSIAC_CONFIG | \
   L2CC_ACR_DPEN_CONFIG  | L2CC_ACR_IPEN_CONFIG  | L2CC_ACR_EBRESP_CONFIG)

#define L2CC_ACR_ALLCONFIGS        (0x7f303c01)
#define L2CC_ACR_CONFIGMASK        (L2CC_ACR_SBZ | L2CC_ACR_ALLCONFIGS)

/* Filter end address */

#define CONFIG_PL310_FLEND         (CONFIG_PL310_FLSTRT + CONFIG_PL310_FLSIZE)

/* Block size.  Used to break up long operations so that interrupts are not
 * disabled for a long time.
 */

#define PL310_GULP_SIZE            4096

/* Misc commoly defined and re-defined things */

#ifndef MIN
#  define MIN(a,b)                 (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#  define MAX(a,b)                 (((a) > (b)) ? (a) : (b))
#endif

#ifndef OK
#  define OK                       0
#endif

/* Data synchronization barrier */

#define dsb(a) __asm__ __volatile__ ("dsb " #a : : : "memory")

/***************************************************************************
 * Private Functions
 ***************************************************************************/

/***************************************************************************
 * Name: pl310_flush_all
 *
 * Description:
 *   Flush all ways using the Clean Invalidate Way Register (CIWR).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

static void pl310_flush_all(void)
{
  /* Flush all ways by writing the set of ways to be cleaned to the Clean
   * Invalidate Way Register (CIWR).
   */

  putreg32(PL310_WAY_MASK, L2CC_CIWR);

  /* Wait for cache operation by way to complete */

  while ((getreg32(L2CC_CIWR) & PL310_WAY_MASK) != 0);

  /* Drain the STB. Operation complete when all buffers, LRB, LFB, STB, and
   * EB, are empty.
   */

  putreg32(0, L2CC_CSR);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/***************************************************************************
 * Name: up_l2ccinitialize
 *
 * Description:
 *   One time configuration of the L2 cache.  The L2 cache will be enabled
 *   upon return.
 *
 * Input Parameters:
 *   None.  The L2 cache configuration is controlled by configuration
 *   settings.
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

void up_l2ccinitialize(void)
{
  uint32_t regval;
  int i;

  /* Make sure that this is a PL310 cache, version r3p2.
   *
   * REVISIT: The SAMA5D4 is supposed to report its ID as 0x410000C8 which is
   * r3p2, but the chip that I have actually* reports 0x410000C9 which is some
   * later revision.
   */

  //DEBUGASSERT((getreg32(L2CC_IDR) & L2CC_IDR_REV_MASK) == L2CC_IDR_REV_R3P2);

  /* Make sure that actual cache configuration agrees with the configured
   * cache configuration.
   */


#if defined(CONFIG_ARMV7A_ASSOCIATIVITY_8WAY)
  DEBUGASSERT((getreg32(L2CC_ACR) & L2CC_ACR_ASS) == 0);
#elif defined(CONFIG_ARMV7A_ASSOCIATIVITY_16WAY)
  DEBUGASSERT((getreg32(L2CC_ACR) & L2CC_ACR_ASS) == 1);
#else
#  error No associativity selected
#endif

#if defined(CONFIG_ARMV7A_WAYSIZE_16KB)
  DEBUGASSERT((getreg32(L2CC_ACR) & L2CC_ACR_WAYSIZE_MASK) == L2CC_ACR_WAYSIZE_16KB);
#elif defined(CONFIG_ARMV7A_WAYSIZE_32KB)
  DEBUGASSERT((getreg32(L2CC_ACR) & L2CC_ACR_WAYSIZE_MASK) == L2CC_ACR_WAYSIZE_32KB);
#elif defined(CONFIG_ARMV7A_WAYSIZE_64KB)
  DEBUGASSERT((getreg32(L2CC_ACR) & L2CC_ACR_WAYSIZE_MASK) == L2CC_ACR_WAYSIZE_64KB);
#elif defined(CONFIG_ARMV7A_WAYSIZE_128KB)
  DEBUGASSERT((getreg32(L2CC_ACR) & L2CC_ACR_WAYSIZE_MASK) == L2CC_ACR_WAYSIZE_128KB);
#elif defined(CONFIG_ARMV7A_WAYSIZE_256KB)
  DEBUGASSERT((getreg32(L2CC_ACR) & L2CC_ACR_WAYSIZE_MASK) == L2CC_ACR_WAYSIZE_256KB);
#elif defined(CONFIG_ARMV7A_WAYSIZE_512KB)
  DEBUGASSERT((getreg32(L2CC_ACR) & L2CC_ACR_WAYSIZE_MASK) == L2CC_ACR_WAYSIZE_512KB);
#else
#  error No way size selected
#endif

  /* L2 configuration can only be changed if the cache is disabled,
   *
   * NOTE: This register access will fail if we are not in secure more.
   */

  if ((getreg32(L2CC_CR) & L2CC_CR_L2CEN) == 0)
    {
#if defined(CONFIG_PL310_TRCR_TSETLAT) && defined(CONFIG_PL310_TRCR_TRDLAT) && \
    defined(CONFIG_PL310_TRCR_TWRLAT)
      /* Configure Tag RAM control */

      regval = ((CONFIG_PL310_TRCR_TSETLAT - 1) << L2CC_TRCR_TSETLAT_SHIFT)
               ((CONFIG_PL310_TRCR_TRDLAT - 1) << L2CC_TRCR_TRDLAT_SHIFT) |
               ((CONFIG_PL310_TRCR_TWRLAT - 1) << L2CC_TRCR_TWRLAT_SHIFT);
      putreg32(regval, L2CC_TRCR);
#endif

#if defined(CONFIG_PL310_DRCR_DSETLAT) && defined(CONFIG_PL310_DRCR_DRDLAT) && \
    defined(CONFIG_PL310_DRCR_DWRLAT)
      /* Configure Data RAM control */

      regval = ((CONFIG_PL310_DRCR_DSETLAT - 1) << L2CC_DRCR_DSETLAT_SHIFT) |
               ((CONFIG_PL310_DRCR_DRDLAT - 1) << L2CC_DRCR_DRDLAT_SHIFT) |
               ((CONFIG_PL310_DRCR_DWRLAT - 1) << L2CC_DRCR_DWRLAT_SHIFT);
      putreg32(regval, L2CC_DRCR);
#endif

#ifdef PL310_ADDRESS_FILTERING
#if defined(CONFIG_PL310_FLSTRT) && defined(CONFIG_PL310_FLSIZE)
      /* Configure the address filter */

      regval = (CONFIG_PL310_FLEND + ~L2CC_FLEND_MASK) & L2CC_FLEND_MASK;
      putreg32(regval, L2CC_FLEND);

      regval = (CONFIG_PL310_FLSTRT & L2CC_FLSTRT_MASK) | L2CC_FLSTRT_ENABLE;
      putreg32(regval | L2X0_ADDR_FILTER_EN, L2CC_FLSTRT);
#endif
#endif

      /* Make sure that the memory is not locked down */

      for (i = 0; i < PL310_NLOCKREGS; i++)
        {
          putreg32(0, L2CC_DLKR(i));
          putreg32(0, L2CC_ILKR(i));
        }

      /* Configure the cache properties */

      regval  = getreg32(L2CC_ACR);
      regval &= ~L2CC_ACR_CONFIGMASK;
      regval |= L2CC_ACR_CONFIG;
      putreg32(regval, L2CC_ACR);

      /* Invalidate and enable the cache */

      l2cc_invalidate_all();
      putreg32(L2CC_CR_L2CEN, L2CC_CR);
    }

  lldbg("(%d ways) * (%d bytes/way) = %d bytes\n",
        PL310_NWAYS, PL310_WAYSIZE, PL310_CACHE_SIZE);
}

/***************************************************************************
 * Name: l2cc_enable
 *
 * Description:
 *    Re-enable the L2CC-P310 L2 cache by setting the enable bit in the
 *    Control Register (CR)
 *
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *    None
 *
 ***************************************************************************/

void l2cc_enable(void)
{
  irqstate_t flags;

  /* Invalidate and enable the cache (must be disabled to do this!) */

  flags = irqsave();
  l2cc_invalidate_all();
  putreg32(L2CC_CR_L2CEN, L2CC_CR);
  irqrestore(flags);
}

/***************************************************************************
 * Name: l2cc_disable
 *
 * Description:
 *    Disable the L2CC-P310 L2 cache by clearing the Control Register (CR)
 *
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *    None
 *
 ***************************************************************************/

void l2cc_disable(void)
{
  irqstate_t flags;

  /* Flush all ways using the Clean Invalidate Way Register (CIWR). */

  flags = irqsave();
  pl310_flush_all();

  /* Disable the L2CC-P310 L2 cache by clearing the Control Register (CR) */

  putreg32(0, L2CC_CR);
  dsb();
  irqrestore(flags);
}

/***************************************************************************
 * Name: l2cc_sync
 *
 * Description:
 *   Drain the STB. Operation complete when all buffers, LRB, LFB, STB, and
 *   EB, are empty.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

void l2cc_sync(void)
{
  irqstate_t flags;

  /* Drain the STB. Operation complete when all buffers, LRB, LFB, STB, and
   * EB, are empty.
   */

  flags = irqsave();
  putreg32(0, L2CC_CSR);
  irqrestore(flags);
}

/***************************************************************************
 * Name: l2cc_invalidate_all
 *
 * Description:
 *   Invalidate all ways using the Invalidate Way Register (IWR).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

void l2cc_invalidate_all(void)
{
  irqstate_t flags;
  uint32_t regval;

  /* Invalidate all ways */

  flags = irqsave();

  /* Disable the L2 cache while we invalidate it */

  regval = getreg32(L2CC_CR);
  l2cc_disable();

  /* Invalidate all ways by writing the bit mask of ways to be invalidated
   * the Invalidate Way Register (IWR).
   */

  putreg32(PL310_WAY_MASK, L2CC_IWR);

  /* Wait for cache operation by way to complete */

  while ((getreg32(L2CC_IWR) & PL310_WAY_MASK) != 0);

  /* Drain the STB. Operation complete when all buffers, LRB, LFB, STB, and
   * EB, are empty.
   */

  putreg32(0, L2CC_CSR);

  /* Then re-enable the L2 cache if it was enabled before */

  putreg32(regval, L2CC_CR);
  irqrestore(flags);
}

/***************************************************************************
 * Name: l2cc_invalidate
 *
 * Description:
 *   Invalidate a range of addresses by writing to the Invalidate Physical
 *   Address Line Register (IPALR) repeatedly.
 *
 * Input Parameters:
 *   startaddr - The first address to be invalidated
 *   endaddr   - The last address to be invalidated
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

void l2cc_invalidate(uintptr_t startaddr, uintptr_t endaddr)
{
  uintptr_t invalsize;
  uintptr_t gulpend;
  irqstate_t flags;

  /* Check if the start address is aligned with a cacheline */

  flags = irqsave();
  if ((startaddr & PL310_CACHE_LINE_MASK) != 0)
    {
      /* No.. align down and flush the cache line by writing the address to
       * the Clean Invalidate Physical Address Line Register (CIPALR).
       */

      startaddr &= ~PL310_CACHE_LINE_MASK;
      putreg32(startaddr, L2CC_CIPALR);

      /* Then start invalidating at the next cache line */

      startaddr += PL310_CACHE_LINE_SIZE;
    }

  /* Check if the end address is aligned with a cache line */

  if ((endaddr & PL310_CACHE_LINE_MASK) != 0)
    {
      /* No.. align down and flush cache line by writing the address to
       * the Clean Invalidate Physical Address Line Register (CIPALR).
       */

      endaddr &= ~PL310_CACHE_LINE_MASK;
      putreg32(endaddr, L2CC_CIPALR);
    }

  irqrestore(flags);

  /* Loop, invalidated the address range by cache line.  Interrupts are re-
   * enabled momentarily every PL310_GULP_SIZE bytes.
   */

  while (startaddr < endaddr)
    {
      /* Get the size of the next gulp of cache lines to invalidate.  We do
       * this in small chunks so that we do not have to keep interrupts
       * disabled throughout the whole flush.
       */

      invalsize = endaddr - startaddr;
      gulpend   = startaddr + MIN(invalsize, PL310_GULP_SIZE);

      /* Disable interrupts and invalidate the gulp */

      flags = irqsave();
      while (startaddr < gulpend)
        {
          /* Invalidate the cache line by writing the address to the
           * Invalidate Physical Address Line Register (IPALR).
           */

          putreg32(startaddr, L2CC_IPALR);

          /* Start of the next cache line */

          startaddr += PL310_CACHE_LINE_SIZE;
        }

      /* Enable interrupts momentarily */

      irqrestore(flags);
    }

  /* Drain the STB. Operation complete when all buffers, LRB, LFB, STB, and
   * EB, are empty.
   */

  flags = irqsave();
  putreg32(0, L2CC_CSR);
  irqrestore(flags);
}

/***************************************************************************
 * Name: l2cc_clean_all
 *
 * Description:
 *   Clean all ways by using the Clean Ways Register (CWR).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

void l2cc_clean_all(void)
{
  irqstate_t flags;

  /* Clean all ways by writing the set of ways to be cleaned to the Clean
   * Ways Register (CWR).
   */

  flags = irqsave();
  putreg32(PL310_WAY_MASK, L2CC_CWR);

  /* Wait for cache operation by way to complete */

  while ((getreg32(L2CC_CWR) & PL310_WAY_MASK) != 0);

  /* Drain the STB. Operation complete when all buffers, LRB, LFB, STB, and
   * EB, are empty.
   */

  putreg32(0, L2CC_CSR);
  irqrestore(flags);
}

/***************************************************************************
 * Name: l2cc_clean
 *
 * Description:
 *   Clean the cache line over a range of addresses uing the Clean Physical
 *   Address Line Register (CPALR) repeatedly.
 *
 * Input Parameters:
 *   startaddr - The first address to be cleaned
 *   endaddr   - The last address to be cleaned
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

void l2cc_clean(uintptr_t startaddr, uintptr_t endaddr)
{
  uintptr_t cleansize;
  uintptr_t gulpend;
  irqstate_t flags;

  /* If the range of addresses to clean is as large or larger the L2 cache,
   * then just clean the whole thing.
   */

  cleansize = endaddr - startaddr;
  if (cleansize >= PL310_CACHE_SIZE)
    {
      l2cc_clean_all();
      return;
    }

  /* Align the starting address to a cache line boundary */

  startaddr &= ~PL310_CACHE_LINE_MASK;

  /* Clean the L2 cache by cache line, enabling interrupts momentarily
   * every PL310_GULP_SIZE bytes.
   */

  while (startaddr < endaddr)
    {
      /* Get the size of the next gulp of cache lines to flush.  We do
       * this in small chunks so that we do not have to keep interrupts
       * disabled throughout the whole flush.
       */

      cleansize = endaddr - startaddr;
      gulpend   = startaddr + MIN(cleansize, PL310_GULP_SIZE);

      /* Disable interrupts and clean the gulp */

      flags = irqsave();
      while (startaddr < gulpend)
        {
          /* Clean the cache line by writing the address to the Clean
           * Physical Address Line Register (CPALR).
           */

          putreg32(startaddr, L2CC_CPALR);

          /* Start of the next cache line */

          startaddr += PL310_CACHE_LINE_SIZE;
        }

      /* Enable interrupts momentarily */

      irqrestore(flags);
    }

  /* Drain the STB. Operation complete when all buffers, LRB, LFB, STB, and
   * EB, are empty.
   */

  flags = irqsave();
  putreg32(0, L2CC_CSR);
  irqrestore(flags);
}

/***************************************************************************
 * Name: l2cc_flush_all
 *
 * Description:
 *   Flush all ways using the Clean Invalidate Way Register (CIWR).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

void l2cc_flush_all(void)
{
  irqstate_t flags;

  /* Flush all ways using the Clean Invalidate Way Register (CIWR). */

  flags = irqsave();
  pl310_flush_all();
  irqrestore(flags);
}

/***************************************************************************
 * Name: l2cc_flush
 *
 * Description:
 *   Flush a range of address by using the Clean Invalidate Physical Address
 *   Line Register (CIPALR) repeatedly.
 *
 * Input Parameters:
 *   startaddr - The first address to be flushed
 *   endaddr   - The last address to be flushed
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

void l2cc_flush(uint32_t startaddr, uint32_t endaddr)
{
  uintptr_t  flushsize;
  uintptr_t  gulpend;
  irqstate_t flags;

  /* If the range of addresses to flush is as large or larger the L2 cache,
   * then just flush the whole thing.
   */

  flushsize = endaddr - startaddr;
  if (flushsize >= PL310_CACHE_SIZE)
    {
      l2cc_flush_all();
      return;
    }

  /* Align the starting address to a cache line boundary */

  startaddr &= ~PL310_CACHE_LINE_MASK;

  /* Flush the L2 cache by cache line, enabling interrupts momentarily
   * every PL310_GULP_SIZE bytes.
   */

  while (startaddr < endaddr)
    {
      /* Get the size of the next gulp of cache lines to flush.  We do
       * this in small chunks so that we do not have to keep interrupts
       * disabled throughout the whole flush.
       */

      flushsize = endaddr - startaddr;
      gulpend   = startaddr + MIN(flushsize, PL310_GULP_SIZE);

      /* Disable interrupts and flush the gulp */

      flags = irqsave();
      while (startaddr < gulpend)
        {
          /* Flush the cache line by writing the address to the Clean
           * Invalidate Physical Address Line Register (CIPALR).
           */

          putreg32(startaddr, L2CC_CIPALR);

          /* Start of the next cache line */

          startaddr += PL310_CACHE_LINE_SIZE;
        }

      /* Enable interrupts momentarily */

      irqrestore(flags);
    }

  /* Drain the STB. Operation complete when all buffers, LRB, LFB, STB, and
   * EB, are empty.
   */

  flags = irqsave();
  putreg32(0, L2CC_CSR);
  irqrestore(flags);
}

#endif /* CONFIG_ARMV7A_L2CC_PL310 */
