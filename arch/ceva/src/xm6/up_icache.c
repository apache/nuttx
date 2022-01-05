/****************************************************************************
 * arch/ceva/src/xm6/up_icache.c
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

#include <nuttx/cache.h>
#include <nuttx/irq.h>

#include "cpm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MSS_PCR                                 0x0404
#define P_ADD0_ATT0                             0x0418
#define P_CCOSAR                                0x0514
#define P_CCOCR                                 0x0518
#define MSS_HDCFG                               0x061c

#define MSS_PCR_CAC_PFE                         0x00000004

#define P_ADD0_ATT0_L1IC                        0x00000001
#define P_ADD0_ATT0_L1IC_LOCK                   0x00000002

#define P_CCOCR_FLUSH                           0x00000001
#define P_CCOCR_L1ICO                           0x00000002
#define P_CCOCR_OT_PREFETCH                     0x00000004
#define P_CCOCR_OT_LOCK                         0x00000008
#define P_CCOCR_OT_UNLOCK                       0x0000000c
#define P_CCOCR_OT_INVALIDATE                   0x00000010
#define P_CCOCR_OT_MASK                         0x0000003c
#define P_CCOCR_OS_ENTIRE                       0x00000080
#define P_CCOCR_QFILL_SHIFT                     12
#define P_CCOCR_QFILL_MASK                      0x00007000
#define P_CCOCR_OF                              0x00008000
#define P_CCOCR_NOBPL_SHIFT                     16
#define P_CCOCR_NOBPL_MASK                      0xffff0000

#define MSS_HDCFG_PCAC_SZE_0KB                  0x00000000
#define MSS_HDCFG_PCAC_SZE_32KB                 0x00001000
#define MSS_HDCFG_PCAC_SZE_64KB                 0x00002000
#define MSS_HDCFG_PCAC_SZE_128KB                0x00003000
#define MSS_HDCFG_PCAC_SZE_MASK                 0x00007000

#define MSS_CACHE_BLOCK_SIZE                    64

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_CEVA_ICACHE
static void maintain_icache_all(uint32_t op)
{
  irqstate_t flags;

  /* Disable irq */

  flags = up_irq_save();

  /* Start the operation on the entire cache */

  putcpm(P_CCOCR, P_CCOCR_L1ICO | op | P_CCOCR_OS_ENTIRE);

  while (getcpm(P_CCOCR) & P_CCOCR_L1ICO)
    {
      /* Loop until the operation finish */;
    }

  /* Restore irq */

  up_irq_restore(flags);
}

static void maintain_icache(uint32_t op, uintptr_t start, uintptr_t end)
{
  static size_t op_maxblocks;

  /* Initialize op_maxblocks if not yet */

  if (op_maxblocks == 0)
    {
      switch (getcpm(MSS_HDCFG) & MSS_HDCFG_PCAC_SZE_MASK)
        {
        case MSS_HDCFG_PCAC_SZE_32KB:
          op_maxblocks = 512;
          break;
        case MSS_HDCFG_PCAC_SZE_64KB:
          op_maxblocks = 1024;
          break;
        case MSS_HDCFG_PCAC_SZE_128KB:
          op_maxblocks = 2048;
          break;
        default:
          op_maxblocks = 1;
          break;
        }
    }

  /* Align the address to the cache block boundary */

  start &= ~(MSS_CACHE_BLOCK_SIZE - 1);
  end   +=  (MSS_CACHE_BLOCK_SIZE - 1);
  end   &= ~(MSS_CACHE_BLOCK_SIZE - 1);

  /* Skip dtcm since it never put into dcache */

  if (end > CONFIG_ARCH_ITCM_SIZE)
    {
      if (start < CONFIG_ARCH_ITCM_SIZE)
        {
          start = CONFIG_ARCH_ITCM_SIZE;
        }

      while (start < end)
        {
          irqstate_t flags;
          size_t op_blocks;

          /* Get the max blocks we can do in one iteration */

          op_blocks = (end - start) / MSS_CACHE_BLOCK_SIZE;
          if (op_blocks > op_maxblocks)
            {
              op_blocks = op_maxblocks;
            }

          /* Disable irq */

          flags = up_irq_save();

          /* Set the cache address */

          putcpm(P_CCOSAR, start);

          /* Start the cache operation */

          putcpm(P_CCOCR, /* Address based operation */
            P_CCOCR_L1ICO | op | (op_blocks << P_CCOCR_NOBPL_SHIFT));

          while (getcpm(P_CCOCR) & P_CCOCR_L1ICO)
            {
              /* Loop until the operation finish */;
            }

          /* Restore irq */

          up_irq_restore(flags);

          /* Prepare the next loop */

          start += op_blocks * MSS_CACHE_BLOCK_SIZE;
        }
    }
}

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

void up_enable_icache(void)
{
  /* Invalidate the entire icache */

  maintain_icache_all(P_CCOCR_OT_INVALIDATE);

  /* Enable prefetch */

  modifycpm(MSS_PCR, 0, MSS_PCR_CAC_PFE);

  /* Enable icache and disable lock */

  modifycpm(P_ADD0_ATT0, P_ADD0_ATT0_L1IC_LOCK, P_ADD0_ATT0_L1IC);
}

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

void up_disable_icache(void)
{
  /* Disable icache */

  modifycpm(P_ADD0_ATT0, P_ADD0_ATT0_L1IC, 0);

  /* Invalidate the entire icache */

  maintain_icache_all(P_CCOCR_OT_INVALIDATE);
}

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

void up_invalidate_icache(uintptr_t start, uintptr_t end)
{
  maintain_icache(P_CCOCR_OT_INVALIDATE, start, end);
}

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

void up_invalidate_icache_all(void)
{
  maintain_icache_all(P_CCOCR_OT_INVALIDATE);
}

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

void up_coherent_dcache(uintptr_t addr, size_t len)
{
  /* Invalidate instruction cache is enough */

  up_invalidate_icache(addr, addr + len);
}
#endif
