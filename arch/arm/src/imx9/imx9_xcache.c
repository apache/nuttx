/****************************************************************************
 * arch/arm/src/imx9/imx9_xcache.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/cache.h>
#include <arch/barriers.h>

#include "arm_internal.h"
#include "hardware/imx9_xcache.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define XCACHE_LINESIZE_BYTE 16
#define XCACHE_SIZE (16*1024)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xcache_wait_busy
 *
 * Description:
 *   Wait for cache command to complete by polling GO bit
 *
 ****************************************************************************/

static inline void xcache_wait_busy(uintptr_t base)
{
  while (getreg32(base + IMX9_XCACHE_CCR_OFFSET) & XCACHE_CCR_GO)
    {
    }
}

/****************************************************************************
 * Name: xcache_wait_line_busy
 *
 * Description:
 *   Wait for cache line command to complete by polling LGO bit
 *
 ****************************************************************************/

static inline void xcache_wait_line_busy(uintptr_t base)
{
  while (getreg32(base + IMX9_XCACHE_CSAR_OFFSET) & XCACHE_CSAR_LGO)
    {
    }
}

/****************************************************************************
 * Name: xcache_invalidate_all
 *
 * Description:
 *   Invalidate entire cache (both ways)
 *
 ****************************************************************************/

static void xcache_invalidate_all(uintptr_t base)
{
  uint32_t regval;

  /* Invalidate all lines in both ways and initiate command */

  regval = XCACHE_CCR_INVW0 | XCACHE_CCR_INVW1 | XCACHE_CCR_GO;
  putreg32(regval, base + IMX9_XCACHE_CCR_OFFSET);

  /* Wait until command completes */

  xcache_wait_busy(base);

  /* Clear command bits, precaution */

  regval = getreg32(base + IMX9_XCACHE_CCR_OFFSET);
  regval &= ~(XCACHE_CCR_INVW0 | XCACHE_CCR_INVW1);
  putreg32(regval, base + IMX9_XCACHE_CCR_OFFSET);
}

/****************************************************************************
 * Name: xcache_clean_all
 *
 * Description:
 *   Clean (push) entire cache (both ways)
 *
 ****************************************************************************/

static void xcache_clean_all(uintptr_t base)
{
  uint32_t regval;

  /* Push all modified lines in both ways */

  regval = XCACHE_CCR_PUSHW0 | XCACHE_CCR_PUSHW1 | XCACHE_CCR_GO;
  putreg32(regval, base + IMX9_XCACHE_CCR_OFFSET);

  /* Wait until command completes */

  xcache_wait_busy(base);

  /* Clear command bits, precaution */

  regval = getreg32(base + IMX9_XCACHE_CCR_OFFSET);
  regval &= ~(XCACHE_CCR_PUSHW0 | XCACHE_CCR_PUSHW1);
  putreg32(regval, base + IMX9_XCACHE_CCR_OFFSET);
}

/****************************************************************************
 * Name: xcache_clean_invalidate_all
 *
 * Description:
 *   Clean and invalidate entire cache (both ways)
 *
 ****************************************************************************/

static void xcache_clean_invalidate_all(uintptr_t base)
{
  uint32_t regval;

  /* Push and invalidate all */

  regval = XCACHE_CCR_PUSHW0 | XCACHE_CCR_PUSHW1 |
           XCACHE_CCR_INVW0 | XCACHE_CCR_INVW1 | XCACHE_CCR_GO;
  putreg32(regval, base + IMX9_XCACHE_CCR_OFFSET);

  /* Wait until command completes */

  xcache_wait_busy(base);

  /* Clear command bits, precaution */

  regval = getreg32(base + IMX9_XCACHE_CCR_OFFSET);
  regval &= ~(XCACHE_CCR_PUSHW0 | XCACHE_CCR_PUSHW1 |
              XCACHE_CCR_INVW0 | XCACHE_CCR_INVW1);
  putreg32(regval, base + IMX9_XCACHE_CCR_OFFSET);
}

/****************************************************************************
 * Name: xcache_op_by_range
 *
 * Description:
 *   Perform cache operation by address range (line by line)
 *
 * Input Parameters:
 *   base   - XCACHE base address
 *   start  - Start address (will be aligned to cache line)
 *   end    - End address + 1
 *   lcmd   - Line command: 1=invalidate, 2=clean, 3=clean+invalidate
 *
 ****************************************************************************/

static void xcache_op_by_range(uintptr_t base, uintptr_t start,
                                uintptr_t end, uint32_t lcmd)
{
  uint32_t regval;
  uintptr_t addr;

  if (start >= end)
    {
      return;
    }

  /* Align start address to cache line size */

  addr = start & ~(XCACHE_LINESIZE_BYTE - 1);

  /* Set line command and use physical address */

  regval = getreg32(base + IMX9_XCACHE_CLCR_OFFSET);
  regval &= ~XCACHE_CLCR_LCMD_MASK;
  regval |= XCACHE_CLCR_LCMD(lcmd) | XCACHE_CLCR_LADSEL;
  putreg32(regval, base + IMX9_XCACHE_CLCR_OFFSET);

  /* Process each cache line */

  while (addr < end)
    {
      /* Set address and initiate line command */

      regval = (addr & XCACHE_CSAR_PHYADDR_MASK) | XCACHE_CSAR_LGO;
      putreg32(regval, base + IMX9_XCACHE_CSAR_OFFSET);

      /* Wait for completion */

      xcache_wait_line_busy(base);

      addr += XCACHE_LINESIZE_BYTE;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_get_icache_linesize
 ****************************************************************************/

#ifdef CONFIG_IMX9_LPCAC_PC
size_t up_get_icache_linesize(void)
{
  return XCACHE_LINESIZE_BYTE;  /* XCACHE line size is 32 bytes */
}
#endif

/****************************************************************************
 * Name: up_get_icache_size
 ****************************************************************************/

#ifdef CONFIG_IMX9_LPCAC_PC
size_t up_get_icache_size(void)
{
  return XCACHE_SIZE;
}
#endif

/****************************************************************************
 * Name: up_enable_icache
 ****************************************************************************/

#ifdef CONFIG_IMX9_LPCAC_PC
void up_enable_icache(void)
{
  uint32_t regval;

  /* Return if already enabled */

  regval = getreg32(IMX9_LPCAC_PC_BASE + IMX9_XCACHE_CCR_OFFSET);
  if (regval & XCACHE_CCR_ENCACHE)
    {
      return;
    }

  /* First, invalidate the entire cache */

  xcache_invalidate_all(IMX9_LPCAC_PC_BASE);

  /* Now enable the cache */

  regval = getreg32(IMX9_LPCAC_PC_BASE + IMX9_XCACHE_CCR_OFFSET);
  regval |= XCACHE_CCR_ENCACHE;
  putreg32(regval, IMX9_LPCAC_PC_BASE + IMX9_XCACHE_CCR_OFFSET);

  UP_ISB();
}
#endif

/****************************************************************************
 * Name: up_disable_icache
 ****************************************************************************/

#ifdef CONFIG_IMX9_LPCAC_PC
void up_disable_icache(void)
{
  uint32_t regval;

  regval = getreg32(IMX9_LPCAC_PC_BASE + IMX9_XCACHE_CCR_OFFSET);
  if (!(regval & XCACHE_CCR_ENCACHE))
    {
      return;
    }

  /* Disable the cache */

  regval &= ~XCACHE_CCR_ENCACHE;
  putreg32(regval, IMX9_LPCAC_PC_BASE + IMX9_XCACHE_CCR_OFFSET);

  UP_DSB();
  UP_ISB();
}
#endif

/****************************************************************************
 * Name: up_invalidate_icache
 ****************************************************************************/

#ifdef CONFIG_IMX9_LPCAC_PC
void up_invalidate_icache(uintptr_t start, uintptr_t end)
{
  xcache_op_by_range(IMX9_LPCAC_PC_BASE, start, end, XCACHE_LCMD_INVALIDATE);
  UP_ISB();
}
#endif

/****************************************************************************
 * Name: up_invalidate_icache_all
 ****************************************************************************/

#ifdef CONFIG_IMX9_LPCAC_PC
void up_invalidate_icache_all(void)
{
  xcache_invalidate_all(IMX9_LPCAC_PC_BASE);
  UP_ISB();
}
#endif

/****************************************************************************
 * Name: up_get_dcache_linesize
 ****************************************************************************/

#ifdef CONFIG_IMX9_LPCAC_PS
size_t up_get_dcache_linesize(void)
{
  return XCACHE_LINESIZE_BYTE;
}
#endif

/****************************************************************************
 * Name: up_get_dcache_size
 ****************************************************************************/

#ifdef CONFIG_IMX9_LPCAC_PS
size_t up_get_dcache_size(void)
{
  return XCACHE_SIZE;
}
#endif

/****************************************************************************
 * Name: up_enable_dcache
 ****************************************************************************/

#ifdef CONFIG_IMX9_LPCAC_PS
void up_enable_dcache(void)
{
  uint32_t regval;

  /* Return if already enabled */

  regval = getreg32(IMX9_LPCAC_PS_BASE + IMX9_XCACHE_CCR_OFFSET);
  if (regval & XCACHE_CCR_ENCACHE)
    {
      return;
    }

  /* First, invalidate the entire cache */

  xcache_invalidate_all(IMX9_LPCAC_PS_BASE);

  /* Now enable the cache */

  regval = getreg32(IMX9_LPCAC_PS_BASE + IMX9_XCACHE_CCR_OFFSET);
  regval |= XCACHE_CCR_ENCACHE;
  putreg32(regval, IMX9_LPCAC_PS_BASE + IMX9_XCACHE_CCR_OFFSET);

  UP_DSB();
}
#endif

/****************************************************************************
 * Name: up_disable_dcache
 ****************************************************************************/

#ifdef CONFIG_IMX9_LPCAC_PS
void up_disable_dcache(void)
{
  uint32_t regval;

  regval = getreg32(IMX9_LPCAC_PS_BASE + IMX9_XCACHE_CCR_OFFSET);
  if (!(regval & XCACHE_CCR_ENCACHE))
    {
      return;
    }

  /* First, clean any modified contents */

  xcache_clean_all(IMX9_LPCAC_PS_BASE);

  /* Now disable the cache */

  regval &= ~XCACHE_CCR_ENCACHE;
  putreg32(regval, IMX9_LPCAC_PS_BASE + IMX9_XCACHE_CCR_OFFSET);

  UP_DSB();
}
#endif

/****************************************************************************
 * Name: up_invalidate_dcache
 ****************************************************************************/

#ifdef CONFIG_IMX9_LPCAC_PS
void up_invalidate_dcache(uintptr_t start, uintptr_t end)
{
  xcache_op_by_range(IMX9_LPCAC_PS_BASE, start, end, XCACHE_LCMD_INVALIDATE);
  UP_DSB();
}
#endif

/****************************************************************************
 * Name: up_invalidate_dcache_all
 ****************************************************************************/

#ifdef CONFIG_IMX9_LPCAC_PS
void up_invalidate_dcache_all(void)
{
  xcache_invalidate_all(IMX9_LPCAC_PS_BASE);
  UP_DSB();
}
#endif

/****************************************************************************
 * Name: up_clean_dcache
 ****************************************************************************/

#ifdef CONFIG_IMX9_LPCAC_PS
void up_clean_dcache(uintptr_t start, uintptr_t end)
{
  xcache_op_by_range(IMX9_LPCAC_PS_BASE, start, end, XCACHE_LCMD_PUSH);
  UP_DSB();
}
#endif

/****************************************************************************
 * Name: up_clean_dcache_all
 ****************************************************************************/

#ifdef CONFIG_IMX9_LPCAC_PS
void up_clean_dcache_all(void)
{
  xcache_clean_all(IMX9_LPCAC_PS_BASE);
  UP_DSB();
}
#endif

/****************************************************************************
 * Name: up_flush_dcache
 ****************************************************************************/

#ifdef CONFIG_IMX9_LPCAC_PS
void up_flush_dcache(uintptr_t start, uintptr_t end)
{
  xcache_op_by_range(IMX9_LPCAC_PS_BASE, start, end, XCACHE_LCMD_CLEAR);
  UP_DSB();
}
#endif

/****************************************************************************
 * Name: up_flush_dcache_all
 ****************************************************************************/

#ifdef CONFIG_IMX9_LPCAC_PS
void up_flush_dcache_all(void)
{
  xcache_clean_invalidate_all(IMX9_LPCAC_PS_BASE);
  UP_DSB();
}
#endif

/****************************************************************************
 * Name: up_coherent_dcache
 ****************************************************************************/

#if defined(CONFIG_IMX9_LPCAC_PS)
void up_coherent_dcache(uintptr_t addr, size_t len)
{
  /* Clean PS cache and invalidate PC cache for code coherency */

  up_clean_dcache(addr, addr + len);
#if defined(CONFIG_IMX9_LPCAC_PC)
  up_invalidate_icache(addr, addr + len);
#endif
}
#endif