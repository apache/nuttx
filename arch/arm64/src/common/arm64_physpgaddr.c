/****************************************************************************
 * arch/arm64/src/common/arm64_physpgaddr.c
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

#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "arm64_arch.h"
#include "arm64_internal.h"

#ifndef CONFIG_DEV_SIMPLE_ADDRENV

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PAR_EL1, Physical Address Register. Bit 0 reports a failed translation;
 * on success bits [51:12] carry the physical frame.
 */

#define PAR_F                (1ull << 0)
#define PAR_PA_MASK          (0x000ffffffffff000ull)
#define VA_PAGE_OFFSET_MASK  (0xfffull)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_addrenv_va_to_pa
 *
 * Description:
 *   Map a virtual address to its physical address.
 *
 *   The translation is asked of the MMU rather than walked in software, so
 *   it answers for whatever is actually mapped: any granule size, block or
 *   page, at any level, and it cannot drift from the tables in use.
 *
 * Input Parameters:
 *   va - The virtual address to be mapped.
 *
 * Returned Value:
 *   Physical address on success; zero if the address is not mapped for a
 *   privileged read.
 *
 ****************************************************************************/

uintptr_t up_addrenv_va_to_pa(void *va)
{
  irqstate_t flags;
  uint64_t par;

  /* PAR_EL1 is a single register per CPU, so nothing may run between the
   * translation and reading the result or it reads someone else's answer.
   * Interrupts are banked with it, so masking them locally is enough.
   */

  flags = up_irq_save();

  __asm__ volatile ("at s1e1r, %0" : : "r" (va) : "memory");
  UP_ISB();
  par = read_sysreg(par_el1);

  up_irq_restore(flags);

  if ((par & PAR_F) != 0)
    {
      return 0;
    }

  return (uintptr_t)((par & PAR_PA_MASK) |
                     ((uintptr_t)va & VA_PAGE_OFFSET_MASK));
}

#endif /* CONFIG_DEV_SIMPLE_ADDRENV */
