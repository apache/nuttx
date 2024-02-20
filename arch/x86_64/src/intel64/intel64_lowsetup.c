/****************************************************************************
 * arch/x86_64/src/intel64/intel64_lowsetup.c
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

#include <nuttx/arch.h>

#include "x86_64_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* The actual address of the page table and gdt/ist after mapping the kernel
 * in high address.
 */

volatile uint64_t *g_pdpt;
volatile uint64_t *g_pd;
volatile uint64_t *g_pt;

volatile struct ist_s       *g_ist64;
volatile struct gdt_entry_s *g_gdt64;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: intel64_lowsetup
 *
 * Description:
 *   Called from __nxstart BEFORE starting the operating system in order
 *   perform any necessary, early initialization.
 *
 ****************************************************************************/

void intel64_lowsetup(void)
{
  /* we should be in long mode at this point */

  /* GDT is loaded with 64bit GDT  */

  /* Paging is enabled */

  /* Setup pointers for accessing Page table and GDT in high address */

  g_pdpt = (uint64_t *)((uintptr_t)&g_pdpt_low + X86_64_LOAD_OFFSET);
  g_pd   = (uint64_t *)((uintptr_t)&g_pd_low   + X86_64_LOAD_OFFSET);
  g_pt   = (uint64_t *)((uintptr_t)&g_pt_low   + X86_64_LOAD_OFFSET);

  g_ist64 = (struct ist_s *)((uintptr_t)&g_ist64_low +
                             X86_64_LOAD_OFFSET);
  g_gdt64 = (struct gdt_entry_s *)((uintptr_t)&g_gdt64_low +
                                   X86_64_LOAD_OFFSET);

  /* reload the GDTR with mapped high memory address */

  setgdt((void *)g_gdt64, (uintptr_t)(&g_gdt64_low_end - &g_gdt64_low) - 1);

  /* Revoke the lower memory */

  __revoke_low_memory();
}
