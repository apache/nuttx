/****************************************************************************
 * boards/tricore/tc4x/a3g-tc4d7-lite/scripts/tricore.lsl
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

#include <nuttx/config.h>

#include "tc1v1_8.lsl"

/* The memory map of tricore.ld for the TASKING locator.
 * CPU0 only: program flash bank A, the data scratchpad RAM and a window of
 * the DLMU of CPU0.
 * The CSA, the interrupt stack and the idle stack are at the top of the
 * data scratchpad RAM, the heap below them.
 */

#define PFLS_BASE       0x80000000
#define PFLS_SIZE       1024k

#define DSPR_BASE     0x70000000
#define DSPR_SIZE     240k

#define DLMU_BASE       0x90020400
#define DLMU_SIZE       171k

#define CSA_SIZE        40k
#define ISTACK_SIZE     CONFIG_ARCH_INTERRUPTSTACK
#define USTACK_SIZE     CONFIG_IDLETHREAD_STACKSIZE

#define CSA0_END        (DSPR_BASE + DSPR_SIZE)
#define CSA0            (CSA0_END - CSA_SIZE)
#define ISTACK0         CSA0
#define ISTACK0_END     (ISTACK0 - ISTACK_SIZE)
#define USTACK0         ISTACK0_END
#define USTACK0_END     (USTACK0 - USTACK_SIZE)

#ifdef CONFIG_ARCH_IRQPRIO
#  define INTTAB_OFFSET 0x20
#else
#  define INTTAB_OFFSET 0x1fe0
#endif

derivative nuttx_tc4x
{
  core tc0
  {
    architecture = TC1V1.8_DPFPU;
  }

  memory pfls
  {
    mau = 8;
    type = rom;
    size = PFLS_SIZE;
    map (dest = bus:tc0:fpi_bus, dest_offset = PFLS_BASE, size = PFLS_SIZE);
  }

  memory dspr
  {
    mau = 8;
    type = ram;
    size = DSPR_SIZE;
    map (dest = bus:tc0:fpi_bus, dest_offset = DSPR_BASE, size = DSPR_SIZE);
  }

  memory dlmu
  {
    mau = 8;
    type = ram;
    size = DLMU_SIZE;
    map (dest = bus:tc0:fpi_bus, dest_offset = DLMU_BASE, size = DLMU_SIZE);
  }
}

processor mpe
{
  derivative = nuttx_tc4x;
}

section_setup mpe:tc0:linear
{
  start_address (symbol = "__start");

  /* The startup code initializes .data and .bss itself, the copy table
   * only has to exist for the locator.
   */

  copytable (align = 4, dest = linear);
}

section_layout mpe:tc0:linear
{
  /* Reset entry at +0, full BIV table at +0x20, single-entry BIV at
   * +0x1fe0, trap table at +0x2000, and text at +0x2100.
   */

  group start (ordered, run_addr = PFLS_BASE)
  {
    select ".text.start";
  }

  group traptab (ordered, run_addr = PFLS_BASE + 0x2000)
  {
    select ".text.traptab";
  }

  group intvec (ordered, run_addr = PFLS_BASE + INTTAB_OFFSET)
  {
    select ".text.inttab";
  }

  group text (ordered, run_addr = PFLS_BASE + 0x2100)
  {
    select ".text*";
    select ".rodata*";
  }

  /* Small data and constants, addressed relative to A0, A1, A8 and A9 */

#include "base_address_groups.lsl"

  group data (ordered, contiguous, align = 4, attributes = rwu, run_addr = mem:mpe:dlmu)
  {
    select ".data*";
  }

  /* The startup code copies .data as one block from flash, so its ROM
   * copies are selected by name to get the same order as in RAM.
   */

  group data_rom (ordered, contiguous, align = 4, load_addr)
  {
    select ".data*";
  }

  group bss (ordered, contiguous, align = 4, attributes = rwu, run_addr = mem:mpe:dlmu)
  {
    select ".bss*";
  }

  group heap_area (ordered, run_addr = DSPR_BASE)
  {
    reserved "heap" (size = USTACK0_END - DSPR_BASE);
  }

  group ustack_area (ordered, run_addr = USTACK0_END)
  {
    reserved "ustack" (size = USTACK_SIZE);
  }

  group istack_area (ordered, run_addr = ISTACK0_END)
  {
    reserved "istack" (size = ISTACK_SIZE);
  }

  group csa_area (ordered, run_addr = CSA0)
  {
    reserved "csa" (size = CSA_SIZE);
  }

  "__TRAPTAB_CPU0" := PFLS_BASE + 0x2000;
#ifdef CONFIG_ARCH_IRQPRIO
  "__INTTAB_CPU0" := PFLS_BASE;
#else
  "__INTTAB_CPU0" := PFLS_BASE + INTTAB_OFFSET;
#endif

  "__a0_mem" := sizeof(group:a0) > 0 ? addressof(group:a0) + 0x8000 : 0;
  "__a1_mem" := sizeof(group:a1) > 0 ? addressof(group:a1) + 0x8000 : 0;
  "__a8_mem" := sizeof(group:a8) > 0 ? addressof(group:a8) + 0x8000 : 0;
  "__a9_mem" := sizeof(group:a9) > 0 ? addressof(group:a9) + 0x8000 : 0;

  "_sdata" = "_lc_gb_data";
  "_edata" = "_lc_ge_data";
  "_data_flash" = "_lc_gb_data_rom";
  "_sbss" = "_lc_gb_bss";
  "_ebss" = "_lc_ge_bss";
  "_sheap" := DSPR_BASE;
  "_eheap" := USTACK0_END;
  "__ustack" := USTACK0;
  "__istack" := ISTACK0;
  "__csa_begin" := CSA0;
  "__csa_end" := CSA0_END;

  /* Boot stack and single boot CSA in the PSPR, which the startup code
   * uses to disable the virtualization before it links the CSAs
   */

  "__psram_ustack" := 0x7010eb00;
  "__psram_boot_csa" := 0x7010ec00;
  "__psram_boot_csa_end" := 0x7010fc00;

  /* The other cores start at the program flash of their own image */

  "__start1" := 0x80200000;
  "__start2" := 0x80580000;
  "__start3" := 0x80680000;
  "__start4" := 0x807e0000;
  "__start5" := 0x8092c000;
}
