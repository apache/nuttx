/****************************************************************************
 * arch/risc-v/src/litex/litex_start.c
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

#include <nuttx/fdt.h>
#include <nuttx/init.h>
#include <arch/board/board.h>

#include "litex_clockconfig.h"
#include "litex.h"
#include "chip.h"

#ifdef CONFIG_BUILD_KERNEL
#  include "litex_mm_init.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c) riscv_lowputc(c)
#else
#  define showprogress(c)
#endif

#if defined (CONFIG_BUILD_KERNEL) && !defined (CONFIG_ARCH_USE_S_MODE)
#  error "Target requires kernel in S-mode, enable CONFIG_ARCH_USE_S_MODE"
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __litex_start
 ****************************************************************************/

void __litex_start(int hart_index, const void * fdt, int arg)
{
  (void)hart_index;
  (void)arg;

  const uint32_t *src;
  uint32_t *dest;

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (dest = (uint32_t *)_sbss; dest < (uint32_t *)_ebss; )
    {
      *dest++ = 0;
    }

  /* Move the initialized data section from his temporary holding spot in
   * FLASH into the correct place in SRAM.  The correct place in SRAM is
   * give by _sdata and _edata.  The temporary location is in FLASH at the
   * end of all of the other read-only data (.text, .rodata) at _eronly.
   */

  /* for vexriscv the full image is loaded in ddr ram */

  for (src = (const uint32_t *)_eronly,
       dest = (uint32_t *)_sdata; dest < (uint32_t *)_edata;
      )
    {
      *dest++ = *src++;
    }

  /* Assume the FDT address was passed in if not NULL */

  if (fdt)
    {
      fdt_register(fdt);
    }
  else
    {
      fdt_register((const char *)CONFIG_LITEX_FDT_MEMORY_ADDRESS);
    }

#ifdef CONFIG_RISCV_PERCPU_SCRATCH
  riscv_percpu_add_hart(0);
#endif

  /* Setup PLL */

  litex_clockconfig();

  /* Configure the UART so we can get debug output */

  litex_lowsetup();

  showprogress('A');

#ifdef USE_EARLYSERIALINIT
  riscv_earlyserialinit();
#endif

  showprogress('B');

  /* Do board initialization */

#ifdef CONFIG_BUILD_KERNEL
  /* Setup page tables for kernel and enable MMU */

  litex_mm_init();
#endif

  showprogress('C');

  /* Call nx_start() */

  nx_start();

  /* Shouldn't get here */

  for (; ; );
}
