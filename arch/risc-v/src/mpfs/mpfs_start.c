/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_start.c
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

#include <nuttx/init.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "chip.h"
#include "mpfs.h"
#include "mpfs_clockconfig.h"
#include "mpfs_ddr.h"
#include "mpfs_cache.h"
#include "mpfs_mm_init.h"
#include "mpfs_userspace.h"

#include "riscv_internal.h"
#include "riscv_percpu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c) riscv_lowputc(c)
#else
#  define showprogress(c)
#endif

#if defined(CONFIG_BUILD_KERNEL) && !defined(CONFIG_ARCH_USE_S_MODE)
#  error "Target requires kernel in S-mode, enable CONFIG_ARCH_USE_S_MODE"
#endif

#if defined(CONFIG_SMP) && !defined(CONFIG_RISCV_PERCPU_SCRATCH)
#  error "Target requires CONFIG_RISCV_PERCPU_SCRATCH if CONFIG_SMP is set"
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __mpfs_start
 ****************************************************************************/

void __mpfs_start(uint64_t mhartid)
{
  const uint32_t *src;
  uint32_t *dest;

  /* Configure FPU (hart 0 don't have an FPU) */

  if (mhartid != 0)
    {
      riscv_fpuconfig();
    }

  /* CPU 0 handles the boot, the rest wait */

  if (riscv_hartid_to_cpuid(mhartid) != 0)
    {
      goto cpux;
    }

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

  for (src = (const uint32_t *)_eronly,
       dest = (uint32_t *)_sdata; dest < (uint32_t *)_edata;
      )
    {
      *dest++ = *src++;
    }

#ifdef CONFIG_RISCV_PERCPU_SCRATCH
  /* Initialize the per CPU areas */

  if (mhartid != 0)
    {
      riscv_percpu_add_hart(mhartid);
    }
#endif /* CONFIG_RISCV_PERCPU_SCRATCH */

  /* Setup PLL if not already provided */

#ifdef CONFIG_MPFS_CLKINIT
  mpfs_clockconfig();
#endif

  /* Configure the UART so we can get debug output */

  mpfs_lowsetup();

  showprogress('A');

#ifdef USE_EARLYSERIALINIT
  riscv_earlyserialinit();
#endif

#ifdef CONFIG_MPFS_DDR_INIT
  if (mpfs_ddr_init() != 0)
    {
      /* We don't allow booting, ddr training failure will cause random
       * behaviour
       */

      showprogress('X');

      /* Reset, but let the progress come out of the uart first */

      up_udelay(1000);
      up_systemreset();
    }
#endif

  showprogress('B');

  /* Do board initialization */

  mpfs_boardinitialize();

  /* Initialize the caches.  Should only be executed from E51 (hart 0) to be
   * functional.  Consider the caches already configured if running without
   * the CONFIG_MPFS_BOOTLOADER -option.
   */

#ifdef CONFIG_MPFS_ENABLE_CACHE
  if (mhartid == 0)
    {
      mpfs_enable_cache();
    }
#endif

  showprogress('C');

  /* For the case of the separate user-/kernel-space build, perform whatever
   * platform specific initialization of the user memory is required.
   * Normally this just means initializing the user space .data and .bss
   * segments.
   */

#ifdef CONFIG_BUILD_PROTECTED
  mpfs_userspace();
  showprogress('D');
#endif

#ifdef CONFIG_BUILD_KERNEL
  mpfs_mm_init();
#endif

  /* Call nx_start() */

  nx_start();

  showprogress('a');

cpux:

#ifdef CONFIG_SMP
  /* Disable local interrupts */

  up_irq_save();

  /* Initialize local PLIC */

  mpfs_plic_init_hart(mhartid);

  /* Then wait for the boot core to start us */

  riscv_cpu_boot(riscv_hartid_to_cpuid(mhartid));
#endif

  while (true)
    {
      asm("WFI");
    }
}
