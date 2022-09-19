/****************************************************************************
 * arch/risc-v/src/k210/k210_start.c
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

#include "riscv_internal.h"
#include "k210_clockconfig.h"
#include "k210_userspace.h"
#include "k210.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c) riscv_lowputc(c)
#else
#  define showprogress(c)
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* NOTE: g_idle_topstack needs to point the top of the idle stack
 * for CPU0 and this value is used in up_initial_state()
 */

uintptr_t g_idle_topstack = K210_IDLESTACK0_TOP;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __k210_start
 ****************************************************************************/

void __k210_start(uint32_t mhartid)
{
  const uint32_t *src;
  uint32_t *dest;

  /* Configure FPU */

  riscv_fpuconfig();

  if (0 < mhartid)
    {
      goto cpu1;
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

  /* Setup PLL */

  k210_clockconfig();

  /* Configure the UART so we can get debug output */

  k210_lowsetup();

  showprogress('A');

#ifdef USE_EARLYSERIALINIT
  riscv_earlyserialinit();
#endif

  showprogress('B');

  /* Do board initialization */

  k210_boardinitialize();

  showprogress('C');

  /* For the case of the separate user-/kernel-space build, perform whatever
   * platform specific initialization of the user memory is required.
   * Normally this just means initializing the user space .data and .bss
   * segments.
   */

#ifdef CONFIG_BUILD_PROTECTED
  k210_userspace();
  showprogress('D');
#endif

  /* Call nx_start() */

  nx_start();

cpu1:

  showprogress('a');

#if defined(CONFIG_SMP) && (CONFIG_SMP_NCPUS == 2)
  riscv_cpu_boot(mhartid);
#endif

  while (true)
    {
      asm("WFI");
    }
}
