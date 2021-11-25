/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_start.c
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
#include "mpfs_userspace.h"
#include "riscv_arch.h"

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

/* g_idle_topstack: _sbss is the start of the BSS region as defined by the
 * linker script. _ebss lies at the end of the BSS region. The idle task
 * stack starts at the end of BSS and is of size CONFIG_IDLETHREAD_STACKSIZE.
 * The IDLE thread is the thread that the system boots on and, eventually,
 * becomes the IDLE, do nothing task that runs only when there is nothing
 * else to run.  The heap continues from there until the end of memory.
 * g_idle_topstack is a read-only variable the provides this computed
 * address.
 */

uintptr_t g_idle_topstack = MPFS_IDLESTACK_TOP;
volatile bool g_serial_ok = false;

extern void mpfs_cpu_boot(uint32_t);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __mpfs_start
 ****************************************************************************/

void __mpfs_start(uint32_t mhartid)
{
  const uint32_t *src;
  uint32_t *dest;

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (dest = &_sbss; dest < &_ebss; )
    {
      *dest++ = 0;
    }

  /* Move the initialized data section from his temporary holding spot in
   * FLASH into the correct place in SRAM.  The correct place in SRAM is
   * give by _sdata and _edata.  The temporary location is in FLASH at the
   * end of all of the other read-only data (.text, .rodata) at _eronly.
   */

  for (src = &_eronly, dest = &_sdata; dest < &_edata; )
    {
      *dest++ = *src++;
    }

  /* Setup PLL if not already provided */

#ifdef CONFIG_MPFS_BOOTLOADER
  mpfs_clockconfig();
#endif

  /* Configure the UART so we can get debug output */

  mpfs_lowsetup();

  showprogress('A');

#ifdef USE_EARLYSERIALINIT
  riscv_earlyserialinit();
#endif

#ifdef CONFIG_MPFS_DDR_INIT
  mpfs_ddr_init();
#endif

  showprogress('B');

  g_serial_ok = true;

  /* Do board initialization */

  mpfs_boardinitialize();

  /* Initialize the caches.  Should only be executed from E51 (hart 0) to be
   * functional.  Consider the caches already configured if running without
   * the CONFIG_MPFS_BOOTLOADER -option.
   */

#ifdef CONFIG_MPFS_BOOTLOADER
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

  /* Call nx_start() */

  nx_start();

  showprogress('a');

  while (true)
    {
      asm("WFI");
    }
}
