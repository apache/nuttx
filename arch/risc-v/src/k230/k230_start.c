/****************************************************************************
 * arch/risc-v/src/k230/k230_start.c
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

#include <debug.h>
#include <nuttx/init.h>
#include <nuttx/arch.h>
#include <nuttx/serial/uart_16550.h>
#include <nuttx/serial/uart_rpmsg.h>
#include <arch/board/board.h>

#include "riscv_internal.h"
#include "chip.h"

#ifdef CONFIG_BUILD_PROTECTED
#  include "k230_userspace.h"
#endif

#ifdef CONFIG_BUILD_KERNEL
#  include "k230_mm_init.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#define showprogress(c) up_putc(c)
#else
#define showprogress(c)
#endif

#if defined (CONFIG_BUILD_KERNEL) && !defined (CONFIG_ARCH_USE_S_MODE)
#  error "Target requires kernel in S-mode, enable CONFIG_ARCH_USE_S_MODE"
#endif

/****************************************************************************
 * Name: k230_clear_bss
 ****************************************************************************/

static void k230_clear_bss(void)
{
  uint32_t *dest;

  /* Doing this inline just to be sure on the state of global variables. */

  for (dest = (uint32_t *)_sbss; dest < (uint32_t *)_ebss; )
    {
      *dest++ = 0;
    }
}

#ifndef CONFIG_BUILD_KERNEL
/****************************************************************************
 * Name: k230_copy_init_data
 ****************************************************************************/

static void k230_copy_init_data(void)
{
  const uint32_t *src;
  uint32_t *dest;

  /* Move the initialized data from their temporary holding spot at FLASH
   * into the correct place in SRAM.  The correct place in SRAM is given
   * by _sdata and _edata.  The temporary location is in FLASH at the
   * end of all of the other read-only data (.text, .rodata) at _eronly.
   */

  for (src = (const uint32_t *)_eronly,
       dest = (uint32_t *)_sdata; dest < (uint32_t *)_edata;
      )
    {
      *dest++ = *src++;
    }
}
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* NOTE: g_idle_topstack needs to point the top of the idle stack
 * for last CPU and this value is used in up_initial_state()
 */

uintptr_t g_idle_topstack = K230_IDLESTACK_BASE +
                              SMP_STACK_SIZE * CONFIG_SMP_NCPUS;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: k230_start
 ****************************************************************************/

void k230_start(int mhartid, const char *dtb)
{
  if (0 == mhartid)
    {
      k230_clear_bss();

      riscv_set_basestack(K230_IDLESTACK_BASE, SMP_STACK_SIZE);

#ifdef CONFIG_RISCV_PERCPU_SCRATCH
      riscv_percpu_add_hart(mhartid);
#else
      k230_copy_init_data();
#endif
    }

#ifndef CONFIG_BUILD_KERNEL
    k230_hart_init();
#endif

  /* Disable MMU */

  WRITE_CSR(CSR_SATP, 0x0);

  /* Configure FPU */

  riscv_fpuconfig();

  if (mhartid > 0)
    {
      goto cpux;
    }

  showprogress('A');

#ifdef USE_EARLYSERIALINIT
  riscv_earlyserialinit();
#endif

  showprogress('B');

  /* Do board initialization */

#ifdef CONFIG_BUILD_PROTECTED
  k230_userspace();
#endif

#ifdef CONFIG_BUILD_KERNEL
  k230_mm_init();
#endif

  showprogress('C');

  /* Call nx_start() */

  nx_start();

cpux:

#ifdef CONFIG_SMP
  riscv_cpu_boot(mhartid);
#endif

  while (true)
    {
      asm("WFI");
    }
}

void riscv_earlyserialinit(void)
{
#ifdef CONFIG_16550_UART
  u16550_earlyserialinit();
#endif
}

void riscv_serialinit(void)
{
#ifdef CONFIG_16550_UART
  u16550_serialinit();
#endif
}

#ifdef CONFIG_RPMSG_UART_CONSOLE
int up_putc(int ch)
{
  /* place holder for now */

  return ch;
}
#endif
