/****************************************************************************
 * arch/risc-v/src/qemu-rv/qemu_rv_start.c
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
#include <nuttx/serial/uart_16550.h>
#include <arch/board/board.h>

#include <debug.h>
#include "riscv_internal.h"
#include "riscv_sbi.h"
#include "chip.h"

#include "qemu_rv_userspace.h"

#ifdef CONFIG_BUILD_KERNEL
#  include "qemu_rv_mm_init.h"
#endif

#ifdef CONFIG_DEVICE_TREE
#  include <nuttx/fdt.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_NUTTSBI_LATE_INIT
#  define NAPOT_RWX     (PMPCFG_A_NAPOT | PMPCFG_RWX_MASK)
#  define NAPOT_RW      (PMPCFG_A_NAPOT | PMPCFG_R | PMPCFG_W)
#  define SIZE_HALF     (UINT32_C(1) << 31)
#endif

#ifdef CONFIG_DEBUG_FEATURES
#define showprogress(c) up_putc(c)
#else
#define showprogress(c)
#endif

#if defined (CONFIG_BUILD_KERNEL) && !defined (CONFIG_ARCH_USE_S_MODE)
#  error "Target requires kernel in S-mode, enable CONFIG_ARCH_USE_S_MODE"
#endif

/****************************************************************************
 * Extern Function Declarations
 ****************************************************************************/

#ifdef CONFIG_ARCH_USE_S_MODE
extern void __start(void);
#endif

/****************************************************************************
 * Name: qemu_rv_clear_bss
 ****************************************************************************/

static void qemu_rv_clear_bss(void)
{
  uint32_t *dest;

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (dest = (uint32_t *)_sbss; dest < (uint32_t *)_ebss; )
    {
      *dest++ = 0;
    }
}

#ifdef CONFIG_BUILD_PROTECTED
static void qemu_rv_copy_data(void)
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

#ifdef CONFIG_ARCH_USE_S_MODE
static void qemu_boot_secondary(int mhartid, uintptr_t dtb)
{
  int i;

  for (i = 0; i < CONFIG_SMP_NCPUS; i++)
    {
      if (i == mhartid)
        {
          continue;
        }

#ifndef CONFIG_NUTTSBI
      riscv_sbi_boot_secondary(i, (uintptr_t)&__start, dtb);
#else
      sinfo("TBD\n");
#endif
    }
}
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ARCH_USE_S_MODE
static bool boot_secondary = false;
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qemu_rv_start
 ****************************************************************************/

void qemu_rv_start(int mhartid, const char *dtb)
{
#ifdef CONFIG_ARCH_USE_S_MODE
  /* Boot other cores */

  if (!boot_secondary)
    {
      boot_secondary = true;
      qemu_boot_secondary(mhartid, (uintptr_t)dtb);
    }
#endif

  /* Configure FPU */

  riscv_fpuconfig();

  if (mhartid > 0)
    {
      goto cpux;
    }

  qemu_rv_clear_bss();

#ifdef CONFIG_BUILD_PROTECTED
  qemu_rv_copy_data();
#endif

#ifdef CONFIG_RISCV_PERCPU_SCRATCH
  riscv_percpu_add_hart(mhartid);
#endif

#ifdef CONFIG_DEVICE_TREE
  fdt_register(dtb);
#endif

  showprogress('A');

#ifdef USE_EARLYSERIALINIT
  riscv_earlyserialinit();
#endif

  showprogress('B');

  /* Do board initialization */

  showprogress('C');

  /* For the case of the separate user-/kernel-space build, perform whatever
   * platform specific initialization of the user memory is required.
   * Normally this just means initializing the user space .data and .bss
   * segments.
   */

#ifdef CONFIG_BUILD_PROTECTED
  qemu_rv_userspace();
  showprogress('D');
#endif

#ifdef CONFIG_BUILD_KERNEL
  /* Setup page tables for kernel and enable MMU */

  qemu_rv_mm_init();
#endif

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

#ifdef CONFIG_NUTTSBI_LATE_INIT
void sbi_late_initialize(void)
{
  /* QEMU 6.2 doesn't support 0 size, so we do it explicitly here */

  riscv_append_pmp_region(NAPOT_RW, 0, SIZE_HALF);
  riscv_append_pmp_region(NAPOT_RWX, SIZE_HALF, SIZE_HALF);
}
#endif
