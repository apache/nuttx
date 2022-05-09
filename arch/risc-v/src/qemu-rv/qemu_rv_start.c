/****************************************************************************
 * arch/risc-v/src/qemu-rv/qemu_rv_start.c
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
#include "chip.h"

#ifdef CONFIG_BUILD_KERNEL
#  include "qemu_rv_mm_init.h"
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
 * Public Data
 ****************************************************************************/

/* NOTE: g_idle_topstack needs to point the top of the idle stack
 * for CPU0 and this value is used in up_initial_state()
 */

uintptr_t g_idle_topstack = QEMU_RV_IDLESTACK_TOP;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qemu_rv_start
 ****************************************************************************/

void qemu_rv_start(int mhartid)
{
  uint32_t *dest;

  /* Configure FPU */

  riscv_fpuconfig();

  if (mhartid > 0)
    {
      goto cpux;
    }

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (dest = &_sbss; dest < &_ebss; )
    {
      *dest++ = 0;
    }

  showprogress('A');

#ifdef USE_EARLYSERIALINIT
  up_earlyserialinit();
#endif

  showprogress('B');

  /* Do board initialization */

#ifdef CONFIG_ARCH_USE_S_MODE
  /* Initialize the per CPU areas */

  riscv_percpu_add_hart(mhartid);
#endif

  showprogress('C');

#ifdef CONFIG_BUILD_KERNEL
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

#ifdef CONFIG_ARCH_USE_S_MODE
void qemu_rv_start_s(int mhartid)
{
  /* Disable MMU and enable PMP */

  SET_CSR(satp, 0x0);
  SET_CSR(pmpaddr0, 0x3fffffffffffffull);
  SET_CSR(pmpcfg0, 0xf);

  /* Set exception and interrupt delegation for S-mode */

  SET_CSR(medeleg, 0xffff);
  SET_CSR(mideleg, 0xffff);

  /* Allow to write satp from S-mode */

  CLEAR_CSR(mstatus, MSTATUS_TVM);

  /* Set mstatus to S-mode and enable SUM */

  CLEAR_CSR(mstatus, ~MSTATUS_MPP_MASK);
  SET_CSR(mstatus, MSTATUS_MPPS | SSTATUS_SUM);

  /* Set the trap vector for S-mode */

  extern void __trap_vec(void);
  SET_CSR(stvec, (uintptr_t)__trap_vec);

  /* Set mepc to the entry */

  SET_CSR(mepc, (uintptr_t)qemu_rv_start);
  asm volatile("mret");
}
#endif

void riscv_serialinit(void)
{
  up_serialinit();
}
