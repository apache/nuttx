/****************************************************************************
 * arch/risc-v/src/jh7110/jh7110_start.c
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

#include "riscv_internal.h"
#include "chip.h"
#include "jh7110_mm_init.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#define showprogress(c) up_putc(c)
#else
#define showprogress(c)
#endif

/****************************************************************************
 * Extern Function Declarations
 ****************************************************************************/

extern void __trap_vec(void);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* NOTE: g_idle_topstack needs to point the top of the idle stack
 * for last CPU and this value is used in up_initial_state()
 */

uintptr_t g_idle_topstack = JH7110_IDLESTACK_BASE +
                              SMP_STACK_SIZE * CONFIG_SMP_NCPUS;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: jh7110_clear_bss
 ****************************************************************************/

void jh7110_clear_bss(void)
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

/****************************************************************************
 * Name: jh7110_start
 ****************************************************************************/

void jh7110_start_s(int mhartid)
{
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

  showprogress('C');

  /* Setup page tables for kernel and enable MMU */

  jh7110_mm_init();

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

/****************************************************************************
 * Name: jh7110_start
 ****************************************************************************/

void jh7110_start(int mhartid)
{
  DEBUGASSERT(mhartid == 0); /* Only Hart 0 supported for now */

  if (0 == mhartid)
    {
      jh7110_clear_bss();

      /* Setup base stack */

      riscv_set_basestack(JH7110_IDLESTACK_BASE, SMP_STACK_SIZE);

      /* Initialize the per CPU areas */

      riscv_percpu_add_hart(mhartid);
    }

  /* Disable MMU */

  WRITE_CSR(CSR_SATP, 0x0);

  /* Set the trap vector for S-mode */

  WRITE_CSR(CSR_STVEC, (uintptr_t)__trap_vec);

  /* Start S-mode */

  jh7110_start_s(mhartid);
}

void riscv_earlyserialinit(void)
{
  u16550_earlyserialinit();
}

void riscv_serialinit(void)
{
  u16550_serialinit();
}
