/****************************************************************************
 * arch/arm/src/efr32/efr32_start.c
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

#include <nuttx/init.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "nvic.h"
#include "chip.h"
#include "efr32_clockconfig.h"
#include "efr32_lowputc.h"
#include "efr32_start.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IDLE_STACK      ((unsigned)&_ebss + CONFIG_IDLETHREAD_STACKSIZE)

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c) arm_lowputc(c)
#else
#  define showprogress(c)
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_ARMV8M_STACKCHECK
/* We need to get r10 set before we can allow instrumentation calls */

void __start(void) noinstrument_function;
#endif

extern const void * const _vectors[];

/****************************************************************************
 * Public Data
 ****************************************************************************/

const uintptr_t g_idle_topstack = IDLE_STACK;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __start
 *
 * Description:
 *   This is the reset entry point.  The core starts in secure state with
 *   SYSCLK running from the HFRCODPLL, set up by the boot ROM.
 *
 ****************************************************************************/

osentry_function
void __start(void)
{
  const uint32_t *src;
  uint32_t *dest;

  /* Make sure that interrupts are disabled and set MSP */

  __asm__ __volatile__ ("CPSID i\n");
  __asm__ __volatile__ ("MSR MSP, %0\n" : : "r" (IDLE_STACK) :);

  /* Make sure that we use MSP from now on */

  __asm__ __volatile__ ("MSR CONTROL, %0\n" : : "r" (0) :);
  __asm__ __volatile__ ("ISB SY\n");

  /* Point VTOR at the NuttX vector table */

  putreg32((uint32_t)_vectors, NVIC_VECTAB);

#ifdef CONFIG_ARMV8M_STACKCHECK
  /* Set the stack limit before we attempt to call any functions */

  __asm__ volatile("sub r10, sp, %0" : :
                   "r"(CONFIG_IDLETHREAD_STACKSIZE - 64) :);
#endif

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (dest = (uint32_t *)_sbss; dest < (uint32_t *)_ebss; )
    {
      *dest++ = 0;
    }

  /* Move the initialized data section from its temporary holding spot in
   * FLASH into the correct place in SRAM.  The correct place in SRAM is
   * given by _sdata and _edata.  The temporary location is in FLASH at the
   * end of all of the other read-only data (.text, .rodata) at _eronly.
   */

  for (src = (const uint32_t *)_eronly,
       dest = (uint32_t *)_sdata; dest < (uint32_t *)_edata;
      )
    {
      *dest++ = *src++;
    }

#ifdef CONFIG_ARCH_RAMFUNCS
  for (src = (const uint32_t *)_framfuncs,
       dest = (uint32_t *)_sramfuncs; dest < (uint32_t *)_eramfuncs;
      )
    {
      *dest++ = *src++;
    }
#endif

#ifdef CONFIG_ARMV8M_STACKCHECK
  arm_stack_check_init();
#endif

  /* Clocks (HFXO and DPLL) and the FPU */

  efr32_clockconfig();
  arm_fpuconfig();

  /* Configure the console UART so we can get debug output */

  efr32_lowsetup();

  showprogress('A');

#ifdef USE_EARLYSERIALINIT
  arm_earlyserialinit();
#endif

  showprogress('B');

  /* Do board initialization */

  efr32_boardinitialize();

  showprogress('C');

  /* Then start NuttX */

  nx_start();

  /* Shouldn't get here */

  for (; ; );
}
