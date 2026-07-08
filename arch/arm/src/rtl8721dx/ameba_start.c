/****************************************************************************
 * arch/arm/src/rtl8721dx/ameba_start.c
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
 * Description
 *
 * NuttX owns the KM4 image2.  The image2 entry is a NuttX-owned app_start()
 * (arch/arm/src/rtl8721dx/ameba_app_start.c, built with the SDK fwlib
 * include set) that runs ALL OS-independent silicon init (cache, data-flash
 * high-speed, MPU, brown-out, XTAL/OSC, ...) and zeroes BSS, then calls
 * main().  Its Img2EntryFun0 descriptor (also in that file) is what the SDK
 * bootloader jumps to.
 *
 * main() is therefore the hand-off point from silicon bring-up to
 * NuttX: it switches MSP onto a NuttX-owned stack (the idle-thread stack),
 * points VTOR at NuttX's own vector table, configures the FPU, and calls
 * nx_start().  nx_start() never returns, so the SDK FreeRTOS scheduler is
 * never launched.
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
#include "ameba_irq.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Reserved idle-thread stack, owned by NuttX (in NuttX's own .bss).  The ARM
 * full-descending stack grows down from the top of this buffer.
 * g_idle_topstack points at the TOP and is consumed by the scheduler / idle
 * thread bring-up logic.
 */

static uint8_t g_idle_stack[CONFIG_IDLETHREAD_STACKSIZE]
  aligned_data(8);

const uintptr_t g_idle_topstack =
  (uintptr_t)g_idle_stack + CONFIG_IDLETHREAD_STACKSIZE;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* NuttX vector table (armv8-m/arm_vectors.c). */

extern const void * const _vectors[];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void ameba_start_continue(void) noreturn_function used_code;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ameba_start_continue
 *
 * Description:
 *   Runs on the NuttX idle stack (MSP already switched by main()).  The SDK
 *   app_start() has already done all silicon init and zeroed BSS, so here we
 *   only take ownership of the vector table / FPU and enter the scheduler.
 *
 ****************************************************************************/

static void ameba_start_continue(void)
{
  /* Configure the FPU. */

  arm_fpuconfig();

  /* Point the vector table at NuttX's own table (app_start left VTOR
   * pointing at the SDK/ROM table).
   */

  putreg32((uint32_t)_vectors, NVIC_VECTAB);

  /* Perform early serial initialization. */

#ifdef USE_EARLYSERIALINIT
  arm_earlyserialinit();
#endif

  /* Start NuttX.  Never returns. */

  nx_start();

  for (; ; );
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: main
 *
 * Description:
 *   Hand-off from app_start() (which ran the silicon init and zeroed BSS)
 *   into NuttX.  Switches MSP onto the NuttX idle stack as the
 *   very first action, then branches to ameba_start_continue() (which runs
 *   on that new stack).  Declared naked so the compiler emits no prologue
 *   that would touch the (about-to-be-replaced) stack.  Never returns.
 *
 ****************************************************************************/

int main(void) naked_function;
int main(void)
{
  __asm__ __volatile__
  (
    "  movs r0, #0\n"
    "  msr  msplim, r0\n"
    "  ldr  r0, =g_idle_topstack\n"
    "  ldr  r0, [r0]\n"
    "  msr  msp, r0\n"
    "  b    ameba_start_continue\n"
  );
}

/****************************************************************************
 * Name: __start
 *
 * Description:
 *   Reset entry referenced by the NuttX armv8-m vector table (_vectors[1]).
 *   In this port the real image2 entry is Img2EntryFun0 -> app_start() ->
 *   main() (see ameba_app_start.c); __start exists so the vector table links
 *   and falls into the NuttX hand-off should the reset vector ever be taken.
 *
 ****************************************************************************/

void __start(void) naked_function;
void __start(void)
{
  __asm__ __volatile__
  (
    "  b main\n"
  );
}
