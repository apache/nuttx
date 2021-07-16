/****************************************************************************
 * arch/xtensa/src/esp32s2/esp32s2_start.c
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
#include <string.h>

#include <nuttx/init.h>
#include <nuttx/irq.h>

#include "xtensa.h"
#include "xtensa_attr.h"

#include "hardware/esp32s2_rtccntl.h"
#include "esp32s2_clockconfig.h"
#include "esp32s2_region.h"
#include "esp32s2_start.h"
#include "esp32s2_lowputc.h"
#include "esp32s2_wdt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c)     up_lowputc(c)
#else
#  define showprogress(c)
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Address of the CPU0 IDLE thread */

uint32_t g_idlestack[IDLETHREAD_STACKWORDS]
  __attribute__((aligned(16), section(".noinit")));

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __start
 *
 * Description:
 *   We arrive here after the bootloader finished loading the program from
 *   flash. The hardware is mostly uninitialized, and the app CPU is in
 *   reset. We do have a stack, so we can do the initialization in C.
 *
 ****************************************************************************/

void IRAM_ATTR __start(void)
{
  uint32_t *dest;
  uint32_t regval;
  uint32_t sp;

  /* Disable any wdt enabled by bootloader */

  esp32s2_wdt_early_deinit();

  regval  = getreg32(DR_REG_BB_BASE + 0x48); /* DR_REG_BB_BASE+48 */
  regval &= ~(1 << 14);
  putreg32(regval, DR_REG_BB_BASE + 0x48);

  /* Make sure that normal interrupts are disabled.  This is really only an
   * issue when we are started in un-usual ways (such as from IRAM).  In this
   * case, we can at least defer some unexpected interrupts left over from
   * the last program execution.
   */

  up_irq_disable();

  /* Set CPU frequency configured in board.h */

  esp32s2_clockconfig();

  esp32s2_lowsetup();

#ifdef USE_EARLYSERIALINIT
  /* Perform early serial initialization */

  xtensa_earlyserialinit();
#endif

  /* Move the stack to a known location.  Although we were given a stack
   * pointer at start-up, we don't know where that stack pointer is
   * positioned with respect to our memory map.  The only safe option is to
   * switch to a well-known IDLE thread stack.
   */

  sp = (uint32_t)g_idlestack + IDLETHREAD_STACKSIZE;
  __asm__ __volatile__("mov sp, %0\n" : : "r"(sp));

  /* Make page 0 access raise an exception */

  esp32s2_region_protection();

  /* Move CPU0 exception vectors to IRAM */

  __asm__ __volatile__ ("wsr %0, vecbase\n"::"r" (&_init_start));

  showprogress('A');

  /* Set .bss to zero */

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (dest = &_sbss; dest < &_ebss; dest++)
    {
      *dest = 0;
    }

  showprogress('B');

  /* Initialize onboard resources */

  esp32s2_board_initialize();

  showprogress('C');

  /* Bring up NuttX */

  nx_start();
  for (; ; ); /* Should not return */
}
