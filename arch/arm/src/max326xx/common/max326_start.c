/****************************************************************************
 * arch/arm/src/max326xx/common/max326_start.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/init.h>
#include <arch/board/board.h>
#include <arch/irq.h>

#include "arm_internal.h"
#include "nvic.h"

#include "max326_clockconfig.h"
#include "max326_userspace.h"
#include "max326_lowputc.h"
#include "max326_serial.h"
#include "max326_icc.h"
#include "max326_start.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory Map:
 *
 * 0x0000:0000 - Beginning of FLASH. Address of exception vectors.
 * 0x0003:ffff - End of flash (assuming 256KB of FLASH)
 * 0x2000:0000 - Start of SRAM and start of .data (_sdata)
 *             - End of .data (_edata) abd start of .bss (_sbss)
 *             - End of .bss (_ebss) and bottom of idle stack
 *             - _ebss + CONFIG_IDLETHREAD_STACKSIZE = end of idle stack,
 *               start of heap
 * 0x2001:7fff - End of SRAM and end of heap (assuming 96KB of SRAM)
 */

#define IDLE_STACK ((uint32_t)&_ebss + CONFIG_IDLETHREAD_STACKSIZE)

/****************************************************************************
 * Name: showprogress
 *
 * Description:
 *   Print a character on the UART to show boot status.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c) arm_lowputc(c)
#else
#  define showprogress(c)
#endif

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
 *   This is the reset entry point.
 *
 ****************************************************************************/

void __start(void)
{
#ifdef CONFIG_BOOT_RUNFROMFLASH
  const uint32_t *src;
#endif
  uint32_t *dest;

  /* Make sure that interrupts are disabled */

  __asm__ __volatile__ ("\tcpsid  i\n");

  /* Configure the clocking and the console UART so that we can get debug
   * output as soon as possible.  NOTE: That this logic must not assume that
   * .bss or .data have been initialized.
   */

  max326_clockconfig(&g_initial_clock_setup);
  max326_lowsetup();
  showprogress('A');

#ifdef CONFIG_MAX326XX_ICC
  /* Enable the instruction cache */

  max326_icc_enable(true);
  showprogress('B');
#endif

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   * REVISIT: Consider using MAX326xx SRAM Zeroize hardware.
   */

  for (dest = &_sbss; dest < &_ebss; )
    {
      *dest++ = 0;
    }

  showprogress('C');

#ifdef CONFIG_BOOT_RUNFROMFLASH
  /* Move the initialized data section from his temporary holding spot in
   * FLASH into the correct place in SRAM.  The correct place in SRAM is
   * give by _sdata and _edata.  The temporary location is in FLASH at the
   * end of all of the other read-only data (.text, .rodata) at _eronly.
   */

  for (src = &_eronly, dest = &_sdata; dest < &_edata; )
    {
      *dest++ = *src++;
    }

  showprogress('D');
#endif

  /* Initialize the FPU (if configured) */

  arm_fpuconfig();
  showprogress('E');

  /* Perform early serial initialization */

#ifdef USE_EARLYSERIALINIT
  max326_earlyserialinit();
#endif
  showprogress('F');

#ifdef CONFIG_BUILD_PROTECTED
  /* For the case of the separate user-/kernel-space build, perform whatever
   * platform specific initialization of the user memory is required.
   * Normally this just means initializing the user space .data and .bss
   * segments.
   */

  max326_userspace();
  showprogress('G');
#endif

  /* Initialize on-board resources */

  max326_board_initialize();
  showprogress('H');

  /* Then start NuttX */

  showprogress('\r');
  showprogress('\n');
  nx_start();

  /* Shouldn't get here */

  for (; ; );
}
