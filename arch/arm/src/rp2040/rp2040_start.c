/****************************************************************************
 * arch/arm/src/rp2040/rp2040_start.c
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

#include "arm_internal.h"
#include "rp2040_config.h"
#include "rp2040_clock.h"
#include "rp2040_uart.h"
#include "hardware/rp2040_sio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IDLE_STACK ((uint32_t)&_ebss+CONFIG_IDLETHREAD_STACKSIZE)

/****************************************************************************
 * Public Data
 ****************************************************************************/

const uintptr_t g_idle_topstack = IDLE_STACK;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: showprogress
 *
 * Description:
 *   Print a character on the UART to show boot status.
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_FEATURES) && defined(HAVE_SERIAL_CONSOLE)
#  define showprogress(c) arm_lowputc((uint32_t)c)
#else
#  define showprogress(c)
#endif

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
#ifdef CONFIG_RP2040_FLASH_BOOT
  const uint32_t *src;
#endif
  uint32_t *dest;
  int i;

  /* Set MSP to the top of the IDLE stack */

  __asm__ __volatile__ ("\tmsr msp, %0\n" :: "r" (g_idle_topstack));

  if (up_cpu_index() != 0)
    {
      while (1)
        {
          __asm__ volatile ("wfe");
        }
    }

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (dest = &_sbss; dest < &_ebss; )
    {
      *dest++ = 0;
    }

  /* Set up clock */

  rp2040_clockconfig();
  rp2040_boardearlyinitialize();

  /* Initialize all spinlock states */

  for (i = 0; i < 32; i++)
    {
      putreg32(0, RP2040_SIO_SPINLOCK(i));
    }

  /* Configure the uart so that we can get debug output as soon as possible */

  rp2040_lowsetup();
  showprogress('A');

  /* Move the initialized data section from his temporary holding spot in
   * FLASH into the correct place in SRAM.  The correct place in SRAM is
   * give by _sdata and _edata.  The temporary location is in FLASH at the
   * end of all of the other read-only data (.text, .rodata) at _eronly.
   */

#ifdef CONFIG_RP2040_FLASH_BOOT
  for (src = &_eronly, dest = &_sdata; dest < &_edata; )
    {
      *dest++ = *src++;
    }
#endif

  showprogress('B');

  /* Perform early serial initialization */

#ifdef USE_EARLYSERIALINIT
  arm_earlyserialinit();
#endif
  showprogress('C');

  /* For the case of the separate user-/kernel-space build, perform whatever
   * platform specific initialization of the user memory is required.
   * Normally this just means initializing the user space .data and .bss
   * segments.
   */

#ifdef CONFIG_BUILD_PROTECTED
  rp2040_userspace();
  showprogress('D');
#endif

  /* Initialize onboard resources */

  rp2040_boardinitialize();
  showprogress('E');

  /* Then start NuttX */

  showprogress('\r');
  showprogress('\n');

  nx_start();

  /* Shouldn't get here */

  for (; ; );
}
