/****************************************************************************
 * arch/arm/src/samd2l2/sam_start.c
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

#include "arm_internal.h"
#include "sam_config.h"
#include "sam_lowputc.h"
#include "sam_clockconfig.h"
#include "sam_userspace.h"
#include "sam_start.h"

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
 * 0x2000:ffff - End of SRAM and end of heap (assuming 64KB of SRAM)
 */

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
#  define showprogress(c) sam_lowputc((uint32_t)c)
#else
#  define showprogress(c)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _start
 *
 * Description:
 *   This is the reset entry point.
 *
 ****************************************************************************/

void __start(void)
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

  /* Configure basic clocking */

  sam_clockconfig();

  /* Configure the uart early so that we can get debug output as soon as
   * possible
   */

  sam_lowsetup();
  showprogress('A');

  /* Perform early serial initialization */

#ifdef USE_EARLYSERIALINIT
  arm_earlyserialinit();
#endif
  showprogress('B');

  /* For the case of the separate user-/kernel-space build, perform whatever
   * platform specific initialization of the user memory is required.
   * Normally this just means initializing the user space .data and .bss
   * segments.
   */

#ifdef CONFIG_BUILD_PROTECTED
  sam_userspace();
  showprogress('C');
#endif

  /* Initialize onboard resources */

  sam_boardinitialize();
  showprogress('D');

  /* Then start NuttX */

  showprogress('\r');
  showprogress('\n');
  nx_start();

  /* Shouldn't get here */

  for (; ; );
}
