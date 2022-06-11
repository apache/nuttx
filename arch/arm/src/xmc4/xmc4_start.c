/****************************************************************************
 * arch/arm/src/xmc4/xmc4_start.c
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

#include "nvic.h"
#include "arm_internal.h"
#include "hardware/xmc4_flash.h"

#include "xmc4_clockconfig.h"
#include "xmc4_lowputc.h"
#include "xmc4_userspace.h"
#include "xmc4_start.h"

/****************************************************************************
 * Private Function prototypes
 ****************************************************************************/

static inline void xmc4_unaligned(void);
static inline void xmc4_flash_waitstates(void);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory Map ***************************************************************/

/* 0x0000:0000 - Beginning of the internal FLASH.   Address of vectors.
 *               Mapped as boot memory address 0x0000:0000 at reset.
 * 0x07ff:ffff - End of flash region (assuming the max of 2MiB of FLASH).
 * 0x1fff:0000 - Start of internal SRAM and start of .data (_sdata)
 *             - End of .data (_edata) and start of .bss (_sbss)
 *             - End of .bss (_ebss) and bottom of idle stack
 *             - _ebss + CONFIG_IDLETHREAD_STACKSIZE = end of idle stack,
 *               start of heap. NOTE that the ARM uses a decrement before
 *               store stack so that the correct initial value is the end of
 *               the stack + 4;
 * 0x2002:ffff - End of internal SRAM and end of heap (a
 */

#define HEAP_BASE  ((uintptr_t)&_ebss+CONFIG_IDLETHREAD_STACKSIZE)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_idle_topstack: _sbss is the start of the BSS region as defined by the
 * linker script. _ebss lies at the end of the BSS region. The idle task
 * stack starts at the end of BSS and is of size CONFIG_IDLETHREAD_STACKSIZE.
 * The IDLE thread is the thread that the system boots on and, eventually,
 * becomes the IDLE, do nothing task that runs only when there is nothing
 * else to run.  The heap continues from there until the end of memory.
 * g_idle_topstack is a read-only variable the provides this computed
 * address.
 */

const uintptr_t g_idle_topstack = HEAP_BASE;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

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

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c) arm_lowputc(c)
#else
#  define showprogress(c)
#endif

#ifdef CONFIG_ARMV7M_STACKCHECK
/* we need to get r10 set before we can allow instrumentation calls */

void __start(void) noinstrument_function;
#endif

/****************************************************************************
 * Name: xmc4_unaligned
 *
 * Description:
 *   Enable unaligned memory access by setting SCB_CCR.UNALIGN_TRP = 0
 *
 ****************************************************************************/

static inline void xmc4_unaligned(void)
{
  uint32_t regval;

  regval = getreg32(NVIC_CFGCON);
  regval &= ~NVIC_CFGCON_UNALIGNTRP;
  putreg32(regval, NVIC_CFGCON);
}

/****************************************************************************
 * Name: xmc4_flash_waitstates
 *
 * Description:
 *   Enable unaligned memory access by setting SCB_CCR.UNALIGN_TRP = 0
 *
 ****************************************************************************/

static inline void xmc4_flash_waitstates(void)
{
  uint32_t regval;

  regval = getreg32(XMC4_FLASH_FCON);
  regval &= ~FLASH_FCON_WSPFLASH_MASK;
  regval |= FLASH_FCON_WSPFLASH(BOARD_FLASH_WS);
  putreg32(regval, XMC4_FLASH_FCON);
}

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
  const uint32_t *src;
  uint32_t *dest;

#ifdef CONFIG_ARMV7M_STACKCHECK
  /* Set the stack limit before we attempt to call any functions */

  __asm__ volatile ("sub r10, sp, %0" : : "r"
                    (CONFIG_IDLETHREAD_STACKSIZE - 64) :);
#endif

  /* Disable the watchdog timer */

  /* TODO - add logic to disable the watchdog timer */

  /* Enable unaligned memory access */

  xmc4_unaligned();

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

  /* Copy any necessary code sections from FLASH to RAM.  The correct
   * destination in SRAM is given by _sramfuncs and _eramfuncs.  The
   * temporary location is in flash after the data initialization code
   * at _framfuncs
   */

#ifdef CONFIG_ARCH_RAMFUNCS
  for (src = &_framfuncs, dest = &_sramfuncs; dest < &_eramfuncs; )
    {
      *dest++ = *src++;
    }
#endif

  /* Set FLASH wait states prior to the configuration of clocking */

  xmc4_flash_waitstates();

  /* Perform clock and Kinetis module initialization (This depends on
   * RAM functions having been copied to RAM).
   */

  xmc4_clock_configure();

  /* Configure the uart and perform early serial initialization so that we
   * can get debug output as soon as possible (This depends on clock
   * configuration).
   */

  xmc4_lowsetup();
  showprogress('A');

  /* Initialize the FPU (if configured) */

  arm_fpuconfig();
  showprogress('B');

#ifdef USE_EARLYSERIALINIT
  /* Perform early serial initialization */

  xmc4_earlyserialinit();
  showprogress('C');
#endif

#ifdef CONFIG_BUILD_PROTECTED
  /* For the case of the separate user-/kernel-space build, perform whatever
   * platform specific initialization of the user memory is required.
   * Normally this just means initializing the user space .data and .bss
   * segments.
   */

  xmc4_userspace();
  showprogress('D');
#endif

  /* Initialize other on-board resources */

  xmc4_board_initialize();

  showprogress('E');

  /* Then start NuttX */

  nx_start();

  /* Shouldn't get here */

  for (; ; );
}
