/****************************************************************************
 * arch/arm/src/stm32u5/stm32_start.c
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
#include "nvic.h"

#include "stm32.h"
#include "stm32_gpio.h"
#include "stm32_userspace.h"
#include "stm32_start.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory Map ***************************************************************/

/* 0x0800:0000 - Beginning of the internal FLASH.   Address of vectors.
 *               Mapped as boot memory address 0x0000:0000 at reset.
 * 0x080f:ffff - End of flash region (assuming the max of 2MiB of FLASH).
 * 0x2000:0000 - Start of internal SRAM1 and start of .data (_sdata)
 *             - End of .data (_edata) and start of .bss (_sbss)
 *             - End of .bss (_ebss) and bottom of idle stack
 *             - _ebss + CONFIG_IDLETHREAD_STACKSIZE = end of idle stack,
 *               start of heap. NOTE that the ARM uses a decrement before
 *               store stack so that the correct initial value is the end of
 *               the stack + 4;
 * 0x2002:ffff - End of internal SRAM1
 * 0x2003:0000 - Start of internal SRAM2
 * 0x2003:ffff - End of internal SRAM2
 */

#define SRAM2_START  STM32U5_SRAM2_BASE
#define SRAM2_END    (SRAM2_START + STM32U5_SRAM2_SIZE)

#define HEAP_BASE  ((uintptr_t)&_ebss + CONFIG_IDLETHREAD_STACKSIZE)

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
 * Private Function prototypes
 ****************************************************************************/

#ifdef CONFIG_ARCH_FPU
static inline void stm32_fpuconfig(void);
#endif

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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_ARMV8M_STACKCHECK
/* we need to get r10 set before we can allow instrumentation calls */

void __start(void) noinstrument_function;
#endif

/****************************************************************************
 * Name: stm32_fpuconfig
 *
 * Description:
 *   Configure the FPU.  Relative bit settings:
 *
 *     CPACR:  Enables access to CP10 and CP11
 *     CONTROL.FPCA: Determines whether the FP extension is active in the
 *       current context:
 *     FPCCR.ASPEN:  Enables automatic FP state preservation, then the
 *       processor sets this bit to 1 on successful completion of any FP
 *       instruction.
 *     FPCCR.LSPEN:  Enables lazy context save of FP state. When this is
 *       done, the processor reserves space on the stack for the FP state,
 *       but does not save that state information to the stack.
 *
 *  Software must not change the value of the ASPEN bit or LSPEN bit while
 *  either:
 *   - the CPACR permits access to CP10 and CP11, that give access to the FP
 *     extension, or
 *   - the CONTROL.FPCA bit is set to 1
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_FPU
#ifndef CONFIG_ARMV8M_LAZYFPU

static inline void stm32_fpuconfig(void)
{
  uint32_t regval;

  /* Set CONTROL.FPCA so that we always get the extended context frame
   * with the volatile FP registers stacked above the basic context.
   */

  regval = getcontrol();
  regval |= CONTROL_FPCA;
  setcontrol(regval);

  /* Ensure that FPCCR.LSPEN is disabled, so that we don't have to contend
   * with the lazy FP context save behaviour.  Clear FPCCR.ASPEN since we
   * are going to turn on CONTROL.FPCA for all contexts.
   */

  regval = getreg32(NVIC_FPCCR);
  regval &= ~(NVIC_FPCCR_ASPEN | NVIC_FPCCR_LSPEN);
  putreg32(regval, NVIC_FPCCR);

  /* Enable full access to CP10 and CP11 */

  regval = getreg32(NVIC_CPACR);
  regval |= NVIC_CPACR_CP_FULL(10) | NVIC_CPACR_CP_FULL(11);
  putreg32(regval, NVIC_CPACR);
}

#else

static inline void stm32_fpuconfig(void)
{
  uint32_t regval;

  /* Clear CONTROL.FPCA so that we do not get the extended context frame
   * with the volatile FP registers stacked in the saved context.
   */

  regval = getcontrol();
  regval &= ~CONTROL_FPCA;
  setcontrol(regval);

  /* Ensure that FPCCR.LSPEN is disabled, so that we don't have to contend
   * with the lazy FP context save behaviour.  Clear FPCCR.ASPEN since we
   * are going to keep CONTROL.FPCA off for all contexts.
   */

  regval = getreg32(NVIC_FPCCR);
  regval &= ~(NVIC_FPCCR_ASPEN | NVIC_FPCCR_LSPEN);
  putreg32(regval, NVIC_FPCCR);

  /* Enable full access to CP10 and CP11 */

  regval = getreg32(NVIC_CPACR);
  regval |= NVIC_CPACR_CP_FULL(10) | NVIC_CPACR_CP_FULL(11);
  putreg32(regval, NVIC_CPACR);
}

#endif

#else
#  define stm32_fpuconfig()
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

#ifdef CONFIG_ARMV8M_STACKCHECK
  /* Set the stack limit before we attempt to call any functions */

  __asm__ volatile
    ("sub r10, sp, %0" : : "r" (CONFIG_IDLETHREAD_STACKSIZE - 64) :);
#endif

#ifdef CONFIG_STM32U5_SRAM2_INIT
  /* The SRAM2 region is parity checked, but upon power up, it will be in
   * a random state and probably invalid with respect to parity, potentially
   * generating faults if accessed.  If elected, we will write zeros to the
   * memory, forcing the parity to be set to a valid state.
   * NOTE:  this is optional because this may be inappropriate, especially
   * if the memory is being used for it's battery backed purpose.  In that
   * case, the first-time initialization needs to be performed by the board
   * under application-specific circumstances.  On the other hand, if we're
   * using this memory for, say, additional heap space, then this is handy.
   */

  for (dest = (uint32_t *)SRAM2_START; dest < (uint32_t *)SRAM2_END; )
    {
      *dest++ = 0;
    }
#endif

  /* Configure the UART so that we can get debug output as soon as possible */

  stm32_clockconfig();
  stm32_fpuconfig();
  stm32_lowsetup();
  stm32_gpioinit();
  showprogress('A');

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (dest = &_sbss; dest < &_ebss; )
    {
      *dest++ = 0;
    }

  showprogress('B');

  /* Move the initialized data section from his temporary holding spot in
   * FLASH into the correct place in SRAM.  The correct place in SRAM is
   * give by _sdata and _edata.  The temporary location is in FLASH at the
   * end of all of the other read-only data (.text, .rodata) at _eronly.
   */

  for (src = &_eronly, dest = &_sdata; dest < &_edata; )
    {
      *dest++ = *src++;
    }

  showprogress('C');

  /* Perform early serial initialization */

#ifdef USE_EARLYSERIALINIT
  arm_earlyserialinit();
#endif
  showprogress('D');

  /* For the case of the separate user-/kernel-space build, perform whatever
   * platform specific initialization of the user memory is required.
   * Normally this just means initializing the user space .data and .bss
   * segments.
   */

#ifdef CONFIG_BUILD_PROTECTED
  stm32_userspace();
  showprogress('E');
#endif

  /* Initialize onboard resources */

  stm32_board_initialize();
  showprogress('F');

  /* Then start NuttX */

  showprogress('\r');
  showprogress('\n');

  nx_start();

  /* Shoulnd't get here */

  for (; ; );
}
