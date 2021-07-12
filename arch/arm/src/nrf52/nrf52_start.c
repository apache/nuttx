/****************************************************************************
 * arch/arm/src/nrf52/nrf52_start.c
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

#include "arm_arch.h"
#include "arm_internal.h"
#include "nvic.h"

#include "nrf52_clockconfig.h"
#include "hardware/nrf52_utils.h"
#include "nrf52_lowputc.h"
#include "nrf52_start.h"
#include "nrf52_gpio.h"
#include "nrf52_serial.h"
#include "nrf52_nvmc.h"

/****************************************************************************
 * Pre-processor Definitions
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
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_ARMV7M_STACKCHECK
/* we need to get r10 set before we can allow instrumentation calls */

void __start(void) __attribute__ ((no_instrument_function));
#endif

/****************************************************************************
 * Name: nrf52_fpuconfig
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
#ifndef CONFIG_ARMV7M_LAZYFPU
static inline void nrf52_fpuconfig(void)
{
  uint32_t regval;

  /* Set CONTROL.FPCA so that we always get the extended context frame
   * with the volatile FP registers stacked above the basic context.
   */

  regval = getcontrol();
  regval |= (1 << 2);
  setcontrol(regval);

  /* Ensure that FPCCR.LSPEN is disabled, so that we don't have to contend
   * with the lazy FP context save behaviour.  Clear FPCCR.ASPEN since we
   * are going to turn on CONTROL.FPCA for all contexts.
   */

  regval = getreg32(NVIC_FPCCR);
  regval &= ~((1 << 31) | (1 << 30));
  putreg32(regval, NVIC_FPCCR);

  /* Enable full access to CP10 and CP11 */

  regval = getreg32(NVIC_CPACR);
  regval |= ((3 << (2 * 10)) | (3 << (2 * 11)));
  putreg32(regval, NVIC_CPACR);
}
#else
static inline void nrf52_fpuconfig(void)
{
  uint32_t regval;

  /* Clear CONTROL.FPCA so that we do not get the extended context frame
   * with the volatile FP registers stacked in the saved context.
   */

  regval = getcontrol();
  regval &= ~(1 << 2);
  setcontrol(regval);

  /* Ensure that FPCCR.LSPEN is disabled, so that we don't have to contend
   * with the lazy FP context save behaviour.  Clear FPCCR.ASPEN since we
   * are going to keep CONTROL.FPCA off for all contexts.
   */

  regval = getreg32(NVIC_FPCCR);
  regval &= ~((1 << 31) | (1 << 30));
  putreg32(regval, NVIC_FPCCR);

  /* Enable full access to CP10 and CP11 */

  regval = getreg32(NVIC_CPACR);
  regval |= ((3 << (2 * 10)) | (3 << (2 * 11)));
  putreg32(regval, NVIC_CPACR);
}
#endif

#else
#  define nrf52_fpuconfig()
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

#ifdef CONFIG_ARMV7M_STACKCHECK
  /* Set the stack limit before we attempt to call any functions */

  __asm__ volatile("sub r10, sp, %0" : :
                   "r"(CONFIG_IDLETHREAD_STACKSIZE - 64) :);
#endif

  /* Make sure that interrupts are disabled */

  __asm__ __volatile__ ("\tcpsid  i\n");

  /* Configure the clocking and the console uart so that we can get debug
   * output as soon as possible.  NOTE: That this logic must not assume that
   * .bss or .data have beeninitialized.
   */

  nrf52_clockconfig();
  nrf52_lowsetup();
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

#if defined(CONFIG_ARCH_CHIP_NRF52832)
  /* Initialize the errdata work-around */

  nrf52832_errdata_init();
#endif

  /* Initialize the FPU (if configured) */

  nrf52_fpuconfig();

#ifdef CONFIG_NRF52_FLASH_PREFETCH
  nrf_nvmc_enable_icache(true);
  nrf_nvmc_enable_profile(true);
#endif

  showprogress('D');

#ifdef USE_EARLYSERIALINIT
  /* Perform early serial initialization */

  nrf52_earlyserialinit();
#endif
  showprogress('E');

#ifdef CONFIG_BUILD_PROTECTED
  /* For the case of the separate user-/kernel-space build, perform whatever
   * platform specific initialization of the user memory is required.
   * Normally this just means initializing the user space .data and .bss
   * segments.
   */

  nrf52_userspace();
  showprogress('F');
#endif

  /* Initialize onboard resources */

  nrf52_board_initialize();
  showprogress('G');

  /* Then start NuttX */

  showprogress('\r');
  showprogress('\n');

  nx_start();

  /* Shouldn't get here */

  for (; ; );
}
