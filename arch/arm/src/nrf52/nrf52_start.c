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

#include "arm_internal.h"
#include "hardware/nrf52_nvmc.h"
#include "hardware/nrf52_utils.h"
#include "nrf52_clockconfig.h"
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

void __start(void) noinstrument_function;
#endif

#ifdef CONFIG_NRF52_FLASH_PREFETCH

/****************************************************************************
 * Name: nrf52_enable_icache
 *
 * Description:
 *   Enable I-Cache for Flash
 *
 * Input Parameter:
 *   enable - enable or disable I-Cache
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void nrf52_enable_icache(bool enable)
{
  if (enable)
    {
      modifyreg32(NRF52_NVMC_ICACHECNF, 0, NVMC_ICACHECNF_CACHEEN);
    }
  else
    {
      modifyreg32(NRF52_NVMC_ICACHECNF, NVMC_ICACHECNF_CACHEEN, 0);
    }
}

/****************************************************************************
 * Name: nrf52_enable_profile
 *
 * Description:
 *   Enable profiling I-Cache for flash
 *
 * Input Parameter:
 *   enable - enable or disable profiling for I-Cache
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void nrf52_enable_profile(bool enable)
{
  if (enable)
    {
      modifyreg32(NRF52_NVMC_ICACHECNF, 0, NVMC_ICACHECNF_CACHEPROFEN);
    }
  else
    {
      modifyreg32(NRF52_NVMC_ICACHECNF, NVMC_ICACHECNF_CACHEPROFEN, 0);
    }
}
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

  for (dest = (uint32_t *)_sbss; dest < (uint32_t *)_ebss; )
    {
      *dest++ = 0;
    }

  showprogress('B');

  /* Move the initialized data section from his temporary holding spot in
   * FLASH into the correct place in SRAM.  The correct place in SRAM is
   * give by _sdata and _edata.  The temporary location is in FLASH at the
   * end of all of the other read-only data (.text, .rodata) at _eronly.
   */

  for (src = (const uint32_t *)_eronly,
       dest = (uint32_t *)_sdata; dest < (uint32_t *)_edata;
      )
    {
      *dest++ = *src++;
    }

  showprogress('C');

#if defined(CONFIG_ARCH_CHIP_NRF52832)
  /* Initialize the errdata work-around */

  nrf52832_errdata_init();
#endif

  /* Initialize the FPU (if configured) */

  arm_fpuconfig();

#ifdef CONFIG_NRF52_FLASH_PREFETCH
  nrf52_enable_icache(true);
  nrf52_enable_profile(true);
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
