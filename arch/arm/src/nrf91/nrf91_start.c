/****************************************************************************
 * arch/arm/src/nrf91/nrf91_start.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/init.h>
#include <arch/board/board.h>
#include <arch/irq.h>

#include "arm_internal.h"
#include "nvic.h"

#include "nrf91_clockconfig.h"
#include "hardware/nrf91_nvmc.h"
#include "hardware/nrf91_utils.h"
#include "hardware/nrf91_uicr.h"
#include "nrf91_lowputc.h"
#include "nrf91_start.h"
#include "nrf91_gpio.h"
#include "nrf91_serial.h"
#ifndef CONFIG_ARCH_TRUSTZONE_NONSECURE
#  include "nrf91_errata.h"
#  include "nrf91_spu.h"
#endif

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

#ifdef CONFIG_ARMV8M_STACKCHECK
/* we need to get r10 set before we can allow instrumentation calls */

void __start(void) noinstrument_function;
#endif

#ifndef CONFIG_ARCH_TRUSTZONE_NONSECURE
/****************************************************************************
 * Name: nrf91_approtect
 ****************************************************************************/

static void nrf91_approtect(void)
{
  /* TODO: missing logic */
}
#endif

#ifdef CONFIG_NRF91_FLASH_PREFETCH
/****************************************************************************
 * Name: nrf91_enable_icache
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

static void nrf91_enable_icache(bool enable)
{
  if (enable)
    {
      modifyreg32(NRF91_NVMC_ICACHECNF, 0, NVMC_ICACHECNF_CACHEEN);
    }
  else
    {
      modifyreg32(NRF91_NVMC_ICACHECNF, NVMC_ICACHECNF_CACHEEN, 0);
    }
}

/****************************************************************************
 * Name: nrf91_enable_profile
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

static void nrf91_enable_profile(bool enable)
{
  if (enable)
    {
      modifyreg32(NRF91_NVMC_ICACHECNF, 0, NVMC_ICACHECNF_CACHEPROFEN);
    }
  else
    {
      modifyreg32(NRF91_NVMC_ICACHECNF, NVMC_ICACHECNF_CACHEPROFEN, 0);
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

#ifdef CONFIG_ARMV8M_STACKCHECK
  /* Set the stack limit before we attempt to call any functions */

  __asm__ volatile("sub r10, sp, %0" : :
                   "r"(CONFIG_IDLETHREAD_STACKSIZE - 64) :);
#endif

  /* Make sure that interrupts are disabled */

  __asm__ __volatile__ ("\tcpsid  i\n");

#ifndef CONFIG_ARCH_TRUSTZONE_NONSECURE
  /* Apply errata */

  nrf91_errata_secure();

  /* Handle APPROTECT configuration */

  nrf91_approtect();

#  ifdef CONFIG_NRF91_FICR_NS_WORKAROUND
  /* Copy FICR */

  nrf91_ficr_ram_copy();
#  endif

  /* Configure SPU */

  nrf91_spu_configure();
#endif

  /* Configure the clocking and the console uart so that we can get debug
   * output as soon as possible.  NOTE: That this logic must not assume that
   * .bss or .data have beeninitialized.
   */

  nrf91_clockconfig();
  nrf91_lowsetup();
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

#ifdef CONFIG_ARCH_FPU
  /* Initialize the FPU (if available) */

  arm_fpuconfig();
#endif

#ifdef CONFIG_NRF91_FLASH_PREFETCH
  nrf91_enable_icache(true);
  nrf91_enable_profile(true);
#endif

#ifdef CONFIG_ARCH_PERF_EVENTS
  up_perf_init((void *)BOARD_SYSTICK_CLOCK);
#endif

  showprogress('D');

#ifdef USE_EARLYSERIALINIT
  /* Perform early serial initialization */

  nrf91_earlyserialinit();
#endif
  showprogress('E');

#ifdef CONFIG_BUILD_PROTECTED
  /* For the case of the separate user-/kernel-space build, perform whatever
   * platform specific initialization of the user memory is required.
   * Normally this just means initializing the user space .data and .bss
   * segments.
   */

  nrf91_userspace();
  showprogress('F');
#endif

  /* Initialize onboard resources */

  nrf91_board_initialize();
  showprogress('G');

  /* Then start NuttX */

  showprogress('\r');
  showprogress('\n');

  nx_start();

  /* Shouldn't get here */

  for (; ; );
}
