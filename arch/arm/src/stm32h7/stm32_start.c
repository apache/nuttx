/****************************************************************************
 * arch/arm/src/stm32h7/stm32_start.c
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

#include <nuttx/cache.h>
#include <nuttx/init.h>
#include <arch/barriers.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "nvic.h"
#include "mpu.h"
#ifdef CONFIG_ARM_MPU
#  include "stm32_mpuinit.h"
#endif

#include "stm32_rcc.h"
#include "stm32_userspace.h"
#include "stm32_lowputc.h"
#include "stm32_start.h"

#ifdef CONFIG_ARCH_STM32H7_DUALCORE
#  include "stm32_dualcore.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory Map ***************************************************************/

/* TODO: Fix this
 * 0x0400:0000 - Beginning of the internal FLASH.   Address of vectors.
 *               Mapped as boot memory address 0x0000:0000 at reset.
 * 0x041f:ffff - End of flash region (assuming the max of 2MiB of FLASH).
 * 0x2000:0000 - Start of internal SRAM and start of .data (_sdata)
 *             - End of .data (_edata) and start of .bss (_sbss)
 *             - End of .bss (_ebss) and bottom of idle stack
 *             - _ebss + CONFIG_IDLETHREAD_STACKSIZE = end of idle stack,
 *               start of heap. NOTE that the ARM uses a decrement before
 *               store stack so that the correct initial value is the end of
 *               the stack + 4;
 * 0x2005:ffff - End of internal SRAM and end of heap (a
 */

#define HEAP_BASE  ((uintptr_t)_ebss + CONFIG_IDLETHREAD_STACKSIZE)

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
 * Private Function prototypes
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

#ifdef CONFIG_ARCH_CHIP_STM32H7_CORTEXM7
/****************************************************************************
 * Name: stm32_tcmenable
 *
 * Description:
 *   Enable/disable tightly coupled memories.  Size of tightly coupled
 *   memory regions is controlled by GPNVM Bits 7-8.
 *
 ****************************************************************************/

static inline void stm32_tcmenable(void)
{
  uint32_t regval;

  UP_MB();

  /* Enabled/disabled ITCM */

#ifdef CONFIG_ARMV7M_ITCM
  regval  = NVIC_TCMCR_EN | NVIC_TCMCR_RMW | NVIC_TCMCR_RETEN;
#else
  regval  = getreg32(NVIC_ITCMCR);
  regval &= ~NVIC_TCMCR_EN;
#endif
  putreg32(regval, NVIC_ITCMCR);

  /* Enabled/disabled DTCM */

#ifdef CONFIG_ARMV7M_DTCM
  /* As DTCM RAM on STM32H7 does not have ECC, so do not enable
   * read-modify-write.
   */

  regval  = NVIC_TCMCR_EN | NVIC_TCMCR_RETEN;
#else
  regval  = getreg32(NVIC_DTCMCR);
  regval &= ~NVIC_TCMCR_EN;
#endif
  putreg32(regval, NVIC_DTCMCR);

  UP_MB();

#ifdef CONFIG_ARMV7M_ITCM
  /* Copy TCM code from flash to ITCM */

#warning Missing logic
#endif
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

#ifdef CONFIG_ARCH_CHIP_STM32H7_CORTEXM4
  /* Wait for CM7 initialization done */

  stm32h7_waitfor_cm7();
#endif

  /* If enabled reset the MPU */

  mpu_early_reset();

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (dest = (uint32_t *)_sbss; dest < (uint32_t *)_ebss; )
    {
      *dest++ = 0;
    }

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

  /* Copy any necessary code sections from FLASH to RAM.  The correct
   * destination in SRAM is given by _sramfuncs and _eramfuncs.  The
   * temporary location is in flash after the data initialization code
   * at _framfuncs.  This should be done before stm32_clockconfig() is
   * called (in case it has some dependency on initialized C variables).
   */

#ifdef CONFIG_ARCH_RAMFUNCS
  for (src = (const uint32_t *)_framfuncs,
       dest = (uint32_t *)_sramfuncs; dest < (uint32_t *)_eramfuncs;
      )
    {
      *dest++ = *src++;
    }
#endif

#ifdef CONFIG_ARMV7M_STACKCHECK
  arm_stack_check_init();
#endif

  /* Configure the UART so that we can get debug output as soon as possible */

  stm32_clockconfig();
  arm_fpuconfig();
  stm32_lowsetup();
  showprogress('A');

#ifdef CONFIG_ARCH_CHIP_STM32H7_CORTEXM7
  /* Enable/disable tightly coupled memories */

  stm32_tcmenable();
#endif

  /* Initialize onboard resources */

  stm32_boardinitialize();
  showprogress('B');

#ifdef CONFIG_ARCH_CHIP_STM32H7_CORTEXM7
  /* Enable I- and D-Caches */

  up_enable_icache();
  up_enable_dcache();
#endif
  showprogress('C');

#ifdef CONFIG_ARCH_PERF_EVENTS
  up_perf_init((void *)STM32_CPUCLK_FREQUENCY);
#endif

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
#endif

#ifdef CONFIG_ARM_MPU
  /* Configure the MPU */

  stm32_mpuinitialize();
#endif
  showprogress('E');

  /* Then start NuttX */

  showprogress('\r');
  showprogress('\n');

#if defined(CONFIG_ARCH_STM32H7_DUALCORE) && \
    defined(CONFIG_ARCH_CHIP_STM32H7_CORTEXM7) && \
    defined(CONFIG_STM32H7_CORTEXM4_ENABLED)

  /* Start CM4 core after clock configration is done */

  stm32h7_start_cm4();
#endif

  nx_start();

  /* Shouldn't get here */

  for (; ; );
}
