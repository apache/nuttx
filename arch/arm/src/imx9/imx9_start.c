/****************************************************************************
 * arch/arm/src/imx9/imx9_start.c
 *
 * SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2024 NXP
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
#include <arch/board/board.h>
#include <arch/barriers.h>

#include "arm_internal.h"
#include "nvic.h"
#include "mpu.h"

#include "imx9_clockconfig.h"
#include "imx9_mpuinit.h"
#include "imx9_userspace.h"
#include "imx9_lowputc.h"
#include "imx9_serial.h"
#include "imx9_start.h"
#include "imx9_gpio.h"

#ifdef CONFIG_IMX9_SCMI
#include "imx9_scmi.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IDLE_STACK      ((unsigned)&_ebss+CONFIG_IDLETHREAD_STACKSIZE)

#ifdef CONFIG_DEBUG_FEATURES
#define showprogress(c) arm_lowputc(c)
#else
#  define showprogress(c)
#endif

/* Memory Map ***************************************************************/

/* 0x2020:0000 - Start of on-chip RAM (OCRAM) and start of .data (_sdata)
 *             - End of .data (_edata) and start of .bss (_sbss)
 *             - End of .bss (_ebss) and bottom of idle stack
 *             - _ebss + CONFIG_IDLETHREAD_STACKSIZE = end of idle stack,
 *               start of heap. NOTE that the ARM uses a decrement before
 *               store stack so that the correct initial value is the end of
 *               the stack + 4;
 * 0x2027:ffff - End of OCRAM and end of heap (assuming 512Kb OCRAM)
 *
 * NOTE:  This assumes that all internal RAM is configured for OCRAM (vs.
 * ITCM or DTCM).  The RAM that holds .data and .bss is called the "Primary
 * RAM".  Many other configurations are possible, including configurations
 * where the primary ram is in external memory.  Those are not considered
 * here.
 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_ARMV7M_STACKCHECK
/* we need to get r10 set before we can allow instrumentation calls */

void __start(void) noinstrument_function;
#endif

extern const void * const _vectors[];

/****************************************************************************
 * Name: imx9_tcmenable
 *
 * Description:
 *   Enable/disable tightly coupled memories.  Size of tightly coupled
 *   memory regions is controlled by GPNVM Bits 7-8.
 *
 ****************************************************************************/

static inline void imx9_tcmenable(void)
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
  regval  = NVIC_TCMCR_EN | NVIC_TCMCR_RMW | NVIC_TCMCR_RETEN;
#else
  regval  = getreg32(NVIC_DTCMCR);
  regval &= ~NVIC_TCMCR_EN;
#endif
  putreg32(regval, NVIC_DTCMCR);

  UP_MB();
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
  register const uint32_t *src;
  register uint32_t *dest;

  /* Make sure that interrupts are disabled and set MSP */

  __asm__ __volatile__ ("CPSID i\n");
  __asm__ __volatile__ ("MSR MSP, %0\n" : : "r" (IDLE_STACK) :);

  /* Make sure that we use MSP from now on */

  __asm__ __volatile__ ("MSR CONTROL, %0\n" : : "r" (0) :);
  __asm__ __volatile__ ("ISB SY\n");

  /* Make sure VECTAB is set to NuttX vector table
   * and not the one from the boot ROM and have consistency
   * with debugger that automatically set the VECTAB
   */

  putreg32((uint32_t)_vectors, NVIC_VECTAB);

#ifdef CONFIG_ARMV7M_STACKCHECK
  /* Set the stack limit before we attempt to call any functions */

  __asm__ volatile("sub r10, sp, %0" : :
                   "r"(CONFIG_IDLETHREAD_STACKSIZE - 64) :);
#endif

  /* If enabled reset the MPU */

  mpu_early_reset();

#if defined(CONFIG_IMX9_INIT_ISRAM)
  imx9_init_isram_functions();
#endif

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (dest = (uint32_t *)_sbss; dest < (uint32_t *)_ebss; )
    {
      *dest++ = 0;
    }

  /* Move the initialized data section from his temporary holding spot in
   * FLASH into the correct place in OCRAM.  The correct place in OCRAM is
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
   * destination in OCRAM is given by _sramfuncs and _eramfuncs.  The
   * temporary location is in flash after the data initialization code
   * at _framfuncs.  This should be done before imx9_clockconfig() is
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

#ifdef CONFIG_IMX9_SCMI
  imx9_scmi_initialize();
#endif

  /* Configure the UART so that we can get debug output as soon as possible */

  imx9_clockconfig();
  arm_fpuconfig();
  imx9_lowsetup();

  /* Enable/disable tightly coupled memories */

  imx9_tcmenable();

  /* Initialize onboard resources */

  imx9_boardinitialize();

#ifdef CONFIG_ARM_MPU
#ifdef CONFIG_BUILD_PROTECTED
  /* For the case of the separate user-/kernel-space build, perform whatever
   * platform specific initialization of the user memory is required.
   * Normally this just means initializing the user space .data and .bss
   * segments.
   */

  imx9_userspace();
#endif

  /* Configure the MPU to permit user-space access to its FLASH and RAM (for
   * CONFIG_BUILD_PROTECTED) or to manage cache properties in external
   * memory regions.
   */

  imx9_mpu_initialize();
#endif

  /* Enable I- and D-Caches */

  up_enable_icache();
  up_enable_dcache();

  /* Perform early serial initialization */

#ifdef USE_EARLYSERIALINIT
  arm_earlyserialinit();
#endif

  /* Then start NuttX */

  nx_start();

  /* Shouldn't get here */

  for (; ; );
}
