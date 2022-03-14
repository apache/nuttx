/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_start.c
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

/* Power-Up Reset Overview
 * -----------------------
 *
 * The ARM core starts executing code on reset with the program counter set
 * to 0x0000:0000.  The CXD56xx contains a shadow pointer register that
 * allows areas of memory to be mapped to address 0x0000:0000. The default,
 * reset value of the shadow pointer is 0x1040:0000 so that on reset code in
 * the boot ROM is always executed first.
 *
 * The boot starts after reset is released.
 * The IRC is selected as CPU clock and the Cortex-M4 starts the boot loader.
 * By default the JTAG access to the chip is disabled at reset.
 * The boot ROM determines the boot mode based on the OTP BOOT_SRC value or
 * reset state pins.
 * For flash-based parts, the part boots from internal flash by default.
 * Otherwise, the boot ROM copies the image to internal SRAM at location
 * 0x1000:0000, sets the ARM's shadow pointer to 0x1000:0000,
 *  and jumps to that location.
 *
 * However, using JTAG the executable image can be also loaded directly into
 * and executed from SRAM.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/init.h>
#include <arch/board/board.h>
#include <arch/irq.h>

#include "chip.h"
#include "arm_internal.h"
#include "nvic.h"
#include "sched/sched.h"
#include "init/init.h"

#include "cxd56_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CXD56_BOOT_ENTRY_POINT      (0x04100000 + 0x1400)

/* XXX */

void weak_function up_cpuctxload(void);

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

#define DEFPRIORITY32 \
  (CXD56M4_SYSH_PRIORITY_DEFAULT << 24 | CXD56M4_SYSH_PRIORITY_DEFAULT << 16 | \
   CXD56M4_SYSH_PRIORITY_DEFAULT << 8 | CXD56M4_SYSH_PRIORITY_DEFAULT)

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uint32_t _vectors[];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fpuconfig
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
#  ifndef CONFIG_ARMV7M_LAZYFPU

void fpuconfig(void)
{
  uint32_t regval;

  /* Set CONTROL.FPCA so that we always get the extended context frame
   * with the volatile FP registers stacked above the basic context.
   */

  regval  = getcontrol();
  regval |= CONTROL_FPCA;
  setcontrol(regval);

  /* Ensure that FPCCR.LSPEN is disabled, so that we don't have to contend
   * with the lazy FP context save behaviour.  Clear FPCCR.ASPEN since we
   * are going to turn on CONTROL.FPCA for all contexts.
   */

  regval  = getreg32(NVIC_FPCCR);
  regval &= ~(NVIC_FPCCR_ASPEN | NVIC_FPCCR_LSPEN);
  putreg32(regval, NVIC_FPCCR);

  /* Enable full access to CP10 and CP11 */

  regval  = getreg32(NVIC_CPACR);
  regval |= NVIC_CPACR_CP_FULL(10) | NVIC_CPACR_CP_FULL(11);
  putreg32(regval, NVIC_CPACR);
}

#  else

void fpuconfig(void)
{
  uint32_t regval;

  /* Clear CONTROL.FPCA so that we do not get the extended context frame
   * with the volatile FP registers stacked in the saved context.
   */

  regval  = getcontrol();
  regval &= ~CONTROL_FPCA;
  setcontrol(regval);

  /* Ensure that FPCCR.LSPEN is disabled, so that we don't have to contend
   * with the lazy FP context save behaviour.  Clear FPCCR.ASPEN since we
   * are going to keep CONTROL.FPCA off for all contexts.
   */

  regval  = getreg32(NVIC_FPCCR);
  regval &= ~(NVIC_FPCCR_ASPEN | NVIC_FPCCR_LSPEN);
  putreg32(regval, NVIC_FPCCR);

  /* Enable full access to CP10 and CP11 */

  regval  = getreg32(NVIC_CPACR);
  regval |= NVIC_CPACR_CP_FULL(10) | NVIC_CPACR_CP_FULL(11);
  putreg32(regval, NVIC_CPACR);
}

#  endif

#else
#  define fpuconfig()
#endif

/****************************************************************************
 * Name: _start
 *
 * Description:
 *   This is the reset entry point.
 *
 ****************************************************************************/
#define CPU_ID (CXD56_CPU_BASE + 0x40)

void __start(void)
{
  uint32_t *dest;
#ifndef CONFIG_CXD56_SUBCORE
  uint32_t cpuid;
#endif

  /* Set MSP/PSP to IDLE stack */

  __asm__ __volatile__("\tmsr msp, %0\n" :
                       : "r" ((uint32_t)&_ebss +
                              CONFIG_IDLETHREAD_STACKSIZE));
  __asm__ __volatile__("\tmsr psp, %0\n" :
                       : "r" ((uint32_t)&_ebss +
                              CONFIG_IDLETHREAD_STACKSIZE));

#ifndef CONFIG_CXD56_SUBCORE
  cpuid = getreg32(CPU_ID);
  if (cpuid != 2)
    {
      for (; ; )
        {
          __asm__ __volatile__("wfi\n");
        }
    }
#endif

  up_irq_disable();

  if (*((uint32_t *)CXD56_BOOT_ENTRY_POINT))
    {
#ifdef CONFIG_HAVE_WEAKFUNCTIONS
      if (up_cpuctxload)
#endif
        {
          up_cpuctxload();
        }
    }

  /* Enable bus snoop */

  putreg32(0, CXD56_EXCCONF_BASE + 0);

#ifndef CONFIG_CXD56_SUBCORE
  cxd56_lowsetup();
#endif
  showprogress('A');

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (dest = &_sbss; dest < &_ebss; )
    {
      *dest++ = 0;
    }

  /* Initialize the FPU (if configured) */

  fpuconfig();

#ifdef CONFIG_ARMV7M_ITMSYSLOG
  /* Perform ARMv7-M ITM SYSLOG initialization */

  itm_syslog_initialize();
#endif

  /* Perform early serial initialization */

#if defined(USE_EARLYSERIALINIT) && !defined(CONFIG_CXD56_SUBCORE)
  arm_earlyserialinit();
#endif
  showprogress('E');

  /* For the case of the separate user-/kernel-space build, perform whatever
   * platform specific initialization of the user memory is required.
   * Normally this just means initializing the user space .data and .bss
   * segments.
   */

#ifdef CONFIG_BUILD_PROTECTED
  showprogress('F');
#endif

  /* Initialize onboard resources */

  cxd56_boardinitialize();
  showprogress('G');

  /* Then start NuttX */

  showprogress('\r');
  showprogress('\n');

  nx_start();

  /* Shouldn't get here */

  for (; ; );
}
