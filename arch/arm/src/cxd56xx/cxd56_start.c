/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_start.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 *   Copyright (C) 2012, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
#include "arm_arch.h"
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
 * Private Function prototypes
 ****************************************************************************/

#ifdef CONFIG_STACK_COLORATION
static void go_nx_start(void *pv, unsigned int nbytes)
  __attribute__((naked, no_instrument_function, noreturn));
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
 * Name: go_nx_start
 *
 * Description:
 *   Set the IDLE stack to the
 *
 ****************************************************************************/

#ifdef CONFIG_STACK_COLORATION
static void go_nx_start(void *pv, unsigned int nbytes)
{
  /* Set the IDLE stack to the stack coloration value then jump to
   * nx_start().  We take extreme care here because were currently
   * executing on this stack.
   *
   * We want to avoid sneak stack access generated by the compiler.
   */

  __asm__ __volatile__
  (
    "\tmovs r1, r1, lsr #2\n"   /* R1 = nwords = nbytes >> 2 */
    "\tbeq  2f\n"               /* (should not happen) */

    "\tbic  r0, r0, #3\n"       /* R0 = Aligned stackptr */
    "\tmovw r2, #0xbeef\n"      /* R2 = STACK_COLOR = 0xdeadbeef */
    "\tmovt r2, #0xdead\n"

    "1:\n"                      /* Top of the loop */
    "\tsub  r1, r1, #1\n"       /* R1 nwords-- */
    "\tcmp  r1, #0\n"           /* Check (nwords == 0) */
    "\tstr  r2, [r0], #4\n"     /* Save stack color word, increment stackptr */
    "\tbne  1b\n"               /* Bottom of the loop */

    "2:\n"
    "\tmov  r14, #0\n"          /* LR = return address (none) */
    "\tb    nx_start\n"         /* Branch to nx_start */
  );
}
#endif

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
  regval |= (1 << 2);
  setcontrol(regval);

  /* Ensure that FPCCR.LSPEN is disabled, so that we don't have to contend
   * with the lazy FP context save behaviour.  Clear FPCCR.ASPEN since we
   * are going to turn on CONTROL.FPCA for all contexts.
   */

  regval  = getreg32(NVIC_FPCCR);
  regval &= ~((1 << 31) | (1 << 30));
  putreg32(regval, NVIC_FPCCR);

  /* Enable full access to CP10 and CP11 */

  regval  = getreg32(NVIC_CPACR);
  regval |= ((3 << (2 * 10)) | (3 << (2 * 11)));
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
  regval &= ~(1 << 2);
  setcontrol(regval);

  /* Ensure that FPCCR.LSPEN is disabled, so that we don't have to contend
   * with the lazy FP context save behaviour.  Clear FPCCR.ASPEN since we
   * are going to keep CONTROL.FPCA off for all contexts.
   */

  regval  = getreg32(NVIC_FPCCR);
  regval &= ~((1 << 31) | (1 << 30));
  putreg32(regval, NVIC_FPCCR);

  /* Enable full access to CP10 and CP11 */

  regval  = getreg32(NVIC_CPACR);
  regval |= ((3 << (2 * 10)) | (3 << (2 * 11)));
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
                              CONFIG_IDLETHREAD_STACKSIZE - 4));
  __asm__ __volatile__("\tmsr psp, %0\n" :
                       : "r" ((uint32_t)&_ebss +
                              CONFIG_IDLETHREAD_STACKSIZE - 4));

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

  cxd56_lowsetup();
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

#ifdef USE_EARLYSERIALINIT
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

#ifdef CONFIG_STACK_COLORATION
  /* Set the IDLE stack to the coloration value and jump into nx_start() */

  go_nx_start((FAR void *)&_ebss, CONFIG_IDLETHREAD_STACKSIZE);
#else
  /* Call nx_start() */

  nx_start();

  /* Shouldn't get here */

  for (; ; );
#endif
}
