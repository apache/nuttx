/****************************************************************************
 * arch/arm/src/xmc4/xmc4_start.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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
#include "arm_arch.h"
#include "arm_internal.h"
#include "hardware/xmc4_flash.h"

#include "xmc4_clockconfig.h"
#include "xmc4_lowputc.h"
#include "xmc4_userspace.h"
#include "xmc4_start.h"

/****************************************************************************
 * Private Function prototypes
 ****************************************************************************/

#ifdef CONFIG_ARCH_FPU
static inline void xmc4_fpu_config(void);
#endif
static inline void xmc4_unaligned(void);
static inline void xmc4_flash_waitstates(void);
#ifdef CONFIG_STACK_COLORATION
static void go_nx_start(void *pv, unsigned int nbytes)
  __attribute__ ((naked, no_instrument_function, noreturn));
#endif

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

#define IDLE_STACK ((uintptr_t)&_ebss+CONFIG_IDLETHREAD_STACKSIZE-4)
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

void __start(void) __attribute__ ((no_instrument_function));
#endif

/****************************************************************************
 * Name: xmc4_fpu_config
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

static inline void xmc4_fpu_config(void)
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

static inline void xmc4_fpu_config(void)
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
#  define xmc4_fpu_config()
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
 * Name: go_nx_start
 *
 * Description:
 *   Set the IDLE stack to the coloration value and jump into nx_start()
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
    "\tcmp  r1, #0\n"           /* Check (nwords == 0) */
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

  xmc4_fpu_config();
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
