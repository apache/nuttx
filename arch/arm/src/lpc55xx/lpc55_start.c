/****************************************************************************
 * arch/arm/src/lpc55xx/lpc55_start.c
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
 * under the License. SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
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

#include "hardware/lpc55_syscon.h"
//#include "lpc55_clockconfig.h"
//#include "lpc55_userspace.h"
//#include "lpc55_lowputc.h"
//#include "lpc55_serial.h"
#include "lpc55_start.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/


/*****************  REMOVE LATER ***************/

/* This heap/stack allocation logic should be moved once we have
 * the lpc55_allocateheap.c logic ported
 */

/* .data is positioned first in the primary RAM followed immediately by .bss.
 * The IDLE thread stack lies just after .bss and has size give by
 * CONFIG_IDLETHREAD_STACKSIZE;  The heap then begins just after the IDLE.
 * ARM EABI requires 64 bit stack alignment.
 */

#define IDLE_STACKSIZE (CONFIG_IDLETHREAD_STACKSIZE & ~7)
#define IDLE_STACK     ((uintptr_t)&_ebss + IDLE_STACKSIZE)
#define HEAP_BASE      ((uintptr_t)&_ebss + IDLE_STACKSIZE)

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
/*****************  END REMOVE LATER ***************/


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
 * Private Data
 ****************************************************************************/

/* This describes the initial PLL configuration */

#if 0
static const struct pll_setup_s g_initial_pll_setup =
{
  .pllclksel = BOARD_PLL_CLKSEL,
  .pllctrl   = SYSCON_SYSPLLCTRL_SELR(BOARD_PLL_SELR) |
               SYSCON_SYSPLLCTRL_SELI(BOARD_PLL_SELI) |
               SYSCON_SYSPLLCTRL_SELP(BOARD_PLL_SELP),
  .pllmdec   = (SYSCON_SYSPLLMDEC_MDEC(BOARD_PLL_MDEC)),
  .pllndec   = (SYSCON_SYSPLLNDEC_NDEC(BOARD_PLL_NDEC)),
  .pllpdec   = (SYSCON_SYSPLLPDEC_PDEC(BOARD_PLL_PDEC)),
  .pllfout   = BOARD_PLL_FOUT,
  .pllflags  = PLL_SETUPFLAG_WAITLOCK | PLL_SETUPFLAG_POWERUP,
  .ahbdiv    = SYSCON_AHBCLKDIV_DIV(BOARD_AHBCLKDIV)
};
#endif
/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc55_fpuconfig
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
 *  Software must not change the value of the ASPEN bit or LSPEN bit while either:
 *   - the CPACR permits access to CP10 and CP11, that give access to the FP
 *     extension, or
 *   - the CONTROL.FPCA bit is set to 1
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_FPU
#ifndef CONFIG_ARMV7M_LAZYFPU
static inline void lpc55_fpuconfig(void)
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
static inline void lpc55_fpuconfig(void)
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
#  define lpc55_fpuconfig()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*****************  REMOVE LATER ***************/

/* Dumping filler functions in here as we add support */

void board_autoled_on(int led) {}
void board_autoled_off(int led) {}
void up_allocate_heap(FAR void **heap_start, size_t *heap_size) {}
void up_timer_initialize(void) {}
void up_putc(char c) {}
void arm_serialinit(void) {}
/*****************  END REMOVE LATER ***************/

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
  uint32_t regval;

  /* Make sure that interrupts are disabled */

  __asm__ __volatile__ ("\tcpsid  i\n");

  /* Enable the SRAM clock to make the stack usable */

  regval = (SYSCON_AHBCLKCTRL0_SRAM1 | SYSCON_AHBCLKCTRL0_SRAM2 |
            SYSCON_AHBCLKCTRL0_SRAM3 | SYSCON_AHBCLKCTRL0_SRAM4);
  putreg32(regval, LPC55_SYSCON_AHBCLKCTRLSET0);

  /* Configure the clocking and the console uart so that we can get debug
   * output as soon as possible.  NOTE: That this logic must not assume that
   * .bss or .data have beeninitialized.
   */
#if 1 /* We will keep moving this as more of the port comes up */
}
#elif
  lpc55_clockconfig(&g_initial_pll_setup);
  lpc55_lowsetup();
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

  /* Initialize the FPU (if configured) */

  lpc55_fpuconfig();
  showprogress('D');

  /* Perform early serial initialization */

#ifdef USE_EARLYSERIALINIT
  lpc55_earlyserialinit();
#endif
  showprogress('E');

  /* For the case of the separate user-/kernel-space build, perform whatever
   * platform specific initialization of the user memory is required.
   * Normally this just means initializing the user space .data and .bss
   * segments.
   */

#ifdef CONFIG_BUILD_PROTECTED
  lpc55_userspace();
  showprogress('F');
#endif

  /* Initialize onboard resources */

  lpc55_board_initialize();
  showprogress('G');

  /* Then start NuttX */

  showprogress('\r');
  showprogress('\n');
  nx_start();

  /* Shouldn't get here */

  for (; ; );
}
#endif
