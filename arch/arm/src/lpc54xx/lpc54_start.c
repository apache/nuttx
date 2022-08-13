/****************************************************************************
 * arch/arm/src/lpc54xx/lpc54_start.c
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

#include "hardware/lpc54_syscon.h"
#include "lpc54_clockconfig.h"
#include "lpc54_userspace.h"
#include "lpc54_lowputc.h"
#include "lpc54_serial.h"
#include "lpc54_start.h"

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
 * Private Data
 ****************************************************************************/

/* This describes the initial PLL configuration */

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
  uint32_t regval;

  /* Make sure that interrupts are disabled */

  __asm__ __volatile__ ("\tcpsid  i\n");

  /* Enable the SRAM clock to make the stack usable */

  regval = (SYSCON_AHBCLKCTRL0_SRAM1 | SYSCON_AHBCLKCTRL0_SRAM2 |
            SYSCON_AHBCLKCTRL0_SRAM3);
  putreg32(regval, LPC54_SYSCON_AHBCLKCTRLSET0);

  /* Configure the clocking and the console uart so that we can get debug
   * output as soon as possible.  NOTE: That this logic must not assume that
   * .bss or .data have beeninitialized.
   */

  lpc54_clockconfig(&g_initial_pll_setup);
  lpc54_lowsetup();
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

  arm_fpuconfig();
  showprogress('D');

  /* Perform early serial initialization */

#ifdef USE_EARLYSERIALINIT
  lpc54_earlyserialinit();
#endif
  showprogress('E');

  /* For the case of the separate user-/kernel-space build, perform whatever
   * platform specific initialization of the user memory is required.
   * Normally this just means initializing the user space .data and .bss
   * segments.
   */

#ifdef CONFIG_BUILD_PROTECTED
  lpc54_userspace();
  showprogress('F');
#endif

  /* Initialize onboard resources */

  lpc54_board_initialize();
  showprogress('G');

  /* Then start NuttX */

  showprogress('\r');
  showprogress('\n');
  nx_start();

  /* Shouldn't get here */

  for (; ; );
}
