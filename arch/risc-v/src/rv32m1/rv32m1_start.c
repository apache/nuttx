/****************************************************************************
 * arch/risc-v/src/rv32m1/rv32m1_start.c
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

#include <nuttx/init.h>
#include <arch/board/board.h>

#include "riscv_internal.h"
#include "rv32m1_clockconfig.h"
#include "rv32m1.h"
#include "rv32m1_gpio.h"
#include "rv32m1_lowputc.h"
#include "hardware/rv32m1_wdog.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c) riscv_lowputc(c)
#else
#  define showprogress(c)
#endif

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

uintptr_t g_idle_topstack = RV32M1_IDLESTACK_TOP;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __rv32m1_start
 ****************************************************************************/

void __rv32m1_start(void)
{
  const uint32_t *src;
  uint32_t *dest;

  uint32_t regval;

  /* Watch dog starts automatically after System resets. Stop it */

  putreg32(WDOG_CNT_UNLOCK, RV32M1_WDOG_CNT);
  putreg32(0xffff, RV32M1_WDOG_TOVAL);

  regval = getreg32(RV32M1_WDOG_CS);
  regval &= ~WDOG_CS_EN;
  regval |= WDOG_CS_UPDATE;
  putreg32(regval, RV32M1_WDOG_CS);

#ifdef CONFIG_RV32M1_ITCM

  src = &_slitcm;
  dest = &_svitcm;

  if (src != dest)
    {
      /* Copy codes from Flash LMA Region to ITCM VMA Region */

      for (; dest < &_evitcm; )
        {
          *dest++ = *src++;
        }
    }

#endif

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

  /* Setup clock */

  rv32m1_clockconfig();

  /* Configure the UART so we can get debug output */

  rv32m1_lowsetup();

  showprogress('A');

#ifdef USE_EARLYSERIALINIT
  riscv_earlyserialinit();
#endif

  showprogress('B');

  /* Do board initialization */

  rv32m1_boardinitialize();

  showprogress('C');

  /* Call nx_start() */

  nx_start();

  /* Shouldn't get here */

  for (; ; );
}
