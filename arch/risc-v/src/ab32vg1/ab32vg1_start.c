/****************************************************************************
 * arch/risc-v/src/ab32vg1/ab32vg1_start.c
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

#include <nuttx/init.h>
#include <arch/board/board.h>

#include "riscv_internal.h"
#include "chip.h"
#include "ab32vg1.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c) riscv_lowputc(c)
#else
#  define showprogress(c)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __ab32vg1_start
 *
 * Description:
 *   C entry point, called from __start with the idle stack in place.  The
 *   whole image (code and initialized data) has already been placed in RAM
 *   by the loader, so only .bss needs clearing.
 *
 ****************************************************************************/

void __ab32vg1_start(void)
{
  uint32_t *dest;

  /* The watchdog may have been left running by the boot code */

  putreg32(WDTCON_DISABLE, AB32VG1_WDTCON);

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (dest = (uint32_t *)_sbss; dest < (uint32_t *)_ebss; )
    {
      *dest++ = 0;
    }

  /* Setup the system clock */

  ab32vg1_clockconfig();

  /* Configure the UART so we can get debug output */

  ab32vg1_lowsetup();

  showprogress('A');

#ifdef USE_EARLYSERIALINIT
  riscv_earlyserialinit();
#endif

  showprogress('B');

  /* Do board initialization */

  ab32vg1_boardinitialize();

  showprogress('C');

  /* Call nx_start() */

  nx_start();

  /* Shouldn't get here */

  for (; ; );
}
