/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_start.c
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

#include "arm_internal.h"
#include "nrf54l_start.h"
#include "nrf54l_clockconfig.h"
#include "nrf54l_lowputc.h"
#include "nrf54l_serial.h"

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
 * Public Function Prototypes
 ****************************************************************************/

void __start(void) noinstrument_function;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __start
 *
 * Description:
 *   Initialize memory and chip hardware before starting NuttX.
 *
 ****************************************************************************/

void __start(void)
{
  const uint32_t *src;
  uint32_t *dest;

#ifdef CONFIG_ARMV8M_STACKCHECK
  /* Set the stack limit before calling instrumented functions. */

  __asm__ volatile("sub r10, sp, %0" : :
                   "r"(CONFIG_IDLETHREAD_STACKSIZE - 64) :);
#endif

  /* Keep interrupts masked until up_irqinitialize() installs the handlers. */

  __asm__ volatile("cpsid i" : : : "memory");

  for (dest = (uint32_t *)_sbss; dest < (uint32_t *)_ebss; )
    {
      *dest++ = 0;
    }

  for (src = (const uint32_t *)_eronly, dest = (uint32_t *)_sdata;
       dest < (uint32_t *)_edata;
      )
    {
      *dest++ = *src++;
    }

#ifdef CONFIG_ARMV8M_STACKCHECK
  arm_stack_check_init();
#endif

#ifdef CONFIG_ARCH_FPU
  arm_fpuconfig();
#endif

  nrf54l_clockconfig();

  nrf54l_lowsetup();
  showprogress('A');

#ifdef CONFIG_ARCH_PERF_EVENTS
  up_perf_init((void *)BOARD_SYSTICK_CLOCK);
#endif

#if defined(CONFIG_NRF54L_UART) && defined(USE_EARLYSERIALINIT)
  nrf54l_earlyserialinit();
#endif

  showprogress('B');

  nrf54l_board_initialize();
  showprogress('C');

  showprogress('\r');
  showprogress('\n');
  nx_start();
}
