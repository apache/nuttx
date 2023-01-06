/****************************************************************************
 * arch/risc-v/src/esp32c6/esp32c6_start.c
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

#include <nuttx/arch.h>
#include <nuttx/init.h>

#include <arch/board/board.h>

#include "chip.h"
#include "esp32c6.h"
#include "esp32c6_irq.h"
#include "esp32c6_lowputc.h"
#include "esp32c6_clockconfig.h"

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

/* Address of the IDLE thread */

uint8_t g_idlestack[CONFIG_IDLETHREAD_STACKSIZE]
  aligned_data(16) locate_data(".noinit");
uintptr_t g_idle_topstack = ESP32C6_IDLESTACK_TOP;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __esp32c6_start
 ****************************************************************************/

void __esp32c6_start(void)
{
  /* Set CPU frequency */

  esp32c6_clockconfig();

  /* Configure the UART so we can get debug output */

  esp32c6_lowsetup();

  showprogress('A');

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (uint32_t *dest = (uint32_t *)_sbss; dest < (uint32_t *)_ebss; )
    {
      *dest++ = 0;
    }

  showprogress('B');

#ifndef CONFIG_SUPPRESS_INTERRUPTS

  /* Put the CPU Interrupts in initial state */

  esp32c6_cpuint_initialize();
#endif

  /* Call nx_start() */

  nx_start();

  for (; ; );
}
