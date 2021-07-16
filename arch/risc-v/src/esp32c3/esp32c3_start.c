/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_start.c
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

#include "esp32c3.h"
#include "esp32c3_clockconfig.h"
#include "esp32c3_irq.h"
#include "esp32c3_lowputc.h"
#include "esp32c3_start.h"
#include "esp32c3_wdt.h"

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
  __attribute__((aligned(16), section(".noinit")));
uint32_t g_idle_topstack = ESP32C3_IDLESTACK_TOP;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __esp32c3_start
 ****************************************************************************/

void __esp32c3_start(void)
{
  uint32_t *dest;

  /* Set CPU frequency */

  esp32c3_clockconfig();

  /* Configure the UART so we can get debug output */

  esp32c3_lowsetup();

#ifdef USE_EARLYSERIALINIT
  /* Perform early serial initialization */

  riscv_earlyserialinit();
#endif

  showprogress('A');

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (dest = &_sbss; dest < &_ebss; dest++)
    {
      *dest = 0;
    }

  showprogress('B');

  /* Disable any wdt enabled by bootloader */

  esp32c3_wdt_early_deinit();

  /* Initialize onboard resources */

  esp32c3_board_initialize();

  /* Bring up NuttX */

  nx_start();

  for (; ; );
}
