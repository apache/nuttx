/****************************************************************************
 * arch/xtensa/src/esp32/esp32_start.c
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
#include <string.h>
#include <assert.h>

#include <nuttx/init.h>
#include <nuttx/irq.h>

#include "xtensa.h"
#include "xtensa_attr.h"

#include "hardware/esp32_dport.h"
#include "hardware/esp32_rtccntl.h"
#include "esp32_clockconfig.h"
#include "esp32_region.h"
#include "esp32_start.h"
#include "esp32_spiram.h"
#include "esp32_wdt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c) up_puts(c)
#else
#  define showprogress(c)
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Address of the CPU0 IDLE thread */

uint32_t g_idlestack[IDLETHREAD_STACKWORDS]
  __attribute__((aligned(16), section(".noinit")));

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifndef CONFIG_SUPPRESS_UART_CONFIG
extern void esp32_lowsetup(void);
#endif

/****************************************************************************
 * Name: __start
 *
 * Description:
 *   We arrive here after the bootloader finished loading the program from
 *   flash. The hardware is mostly uninitialized, and the app CPU is in
 *   reset. We do have a stack, so we can do the initialization in C.
 *
 *   The app CPU will remain in reset unless CONFIG_SMP is selected and
 *   up_cpu_start() is called later in the bring-up sequeuence.
 *
 ****************************************************************************/

void IRAM_ATTR __start(void)
{
  uint32_t regval;
  uint32_t sp;

  /* Make sure that normal interrupts are disabled.  This is really only an
   * issue when we are started in un-usual ways (such as from IRAM).  In this
   * case, we can at least defer some unexpected interrupts left over from
   * the last program execution.
   */

  up_irq_disable();

  /* Move the stack to a known location.  Although we were given a stack
   * pointer at start-up, we don't know where that stack pointer is
   * positioned with respect to our memory map.  The only safe option is to
   * switch to a well-known IDLE thread stack.
   */

  sp = (uint32_t)g_idlestack + IDLETHREAD_STACKSIZE;
  __asm__ __volatile__("mov sp, %0\n" : : "r"(sp));

  /* Make page 0 access raise an exception */

  esp32_region_protection();

  /* Move CPU0 exception vectors to IRAM */

  __asm__ __volatile__ ("wsr %0, vecbase\n"::"r" (&_init_start));

  /* Set .bss to zero */

  memset(&_sbss, 0, (&_ebss - &_sbss) * sizeof(_sbss));

  /* Make sure that the APP_CPU is disabled for now */

  regval  = getreg32(DPORT_APPCPU_CTRL_B_REG);
  regval &= ~DPORT_APPCPU_CLKGATE_EN;
  putreg32(regval, DPORT_APPCPU_CTRL_B_REG);

  /* The 2nd stage bootloader enables RTC WDT to check on startup sequence
   * related issues in application. Hence disable that as we are about to
   * start the NuttX environment.
   */

  esp32_wdt_early_deinit();

  /* Set CPU frequency configured in board.h */

  esp32_clockconfig();

#ifndef CONFIG_SUPPRESS_UART_CONFIG
  /* Configure the UART so we can get debug output */

  esp32_lowsetup();
#endif

#ifdef USE_EARLYSERIALINIT
  /* Perform early serial initialization */

  xtensa_earlyserialinit();
#endif

  showprogress("A");

#if defined(CONFIG_ESP32_SPIRAM_BOOT_INIT)
  esp_spiram_init_cache();
  if (esp_spiram_init() != OK)
    {
#  if defined(ESP32_SPIRAM_IGNORE_NOTFOUND)
      mwarn("SPIRAM Initialization failed!\n");
#  else
      PANIC();
#  endif
    }

  /* Set external memory bss section to zero */

#  ifdef CONFIG_XTENSA_EXTMEM_BSS
     memset(&_sbss_extmem, 0,
            (&_ebss_extmem - &_sbss_extmem) * sizeof(_sbss_extmem));
#  endif

#endif

  /* Initialize onboard resources */

  esp32_board_initialize();

  showprogress("B");

  /* Bring up NuttX */

  nx_start();
  for (; ; ); /* Should not return */
}
