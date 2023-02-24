/****************************************************************************
 * arch/risc-v/src/espressif/esp_start.c
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

#include <nuttx/arch.h>
#include <nuttx/init.h>

#include "riscv_internal.h"

#include "esp_irq.h"
#include "esp_libc_stubs.h"
#include "esp_lowputc.h"
#include "esp_start.h"

#include "brownout.h"
#include "esp_clk_internal.h"
#include "esp_cpu.h"
#include "hal/wdt_hal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c)     riscv_lowputc(c)
#else
#  define showprogress(c)
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Address of the IDLE thread */

uint8_t g_idlestack[CONFIG_IDLETHREAD_STACKSIZE]
  aligned_data(16) locate_data(".noinit");
uintptr_t g_idle_topstack = ESP_IDLESTACK_TOP;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __esp_start
 ****************************************************************************/

void __esp_start(void)
{
#ifdef CONFIG_ESPRESSIF_REGION_PROTECTION
  /* Configure region protection */

  esp_cpu_configure_region_protection();
#endif

  /* Configures the CPU clock, RTC slow and fast clocks, and performs
   * RTC slow clock calibration.
   */

  esp_clk_init();

  /* Disable clock of unused peripherals */

  esp_perip_clk_init();

#ifdef CONFIG_ESPRESSIF_BROWNOUT_DET
  /* Initialize hardware brownout check and reset */

  esp_brownout_init();
#endif

  /* Configure the UART so we can get debug output */

  esp_lowsetup();

#ifdef USE_EARLYSERIALINIT
  /* Perform early serial initialization */

  riscv_earlyserialinit();
#endif

  showprogress('A');

  /* Clear .bss. We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (uint32_t *dest = (uint32_t *)_sbss; dest < (uint32_t *)_ebss; )
    {
      *dest++ = 0;
    }

  /* Setup the syscall table needed by the ROM code */

  esp_setup_syscall_table();

  showprogress('B');

  /* The 2nd stage bootloader enables RTC WDT to monitor any issues that may
   * prevent the startup sequence from finishing correctly. Hence disable it
   * as NuttX is about to start.
   */

  wdt_hal_context_t rwdt_ctx = RWDT_HAL_CONTEXT_DEFAULT();
  wdt_hal_write_protect_disable(&rwdt_ctx);
  wdt_hal_disable(&rwdt_ctx);
  wdt_hal_write_protect_enable(&rwdt_ctx);

  /* Initialize onboard resources */

  esp_board_initialize();

  showprogress('C');

  /* Bring up NuttX */

  nx_start();

  for (; ; );
}
