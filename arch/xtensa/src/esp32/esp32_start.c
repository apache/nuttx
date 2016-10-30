/****************************************************************************
 * arch/xtensa/src/common/esp32_start.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Basic initialize sequence derives from logic originally provided by
 * Espressif Systems:
 *
 *   Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>

#include <nuttx/init.h>

#include "xtensa_attr.h"
#include "chip/esp32_dport.h"
#include "chip/esp32_rtccntl.h"
#include "esp32_clockconfig.h"
#include "esp32_region.h"
#include "esp32_start.h"
#include "xtensa.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

  /* Kill the watchdog timer */

  regval  = getreg32(RTC_CNTL_WDTCONFIG0_REG);
  regval &= ~RTC_CNTL_WDT_FLASHBOOT_MOD_EN;
  putreg32(regval, RTC_CNTL_WDTCONFIG0_REG);

  regval  = getreg32(0x6001f048); /* DR_REG_BB_BASE+48 */
  regval &= ~(1 << 14);
  putreg32(regval, 0x6001f048);

  /* Make page 0 access raise an exception */

  esp32_region_protection();

  /* Move CPU0 exception vectors to IRAM */

  asm volatile ("wsr %0, vecbase\n"::"r" (&_init_start));

  /* Set .bss to zero */

  memset(&_sbss, 0, (&_ebss - &_sbss) * sizeof(_sbss));

  /* Make sure that the APP_CPU is disabled for now */

  regval  = getreg32(DPORT_APPCPU_CTRL_B_REG);
  regval &= ~DPORT_APPCPU_CLKGATE_EN;
  putreg32(regval, DPORT_APPCPU_CTRL_B_REG);

  /* Set CPU frequency configured in board.h */

  esp32_clockconfig();

#ifdef USE_EARLYSERIALINIT
 /* Perform early serial initialization */

  xtensa_early_serial_initialize();
#endif

  /* Initialize onboard resources */

  esp32_board_initialize();

  /* Bring up NuttX */

  os_start();
}
