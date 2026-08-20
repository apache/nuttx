/****************************************************************************
 * boards/arm/stm32h7/linum-stm32h753bi/src/stm32_boot.c
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

#include <nuttx/debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "linum-stm32h753bi.h"

#if defined(CONFIG_ARM_MPU) && defined(BOARD_SDRAM1_SIZE)
#  include "mpu.h"
#  include "hardware/stm32_memorymap.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

#if defined(CONFIG_ARM_MPU) && defined(BOARD_SDRAM1_SIZE)

/****************************************************************************
 * Name: linum_sdram_cacheable
 *
 * Description:
 *   Let the caches work on the external SDRAM.
 *
 *   It sits at 0xc0000000, and the Cortex-M7 default memory map calls
 *   that range External Device:  uncached, and every write strongly
 *   ordered.  Anything held there is then read and written at the speed
 *   of the SDRAM bus with no cache in front of it, a comparison of a
 *   128 KiB buffer measured 97 ms there against 6 ms in internal SRAM,
 *   and a graphics stack drawing into a framebuffer in SDRAM pays that
 *   on every pixel.
 *
 *   Nothing else is reading the memory behind the CPU's back:  the
 *   framebuffer this board can put there is drawn and read by software
 *   on the same core, and the data cache is write-through, so what the
 *   LTDC would see if it were enabled is what was written.
 *
 ****************************************************************************/

static void linum_sdram_cacheable(void)
{
  mpu_configure_region(STM32_FMC_BANK5, BOARD_SDRAM1_SIZE,
                       MPU_RASR_TEX_NOR |    /* Normal memory        */
                       MPU_RASR_C       |    /* Cacheable            */
                       MPU_RASR_B       |    /* Bufferable           */
                       MPU_RASR_AP_RWRW);    /* Read and write       */

  /* Everything outside the region keeps the default map */

  mpu_control(true, false, true);
}
#endif

void stm32_boardinitialize(void)
{
#if defined(CONFIG_ARM_MPU) && defined(BOARD_SDRAM1_SIZE)
  linum_sdram_cacheable();
#endif

#ifdef CONFIG_ARCH_LEDS
  /* Configure on-board LEDs if LED support has been selected. */

  board_autoled_initialize();
#endif

#ifdef CONFIG_STM32_OTGFS
  /* Initialize USB-related GPIOs (VBUS sensing, power switch enable and
   * over-current) so the OTG FS port can drive VBUS in host mode.
   */

  stm32_usbinitialize();
#endif
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize().  board_late_initialize()
 *   will be called immediately after up_initialize() is called and just
 *   before the initial application is started.  This additional
 *   initialization phase may be used, for example, to initialize board-
 *   specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  stm32_bringup();
}
#endif
