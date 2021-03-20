/****************************************************************************
 * boards/arm/stm32/stm32f4discovery/src/stm32_boot.c
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

#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "arm_arch.h"
#include "nvic.h"
#include "itm.h"

#include "stm32.h"
#include "stm32f4discovery.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This
 *   entry point is called early in the initialization -- after all memory
 *   has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void stm32_boardinitialize(void)
{
#ifdef CONFIG_SCHED_CRITMONITOR
  /* Enable ITM and DWT resources, if not left enabled by debugger. */

  modifyreg32(NVIC_DEMCR, 0, NVIC_DEMCR_TRCENA);

  /* Make sure the high speed cycle counter is running.  It will be started
   * automatically only if a debugger is connected.
   */

  putreg32(0xc5acce55, ITM_LAR);
  modifyreg32(DWT_CTRL, 0, DWT_CTRL_CYCCNTENA_MASK);
#endif

#if defined(CONFIG_STM32_SPI1) || defined(CONFIG_STM32_SPI2) || defined(CONFIG_STM32_SPI3)
  /* Configure SPI chip selects if 1) SPI is not disabled, and 2) the weak
   * function stm32_spidev_initialize() has been brought into the link.
   */

  if (stm32_spidev_initialize)
    {
      stm32_spidev_initialize();
    }
#endif

#ifdef CONFIG_STM32_OTGFS
  /* Initialize USB if the 1) OTG FS controller is in the configuration and
   * 2) disabled, and 3) the weak function stm32_usbinitialize() has been
   * brought into the build. Presumably either CONFIG_USBDEV or
   * CONFIG_USBHOST is also selected.
   */

  if (stm32_usbinitialize)
    {
      stm32_usbinitialize();
    }
#endif

#ifdef HAVE_NETMONITOR
  /* Configure board resources to support networking. */

  if (stm32_netinitialize)
    {
      stm32_netinitialize();
    }
#endif

#ifdef CONFIG_ARCH_LEDS
  /* Configure on-board LEDs if LED support has been selected. */

  board_autoled_initialize();
#endif
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize(). board_late_initialize() will be
 *   called immediately after up_initialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  /* Perform board-specific initialization */

  stm32_bringup();
}
#endif
