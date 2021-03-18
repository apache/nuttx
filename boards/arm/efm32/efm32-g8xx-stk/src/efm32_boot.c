/****************************************************************************
 * boards/arm/efm32/efm32-g8xx-stk/src/efm32_boot.c
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

#include <nuttx/board.h>

#include "efm32_gpio.h"
#include "efm32_start.h"
#include "efm32-g8xx-stk.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: efm32_boardinitialize
 *
 * Description:
 *   All EFM32 architectures must provide the following entry point.  This
 *   entry point is called early in the initialization before any devices
 *   have been initialized.
 *
 ****************************************************************************/

void efm32_boardinitialize(void)
{
#ifdef CONFIG_EFM32_UART0
  /*   The control MCU acts as a board controller (BC). There is a UART
   *   connection between the EFM and the BC. The connection is made by
   *   setting the EFM_BC_EN (PD13) line high. The EFM can then use the BSP
   *   to send commands to the BC. When EFM_BC_EN is low, EFM_BC_TX and
   *   EFM_BC_RX can be used by other applications.
   */

  efm32_configgpio(GPIO_BC_EN);
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
}
#endif
