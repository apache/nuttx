/****************************************************************************
 * boards/arm/rm57/rm57l843-launchxl2/src/rm57_initialize.c
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

#include <nuttx/board.h>

#include <arch/board/board.h>

#include "rm57l843-launchxl2.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_board_initialize
 *
 * Description:
 *   All RM57 architectures must provide the following entry point.  This
 *   function is called near the beginning of _start.  This function is
 *   called after clocking has been configured but before caches have been
 *   enabled and before any devices have been initialized.  .data/.bss
 *   memory may or may not have been initialized (see the "special
 *   precautions" below).
 *
 *   This function must perform low level initialization including
 *
 *   - Initialization of board-specific memory resources
 *   - Configuration of board specific resources (GPIOs, LEDs, etc).
 *   - Setup of the console UART.  This UART done early so that the serial
 *     console is available for debugging very early in the boot sequence.
 *
 *   Special precautions must be taken if .data/.bss lie in SRAM.  In that
 *   case, the boot logic cannot initialize .data or .bss.  The function
 *   must then:
 *
 *   - Take precautions to assume that logic does not access any global
 *     data that might lie in SRAM before it is initialized.
 *   - Call the function arm_data_initialize() as soon as SRAM has been
 *     properly configured for use.
 *
 ****************************************************************************/

void rm57_board_initialize(void)
{
#ifdef CONFIG_ARCH_LEDS
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
  /* Perform application level board initialization */

  rm57_bringup();
}
#endif
