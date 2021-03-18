/****************************************************************************
 * boards/arm/tms570/tms570ls31x-usb-kit/src/tms570_initialize.c
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

#include "tms570ls31x_usb_kit.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tms570_boardinitialize
 *
 * Description:
 *   All TMS570 architectures must provide the following entry point.  This
 *   function is called near the beginning of _start.  This function is
 *   called after clocking has been configured but before caches have been
 *   enabled and before any devices have been initialized.  .data/.bss
 *   memory may or may not have been initialized (see the "special
 *   precautions" below).
 *
 *   This function must perform low level initialization including
 *
 *   - Initialization of board-specific memory resources (e.g., SDRAM)
 *   - Configuration of board specific resources (GPIOs, LEDs, etc).
 *   - Setup of the console UART.  This UART done early so that the serial
 *     console is available for debugging very early in the boot sequence.
 *
 *   Special precautions must be taken if .data/.bss lie in SRAM.  in that
 *   case, the boot logic cannot initialize .data or .bss.  The function
 *   must then:
 *
 *   - Take precautions to assume that logic does not access any global
 *     data that might lie in SDRAM.
 *   - Call the function arm_data_initialize() as soon as SDRAM has been
 *     properly configured for use.
 *
 ****************************************************************************/

void tms570_board_initialize(void)
{
#ifdef CONFIG_ARCH_LEDS
  board_autoled_initialize();
#endif

#if 0
  putreg32(0x01000001, 0xfff7b800 + 0);

  /* - PULL functionality is enabled */

  putreg32(0x00000000, 0xfff7b800 + 0x64);

  /* - This if statement is a placeholder for ArgoBoard/USBStick check */

  /* - Configure NHET pins as output */

  putreg32(0xaa178035, 0xfff7b800 + 0x004c);

  /* - Turn all LEDs off */

  putreg32(0x08110034, 0xfff7b800 + 0x0054);

  /* - Set only NHET[25,0] */

  putreg32(0x0a110035, 0xfff7b800 + 0x0054);
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

  tms570_bringup();
}
#endif
