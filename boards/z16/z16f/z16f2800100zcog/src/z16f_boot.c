/****************************************************************************
 * boards/z16/z16f/z16f2800100zcog/src/z16f_boot.c
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

#include "chip.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z16f_gpioinit
 *
 * Description:
 *   Configure board-specific GPIO usage here.  Driver pin configurations
 *   are set in the associated device drivers (such as UART, SPI, I2C,
 *   etc.) and must be preserved.
 *
 *   Based upon sample code included with the Zilog ZDS-II toolchain.
 *
 ****************************************************************************/

static void z16f_gpioinit(void)
{
  /* Configure LEDs and Run/Stop switch port */

  putreg8(getreg8(Z16F_GPIOA_DD) | 0x87, Z16F_GPIOA_DD);
  putreg8(getreg8(Z16F_GPIOA_OUT) | 0x07, Z16F_GPIOA_OUT);
  putreg8(getreg8(Z16F_GPIOA_DD) & 0xf8, Z16F_GPIOA_DD);

  /* Configure rate switch port */

  putreg8(getreg8(Z16F_GPIOB_DD) | 0x20, Z16F_GPIOB_DD);
  putreg8(getreg8(Z16F_GPIOB_AFL) | 0x20, Z16F_GPIOB_AFL);

#if 0 /* Not yet */
  putreg8(0x05, Z16F_ADC0_MAX);
  putreg8(0xf5, Z16F_ADC0_CTL);
#endif

  /* Configure Direction switch port */

  putreg8(getreg8(Z16F_GPIOC_DD) | 0x01, Z16F_GPIOC_DD);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z16f_board_initialize
 *
 * Description:
 *   All Z16 architectures must provide the following entry point.  This
 *   entry point is called early in the initialization -- after all memory
 *   has been configured but before any devices have been initialized.
 *
 ****************************************************************************/

void z16f_board_initialize(void)
{
  z16f_gpioinit();
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize().  board_late_initialize() will
 *   be called immediately after up_initialize() is called and just before
 *   the initial application is started.  This additional initialization
 *   phase may be used, for example, to initialize board-specific device
 *   drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
}
#endif
