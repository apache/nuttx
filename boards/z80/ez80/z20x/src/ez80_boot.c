/****************************************************************************
 * boards/z80/ez80/z20x/src/ez80_boot.c
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

#include <stdbool.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mm/mm.h>
#include <arch/chip/io.h>

#include "chip.h"
#include "z80_internal.h"
#include "z20x.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ez80_board_initialize
 *
 * Description:
 *   All eZ80 architectures must provide the following entry point.  This
 *   entry point is called early in the initialization -- after basic CPU
 *   configuration is complete but before any devices have been initialized.
 *
 ****************************************************************************/

void ez80_board_initialize(void)
{
#ifdef CONFIG_Z20X_PROGRAM
  /* Recover memory used by the bootloader */

  kmm_addregion((FAR void *)PROGSTART, PROGSIZE);
#endif

#ifdef CONFIG_EZ80_SPI
  /* Initialize SPI chip selects */

  ez80_spidev_initialize();
#endif
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
  /* Perform board-specific initialization */

  ez80_bringup();
}
#endif
