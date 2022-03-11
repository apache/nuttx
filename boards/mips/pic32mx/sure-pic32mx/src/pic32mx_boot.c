/****************************************************************************
 * boards/mips/pic32mx/sure-pic32mx/src/pic32mx_boot.c
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

#include <arch/board/board.h>

#include "mips_internal.h"
#include "pic32mx.h"
#include "sure-pic32mx.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mx_boardinitialize
 *
 * Description:
 *   All PIC32MX architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void pic32mx_boardinitialize(void)
{
  /* Configure SPI chip selects if 1) at least one SPI is enabled, and 2)
   * the weak function pic32mx_spidev_initialize() has been brought into
   * the link.
   */

#if defined(CONFIG_PIC32MX_SPI2)
  if (pic32mx_spidev_initialize)
    {
      pic32mx_spidev_initialize();
    }
#endif

  /* Configure the mini-B PHY on the Sure PIC32MX board for the USB device */

#if defined(CONFIG_PIC32MX_USBDEV)
  if (pic32mx_usbdevinitialize)
    {
      pic32mx_usbdevinitialize();
    }
#endif

  /* Configure on-board LEDs if LED support has been selected. */

#ifdef CONFIG_ARCH_LEDS
  pic32mx_led_initialize();
#endif
}
