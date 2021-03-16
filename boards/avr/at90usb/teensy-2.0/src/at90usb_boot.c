/****************************************************************************
 * boards/avr/at90usb/teensy-2.0/src/at90usb_boot.c
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

#include "up_arch.h"
#include "up_internal.h"

#include "at90usb.h"
#include "teensy-20.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: at90usb_boardinitialize
 *
 * Description:
 *   All AT90USB architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after
 *   all memory has been configured and mapped but before any devices
 *   have been initialized.
 *
 ****************************************************************************/

void at90usb_boardinitialize(void)
{
#ifdef CONFIG_AVR_SPI
  /* Configure SSP chip selects if 1) at least one SSP is enabled, and 2) the
   * weak function at90usb_spidev_initialize() has been brought into the
   * link.
   */

  if (at90usb_spidev_initialize)
    {
      at90usb_spidev_initialize();
    }
#endif

#ifdef CONFIG_ARCH_LEDS
  /* Configure on-board LEDs if LED support has been selected. */

  at90usb_led_initialize();
#endif
}
