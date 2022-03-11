/****************************************************************************
 * boards/arm/nuc1xx/nutiny-nuc120/src/nuc_boardinitialize.c
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

#include "arm_internal.h"
#include "nutiny-nuc120.h"

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
 * Name: nuc_boardinitialize
 *
 * Description:
 *   All NUC1XX architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void nuc_boardinitialize(void)
{
  /* Configure SPI chip selects if 1) SPI is not disabled, and 2) the weak
   * function nuc_spidev_initialize() has been brought into the link.
   */

#if defined(CONFIG_NUC1XX_SPI1) || defined(CONFIG_NUC1XX_SPI2) || defined(CONFIG_NUC1XX_SPI3)
  if (nuc_spidev_initialize)
    {
      nuc_spidev_initialize();
    }
#endif

  /* Initialize USB if the 1) USB device controller is in the configuration
   * and 2) disabled, and 3) the weak function nuc_usbinitialize() has been
   * brought into the build.
   * Presumeably either CONFIG_USBDEV is also selected.
   */

#ifdef CONFIG_NUC1XX_USB
  if (nuc_usbinitialize)
    {
      nuc_usbinitialize();
    }
#endif

  /* Configure on-board LED if LED support has been selected. */

#ifdef CONFIG_ARCH_LEDS
  nuc_led_initialize();
#endif
}
