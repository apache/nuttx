/****************************************************************************
 * boards/arm/kinetis/teensy-3.x/src/k20_boot.c
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

#include <debug.h>
#include <sys/types.h>
#include <syslog.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "kinetis_usbotg.h"
#include "arm_internal.h"
#include "teensy-3x.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_boardinitialize
 *
 * Description:
 *   All Kinetis architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void kinetis_boardinitialize(void)
{
#if defined(CONFIG_KINETIS_SPI1) || defined(CONFIG_KINETIS_SPI2)
  /* Configure SPI chip selects if 1) SPI is not disabled, and 2) the weak
   * function kinetis_spidev_initialize() has been brought into the link.
   */

  if (kinetis_spidev_initialize)
    {
      kinetis_spidev_initialize();
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
 *   function called board_late_initialize().  board_late_initialize() will
 *   be called immediately after up_intitialize() is called and just before
 *   the initial application is started.  This additional initialization
 *   phase may be used, for example, to initialize board-specific device
 *   drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  int ret;

#if defined(CONFIG_KINETIS_I2C0) || defined(CONFIG_KINETIS_I2C1)
  kinetis_i2cdev_initialize();
#endif

#ifdef CONFIG_PWM
  /* Initialize PWM and register the PWM device. */

  ret = kinetis_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: kinetis_pwm_setup() failed: %d\n", ret);
    }
#endif

  UNUSED(ret);
}
#endif /* CONFIG_BOARD_LATE_INITIALIZE */
