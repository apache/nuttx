/****************************************************************************
 * boards/arm/lpc43xx/lpc4370-link2/src/lpc43_boot.c
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

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/i2c/i2c_master.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "lpc4370-link2.h"
#include "lpc43_i2c.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_i2c_register
 *
 * Description:
 *   Register one I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#ifdef HAVE_I2CTOOL
static void lpc43_i2c_register(int bus)
{
  struct i2c_master_s *i2c;
  int ret;

  i2c = lpc43_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      _err("ERROR: Failed to get I2C%d interface\n", bus);
    }
  else
    {
      ret = i2c_register(i2c, bus);
      if (ret < 0)
        {
          _err("ERROR: Failed to register I2C%d driver: %d\n", bus, ret);
          lpc43_i2cbus_uninitialize(i2c);
        }
    }
}
#endif

/****************************************************************************
 * Name: lpc43_i2ctool
 *
 * Description:
 *   Register I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#ifdef HAVE_I2CTOOL
static void lpc43_i2ctool(void)
{
#ifdef CONFIG_LPC43_I2C0
  lpc43_i2c_register(0);
#endif
#ifdef CONFIG_STM32_I2C1
  lpc43_i2c_register(1);
#endif
}
#else
#  define lpc43_i2ctool()
#endif


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_boardinitialize
 *
 * Description:
 *   All LPC43xx architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void lpc43_boardinitialize(void)
{
  /* Configure on-board LEDs if LED support has been selected. */

#ifdef CONFIG_ARCH_LEDS
  board_autoled_initialize();
#endif

#ifdef CONFIG_SPIFI_LIBRARY
  board_spifi_initialize();
#endif
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize().
 *   board_late_initialize() will be called immediately after up_initialize()
 *   is called and just before the initial application is started.
 *   This additional initialization phase may be used, for example, to
 *   initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  int ret;

  /* Register I2C drivers on behalf of the I2C tool */

  lpc43_i2ctool();

#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC driver. */

  ret = lpc43_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: lpc43_adc_setup failed: %d\n", ret);
    }
#endif

  UNUSED(ret);
}
#endif
