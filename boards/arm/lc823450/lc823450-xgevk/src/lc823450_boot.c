/****************************************************************************
 * boards/arm/lc823450/lc823450-xgevk/src/lc823450_boot.c
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
#include <stdio.h>
#include <syslog.h>

#include <nuttx/board.h>
#include <nuttx/i2c/i2c_master.h>

#ifdef CONFIG_MTD
#  include "lc823450_mtd.h"
#endif

#include "lc823450_i2c.h"
#include "lc823450-xgevk.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lc823450_i2c_register
 *
 * Description:
 *   Register one I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#ifdef HAVE_I2CTOOL
static void lc823450_i2c_register(int bus)
{
  struct i2c_master_s *i2c;
  int ret;

  i2c = lc823450_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", bus);
    }
  else
    {
      ret = i2c_register(i2c, bus);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
                 bus, ret);
          lc823450_i2cbus_uninitialize(i2c);
        }
    }
}
#endif

/****************************************************************************
 * Name: lc823450_i2ctool
 *
 * Description:
 *   Register I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#ifdef HAVE_I2CTOOL
static void lc823450_i2ctool(void)
{
#ifdef CONFIG_LC823450_I2C0
  lc823450_i2c_register(0);
#endif
#ifdef CONFIG_LC823450_I2C1
  lc823450_i2c_register(1);
#endif
}
#else
#  define lc823450_i2ctool()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize().  board_late_initialize() will
 *   be called immediately after up_intitialize() is called and just before
 *   the initial application is started. This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  int ret;

#ifdef CONFIG_ADC
  ret = lc823450_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: lc82450_adc_setup failed: %d\n", ret);
    }
#endif

  /* Register I2C drivers on behalf of the I2C tool */

  lc823450_i2ctool();

#ifdef CONFIG_LC823450_MTD
  /* Initialize eMMC */

  ret = lc823450_mtd_initialize(CONFIG_MTD_DEVNO_EMMC);
  if (ret != OK)
    {
      syslog(LOG_ERR, "Failed to initialize eMMC: ret=%d\n", ret);
    }

#ifdef CONFIG_LC823450_SDIF_SDC
  /* Initialize uSD */

  ret = lc823450_mtd_initialize(CONFIG_MTD_DEVNO_SDC);
  if (ret != OK)
    {
      syslog(LOG_ERR, "Failed to initialize uSD: ret=%d\n", ret);
    }
#endif /* CONFIG_LC823450_SDIF_SDC */

#endif /* CONFIG_LC823450_MTD */

  UNUSED(ret); /* May not be used */

  lc823450_bringup();
}
#endif /* CONFIG_BOARD_LATE_INITIALIZE */
