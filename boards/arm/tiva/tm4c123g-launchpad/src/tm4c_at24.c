/****************************************************************************
 * boards/arm/tiva/tm4c123g-launchpad/src/tm4c_at24.c
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

/* AT24 Serial EEPROM
 *
 * A AT24C512 Serial EEPPROM was used for tested I2C.  There are no I2C
 * devices on-board the Launchpad, but an external serial EEPROM module
 * module was used.
 *
 * The Serial EEPROM was mounted on an external adaptor board and connected
 * to the LaunchPad thusly:
 *
 *   - VCC -- VCC
 *   - GND -- GND
 *   - PB2 -- SCL
 *   - PB3 -- SDA
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/nxffs.h>

#include "tiva_i2c.h"
#include "tm4c123g-launchpad.h"

#ifdef HAVE_AT24

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tm4c_at24_automount
 *
 * Description:
 *   Initialize and configure the AT24 serial EEPROM
 *
 ****************************************************************************/

int tm4c_at24_automount(int minor)
{
  FAR struct i2c_master_s *i2c;
  FAR struct mtd_dev_s *mtd;
  static bool initialized = false;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* No.. Get the I2C bus driver */

      i2c = tiva_i2cbus_initialize(AT24_BUS);
      if (!i2c)
        {
          syslog(LOG_ERR, "ERROR: Failed to initialize SPI%d\n", AT24_BUS);
          return -ENODEV;
        }

      /* Now bind the I2C interface to the AT24 I2C EEPROM driver */

      mtd = at24c_initialize(i2c);
      if (!mtd)
        {
          syslog(LOG_ERR,
                 "ERROR: Failed to bind SPI%d to the AT24 EEPROM driver\n",
                 AT24_BUS);
          return -ENODEV;
        }

#if defined(CONFIG_TM4C123G_LAUNCHPAD_AT24_FTL)
      /* And use the FTL layer to wrap the MTD driver as a block driver */

      ret = ftl_initialize(AT24_MINOR, mtd);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to initialize the FTL layer: %d\n",
                 ret);
          return ret;
        }

#elif defined(CONFIG_TM4C123G_LAUNCHPAD_AT24_NXFFS)
      /* Initialize to provide NXFFS on the MTD interface */

      ret = nxffs_initialize(mtd);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: NXFFS initialization failed: %d\n", ret);
          return ret;
        }

      /* Mount the file system at /mnt/at24 */

      ret = nx_mount(NULL, "/mnt/at24", "nxffs", 0, NULL);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to mount the NXFFS volume: %d\n",
                 ret);
          return ret;
        }
#endif

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}

#endif /* HAVE_AT24 */
