/****************************************************************************
 * boards/arm/sama5/sama5d3x-ek/src/sam_at24.c
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
 * A AT24C512 Serial EEPPROM was used for tested I2C.  There are other
 * I2C/TWI devices on-board, but the serial EEPROM is the simplest test.
 *
 * There is, however, no AT24 EEPROM on board the SAMA5D3x-EK:  The Serial
 * EEPROM was mounted on an external adaptor board and connected to the
 * SAMA5D3x-EK thusly:
 *
 *   - VCC -- VCC
 *   - GND -- GND
 *   - TWCK0(PA31) -- SCL
 *   - TWD0(PA30)  -- SDA
 *
 * By default, PA30 and PA31 are SWJ-DP pins, it can be used as a pin for TWI
 * peripheral in the end application.
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

#include "sam_twi.h"
#include "sama5d3x-ek.h"

#ifdef HAVE_AT24

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_at24_automount
 *
 * Description:
 *   Initialize and configure the AT24 serial EEPROM
 *
 ****************************************************************************/

int sam_at24_automount(int minor)
{
  struct i2c_master_s *i2c;
  struct mtd_dev_s *mtd;
  static bool initialized = false;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* No.. Get the I2C bus driver */

      finfo("Initialize TWI%d\n", AT24_BUS);
      i2c = sam_i2cbus_initialize(AT24_BUS);
      if (!i2c)
        {
          ferr("ERROR: Failed to initialize TWI%d\n", AT24_BUS);
          return -ENODEV;
        }

      /* Now bind the I2C interface to the AT24 I2C EEPROM driver */

      finfo("Bind the AT24 EEPROM driver to TWI%d\n", AT24_BUS);
      mtd = at24c_initialize(i2c);
      if (!mtd)
        {
          ferr("ERROR: Failed to bind TWI%d to the AT24 EEPROM driver\n",
               AT24_BUS);
          return -ENODEV;
        }

#if defined(CONFIG_SAMA5D3XEK_AT24_FTL)
      /* And use the FTL layer to wrap the MTD driver as a block driver */

      finfo("Initialize the FTL layer to create /dev/mtdblock%d\n",
            AT24_MINOR);
      ret = ftl_initialize(AT24_MINOR, mtd);
      if (ret < 0)
        {
          ferr("ERROR: Failed to initialize the FTL layer: %d\n", ret);
          return ret;
        }

#elif defined(CONFIG_SAMA5D3XEK_AT24_NXFFS)
      /* Initialize to provide NXFFS on the MTD interface */

      finfo("Initialize the NXFFS file system\n");
      ret = nxffs_initialize(mtd);
      if (ret < 0)
        {
          ferr("ERROR: NXFFS initialization failed: %d\n", ret);
          return ret;
        }

      /* Mount the file system at /mnt/at24 */

      finfo("Mount the NXFFS file system at /dev/at24\n");
      ret = nx_mount(NULL, "/mnt/at24", "nxffs", 0, NULL);
      if (ret < 0)
        {
          ferr("ERROR: Failed to mount the NXFFS volume: %d\n", ret);
          return ret;
        }
#endif

      /* Now we are initializeed */

      initialized = true;
    }

  return OK;
}

#endif /* HAVE_AT24 */
