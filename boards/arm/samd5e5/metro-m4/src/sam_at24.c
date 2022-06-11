/****************************************************************************
 * boards/arm/samd5e5/metro-m4/src/sam_at24.c
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
#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/nxffs.h>

#include "metro-m4.h"

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

extern struct i2c_master_s *g_i2c5_dev;

int sam_at24_automount(int minor)
{
  struct mtd_dev_s *mtd;
  static bool initialized = false;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* Now bind the I2C interface to the AT24 I2C EEPROM driver */

      finfo("Bind the AT24 EEPROM driver to I2C%d\n", AT24_I2C_BUS);
      mtd = at24c_initialize(g_i2c5_dev);
      if (!mtd)
        {
          ferr("ERROR: Failed to bind TWI%d to the AT24 EEPROM driver\n",
               AT24_I2C_BUS);
          return -ENODEV;
        }

#if defined(CONFIG_METRO_M4_AT24_FTL)
      /* And finally, use the FTL layer
       * to wrap the MTD driver as a block driver
       */

      finfo("Initialize the FTL layer to create /dev/mtdblock%d\n",
            AT24_MINOR);
      ret = ftl_initialize(AT24_MINOR, mtd);
      if (ret < 0)
        {
          ferr("ERROR: Failed to initialize the FTL layer: %d\n", ret);
          return ret;
        }

#elif defined(CONFIG_METRO_M4_AT24_NXFFS)
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
          ferr("ERROR: Failed to mount the NXFFS volume: %d\n",
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
