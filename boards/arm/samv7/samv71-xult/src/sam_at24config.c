/****************************************************************************
 * boards/arm/samv7/samv71-xult/src/sam_at24config.c
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

#include <errno.h>
#include <debug.h>

#include <nuttx/mtd/configdata.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/ioctl.h>

#include "sam_twihs.h"
#include "samv71-xult.h"

#ifdef HAVE_MTDCONFIG

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_at24config
 *
 * Description:
 *   Create an AT24xx-based MTD configuration device for storage device
 *   configuration information.
 *
 ****************************************************************************/

int sam_at24config(void)
{
  struct i2c_master_s *i2c;
  struct mtd_dev_s *at24;
  int ret;

  /* Get an instance of the TWI0 interface */

  i2c = sam_i2cbus_initialize(0);
  if (!i2c)
    {
      ferr("ERROR: Failed to initialize TWI0\n");
      return -ENODEV;
    }

  /* Initialize the AT24 driver */

  at24 = at24c_initialize(i2c);
  if (!at24)
    {
      ferr("ERROR: Failed to initialize the AT24 driver\n");
      sam_i2cbus_uninitialize(i2c);
      return -ENODEV;
    }

  /* Make sure that the AT24 is in normal memory access mode */

  ret = at24->ioctl(at24, MTDIOC_EXTENDED, 0);
  if (ret < 0)
    {
      ferr("ERROR: AT24 ioctl(MTDIOC_EXTENDED) failed: %d\n", ret);
    }

  /* Bind the instance of an MTD device to the /dev/config device. */

  ret = mtdconfig_register(at24);
  if (ret < 0)
    {
      ferr("ERROR: Failed to bind AT24 driver to the MTD config device\n");
      sam_i2cbus_uninitialize(i2c);
    }

  return ret;
}

#endif /* HAVE_MTDCONFIG */
