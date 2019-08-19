/****************************************************************************
 * boards/arm/stm32/stm32f103-minimum/src/stm32_at24.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/mount.h>

#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/nxffs.h>

#include "stm32_i2c.h"
#include "stm32f103_minimum.h"

#ifdef HAVE_AT24

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_at24_automount
 *
 * Description:
 *   Initialize and configure the AT24 serial EEPROM
 *
 ****************************************************************************/

int stm32_at24_automount(int minor)
{
  FAR struct i2c_master_s *i2c;
  FAR struct mtd_dev_s *mtd;
  static bool initialized = false;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* No.. Get the I2C bus driver */

      finfo("Initialize I2C%d\n", AT24_I2C_BUS);
      i2c = stm32_i2cbus_initialize(AT24_I2C_BUS);
      if (!i2c)
        {
          ferr("ERROR: Failed to initialize I2C%d\n", AT24_I2C_BUS);
          return -ENODEV;
        }

      /* Now bind the I2C interface to the AT24 I2C EEPROM driver */

      finfo("Bind the AT24 EEPROM driver to I2C%d\n", AT24_I2C_BUS);
      mtd = at24c_initialize(i2c);
      if (!mtd)
        {
          ferr("ERROR: Failed to bind TWI%d to the AT24 EEPROM driver\n",
               AT24_I2C_BUS);
          return -ENODEV;
        }

#if defined(CONFIG_STM32F103MINIMUM_AT24_FTL)
      /* And finally, use the FTL layer to wrap the MTD driver as a block driver */

      finfo("Initialize the FTL layer to create /dev/mtdblock%d\n", AT24_MINOR);
      ret = ftl_initialize(AT24_MINOR, mtd);
      if (ret < 0)
        {
          ferr("ERROR: Failed to initialize the FTL layer: %d\n", ret);
          return ret;
        }

#elif defined(CONFIG_STM32F103MINIMUM_AT24_NXFFS)
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
      ret = mount(NULL, "/mnt/at24", "nxffs", 0, NULL);
      if (ret < 0)
        {
          ferr("ERROR: Failed to mount the NXFFS volume: %d\n", errno);
          return ret;
        }
#endif
      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}

#endif /* HAVE_AT24 */
