/****************************************************************************
 * config/sama5d3x-ek/src/sam_at24.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

/* AT24 Serial EEPROM
 *
 * A AT24C512 Serial EEPPROM was used for tested I2C.  There are other I2C/TWI
 * devices on-board, but the serial EEPROM is the simplest test.
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

#include <sys/mount.h>

#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/i2c.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/nxffs.h>

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
  FAR struct i2c_dev_s *i2c;
  FAR struct mtd_dev_s *mtd;
  static bool initialized = false;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* No.. Get the I2C bus driver */

      fvdbg("Initialize TWI%d\n", AT24_BUS);
      i2c = up_i2cinitialize(AT24_BUS);
      if (!i2c)
        {
          fdbg("ERROR: Failed to initialize TWI%d\n", AT24_BUS);
          return -ENODEV;
        }

      /* Now bind the I2C interface to the AT24 I2C EEPROM driver */

      fvdbg("Bind the AT24 EEPROM driver to TWI%d\n", AT24_BUS);
      mtd = at24c_initialize(i2c);
      if (!mtd)
        {
          fdbg("ERROR: Failed to bind TWI%d to the AT24 EEPROM driver\n",
               AT24_BUS);
          return -ENODEV;
        }

#if defined(CONFIG_SAMA5D3xEK_AT24_FTL)
      /* And finally, use the FTL layer to wrap the MTD driver as a block driver */

      fvdbg("Initialize the FTL layer to create /dev/mtdblock%d\n", AT24_MINOR);
      ret = ftl_initialize(AT24_MINOR, mtd);
      if (ret < 0)
        {
          fdbg("ERROR: Failed to initialize the FTL layer: %d\n", ret);
          return ret;
        }

#elif defined(CONFIG_SAMA5D3xEK_AT24_NXFFS)
      /* Initialize to provide NXFFS on the MTD interface */

      fvdbg("Initialize the NXFFS file system\n");
      ret = nxffs_initialize(mtd);
      if (ret < 0)
        {
          fdbg("ERROR: NXFFS initialization failed: %d\n", ret);
          return ret;
        }

      /* Mount the file system at /mnt/at24 */

      fvdbg("Mount the NXFFS file system at /dev/at24\n");
      ret = mount(NULL, "/mnt/at24", "nxffs", 0, NULL);
      if (ret < 0)
        {
          fdbg("ERROR: Failed to mount the NXFFS volume: %d\n", errno);
          return ret;
        }
#endif
      /* Now we are initializeed */

      initialized = true;
    }

  return OK;
}

#endif /* HAVE_AT24 */
