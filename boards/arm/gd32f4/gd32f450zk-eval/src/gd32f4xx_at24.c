/****************************************************************************
 * boards/arm/gd32f4/gd32f450zk-eval/src/gd32f4xx_at24.c
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
#include <nuttx/kmalloc.h>

#include "gd32f4xx.h"
#include "gd32f450z_eval.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_at24_wr_test
 *
 * Description:
 *   Write and read the AT24 serial EEPROM test.
 *
 ****************************************************************************/

#ifdef HAVE_AT24

#define BUFFSIZE  16
#define START_BLOCK 0

#if BUFFSIZE>=CONFIG_AT24XX_MTD_BLOCKSIZE
#  define NBLOCK    (BUFFSIZE/CONFIG_AT24XX_MTD_BLOCKSIZE)
#else
#  error "BUFFSIZE should bigger than CONFIG_AT24XX_MTD_BLOCKSIZE"
#endif

const uint8_t write_buf[BUFFSIZE] =
{
  0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7,
  0x8, 0x9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf
};

int gd32_at24_wr_test(int minor)
{
  struct i2c_master_s *i2c;
  struct mtd_dev_s *at24;
  static bool initialized = false;
  int ret;
  ssize_t nblocks;
  uint8_t *read_buf;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* No.. Get the I2C port driver */

      finfo("Initialize TWI%d\n", AT24_BUS);
      i2c = gd32_i2cbus_initialize(AT24_BUS);
      if (!i2c)
        {
          ferr("ERROR: Failed to initialize TWI%d\n", AT24_BUS);
          return -ENODEV;
        }

      /* Now bind the I2C interface to the AT24 I2C EEPROM driver */

      finfo("Bind the AT24 EEPROM driver to TWI%d\n", AT24_BUS);
      at24 = at24c_initialize(i2c);
      if (!at24)
        {
          ferr("ERROR: Failed to bind TWI%d to the AT24 EEPROM driver\n",
               AT24_BUS);
          return -ENODEV;
        }

      /* Now we are initializeed */

      initialized = true;
    }

  /* Write start block is START_BLOCK, number of block is 2 */

  nblocks = at24->bwrite(at24, START_BLOCK, NBLOCK, write_buf);
  if (nblocks < NBLOCK)
    {
      ferr("ERROR: AT24 write failed: %zd\n", nblocks);
      gd32_i2cbus_uninitialize(i2c);
      return (int)nblocks;
    }

  read_buf = (uint8_t *)kmm_malloc(BUFFSIZE);

  /* Read the data write before */

  nblocks = at24->bread(at24, START_BLOCK, NBLOCK, read_buf);
  if (nblocks < NBLOCK)
    {
      ferr("ERROR: AT24 read failed: %zd\n", nblocks);
      gd32_i2cbus_uninitialize(i2c);
      return (int)nblocks;
    }

  if (memcmp(read_buf, write_buf, BUFFSIZE) != 0)
    {
      ferr("ERROR: Read buffer does not match write buffer\n");
      return -1;
    }

  /* Release the I2C instance.
   * REVISIT:  Need an interface to release the AT24 instance too
   */

  ret = gd32_i2cbus_uninitialize(i2c);
  if (ret < 0)
    {
      ferr("ERROR: Failed to release the I2C interface: %d\n", ret);
    }

  syslog(LOG_INFO, "INFO: I2C EEPROM write and read success: \
         %d\n", ret);

  return OK;
}

#endif /* HAVE_AT24 */
