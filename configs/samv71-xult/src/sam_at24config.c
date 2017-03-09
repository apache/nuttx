/************************************************************************************
 * configs/samv71-xult/src/sam_at24config.c
 *
 *   Copyright (C) 2015-2016 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/mtd/configdata.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/ioctl.h>

#include "sam_twihs.h"
#include "samv71-xult.h"

#ifdef HAVE_MTDCONFIG

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: sam_at24config
 *
 * Description:
 *   Create an AT24xx-based MTD configuration device for storage device configuration
 *   information.
 *
 ************************************************************************************/

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
      (void)sam_i2cbus_uninitialize(i2c);
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
      (void)sam_i2cbus_uninitialize(i2c);
    }

  return ret;
}

#endif /* HAVE_MTDCONFIG */
