/****************************************************************************
 * configs/lpcxpresso-lpc54628/src/lpc54_i2c.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include <syslog.h>

#include <nuttx/i2c/i2c_master.h>

#include "lpc54_config.h"
#include "lpc54_i2c_master.h"
#include "lpcxpresso-lpc54628.h"

#if defined(HAVE_I2CTOOL) || defined(HAVE_FT5x06)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct i2c_master_s *g_i2c_handle[NI2C];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_i2c_handle
 *
 * Description:
 *   Create (or reuse) an I2C handle
 *
 ****************************************************************************/

FAR struct i2c_master_s *lpc54_i2c_handle(int bus, int ndx)
{
  FAR struct i2c_master_s *i2c = g_i2c_handle[ndx];

  if (i2c == NULL)
    {
      i2c = lpc54_i2cbus_initialize(bus);
      if (i2c == NULL)
        {
          syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", bus);
        }
      else
        {
          g_i2c_handle[ndx] = i2c;
        }
    }

  return i2c;
}

/****************************************************************************
 * Name: lpc54_i2c_free
 *
 * Description:
 *   Free an I2C handle created by lpc54_i2c_handle
 *
 ****************************************************************************/

void lpc54_i2c_free(int ndx)
{
  if (g_i2c_handle[ndx] != NULL)
    {
      lpc54_i2cbus_uninitialize(g_i2c_handle[ndx]);
      g_i2c_handle[ndx] = NULL;
    }
}

#endif /* HAVE_I2CTOOL || HAVE_FT5x06*/
