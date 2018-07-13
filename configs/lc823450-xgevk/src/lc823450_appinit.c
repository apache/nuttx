/****************************************************************************
 * configs/lc823450-xgevk/src/lc823450_appinit.c
 *
 *   Copyright 2017 Sony Video & Sound Products Inc.
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
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

#include <stdio.h>
#include <syslog.h>

#include <nuttx/board.h>
#include <nuttx/i2c/i2c_master.h>

#ifdef CONFIG_MTD
#  include "lc823450_mtd.h"
#endif

#include "lc823450_i2c.h"
#include "lc823450-xgevk.h"

#ifdef CONFIG_LIB_BOARDCTL

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
  FAR struct i2c_master_s *i2c;
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
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initalization logic and the the
 *         matching application logic.  The value cold be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
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

#ifndef CONFIG_BOARD_INITIALIZE
  /* Perform board initialization */

  return lc823450_bringup();
#else
  return OK;
#endif
}

#endif /* CONFIG_LIB_BOARDCTL */
