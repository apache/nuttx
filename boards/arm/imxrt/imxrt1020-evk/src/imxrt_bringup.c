/****************************************************************************
 * boards/arm/imxrt/imxrt1020-evk/src/imxrt_bringup.c
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

#include <sys/types.h>
#include <syslog.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/wireless/bluetooth/bt_uart.h>

#include "hardware/imxrt_pinmux.h"
#include "hardware/imxrt_ccm.h"
#include "imxrt_periphclks.h"

#include "imxrt_lpi2c.h"
#include "imxrt_flexspi_nor_boot.h"
#include "imxrt1020-evk.h"

#ifdef CONFIG_IMXRT_USDHC
#  include "imxrt_usdhc.h"
#endif

#ifdef CONFIG_IMXRT_ENET
#  include "imxrt_enet.h"
#endif

#ifdef CONFIG_IMXRT_LPSPI
#  include "nuttx/spi/spi_transfer.h"
#  include <imxrt_lpspi.h>
#endif

#include "imxrt1020-evk.h"

#include <arch/board/board.h>  /* Must always be included last */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Checking needed by MMC/SDCard */

#ifdef CONFIG_NSH_MMCSDMINOR
#  define MMCSD_MINOR   CONFIG_NSH_MMCSDMINOR
#else
#  define MMCSD_MINOR   0
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_I2C_DRIVER) && defined(CONFIG_IMXRT_LPI2C)
static void imxrt_i2c_register(int bus)
{
  struct i2c_master_s   *i2c;
  int                    ret;

  i2c = imxrt_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      serr("ERROR: Failed to get I2C%d interface\n", bus);
    }
  else
    {
      ret = i2c_register(i2c, bus);
      if (ret < 0)
        {
          serr("ERROR: Failed to register I2C%d driver: %d\n", bus, ret);
          imxrt_i2cbus_uninitialize(i2c);
        }
    }
}

#endif

#ifdef CONFIG_IMXRT_USDHC
static int nsh_sdmmc_initialize(void)
{
  struct sdio_dev_s *sdmmc;
  int               ret = 0;

  /* Get an instance of the SDIO interface */

  sdmmc = imxrt_usdhc_initialize(BOARD_USDHC_SD_ID);
  if (!sdmmc)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SD/MMC\n");
    }
  else
    {
      /* Bind the SDIO interface to the MMC/SD driver */

      ret = mmcsd_slotinitialize(BOARD_USDHC_SD_ID, sdmmc);
      if (ret != OK)
        {
          syslog(LOG_ERR,
                 "ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n",
                 ret);
        }
    }

  return OK;
}

#else
#  define nsh_sdmmc_initialize()  (OK)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int imxrt_bringup(void)
{
  int ret;

  /* If we got here then perhaps not all initialization was successful, but
   * at least enough succeeded to bring-up NSH with perhaps reduced
   * capabilities.
   */

#ifdef CONFIG_USBHOST
  ret = imxrt_usbhost_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Couldn't start usbotg %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#if defined(CONFIG_I2C_DRIVER)
    struct i2c_master_s *i2c;

#if defined(CONFIG_IMXRT_LPI2C1)
  i2c = imxrt_i2c_register(1);
#endif
#if defined(CONFIG_IMXRT_LPI2C4)
  i2c = imxrt_i2cbus_initialize(4);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "Failed to get i2c bus4\n");
    }
  else
    {
      i2c_register(i2c, 4);
    }
#endif
#endif

#if defined(CONFIG_SPI_DRIVER)
#if defined(CONFIG_IMXRT_LPSPI1)
  imxrt_config_gpio(GPIO_LPSPI1_CS);
  imxrt_spi_register(1);
#endif
#if defined(CONFIG_IMXRT_LPSPI2)
  imxrt_config_gpio(GPIO_LPSPI2_CS);
  imxrt_spi_register(2);
#endif
#if defined(CONFIG_IMXRT_LPSPI3)
  imxrt_config_gpio(GPIO_LPSPI3_CS);
  imxrt_spi_register(3);
#endif
#if defined(CONFIG_IMXRT_LPSPI4)
  imxrt_config_gpio(GPIO_LPSPI4_CS);
  imxrt_spi_register(4);
#endif
#endif

#ifdef CONFIG_IMXRT_USDHC
  /* Initialize SDHC-base MMC/SD card support */

  imxrt_config_gpio(GPIO_VSDHIGH);
  imxrt_config_gpio(PIN_USDHC1_PWREN);

  nsh_sdmmc_initialize();
#endif

#ifdef CONFIG_DEV_GPIO
  ret = imxrt_gpio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize GPIO Driver: %d\n", ret);
      return ret;
    }

#endif

  return ret;
}
