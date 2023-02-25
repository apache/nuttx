/****************************************************************************
 * boards/arm/imxrt/teensy-4.x/src/imxrt_bringup.c
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
#include <debug.h>

#include <syslog.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/video/fb.h>
#include <imxrt_lpi2c.h>
#include <imxrt_lpspi.h>
#include <nuttx/input/buttons.h>
#include <nuttx/usb/cdcacm.h>

#ifdef CONFIG_IMXRT_USDHC
#  include "imxrt_usdhc.h"
#endif

#ifdef CONFIG_USBMONITOR
#  include <nuttx/usb/usbmonitor.h>
#endif

#ifdef CONFIG_VIDEO_FB
#  include <nuttx/video/fb.h>
#endif

#include "imxrt_enet.h"
#include "teensy-4.h"

#include <arch/board/board.h>   /* Must always be included last */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Checking needed by MMC/SDCard */

#ifdef CONFIG_NSH_MMCSDMINOR
#  define MMCSD_MINOR CONFIG_NSH_MMCSDMINOR
#else
#  define MMCSD_MINOR 0
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined (CONFIG_IMXRT_USDHC) && (CONFIG_TEENSY_41)
static int nsh_sdmmc_initialize(void)
{
  struct sdio_dev_s *sdmmc;
  int ret = 0;

  /* Get an instance of the SDIO interface */

  sdmmc = imxrt_usdhc_initialize(0);
  if (!sdmmc)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SD/MMC\n");
    }
  else
    {
      /* Bind the SDIO interface to the MMC/SD driver */

      ret = mmcsd_slotinitialize(0, sdmmc);
      if (ret != OK)
        {
          syslog(LOG_ERR,
                 "ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n",
                 ret);
        }

      imxrt_usdhc_set_sdio_card_isr(sdmmc, NULL, NULL);
    }

  return OK;
}
#else
#  define nsh_sdmmc_initialize() (OK)
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

  /* If we got here then perhaps not all initialization was successful,
   * but at least enough succeeded to bring-up NSH with perhaps reduced
   * capabilities.
   */

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#if !defined(CONFIG_BOARDCTL_USBDEVCTRL) && !defined(CONFIG_USBDEV_COMPOSITE)
# ifdef CONFIG_CDCACM
    cdcacm_initialize(0, NULL);
# endif
#endif

#if defined(CONFIG_I2C_DRIVER)
  imxrt_i2c_setup();
#endif

#ifdef CONFIG_IMXRT_USDHC
  /* Initialize SDHC-base MMC/SD card support */

  nsh_sdmmc_initialize();
#endif

#if defined(CONFIG_IMXRT_ENET) && defined(CONFIG_NETDEV_LATEINIT)
  ret = imxrt_netinitialize(0);
#endif

#if defined(CONFIG_IMXRT_FLEXCAN) && defined(CONFIG_NETDEV_LATEINIT)
  ret = imxrt_can_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: imxrt_can_setup() failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_IMXRT_FLEXPWM
  ret = imxrt_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: imxrt_pwm_setup() failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_VIDEO_FB
  /* Initialize and register the framebuffer driver */

  ret = fb_register(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fb_register() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_IMXRT_ADC
  /* Initialize ADC and register the ADC driver. */

  ret = imxrt_adc_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: imxrt_adc_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_DEV_GPIO
  /* Initialize GPIO driver */

  ret = imxrt_gpio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: imxrt_gpio_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_IMXRT_ENC
  /* Initialize ENC and register the ENC driver. */

  ret = imxrt_enc_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: imxrt_enc_initialize() failed: %d\n", ret);
    }
#endif

  UNUSED(ret);
  return OK;
}
