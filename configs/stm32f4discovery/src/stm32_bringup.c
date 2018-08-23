/****************************************************************************
 * config/stm32f4discovery/src/stm32_bringup.c
 *
 *   Copyright (C) 2012, 2014-2018 Gregory Nutt. All rights reserved.
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
#include <debug.h>
#include <errno.h>

#ifdef CONFIG_USBMONITOR
#  include <nuttx/usb/usbmonitor.h>
#endif

#include "stm32.h"
#include "stm32_romfs.h"

#ifdef CONFIG_STM32_OTGFS
#  include "stm32_usbhost.h"
#endif

#ifdef CONFIG_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_RNDIS
#  include <nuttx/usb/rndis.h>
#endif

#include "stm32f4discovery.h"

/* Conditional logic in stm32f4discovery.h will determine if certain features
 * are supported.  Tests for these features need to be made after including
 * stm32f4discovery.h.
 */

#ifdef HAVE_RTC_DRIVER
#  include <nuttx/timers/rtc.h>
#  include "stm32_rtc.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_INITIALIZE=y :
 *     Called from board_initialize().
 *
 *   CONFIG_BOARD_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void)
{
#ifdef HAVE_RTC_DRIVER
  FAR struct rtc_lowerhalf_s *lower;
#endif
  int ret = OK;

#ifdef CONFIG_SENSORS_BMP180
  stm32_bmp180initialize("/dev/press0");
#endif

#ifdef CONFIG_SENSORS_BH1750FVI
  ret = stm32_bh1750initialize("/dev/light0");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_bh1750initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_ZEROCROSS
  /* Configure the zero-crossing driver */

  stm32_zerocross_initialize();
#endif

#ifdef CONFIG_LEDS_MAX7219
  ret = stm32_max7219init("/dev/numdisp0");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: max7219_leds_register failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_RGBLED
  /* Configure the RGB LED driver */

  stm32_rgbled_setup();
#endif

#if defined(CONFIG_PCA9635PW)
  /* Initialize the PCA9635 chip */

  ret = stm32_pca9635_initialize();
  if (ret < 0)
    {
      serr("ERROR: stm32_pca9635_initialize failed: %d\n", ret);
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

#ifdef HAVE_SDIO
  /* Initialize the SDIO block driver */

  ret = stm32_sdio_initialize();
  if (ret != OK)
    {
      ferr("ERROR: Failed to initialize MMC/SD driver: %d\n", ret);
      return ret;
    }
#endif

#ifdef HAVE_USBHOST
  /* Initialize USB host operation.  stm32_usbhost_initialize() starts a thread
   * will monitor for USB connection and disconnection events.
   */

  ret = stm32_usbhost_initialize();
  if (ret != OK)
    {
      uerr("ERROR: Failed to initialize USB host: %d\n", ret);
      return ret;
    }
#endif

#ifdef HAVE_USBMONITOR
  /* Start the USB Monitor */

  ret = usbmonitor_start();
  if (ret != OK)
    {
      uerr("ERROR: Failed to start USB monitor: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_PWM
  /* Initialize PWM and register the PWM device. */

  ret = stm32_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_pwm_setup() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_CAN
  /* Initialize CAN and register the CAN driver. */

  ret = stm32_can_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_can_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_BUTTONS
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_INPUT_NUNCHUCK
  /* Register the Nunchuck driver */

  ret = nunchuck_initialize("/dev/nunchuck0");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: nunchuck_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_QENCODER
  /* Initialize and register the qencoder driver */

  ret = stm32_qencoder_initialize("/dev/qe0", CONFIG_STM32F4DISCO_QETIMER);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_USERLED
  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef HAVE_RTC_DRIVER
  /* Instantiate the STM32 lower-half RTC driver */

  lower = stm32_rtc_lowerhalf();
  if (!lower)
    {
      serr("ERROR: Failed to instantiate the RTC lower-half driver\n");
      return -ENOMEM;
    }
  else
    {
      /* Bind the lower half driver and register the combined RTC driver
       * as /dev/rtc0
       */

      ret = rtc_initialize(0, lower);
      if (ret < 0)
        {
          serr("ERROR: Failed to bind/register the RTC driver: %d\n", ret);
          return ret;
        }
    }
#endif

#ifdef HAVE_CS43L22
  /* Configure CS43L22 audio */

  ret = stm32_cs43l22_initialize(1);
  if (ret != OK)
    {
      serr("Failed to initialize CS43L22 audio: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_MAX31855
  /* Register device 0 on spi channel 2 */

  ret = stm32_max31855initialize("/dev/temp0", 2, 0);
  if (ret < 0)
    {
      serr("ERROR:  stm32_max31855initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_MAX6675
  ret = stm32_max6675initialize("/dev/temp0");
  if (ret < 0)
    {
      serr("ERROR:  stm32_max6675initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = mount(NULL, STM32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      serr("ERROR: Failed to mount procfs at %s: %d\n",
           STM32_PROCFS_MOUNTPOINT, ret);
    }
#endif

#ifdef CONFIG_STM32_ROMFS
  ret = stm32_romfs_initialize();
  if (ret < 0)
    {
      serr("ERROR: Failed to mount romfs at %s: %d\n",
           STM32_ROMFS_MOUNTPOINT, ret);
    }
#endif

#ifdef CONFIG_SENSORS_XEN1210
  ret = xen1210_archinitialize(0);
  if (ret < 0)
    {
      serr("ERROR:  xen1210_archinitialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_STM32F4DISCO_LIS3DSH
  /* Create a lis3dsh driver instance fitting the chip built into stm32f4discovery */

  ret = stm32_lis3dshinitialize("/dev/acc0");
  if (ret < 0)
    {
      serr("ERROR: Failed to initialize LIS3DSH driver: %d\n", ret);
    }
#endif

#ifdef HAVE_HCIUART
  ret = hciuart_dev_initialize();
  if (ret < 0)
    {
      serr("ERROR: Failed to initialize HCI UART driver: %d\n", ret);
    }
#endif

#if defined(CONFIG_RNDIS) && defined(CONFIG_NSH_MACADDR)
  uint8_t mac[6];
  mac[0] = 0xaa; /* TODO */
  mac[1] = (CONFIG_NSH_MACADDR >> (8 * 4)) & 0xff;
  mac[2] = (CONFIG_NSH_MACADDR >> (8 * 3)) & 0xff;
  mac[3] = (CONFIG_NSH_MACADDR >> (8 * 2)) & 0xff;
  mac[4] = (CONFIG_NSH_MACADDR >> (8 * 1)) & 0xff;
  mac[5] = (CONFIG_NSH_MACADDR >> (8 * 0)) & 0xff;
  usbdev_rndis_initialize(mac);
#endif

  return ret;
}
