/****************************************************************************
 * boards/arm/stm32/alientek-m144z-m4/src/stm32_bringup.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>

#ifdef CONFIG_USBMONITOR
#  include <nuttx/usb/usbmonitor.h>
#endif

#ifdef CONFIG_STM32_OTGFS
#  include "stm32_usbhost.h"
#endif

#include "stm32.h"
#include "alientek-m144z-m4.h"

#ifdef CONFIG_FS_PROCFS
#  include <nuttx/fs/fs.h>
#endif

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#ifdef CONFIG_MTD_W25
#  include <nuttx/spi/spi.h>
#  include <nuttx/mtd/mtd.h>
#  include <nuttx/fs/fs.h>
#endif

/* The following are includes from board-common logic */

#ifdef CONFIG_SENSORS_BMP180
#include "stm32_bmp180.h"
#endif

#ifdef CONFIG_SENSORS_INA219
#include "stm32_ina219.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define HAVE_USBDEV     1
#define HAVE_USBHOST    1
#define HAVE_USBMONITOR 1
/* #define HAVE_I2CTOOL    1 */

/* Can't support USB host or device features if USB OTG HS is not enabled */

#ifndef CONFIG_STM32_OTGHS
#  undef HAVE_USBDEV
#  undef HAVE_USBHOST
#endif

/* Can't support USB device USB device is not enabled */

#ifndef CONFIG_USBDEV
#  undef HAVE_USBDEV
#endif

/* Can't support USB host is USB host is not enabled */

#ifndef CONFIG_USBHOST
#  undef HAVE_USBHOST
#endif

/* Check if we should enable the USB monitor before starting NSH */

#ifndef CONFIG_USBMONITOR
#  undef HAVE_USBMONITOR
#endif

#ifndef HAVE_USBDEV
#  undef CONFIG_USBDEV_TRACE
#endif

#ifndef HAVE_USBHOST
#  undef CONFIG_USBHOST_TRACE
#endif

#if !defined(CONFIG_USBDEV_TRACE) && !defined(CONFIG_USBHOST_TRACE)
#  undef HAVE_USBMONITOR
#endif

#if !defined(CONFIG_STM32_CAN1) && !defined(CONFIG_STM32_CAN2)
#  undef CONFIG_CAN
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_i2c_register
 *
 * Description:
 *   Register one I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#ifdef HAVE_I2CTOOL
static void stm32_i2c_register(int bus)
{
  struct i2c_master_s *i2c;
  int ret;

  i2c = stm32_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      _err("ERROR: Failed to get I2C%d interface\n", bus);
    }
  else
    {
      ret = i2c_register(i2c, bus);
      if (ret < 0)
        {
          _err("ERROR: Failed to register I2C%d driver: %d\n", bus, ret);
          stm32_i2cbus_uninitialize(i2c);
        }
    }
}
#endif

/****************************************************************************
 * Name: stm32_i2ctool
 *
 * Description:
 *   Register I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#ifdef HAVE_I2CTOOL
static void stm32_i2ctool(void)
{
#ifdef CONFIG_STM32_I2C1
  stm32_i2c_register(1);
#endif
#ifdef CONFIG_STM32_I2C2
  stm32_i2c_register(2);
#endif
#ifdef CONFIG_STM32_I2C3
  stm32_i2c_register(3);
#endif
}
#else
#  define stm32_i2ctool()
#endif

/****************************************************************************
 * Name: stm32_w25_bringup
 *
 * Description:
 *   Probe the on-board W25Q128 NOR flash on SPI1, register it as the MTD
 *   character device /dev/w25 and (when CONFIG_FS_LITTLEFS=y) mount a
 *   littlefs filesystem at /data.  The raw JEDEC ID / Unique ID dump is
 *   emitted via _info() so the chip is visible in the boot log when
 *   CONFIG_DEBUG_INFO=y.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_W25
static int stm32_w25_bringup(void)
{
  FAR struct spi_dev_s *spi;
  FAR struct mtd_dev_s *mtd;
  uint8_t jedec[3];
  uint8_t manufdev[2];
  uint8_t uniqueid[8];
  int ret;
  int i;

  spi = stm32_spibus_initialize(1);
  if (spi == NULL)
    {
      syslog(LOG_ERR, "ERROR: stm32_spibus_initialize(1) failed\n");
      return -ENODEV;
    }

  /* Raw JEDEC-ID probe before invoking the W25 driver.  This is the first
   * piece of useful evidence when bring-up fails: it isolates "is the SPI
   * bus + CS + chip alive at all?" from "is the W25 driver happy with the
   * answer?".  Expected for the on-board W25Q128:
   *
   *   manufacturer = 0xEF (Winbond)
   *   memory type  = 0x40 (SPI flash)
   *   capacity     = 0x18 (128 Mbit / 16 MiB)
   */

  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);
  SPI_SETFREQUENCY(spi, 1000000);

  /* (1) Enable-Reset + Reset (0x66, 0x99).  If the chip is wedged in some
   * legacy / power-on state this should put it back to its documented
   * power-on defaults.  Harmless on chips that don't implement the command
   * (they just ignore the unknown opcode).
   */

  SPI_SELECT(spi, SPIDEV_FLASH(0), true);
  SPI_SEND(spi, 0x66);
  SPI_SELECT(spi, SPIDEV_FLASH(0), false);
  up_udelay(10);
  SPI_SELECT(spi, SPIDEV_FLASH(0), true);
  SPI_SEND(spi, 0x99);
  SPI_SELECT(spi, SPIDEV_FLASH(0), false);
  up_udelay(50);

  /* (2) JEDEC ID (0x9F): manufacturer / memory type / capacity. */

  SPI_SELECT(spi, SPIDEV_FLASH(0), true);
  SPI_SEND(spi, 0x9f);
  jedec[0] = (uint8_t)SPI_SEND(spi, 0xff);
  jedec[1] = (uint8_t)SPI_SEND(spi, 0xff);
  jedec[2] = (uint8_t)SPI_SEND(spi, 0xff);
  SPI_SELECT(spi, SPIDEV_FLASH(0), false);

  /* (3) Manufacturer/Device ID (0x90 + 24-bit addr 0x000000). */

  SPI_SELECT(spi, SPIDEV_FLASH(0), true);
  SPI_SEND(spi, 0x90);
  SPI_SEND(spi, 0x00);
  SPI_SEND(spi, 0x00);
  SPI_SEND(spi, 0x00);
  manufdev[0] = (uint8_t)SPI_SEND(spi, 0xff);
  manufdev[1] = (uint8_t)SPI_SEND(spi, 0xff);
  SPI_SELECT(spi, SPIDEV_FLASH(0), false);

  /* (4) Unique ID (0x4B + 4 dummies, then 8 bytes). */

  SPI_SELECT(spi, SPIDEV_FLASH(0), true);
  SPI_SEND(spi, 0x4b);
  for (i = 0; i < 4; i++)
    {
      SPI_SEND(spi, 0xff);
    }

  for (i = 0; i < 8; i++)
    {
      uniqueid[i] = (uint8_t)SPI_SEND(spi, 0xff);
    }

  SPI_SELECT(spi, SPIDEV_FLASH(0), false);
  SPI_LOCK(spi, false);

  _info("W25 probe: 0x9F JEDEC = %02x %02x %02x (expect ef 40 18)\n",
        jedec[0], jedec[1], jedec[2]);
  _info("W25 probe: 0x90 M/Dev = %02x %02x (expect ef 17)\n",
        manufdev[0], manufdev[1]);
  _info("W25 probe: 0x4B UID   = "
        "%02x %02x %02x %02x %02x %02x %02x %02x\n",
        uniqueid[0], uniqueid[1], uniqueid[2], uniqueid[3],
        uniqueid[4], uniqueid[5], uniqueid[6], uniqueid[7]);

  mtd = w25_initialize(spi);
  if (mtd == NULL)
    {
      syslog(LOG_ERR, "ERROR: w25_initialize() failed\n");
      return -EIO;
    }

  ret = register_mtddriver("/dev/w25", mtd, 0755, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: register_mtddriver(/dev/w25) failed: %d\n", ret);
      return ret;
    }

#ifdef CONFIG_FS_LITTLEFS
  /* Mount littlefs directly on the raw MTD node.  littlefs_bind() accepts
   * MTD inodes without an FTL / BCH wrapper.  The "autoformat" option makes
   * the FS call lfs_format() on first boot (LFS_ERR_CORRUPT -> -EFAULT
   * from lfs_mount triggers the VFS auto-format path).  Subsequent boots
   * find a valid superblock and mount in-place, so data persists.
   */

  ret = nx_mount("/dev/w25", "/data", "littlefs", 0, "autoformat");
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: mount littlefs /dev/w25 -> /data failed: %d\n", ret);
      return ret;
    }

  _info("littlefs mounted at /data on /dev/w25\n");
#endif

  return OK;
}
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
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret;

  /* Register I2C drivers on behalf of the I2C tool */

  stm32_i2ctool();

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system at /proc so that ps, top and the
   * cat /proc/<file> idiom work out of the box.  CONFIG_NSH_PROC_MOUNTPOINT
   * only tells the NSH commands where to read procfs from; the actual
   * mount has to happen here in board bring-up.
   */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef CONFIG_USERLED
  /* Register the user LED driver at /dev/userleds.
   *
   * On ALIENTEK M144Z-M4 the LEDs are:
   *   LED0 (red, PF9)   - bit 0 of /dev/userleds
   *   LED1 (green, PF10) - bit 1 of /dev/userleds
   * Both are active LOW (handled inside stm32_userleds.c).
   */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_INPUT_BUTTONS
  /* Register the button driver at /dev/buttons.
   *
   * On ALIENTEK M144Z-M4 the buttons are:
   *   KEY0  (PE4, active LOW)  - bit 0 of /dev/buttons
   *   WK_UP (PA0, active HIGH) - bit 1 of /dev/buttons
   * The active polarity is normalised inside stm32_buttons.c so userspace
   * always sees "pressed = 1".
   */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_MTD_W25
  /* Bring up the on-board W25Q128 NOR flash on SPI1 and expose it as
   * /dev/w25 (raw MTD character device).  When CONFIG_FS_LITTLEFS=y, a
   * littlefs filesystem is also auto-mounted at /data on top of the same
   * device.  See stm32_w25_bringup() above for the full probe sequence.
   */

  ret = stm32_w25_bringup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_w25_bringup() failed: %d\n", ret);
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

#ifdef HAVE_USBHOST
  /* Initialize USB host operation.  stm32_usbhost_initialize() starts a
   * thread will monitor for USB connection and disconnection events.
   */

  ret = stm32_usbhost_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize USB host: %d\n", ret);
      return ret;
    }
#endif

#ifdef HAVE_USBMONITOR
  /* Start the USB Monitor */

  ret = usbmonitor_start();
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to start USB monitor: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_BMP180
  /* Initialize the BMP180 pressure sensor. */

  ret = board_bmp180_initialize(0, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize BMP180, error %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_DAC
  /* Initialize DAC and register the DAC driver. */

  ret = stm32_dac_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to start ADC1: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_INA219
  /* Configure and initialize the INA219 sensor */

  ret = board_ina219_initialize(0, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_ina219initialize() failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_TIMER)
  /* Initialize the timer, at this moment it's only Timer 1,2,3 */

  #if defined(CONFIG_STM32_TIM1)
    stm32_timer_driver_setup("/dev/timer1", 1);
  #endif
  #if defined(CONFIG_STM32_TIM2)
    stm32_timer_driver_setup("/dev/timer2", 2);
  #endif
  #if defined(CONFIG_STM32_TIM3)
    stm32_timer_driver_setup("/dev/timer3", 3);
  #endif
#endif

#ifdef CONFIG_IEEE802154_MRF24J40
  /* Configure MRF24J40 wireless */

  ret = stm32_mrf24j40_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_mrf24j40_initialize() failed:"
                      " %d\n", ret);
    }
#endif

  UNUSED(ret);
  return OK;
}
