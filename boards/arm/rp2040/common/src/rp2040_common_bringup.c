/****************************************************************************
 * boards/arm/rp2040/common/src/rp2040_common_bringup.c
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

#include <debug.h>
#include <errno.h>
#include <stddef.h>
#include <string.h>
#include <sys/stat.h>

#include <nuttx/fs/fs.h>

#include <arch/board/board.h>

#include "rp2040_pico.h"
#include "rp2040_common_bringup.h"

#ifdef CONFIG_LCD_BACKPACK
#include "rp2040_lcd_backpack.h"
#endif

#ifdef CONFIG_LCD
#include <nuttx/board.h>
#endif

#ifdef CONFIG_LCD_DEV
#include <nuttx/lcd/lcd_dev.h>
#endif

#ifdef CONFIG_VIDEO_FB
#include <nuttx/video/fb.h>
#endif

#ifdef CONFIG_SENSORS_INA219
#include <nuttx/sensors/ina219.h>
#include "rp2040_ina219.h"
#endif

#ifdef CONFIG_SENSORS_BMP180
#include <nuttx/sensors/bmp180.h>
#include "rp2040_bmp180.h"
#endif

#ifdef CONFIG_SENSORS_BMP280
#include <nuttx/sensors/bmp280.h>
#include "rp2040_bmp280.h"
#endif

#ifdef CONFIG_SENSORS_SHT4X
#include <nuttx/sensors/sht4x.h>
#include "rp2040_i2c.h"
#endif

#ifdef CONFIG_SENSORS_MCP9600
#include <nuttx/sensors/mcp9600.h>
#include "rp2040_i2c.h"
#endif

#ifdef CONFIG_SENSORS_MS56XX
#include <nuttx/sensors/ms56xx.h>
#include "rp2040_i2c.h"
#endif

#ifdef CONFIG_SENSORS_MAX6675
#include <nuttx/sensors/max6675.h>
#include "rp2040_max6675.h"
#endif

#ifdef CONFIG_RP2040_PWM
#include "rp2040_pwm.h"
#include "rp2040_pwmdev.h"
#endif

#if defined(CONFIG_ADC) && defined(CONFIG_RP2040_ADC)
#include "rp2040_adc.h"
#endif

#if defined(CONFIG_ADC) && defined(CONFIG_ADC_MCP3008)
#include <nuttx/analog/mcp3008.h>
#include <nuttx/analog/adc.h>
#include "rp2040_spi.h"
#endif

#if defined(CONFIG_RP2040_BOARD_HAS_WS2812) && defined(CONFIG_WS2812)
#include "rp2040_ws2812.h"
#endif

#ifdef CONFIG_WATCHDOG
#  include "rp2040_wdt.h"
#endif

#if defined(CONFIG_RP2040_ROMFS_ROMDISK_DEVNAME)
#  include <rp2040_romfsimg.h>
#endif

#ifdef CONFIG_RP2040_FLASH_FILE_SYSTEM
#  include "rp2040_flash_mtd.h"
#endif

#ifdef CONFIG_WS2812_HAS_WHITE
#define HAS_WHITE true
#else /* CONFIG_WS2812_HAS_WHITE */
#define HAS_WHITE false
#endif /* CONFIG_WS2812_HAS_WHITE */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_common_bringup
 ****************************************************************************/

int rp2040_common_bringup(void)
{
  int ret = 0;

#ifdef CONFIG_RP2040_FLASH_FILE_SYSTEM
  struct mtd_dev_s *mtd_dev;
#endif

#ifdef CONFIG_RP2040_I2C_DRIVER
  #ifdef CONFIG_RP2040_I2C0
  ret = board_i2cdev_initialize(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2C0.\n");
    }
  #endif

  #ifdef CONFIG_RP2040_I2C1
  ret = board_i2cdev_initialize(1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2C1.\n");
    }
  #endif
#endif

#ifdef CONFIG_RP2040_SPI_DRIVER
  #ifdef CONFIG_RP2040_SPI0
  ret = board_spidev_initialize(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize SPI0.\n");
    }
  #endif

  #ifdef CONFIG_RP2040_SPI1
  ret = board_spidev_initialize(1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize SPI1.\n");
    }
  #endif
#endif

#ifdef CONFIG_RP2040_PWM
#  ifdef CONFIG_RP2040_PWM0
#    if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
  ret = rp2040_pwmdev_initialize(0,
                                 CONFIG_RP2040_PWM0A_GPIO,
                                 CONFIG_RP2040_PWM0B_GPIO,
                                 (0
#      ifdef CONFIG_RP2040_PWM0A_INVERT
                                  | RP2040_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP2040_PWM0B_INVERT
                                  | RP2040_PWM_CSR_B_INV
#      endif
#      ifdef CONFIG_RP2040_PWM0_PHASE_CORRECT
                                  | RP2040_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    else
  ret = rp2040_pwmdev_initialize(0,
                                 CONFIG_RP2040_PWM0A_GPIO,
                                 (0
#      ifdef CONFIG_RP2040_PWM0A_INVERT
                                  | RP2040_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP2040_PWM0_PHASE_CORRECT
                                  | RP2040_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    endif
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize PWM0.\n");
    }
#  endif

#  ifdef CONFIG_RP2040_PWM1
#    if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
  ret = rp2040_pwmdev_initialize(1,
                                 CONFIG_RP2040_PWM1A_GPIO,
                                 CONFIG_RP2040_PWM1B_GPIO,
                                 (0
#      ifdef CONFIG_RP2040_PWM1A_INVERT
                                  | RP2040_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP2040_PWM1B_INVERT
                                  | RP2040_PWM_CSR_B_INV
#      endif
#      ifdef CONFIG_RP2040_PWM1_PHASE_CORRECT
                                  | RP2040_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    else
  ret = rp2040_pwmdev_initialize(1,
                                 CONFIG_RP2040_PWM1A_GPIO,
                                 (0
#      ifdef CONFIG_RP2040_PWM1A_INVERT
                                  | RP2040_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP2040_PWM1_PHASE_CORRECT
                                  | RP2040_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    endif
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize PWM1.\n");
    }
#  endif

#  ifdef CONFIG_RP2040_PWM2
#    if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
  ret = rp2040_pwmdev_initialize(2,
                                 CONFIG_RP2040_PWM2A_GPIO,
                                 CONFIG_RP2040_PWM2B_GPIO,
                                 (0
#      ifdef CONFIG_RP2040_PWM2A_INVERT
                                  | RP2040_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP2040_PWM2B_INVERT
                                  | RP2040_PWM_CSR_B_INV
#      endif
#      ifdef CONFIG_RP2040_PWM2_PHASE_CORRECT
                                  | RP2040_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    else
  ret = rp2040_pwmdev_initialize(2,
                                 CONFIG_RP2040_PWM2A_GPIO,
                                 (0
#      ifdef CONFIG_RP2040_PWM2A_INVERT
                                  | RP2040_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP2040_PWM2_PHASE_CORRECT
                                  | RP2040_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    endif
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize PWM2.\n");
    }
#  endif

#  ifdef CONFIG_RP2040_PWM3
#    if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
  ret = rp2040_pwmdev_initialize(3,
                                 CONFIG_RP2040_PWM3A_GPIO,
                                 CONFIG_RP2040_PWM3B_GPIO,
                                 (0
#      ifdef CONFIG_RP2040_PWM3A_INVERT
                                  | RP2040_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP2040_PWM3B_INVERT
                                  | RP2040_PWM_CSR_B_INV
#      endif
#      ifdef CONFIG_RP2040_PWM3_PHASE_CORRECT
                                  | RP2040_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    else
  ret = rp2040_pwmdev_initialize(3,
                                 CONFIG_RP2040_PWM3A_GPIO,
                                 (0
#      ifdef CONFIG_RP2040_PWM3A_INVERT
                                  | RP2040_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP2040_PWM3_PHASE_CORRECT
                                  | RP2040_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    endif
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize PWM3.\n");
    }
#  endif

#  ifdef CONFIG_RP2040_PWM4
#    if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
  ret = rp2040_pwmdev_initialize(4,
                                 CONFIG_RP2040_PWM4A_GPIO,
                                 CONFIG_RP2040_PWM4B_GPIO,
                                 (0
#      ifdef CONFIG_RP2040_PWM4A_INVERT
                                  | RP2040_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP2040_PWM4B_INVERT
                                  | RP2040_PWM_CSR_B_INV
#      endif
#      ifdef CONFIG_RP2040_PWM4_PHASE_CORRECT
                                  | RP2040_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    else
  ret = rp2040_pwmdev_initialize(4,
                                 CONFIG_RP2040_PWM4A_GPIO,
                                 (0
#      ifdef CONFIG_RP2040_PWM4A_INVERT
                                  | RP2040_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP2040_PWM4_PHASE_CORRECT
                                  | RP2040_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    endif
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize PWM4.\n");
    }
#  endif

#  ifdef CONFIG_RP2040_PWM5
#    if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
  ret = rp2040_pwmdev_initialize(5,
                                 CONFIG_RP2040_PWM5A_GPIO,
                                 CONFIG_RP2040_PWM5B_GPIO,
                                 (0
#      ifdef CONFIG_RP2040_PWM5A_INVERT
                                  | RP2040_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP2040_PWM5B_INVERT
                                  | RP2040_PWM_CSR_B_INV
#      endif
#      ifdef CONFIG_RP2040_PWM5_PHASE_CORRECT
                                  | RP2040_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#  else
  ret = rp2040_pwmdev_initialize(5,
                                 CONFIG_RP2040_PWM5A_GPIO,
                                 (0
#      ifdef CONFIG_RP2040_PWM5A_INVERT
                                  | RP2040_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP2040_PWM5_PHASE_CORRECT
                                  | RP2040_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#  endif
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize PWM5.\n");
    }
#  endif

#  ifdef CONFIG_RP2040_PWM6
#    if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
  ret = rp2040_pwmdev_initialize(6,
                                 CONFIG_RP2040_PWM6A_GPIO,
                                 CONFIG_RP2040_PWM6B_GPIO,
                                 (0
#      ifdef CONFIG_RP2040_PWM6A_INVERT
                                  | RP2040_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP2040_PWM6B_INVERT
                                  | RP2040_PWM_CSR_B_INV
#      endif
#      ifdef CONFIG_RP2040_PWM6_PHASE_CORRECT
                                  | RP2040_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    else
  ret = rp2040_pwmdev_initialize(6,
                                 CONFIG_RP2040_PWM6A_GPIO,
                                 (0
#      ifdef CONFIG_RP2040_PWM6A_INVERT
                                  | RP2040_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP2040_PWM6_PHASE_CORRECT
                                  | RP2040_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    endif
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize PWM6.\n");
    }
#  endif

#  ifdef CONFIG_RP2040_PWM7
#    if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
  ret = rp2040_pwmdev_initialize(7,
                                 CONFIG_RP2040_PWM7A_GPIO,
                                 CONFIG_RP2040_PWM7B_GPIO,
                                 (0
#      ifdef CONFIG_RP2040_PWM7A_INVERT
                                  | RP2040_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP2040_PWM7B_INVERT
                                  | RP2040_PWM_CSR_B_INV
#      endif
#      ifdef CONFIG_RP2040_PWM7_PHASE_CORRECT
                                  | RP2040_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    else
  ret = rp2040_pwmdev_initialize(7,
                                 CONFIG_RP2040_PWM7A_GPIO,
                                 (0
#      ifdef CONFIG_RP2040_PWM7A_INVERT
                                  | RP2040_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP2040_PWM7_PHASE_CORRECT
                                  | RP2040_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    endif
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize PWM7.\n");
    }
#  endif
#endif

#ifdef CONFIG_RP2040_SPISD
  /* Mount the SPI-based MMC/SD block driver */

  ret = board_spisd_initialize(0, CONFIG_RP2040_SPISD_SPI_CH);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize SPI device to MMC/SD: %d\n",
           ret);
    }
#endif

#ifdef CONFIG_ADC_MCP3008
  /* Register MCP3008 ADC. */

  struct spi_dev_s *spi = rp2040_spibus_initialize(0);
  if (spi == NULL)
    {
      syslog(LOG_ERR, "Failed to initialize SPI bus 0\n");
    }

  struct adc_dev_s *mcp3008 = mcp3008_initialize(spi);
  if (mcp3008 == NULL)
    {
      syslog(LOG_ERR, "Failed to initialize MCP3008\n");
    }

  ret = adc_register("/dev/adc1", mcp3008);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to register MCP3008 device driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      serr("ERROR: Failed to mount procfs at %s: %d\n", "/proc", ret);
    }
#endif

#ifdef CONFIG_SENSORS_BMP180
  /* Try to register BMP180 device in I2C0 */

  ret = board_bmp180_initialize(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize BMP180 driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_BMP280
  /* Try to register BMP280 device in I2C0 */

  ret = board_bmp280_initialize(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize BMP280 driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_MAX6675
  /* Try to register MAX6675 device as /dev/temp0 at SPI0 */

  ret = board_max6675_initialize(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize MAX6675 driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_INA219
  /* Configure and initialize the INA219 sensor in I2C0 */

  ret = board_ina219_initialize(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: rp2040_ina219_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_SHT4X

  /* Try to register SHT4X device on I2C0 */

  ret = sht4x_register(rp2040_i2cbus_initialize(0), 0,
                       CONFIG_SHT4X_I2C_ADDR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: couldn't initialize SHT4x: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_MCP9600
  /* Try to register MCP9600 device as /dev/thermo0 at I2C0. */

  ret = mcp9600_register("/dev/thermo0", rp2040_i2cbus_initialize(0), 0x60);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: couldn't initialize MCP9600: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_MS56XX
  /* Try to register MS56xx device at I2C0 */

  ret = ms56xx_register(rp2040_i2cbus_initialize(0), 0, MS56XX_ADDR0,
                        MS56XX_MODEL_MS5611);
  if (ret < 0)
    {
        syslog(LOG_ERR, "ERROR: couldn't register MS5611: %d\n", ret);
    }
#endif

#ifdef CONFIG_VIDEO_FB
  ret = fb_register(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize Frame Buffer Driver.\n");
    }
#elif defined(CONFIG_LCD)
  ret = board_lcd_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize LCD.\n");
    }
#endif

#ifdef CONFIG_LCD_DEV
  ret = lcddev_register(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: lcddev_register() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_LCD_BACKPACK
  /* slcd:0, i2c:0, rows=2, cols=16 */

  ret = board_lcd_backpack_init(0, 0, 2, 16);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize PCF8574 LCD, error %d\n", ret);
    }
#endif

#ifdef CONFIG_RP2040_I2S
  ret = board_i2sdev_initialize(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2S.\n");
    }
#endif

#ifdef CONFIG_DEV_GPIO
  ret = rp2040_dev_gpio_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize GPIO Driver: %d\n", ret);
    }
#endif

  /* Initialize ADC */

#if defined(CONFIG_ADC) && defined(CONFIG_RP2040_ADC)

#  ifdef CONFIG_RPC2040_ADC_CHANNEL0
#    define ADC_0 true
#  else
#    define ADC_0 false
#  endif

#  ifdef CONFIG_RPC2040_ADC_CHANNEL1
#    define ADC_1 true
#  else
#    define ADC_1 false
#  endif

#  ifdef CONFIG_RPC2040_ADC_CHANNEL2
#    define ADC_2 true
#  else
#    define ADC_2 false
#  endif

#  ifdef CONFIG_RPC2040_ADC_CHANNEL3
#    define ADC_3 true
#  else
#    define ADC_3 false
#  endif

#  ifdef CONFIG_RPC2040_ADC_TEMPERATURE
#    define ADC_TEMP true
#  else
#    define ADC_TEMP false
#  endif

  ret = rp2040_adc_setup("/dev/adc0", ADC_0, ADC_1, ADC_2, ADC_3, ADC_TEMP);
  if (ret != OK)
    {
      syslog(LOG_ERR, "Failed to initialize ADC Driver: %d\n", ret);
    }

#endif /* defined(CONFIG_ADC) && defined(CONFIG_RP2040_ADC) */

  /* Initialize board neo-pixel */

#if defined(CONFIG_RP2040_BOARD_HAS_WS2812) && defined(CONFIG_WS2812)

  if (rp2040_ws2812_setup("/dev/leds0",
                          CONFIG_RP2040_WS2812_GPIO_PIN,
                          CONFIG_RP2040_WS2812_PWR_GPIO,
                          CONFIG_WS2812_LED_COUNT,
                          HAS_WHITE) == NULL)
    {
      syslog(LOG_ERR, "Failed to initialize WS2812: %d\n", errno);
    }
#endif

#ifdef CONFIG_WATCHDOG
  /* Configure watchdog timer */

  ret = rp2040_wdt_init();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize watchdog drivers: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_RP2040_FLASH_FILE_SYSTEM

  mtd_dev = rp2040_flash_mtd_initialize();

  if (mtd_dev == NULL)
    {
      syslog(LOG_ERR, "ERROR: flash_mtd_initialize failed: %d\n", errno);
    }
  else
    {
      ret = smart_initialize(0, mtd_dev, NULL);

      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: smart_initialize failed: %d\n", -ret);
        }
      else if (sizeof(CONFIG_RP2040_FLASH_MOUNT_POINT) > 1)
        {
          mkdir(CONFIG_RP2040_FLASH_MOUNT_POINT, 0777);

          /* Mount the file system */

          ret = nx_mount("/dev/smart0",
                        CONFIG_RP2040_FLASH_MOUNT_POINT,
                        "smartfs",
                        0,
                        NULL);
          if (ret < 0)
            {
              syslog(LOG_ERR,
                    "ERROR: nx_mount(\"/dev/smart0\", \"%s\", \"smartfs\","
                    " 0, NULL) failed: %d\n",
                    CONFIG_RP2040_FLASH_MOUNT_POINT,
                    ret);
            }
        }
    }

#endif

#if defined(CONFIG_RP2040_ROMFS_ROMDISK_DEVNAME)
  /* Register the ROM disk */

  ret = romdisk_register(CONFIG_RP2040_ROMFS_ROMDISK_MINOR,
                         rp2040_romfs_img,
                         NSECTORS(rp2040_romfs_img_len),
                         CONFIG_RP2040_ROMFS_ROMDISK_SECTSIZE);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: romdisk_register failed: %d\n", -ret);
    }
  else
    {
      /* Mount the file system */

      ret = nx_mount(CONFIG_RP2040_ROMFS_ROMDISK_DEVNAME,
                     CONFIG_RP2040_ROMFS_MOUNT_MOUNTPOINT,
                     "romfs",
                     MS_RDONLY,
                     NULL);
      if (ret < 0)
        {
          syslog(LOG_ERR,
                 "ERROR: nx_mount(%s,%s,romfs) failed: %d\n",
                 CONFIG_RP2040_ROMFS_ROMDISK_DEVNAME,
                 CONFIG_RP2040_ROMFS_MOUNT_MOUNTPOINT,
                 ret);
        }
    }

#endif
  return ret;
}
