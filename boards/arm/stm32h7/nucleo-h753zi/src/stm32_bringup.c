/****************************************************************************
 * boards/arm/stm32h7/nucleo-h753zi/src/stm32_bringup.c
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
 * http://www.apache.org/licenses/LICENSE-2.0
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

/* System includes */

#include <nuttx/config.h>
#include <arch/board/board.h>
#include <sys/types.h>
#include <syslog.h>
#include <errno.h>

/* Filesystem includes */

#include <nuttx/fs/fs.h>

/* Communication protocol includes */

#ifdef CONFIG_I2C
#  include <nuttx/i2c/i2c_master.h>
#endif

#ifdef CONFIG_SPI
#  include <nuttx/spi/spi.h>
#endif

/* STM32-specific includes */

#include "stm32_gpio.h"
#include "hardware/stm32_gpio.h"
#include "arm_internal.h"
#include "hardware/stm32h7x3xx_rcc.h"

#ifdef CONFIG_STM32H7_I2C
#  include "stm32_i2c.h"
#endif

#ifdef CONFIG_STM32H7_SPI
#  include "stm32_spi.h"
#endif

/* Display includes */

#ifdef CONFIG_LCD_ST7796
#  include <nuttx/video/fb.h>
#  include <nuttx/lcd/st7796.h>
#endif

#ifdef CONFIG_LCD_DEV
#  include <nuttx/lcd/lcd_dev.h>
#endif

#ifdef CONFIG_LCD_SSD1306
#  include <nuttx/lcd/lcd.h>
#  include <nuttx/lcd/ssd1306.h>
#endif

#include <nuttx/board.h>
#include "nucleo-h753zi.h"

/* USB-related includes */

#ifdef CONFIG_USBMONITOR
#  include <nuttx/usb/usbmonitor.h>
#endif

#ifdef CONFIG_STM32H7_OTGFS
#  include "stm32_usbhost.h"
#endif

#ifdef CONFIG_RNDIS
#  include <nuttx/usb/rndis.h>
#endif

/* Input device includes */

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

/* LED includes */

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

/* Timer-related includes */

#ifdef HAVE_RTC_DRIVER
#  include <nuttx/timers/rtc.h>
#  include "stm32_rtc.h"
#endif

#ifdef CONFIG_CAPTURE
#  include <nuttx/timers/capture.h>
#  include "stm32_capture.h"
#endif

#ifdef CONFIG_STM32H7_IWDG
#  include "stm32_wdg.h"
#endif

/* Storage includes */

#ifdef CONFIG_STM32_ROMFS
#  include "drivers/driver_middleware/stm32_romfs.h"
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Initialization functions organized by name length (longest to shortest) */

static int nucleo_automotive_initialize(void);
static int nucleo_communication_initialize(void);
static int nucleo_connectivity_initialize(void);
static int nucleo_filesystem_initialize(void);
static int nucleo_watchdog_initialize(void);
static int nucleo_sensors_initialize(void);
static int nucleo_display_initialize(void);
static int nucleo_storage_initialize(void);
static int nucleo_timers_initialize(void);
static int nucleo_input_initialize(void);
static int nucleo_gpio_initialize(void);
static int nucleo_led_initialize(void);
static int nucleo_usb_initialize(void);
static int nucleo_rtc_initialize(void);
static int nucleo_adc_initialize(void);

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
  static int nucleo_i2c_tools_initialize(void);
#endif

#ifdef CONFIG_CAPTURE
  static int nucleo_capture_initialize(void);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nucleo_led_initialize
 *
 * Description:
 * Initialize LED subsystem based on configuration
 * Priority: HIGH (provides early visual feedback)
 *
 * Dependencies: None
 *
 * Returned Value:
 * Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nucleo_led_initialize(void)
{
  int ret = OK;

#ifdef CONFIG_NUCLEO_H753ZI_LEDS_USER

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "[ERROR: BRINGUP] - userled_lower_initialize() "
             "failed: %d\n", ret);
    }
  else
    {
      syslog(LOG_INFO,
             "[INFO: BRINGUP] - User LEDs initialized at "
             "/dev/userleds\n");
    }

#elif defined(CONFIG_NUCLEO_H753ZI_LEDS_AUTO)

    syslog(LOG_INFO,
          "[INFO: BRINGUP] - Auto LEDs enabled for system status "
          "indication\n");

#elif defined(CONFIG_NUCLEO_H753ZI_LEDS_DISABLED)

     syslog(LOG_INFO,
            "[INFO: BRINGUP] - LEDs disabled by configuration\n");

#endif

  return ret;
}

/****************************************************************************
 * Name: nucleo_filesystem_initialize
 *
 * Description:
 * Initialize filesystem support (PROCFS, ROMFS)
 * Priority: HIGH (required for logging and configuration)
 *
 * Dependencies: None
 *
 * Returned Value:
 * Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nucleo_filesystem_initialize(void)
{
  int ret = OK;

#ifdef CONFIG_FS_PROCFS

  int local_ret = nx_mount(NULL, STM32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (local_ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount PROCFS: %d\n", local_ret);
      if (ret == OK)
        {
          ret = local_ret;
        }
    }
  else
    {
      syslog(LOG_INFO,
             "PROCFS mounted at %s\n", STM32_PROCFS_MOUNTPOINT);
    }

#endif /* CONFIG_FS_PROCFS */

#ifdef CONFIG_STM32_ROMFS

  int local_ret = stm32_romfs_initialize();
  if (local_ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount ROMFS at %s: %d\n",
          CONFIG_STM32_ROMFS_MOUNTPOINT, local_ret);

      if (ret == OK)
        {
          ret = local_ret;
        }
    }
  else
    {
      syslog(LOG_INFO,
             "ROMFS mounted at %s\n", CONFIG_STM32_ROMFS_MOUNTPOINT);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: nucleo_rtc_initialize
 *
 * Description:
 * Initialize Real-Time Clock driver
 * Priority: HIGH (time services for other subsystems)
 *
 * Dependencies: None
 *
 * Returned Value:
 * Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nucleo_rtc_initialize(void)
{
  int ret = OK;

#ifdef HAVE_RTC_DRIVER

  struct rtc_lowerhalf_s *lower;

  lower = stm32_rtc_lowerhalf();
  if (!lower)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to instantiate RTC lower-half driver\n");
      return -ENOMEM;
    }

  ret = rtc_initialize(0, lower);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to bind/register RTC driver: %d\n", ret);
    }
  else
    {
      syslog(LOG_INFO,
             "RTC driver registered as /dev/rtc0\n");
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: nucleo_gpio_initialize
 *
 * Description:
 * Initialize GPIO driver for user applications
 * Priority: HIGH (required by many other drivers)
 *
 * Dependencies: None
 *
 * Returned Value:
 * Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nucleo_gpio_initialize(void)
{
  int ret = OK;

#ifdef CONFIG_DEV_GPIO

  ret = stm32_gpio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize GPIO driver: %d\n", ret);
    }
  else
    {
     syslog(LOG_INFO,
            "GPIO driver initialized\n");
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: nucleo_automotive_initialize
 *
 * Description:
 * Initialize automotive/industrial communication protocols (CAN, LIN, etc.)
 * Priority: MEDIUM
 *
 * Dependencies: GPIO, Clock configuration
 *
 * Returned Value:
 * Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nucleo_automotive_initialize(void)
{
  int ret = OK;

#ifdef CONFIG_STM32H7_FDCAN1

  /* syslog(LOG_INFO,
   * "[FDCAN1] Starting initialization...\n");
   */

  /* Initialize FDCAN1 driver FIRST */

  int local_ret = stm32_fdcansockinitialize(0);
  if (local_ret < 0)
    {
      syslog(LOG_ERR,
             "[FDCAN1] ERROR: Failed to initialize: %d\n", local_ret);

      if (ret == OK)
        {
          ret = local_ret;
        }
    }
  else
    {
      syslog(LOG_INFO,
             "[FDCAN1] Driver initialized as /dev/can0\n");

      /* DEBUG
       *
       * NOW force GPIO configuration AFTER driver init
       * syslog(LOG_INFO,
       * "[FDCAN1] Forcing GPIO reconfiguration...\n");
       *
       * syslog(LOG_INFO,
       * "[FDCAN1] GPIO_CAN1_RX: 0x%08lx\n",
       *      (unsigned long)GPIO_CAN1_RX);
       *
       * syslog(LOG_INFO,
       *  "[FDCAN1] GPIO_CAN1_TX: 0x%08lx\n",
       *      (unsigned long)GPIO_CAN1_TX);
       *
       *
       * Verify GPIO and RCC configuration
       * syslog(LOG_INFO,
       *
       *  uint32_t rcc_apb1henr = getreg32(STM32_RCC_APB1HENR);
       *  uint32_t rcc_d2ccip1r = getreg32(STM32_RCC_D2CCIP1R);
       *
       *  uint32_t gpiob_otyper = getreg32(STM32_GPIOB_OTYPER);
       *
       * "[FDCAN1] POST-CONFIG:\n");
       * syslog(LOG_INFO,
       * "[FDCAN1]   RCC_APB1HENR: 0x%08lx (bit 8=%d)\n",
       *      (unsigned long)rcc_apb1henr,
       *      (int)((rcc_apb1henr >> 8) & 1));
       * syslog(LOG_INFO, "[FDCAN1]   RCC_D2CCIP1R: 0x%08lx (clk_sel=%lu)\n",
       *      (unsigned long)rcc_d2ccip1r,
       *      (unsigned long)((rcc_d2ccip1r >> 28) & 0x3));
       * syslog(LOG_INFO, "[FDCAN1]   GPIOB_MODER: 0x%08lx "
       *      "(PB8[17:16]=%lu, PB9[19:18]=%lu)\n",
       *      (unsigned long)gpiob_moder,
       *      (unsigned long)((gpiob_moder >> 16) & 0x3),
       *      (unsigned long)((gpiob_moder >> 18) & 0x3));
       * syslog(LOG_INFO, "[FDCAN1]   GPIOB_AFRH: 0x%08lx "
       *      "(PB8[3:0]=%lu, PB9[7:4]=%lu)\n",
       *      (unsigned long)gpiob_afrh,
       *      (unsigned long)(gpiob_afrh & 0xf),
       *      (unsigned long)((gpiob_afrh >> 4) & 0xf));
       * syslog(LOG_INFO, "[FDCAN1]   GPIOB_OTYPER: 0x%08lx "
       *      "(PB8[8]=%d, PB9[9]=%d)\n",
       *      (unsigned long)gpiob_otyper,
       *      (int)((gpiob_otyper >> 8) & 1),
       *      (int)((gpiob_otyper >> 9) & 1));
       *
       *
       * Expected values:
       * MODER: PB8=10b (AF), PB9=10b (AF)
       * AFRH: PB8=9 (AF9), PB9=9 (AF9)
       * OTYPER: PB8=0 (push-pull), PB9=0 (push-pull)
       */

      stm32_configgpio(GPIO_CAN1_RX);  /* PB8 */
      stm32_configgpio(GPIO_CAN1_TX);  /* PB9 */

      uint32_t gpiob_moder = getreg32(STM32_GPIOB_MODER);
      uint32_t gpiob_afrh = getreg32(STM32_GPIOB_AFRH);

      if (((gpiob_moder >> 16) & 0x3) != 0x2 ||
          ((gpiob_moder >> 18) & 0x3) != 0x2)
        {
          syslog(LOG_ERR,
                 "[FDCAN1] ERROR: GPIO not in AF mode!\n");
        }

      if ((gpiob_afrh & 0xf) != 9 || ((gpiob_afrh >> 4) & 0xf) != 9)
        {
          syslog(LOG_ERR,
                 "[FDCAN1] ERROR: GPIO not set to AF9!\n");
        }
    }
#endif

#ifdef CONFIG_STM32H7_FDCAN2

  /* Initialize FDCAN2 and register as /dev/can1 */

  int local_ret = stm32_fdcansockinitialize(1);
  if (local_ret < 0)
    {
      syslog(LOG_ERR, "[FDCAN2] ERROR: Failed to initialize: %d\n",
             local_ret);
      if (ret == OK)
        {
          ret = local_ret;
        }
    }
  else
    {
      syslog(LOG_INFO,
             "[FDCAN2] Initialized as /dev/can1\n");
    }
#endif

  /* Future: LIN, FlexRay initialization here */

  return ret;
}

/****************************************************************************
 * Name: nucleo_communication_initialize
 *
 * Description:
 *   Initialize general-purpose communication bus drivers (SPI, I2C)
 *   Priority: HIGH (required by sensors and other peripherals)
 *
 * Dependencies: GPIO
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nucleo_communication_initialize(void)
{
  int ret = OK;
  int local_ret;

  UNUSED(local_ret);

#ifdef CONFIG_STM32H7_SPI
  local_ret = stm32_spi_initialize();
  if (local_ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_spi_initialize failed: %d\n",
             local_ret);
      if (ret == OK)
        {
          ret = local_ret;
        }
    }
  else
    {
      syslog(LOG_INFO, "SPI buses initialized\n");
    }
#endif

#ifdef CONFIG_SPI_DRIVER
  local_ret = stm32_spidev_register_all();
  if (local_ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_spidev_register_all failed: %d\n",
             local_ret);
    }
  else
    {
      syslog(LOG_INFO, "SPI character drivers registered\n");
    }
#endif

#ifdef CONFIG_STM32H7_I2C
  local_ret = stm32_i2c_initialize();
  if (local_ret < 0)
    {
      syslog(LOG_ERR, "ERROR: I2C bus initialization failed: %d\n",
             local_ret);
      if (ret == OK)
        {
          ret = local_ret;
        }
    }
  else
    {
      syslog(LOG_INFO, "I2C buses initialized\n");
#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
      local_ret = nucleo_i2c_tools_initialize();
      if (local_ret < 0)
        {
          syslog(LOG_ERR, "ERROR: I2C tools registration failed: %d\n",
                 local_ret);
        }
#endif
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: nucleo_input_initialize
 *
 * Description:
 * Initialize input devices (buttons, etc.)
 * Priority: MEDIUM
 *
 * Dependencies: GPIO
 *
 * Returned Value:
 * Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nucleo_input_initialize(void)
{
  int ret = OK;

#ifdef CONFIG_INPUT_BUTTONS

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
  else
    {
      syslog(LOG_INFO,
             "Buttons driver registered as /dev/buttons\n");
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: nucleo_usb_initialize
 *
 * Description:
 * Initialize USB subsystem (host, device, monitoring)
 * Priority: MEDIUM
 *
 * Dependencies: None
 *
 * Returned Value:
 * Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nucleo_usb_initialize(void)
{
  int ret = OK;

#ifdef HAVE_USBHOST

  int local_ret = stm32_usbhost_initialize();
  if (local_ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize USB host: %d\n", local_ret);
      if (ret == OK)
        {
          ret = local_ret;
        }
    }
  else
    {
      syslog(LOG_INFO,
             "USB host initialized\n");
    }
#endif

#ifdef HAVE_USBMONITOR

  int local_ret = usbmonitor_start();
  if (local_ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to start USB monitor: %d\n",
             local_ret);
      if (ret == OK)
        {
          ret = local_ret;
        }
    }
  else
    {
      syslog(LOG_INFO,
             "USB monitor started\n");
    }
#endif

#if defined(CONFIG_CDCACM) && !defined(CONFIG_CDCACM_CONSOLE) && \
    !defined(CONFIG_CDCACM_COMPOSITE)

  syslog(LOG_INFO, "Initializing CDC/ACM device\n");

  int local_ret = cdcacm_initialize(0, NULL);
  if (local_ret < 0)
    {
      syslog(LOG_ERR, "ERROR: cdcacm_initialize failed: %d\n", local_ret);
      if (ret == OK)
        {
          ret = local_ret;
        }
    }
  else
    {
      syslog(LOG_INFO,
             "CDC/ACM device initialized\n");
    }
#endif

#if defined(CONFIG_RNDIS) && !defined(CONFIG_RNDIS_COMPOSITE)

  uint8_t mac[6];
  mac[0] = 0xa0;
  mac[1] = (CONFIG_NETINIT_MACADDR_2 >> (8 * 0)) & 0xff;
  mac[2] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 3)) & 0xff;
  mac[3] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 2)) & 0xff;
  mac[4] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 1)) & 0xff;
  mac[5] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 0)) & 0xff;

  int local_ret = usbdev_rndis_initialize(mac);
  if (local_ret < 0)
    {
      syslog(LOG_ERR, "ERROR: RNDIS initialization failed: %d\n",
             local_ret);
      if (ret == OK)
        {
          ret = local_ret;
        }
    }
  else
    {
      syslog(LOG_INFO,
             "RNDIS USB device initialized\n");
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: nucleo_adc_initialize
 *
 * Description:
 * Initialize Analog-to-Digital Converter
 * Priority: MEDIUM
 *
 * Dependencies: GPIO
 *
 * Returned Value:
 * Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nucleo_adc_initialize(void)
{
  int ret = OK;

#ifdef CONFIG_ADC

  ret = stm32_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_adc_setup failed: %d\n", ret);
    }
  else
    {
      syslog(LOG_INFO,
             "ADC driver initialized\n");
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: nucleo_sensors_initialize
 *
 * Description:
 * Initialize sensor drivers (IMU, magnetometer, RFID, etc.)
 * Priority: LOW (application-specific)
 *
 * Dependencies: I2C, SPI, GPIO
 *
 * Returned Value:
 * Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nucleo_sensors_initialize(void)
{
  int ret = OK;

#ifdef CONFIG_SENSORS_LSM6DSL

  int local_ret = stm32_lsm6dsl_initialize("/dev/lsm6dsl0");
  if (local_ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize LSM6DSL driver: %d\n",
             local_ret);
      if (ret == OK)
        {
          ret = local_ret;
        }
    }
  else
    {
      syslog(LOG_INFO,
             "LSM6DSL sensor initialized as /dev/lsm6dsl0\n");
    }
#endif

#ifdef CONFIG_SENSORS_LSM9DS1

  int local_ret = stm32_lsm9ds1_initialize();
  if (local_ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize LSM9DS1 driver: %d\n",
             local_ret);
      if (ret == OK)
        {
          ret = local_ret;
        }
    }
  else
    {
      syslog(LOG_INFO,
             "LSM9DS1 sensor initialized\n");
    }
#endif

#ifdef CONFIG_SENSORS_LSM303AGR

  int local_ret = stm32_lsm303agr_initialize("/dev/lsm303mag0");
  if (local_ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize LSM303AGR driver: %d\n",
             local_ret);
      if (ret == OK)
        {
          ret = local_ret;
        }
    }
  else
    {
      syslog(LOG_INFO,
             "LSM303AGR magnetometer initialized as /dev/lsm303mag0\n");
    }
#endif

#ifdef CONFIG_NUCLEO_H753ZI_MFRC522_ENABLE

  int local_ret = stm32_mfrc522initialize(MFRC522_DEVPATH);
  if (local_ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_mfrc522initialize() failed: %d\n",
             local_ret);

      if (ret == OK)
        {
          ret = local_ret;
        }
    }
  else
    {
      syslog(LOG_INFO,
             "MFRC522 RFID reader initialized successfully at %s\n",
             MFRC522_DEVPATH);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: nucleo_display_initialize
 *
 * Description:
 *   Initialize display drivers (LCD, OLED, TFT, etc.)
 *   Priority: MEDIUM
 *
 * Dependencies: SPI, I2C, GPIO
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nucleo_display_initialize(void)
{
  int ret = OK;

#if defined(CONFIG_LCD_SSD1306) && \
    defined(CONFIG_NUCLEO_H753ZI_SSD1306_ENABLE)

  ret = board_lcd_initialize();
  if (ret == OK)
    {
      struct lcd_dev_s *lcd = board_lcd_getdev(NUCLEO_SSD1306_DEVNO);
      if (lcd != NULL)
        {
#ifdef CONFIG_LCD_DEV

          ret = lcddev_register(NUCLEO_SSD1306_DEVNO);
          if (ret < 0)
            {
              syslog(LOG_ERR,
                     "ERROR: lcddev_register(%d) failed: %d\n",
                     NUCLEO_SSD1306_DEVNO, ret);
            }
          else
            {
              syslog(LOG_INFO,
                     "SSD1306 OLED registered at /dev/lcd%d\n",
                     NUCLEO_SSD1306_DEVNO);
            }
#endif
        }
    }
#endif

#if defined(CONFIG_LCD_ST7796) && \
    defined(CONFIG_NUCLEO_H753ZI_ST7796_ENABLE)

  /* syslog(LOG_INFO, "Initializing ST7796 TFT display...\n"); */

  int local_ret = stm32_st7796initialize(0);
  if (local_ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_st7796initialize() failed: %d\n",
             local_ret);
      if (ret == OK)
        {
          ret = local_ret;
        }
    }
  else
    {
      /* syslog(LOG_INFO,
       * "ST7796 TFT display initialized successfully at %s\n",
       *  ST7796_FB_PATH);
       */

      /* CRITICAL: Flush splashscreen from RAM to SPI display */

      local_ret = stm32_st7796_flush_fb();
      if (local_ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to flush splashscreen: %d\n",
                 local_ret);
        }
      else
        {
          /* syslog(LOG_INFO, "
           *        ST7796: Splashscreen flushed to display\n");
           */
        }

      /* Enable backlight after flush */

      stm32_st7796_backlight(true);

      /* syslog(LOG_INFO,
       *        "ST7796 backlight enabled\n");
       */
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: nucleo_connectivity_initialize
 *
 * Description:
 * Initialize wireless connectivity modules (Wi-Fi, Bluetooth, etc.)
 * Priority: LOW (application-specific)
 *
 * Dependencies: SPI, I2C, GPIO
 *
 * Returned Value:
 * Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nucleo_connectivity_initialize(void)
{
  int ret = OK;

#ifdef CONFIG_PCA9635PW

  ret = stm32_pca9635_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_pca9635_initialize failed: %d\n", ret);
    }
  else
    {
      syslog(LOG_INFO, "PCA9635 LED controller initialized\n");
    }
#endif

#ifdef CONFIG_WL_NRF24L01

  ret = stm32_wlinitialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize wireless driver: %d\n",
             ret);
    }
  else
    {
      syslog(LOG_INFO, "NRF24L01 wireless driver initialized\n");
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: nucleo_storage_initialize
 *
 * Description:
 * Initialize storage devices (SD card, flash, etc.)
 * Priority: MEDIUM
 *
 * Dependencies: SPI (for SD cards), filesystem
 *
 * Returned Value:
 * Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nucleo_storage_initialize(void)
{
  int ret = OK;

#ifdef CONFIG_MMCSD_SPI

  ret = stm32_mmcsd_initialize(CONFIG_NSH_MMCSDMINOR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SD slot %d: %d\n",
             CONFIG_NSH_MMCSDMINOR, ret);
    }
  else
    {
      syslog(LOG_INFO, "MMC/SD SPI driver initialized (slot %d)\n",
             CONFIG_NSH_MMCSDMINOR);
    }
#endif

#ifdef CONFIG_MTD
#ifdef HAVE_PROGMEM_CHARDEV

  ret = stm32_progmem_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize MTD progmem: %d\n", ret);
    }
  else
    {
      syslog(LOG_INFO, "MTD program memory initialized\n");
    }
#endif
#endif

  return ret;
}

/****************************************************************************
 * Name: nucleo_timers_initialize
 *
 * Description:
 * Initialize timer-related drivers (PWM, capture, etc.)
 * Priority: LOW
 *
 * Dependencies: GPIO
 *
 * Returned Value:
 * Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nucleo_timers_initialize(void)
{
  int ret = OK;

#ifdef CONFIG_PWM

  int local_ret = stm32_pwm_setup();
  if (local_ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_pwm_setup() failed: %d\n", local_ret);
      if (ret == OK)
        {
          ret = local_ret;
        }
    }
  else
    {
      syslog(LOG_INFO, "PWM drivers initialized\n");
    }
#endif

#ifdef CONFIG_CAPTURE

  int local_ret = nucleo_capture_initialize();
  if (local_ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: nucleo_capture_initialize() failed: %d\n",
             local_ret);
      if (ret == OK)
        {
          ret = local_ret;
        }
    }
  else
    {
      syslog(LOG_INFO, "Timer capture drivers initialized\n");
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: nucleo_watchdog_initialize
 *
 * Description:
 * Initialize watchdog timer
 * Priority: LOW (should be last - for system monitoring)
 *
 * Dependencies: None (intentionally independent)
 *
 * Returned Value:
 * Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nucleo_watchdog_initialize(void)
{
  int ret = OK;

#ifdef CONFIG_STM32H7_IWDG

  ret = stm32_iwdginitialize("/dev/watchdog0", STM32_LSI_FREQUENCY);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize watchdog: %d\n", ret);
    }
  else
    {
      syslog(LOG_INFO, "Watchdog initialized as /dev/watchdog0\n");
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: nucleo_i2c_tools_initialize
 *
 * Description:
 * Initialize I2C tools for debugging and development. This registers the
 * I2C buses as character drivers (e.g., /dev/i2c1) to allow tools like
 * i2c-tools to interact with the bus from user space.
 *
 * Priority: LOW (development/debugging only)
 *
 * Dependencies: I2C hardware must be initialized first.
 *
 * Returned Value:
 * Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
static int stm32_i2c_register(int bus)
{
  struct i2c_master_s *i2c;
  int ret;

  /* Get the I2C bus instance */

  i2c = stm32_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", bus);
      return -ENODEV;
    }

  /* Register the bus as a character driver (/dev/i2cN) */

  ret = i2c_register(i2c, bus);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
             bus, ret);

      /* Only uninitialize if registration fails and no other
       * peripheral is using this bus instance.
       */

      stm32_i2cbus_uninitialize(i2c);
      return ret;
    }

  syslog(LOG_INFO, "I2C%d registered for tools at /dev/i2c%d\n", bus, bus);
  return OK;
}

static int nucleo_i2c_tools_initialize(void)
{
  int ret = OK;
  int local_ret;

#ifdef CONFIG_STM32H7_I2C1
  local_ret = stm32_i2c_register(1);
  if (local_ret < 0)
    {
      ret = local_ret;
    }
#endif

#ifdef CONFIG_STM32H7_I2C2
  local_ret = stm32_i2c_register(2);
  if (local_ret < 0 && ret == OK)
    {
      ret = local_ret;
    }
#endif

#ifdef CONFIG_STM32H7_I2C3
  local_ret = stm32_i2c_register(3);
  if (local_ret < 0 && ret == OK)
    {
      ret = local_ret;
    }
#endif

#ifdef CONFIG_STM32H7_I2C4
  local_ret = stm32_i2c_register(4);
  if (local_ret < 0 && ret == OK)
    {
      ret = local_ret;
    }
#endif

  return ret;
}
#endif

/****************************************************************************
 * Name: nucleo_capture_initialize
 *
 * Description:
 * Initialize and register capture drivers
 * Priority: LOW
 *
 * Dependencies: Timer peripherals
 *
 * Returned Value:
 * Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_CAPTURE
static int nucleo_capture_initialize(void)
{
  int ret;

  struct cap_lowerhalf_s *lower[] =
    {
#if defined(CONFIG_STM32H7_TIM1_CAP)
      stm32_cap_initialize(1),
#endif
#if defined(CONFIG_STM32H7_TIM2_CAP)
      stm32_cap_initialize(2),
#endif
#if defined(CONFIG_STM32H7_TIM3_CAP)
      stm32_cap_initialize(3),
#endif
#if defined(CONFIG_STM32H7_TIM4_CAP)
      stm32_cap_initialize(4),
#endif
#if defined(CONFIG_STM32H7_TIM5_CAP)
      stm32_cap_initialize(5),
#endif
#if defined(CONFIG_STM32H7_TIM8_CAP)
      stm32_cap_initialize(8),
#endif
#if defined(CONFIG_STM32H7_TIM12_CAP)
      stm32_cap_initialize(12),
#endif
#if defined(CONFIG_STM32H7_TIM13_CAP)
      stm32_cap_initialize(13),
#endif
#if defined(CONFIG_STM32H7_TIM14_CAP)
      stm32_cap_initialize(14),
#endif
#if defined(CONFIG_STM32H7_TIM15_CAP)
      stm32_cap_initialize(15),
#endif
#if defined(CONFIG_STM32H7_TIM16_CAP)
      stm32_cap_initialize(16),
#endif
#if defined(CONFIG_STM32H7_TIM17_CAP)
      stm32_cap_initialize(17),
#endif
    };

  size_t count = sizeof(lower) / sizeof(lower[0]);

  if (count == 0)
    {
      return OK;
    }

  ret = cap_register_multiple("/dev/cap", lower, count);
  if (ret == EINVAL)
    {
      syslog(LOG_ERR, "ERROR: cap_register_multiple path is invalid\n");
    }
  else if (ret == EEXIST)
    {
      syslog(LOG_ERR,
             "ERROR: cap_register_multiple inode already exists\n");
    }
  else if (ret == ENOMEM)
    {
      syslog(LOG_ERR,
             "ERROR: cap_register_multiple not enough memory\n");
    }
  else if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: cap_register_multiple failed: %d\n", ret);
    }

  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 * Perform architecture-specific initialization with dependency-aware
 * ordering
 *
 * This function initializes all board-specific drivers and subsystems
 * in a controlled manner, ensuring that dependencies between subsystems
 * are respected and failures in one subsystem do not prevent
 * initialization of others.
 *
 * INITIALIZATION PHASES WITH DEPENDENCY MANAGEMENT:
 *
 * Phase 1 - Basic System & Visual Feedback
 * Phase 2 - Hardware Interfaces & Communication Protocols
 * Phase 3 - User Interface & USB Services
 * Phase 4 - Analog Measurement (ADC)
 * Phase 5 - Display Drivers (LCD, OLED, TFT)
 * Phase 6 - Sensors & Wireless Connectivity
 * Phase 7 - Storage Devices (SD Card, Flash)
 * Phase 8 - Timers, PWM & Signal Processing
 * Phase 9 - System Monitoring (Watchdog)
 *
 * ERROR HANDLING STRATEGY:
 * Individual subsystem failures are logged via syslog.
 * The function continues to initialize other subsystems even if one fails.
 * A single return value (ret) tracks the status of the first failure.
 *
 * Input Parameters:
 * None
 *
 * Returned Value:
 * Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret = OK;
  int subsys_ret;

  /* syslog(LOG_INFO,
   *       "\n[INFO: BRINGUP] Nucleo-H753ZI initialization...\n");
   */

  /* PHASE 1: BASIC SYSTEM & VISUAL FEEDBACK */

  /* syslog(LOG_INFO,
   *        "[INFO: BRINGUP] Phase 1: Initializing basic system & "
   *          "visual feedback\n");
   */

  subsys_ret = nucleo_led_initialize();
  if (subsys_ret != OK && ret == OK)
    {
      ret = subsys_ret;
    }

  subsys_ret = nucleo_filesystem_initialize();
  if (subsys_ret != OK && ret == OK)
    {
      ret = subsys_ret;
    }

  subsys_ret = nucleo_rtc_initialize();
  if (subsys_ret != OK && ret == OK)
    {
      ret = subsys_ret;
    }

  /* PHASE 2: HARDWARE INTERFACES & COMMUNICATION PROTOCOLS */

  /* syslog(LOG_INFO,
   *       "[INFO: BRINGUP] Phase 2: Initializing hardware interfaces & "
   *       "communication protocols\n");
   */

  subsys_ret = nucleo_gpio_initialize();
  if (subsys_ret != OK && ret == OK)
    {
      ret = subsys_ret;
    }

  subsys_ret = nucleo_communication_initialize();
  if (subsys_ret != OK && ret == OK)
    {
      ret = subsys_ret;
    }

  subsys_ret = nucleo_automotive_initialize();
  if (subsys_ret != OK && ret == OK)
    {
      ret = subsys_ret;
    }

  /* PHASE 3: USER INTERFACE & USB SERVICES */

  /* syslog(LOG_INFO,
   *       "[INFO: BRINGUP] Phase 3: Initializing user interface & "
   *       "USB services\n");
   */

  subsys_ret = nucleo_input_initialize();
  if (subsys_ret != OK && ret == OK)
    {
      ret = subsys_ret;
    }

  subsys_ret = nucleo_usb_initialize();
  if (subsys_ret != OK && ret == OK)
    {
      ret = subsys_ret;
    }

  /* PHASE 4: ANALOG MEASUREMENT (ADC) */

  /* syslog(LOG_INFO,
   *      "[INFO: BRINGUP] Phase 4: Initializing analog measurement "
   *       "(ADC)\n");
   */

  subsys_ret = nucleo_adc_initialize();
  if (subsys_ret != OK && ret == OK)
    {
      ret = subsys_ret;
    }

  /* PHASE 5: DISPLAY DRIVERS (LCD, OLED, TFT) */

  /* syslog(LOG_INFO,
   *      "[INFO: BRINGUP] Phase 5: Initializing display drivers\n");
   */

  subsys_ret = nucleo_display_initialize();
  if (subsys_ret != OK && ret == OK)
    {
      ret = subsys_ret;
    }

  /* PHASE 6: SENSORS & WIRELESS CONNECTIVITY */

  /* syslog(LOG_INFO,
   *     "[INFO: BRINGUP] Phase 6: Initializing sensors & "
   *        "wireless connectivity\n");
   */

  subsys_ret = nucleo_sensors_initialize();
  if (subsys_ret != OK && ret == OK)
    {
      ret = subsys_ret;
    }

  subsys_ret = nucleo_connectivity_initialize();
  if (subsys_ret != OK && ret == OK)
    {
      ret = subsys_ret;
    }

  /* PHASE 7: STORAGE DEVICES (SD CARD, FLASH) */

  /* syslog(LOG_INFO,
   *      "[INFO: BRINGUP] Phase 7: Initializing storage devices "
   *      "(SD card, flash)\n");
   */

  subsys_ret = nucleo_storage_initialize();
  if (subsys_ret != OK && ret == OK)
    {
      ret = subsys_ret;
    }

  /* PHASE 8: TIMERS, PWM & SIGNAL PROCESSING */

  /* syslog(LOG_INFO,
   *      "[INFO: BRINGUP] Phase 8: Initializing timers, PWM & "
   *      "signal processing\n");
   */

  subsys_ret = nucleo_timers_initialize();
  if (subsys_ret != OK && ret == OK)
    {
      ret = subsys_ret;
    }

  /* PHASE 9: SYSTEM MONITORING (WATCHDOG) */

  /* syslog(LOG_INFO,
   *      "[INFO: BRINGUP] Phase 9: Initializing system monitoring "
   *      "(watchdog)\n");
   */

  subsys_ret = nucleo_watchdog_initialize();
  if (subsys_ret != OK && ret == OK)
    {
      ret = subsys_ret;
    }

  /* INITIALIZATION COMPLETE */

  if (ret == OK)
    {
      syslog(LOG_INFO,
             "[INFO: BRINGUP] Nucleo-H753ZI board initialization "
             "completed successfully\n");
    }
  else
    {
      syslog(LOG_WARNING,
             "[WARNING: BRINGUP] Nucleo-H753ZI board initialization "
             "completed with errors: %d\n",
             ret);
      syslog(LOG_INFO,
             "[INFO: BRINGUP] System is functional, but some drivers "
             "may be unavailable\n");
    }

  return ret;
}
