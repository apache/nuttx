/****************************************************************************
 * boards/arm/stm32/stm32f401rc-rs485/src/stm32_bringup.c
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

#include <stdio.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/arch.h>

#include <stm32.h>
#include <stm32_uart.h>

#include <arch/board/board.h>

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#include "stm32f401rc-rs485.h"

#include <nuttx/board.h>

#ifdef CONFIG_SENSORS_LM75
#include "stm32_lm75.h"
#endif

#ifdef CONFIG_SENSORS_QENCODER
#include "board_qencoder.h"
#endif

#ifdef CONFIG_RNDIS
#  include <nuttx/usb/rndis.h>
#endif

#ifdef CONFIG_SENSORS_HCSR04
#include "stm32_hcsr04.h"
#endif

#ifdef CONFIG_LCD_MAX7219
#include "stm32_max7219_matrix.h"
#endif

#ifdef CONFIG_CL_MFRC522
#include "stm32_mfrc522.h"
#endif

#ifdef CONFIG_STEPPER_DRV8825
#include "stm32_drv8266.h"
#endif

#ifdef CONFIG_SENSORS_BMP280
#include "stm32_bmp280.h"
#endif

#ifdef CONFIG_LCD_BACKPACK
#include "stm32_lcd_backpack.h"
#endif

#ifdef CONFIG_WS2812
#include "stm32_ws2812.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_i2c_register
 *
 * Description:
 *   Register one I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
static void stm32_i2c_register(int bus)
{
  struct i2c_master_s *i2c;
  int ret;

  i2c = stm32_i2cbus_initialize(bus);
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

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
static void stm32_i2ctool(void)
{
  stm32_i2c_register(1);
#if 0
  stm32_i2c_register(1);
  stm32_i2c_register(2);
#endif
}
#else
#  define stm32_i2ctool()
#endif

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
  int ret = OK;

#ifdef CONFIG_USERLED
  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
  stm32_i2ctool();
#endif

#ifdef CONFIG_I2C_EE_24XX
  ret = stm32_at24_init("/dev/eeprom");
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize EEPROM HX24LCXXB: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_LM75_I2C
  /* Configure and initialize the LM75 sensor */

  ret = board_lm75_initialize(0, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_lm75_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_INPUT_BUTTONS
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC driver. */

  ret = stm32_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_adc_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_PWM
  /* Initialize PWM and register the PWM device */

  ret = stm32_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_pwm_setup() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_LCD_MAX7219
  /* Configure and initialize the MAX7219 driver */

  ret = board_max7219_matrix_initialize(1);
  if (ret < 0)
    {
      syslog(LOG_ERR, \
       "ERROR: board_max7219_matrix_initialize failed: %d\n", ret);
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
      syslog(LOG_ERR, "ERROR: Failed to initialize MMC/SD driver: %d\n",
              ret);
      return ret;
    }
#endif

#ifdef CONFIG_SENSORS_QENCODER
  /* Initialize and register the qencoder driver */

  ret = board_qencoder_initialize(0, STM32F401RCRS485_QETIMER);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
#endif

#if defined(CONFIG_RNDIS) && !defined(CONFIG_RNDIS_COMPOSITE)
  uint8_t mac[6];
  mac[0] = 0xa0; /* TODO */
  mac[1] = (CONFIG_NETINIT_MACADDR_2 >> (8 * 0)) & 0xff;
  mac[2] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 3)) & 0xff;
  mac[3] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 2)) & 0xff;
  mac[4] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 1)) & 0xff;
  mac[5] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 0)) & 0xff;
  usbdev_rndis_initialize(mac);
#endif

#ifdef CONFIG_SENSORS_HCSR04
  /* Configure and initialize the HC-SR04 distance sensor */

  ret = board_hcsr04_initialize(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_hcsr04_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_STEPPER_DRV8825
  /* Configure and initialize the drv8825 driver */

  ret = board_drv8825_initialize(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_drv8825_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_CL_MFRC522
  ret = stm32_mfrc522initialize("/dev/rfid0");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_mfrc522initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_BMP280
  /* Initialize the BMP280 pressure sensor. */

  ret = board_bmp280_initialize(0, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize BMP280, error %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_LCD_BACKPACK
  /* slcd:0, i2c:1, rows=2, cols=16 */

  ret = board_lcd_backpack_init(0, 1, 2, 16);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize PCF8574 LCD, error %d\n", ret);
      return ret;
    }
#endif

#if defined(CONFIG_WS2812) && defined(CONFIG_WS2812_LED_COUNT)
  /* Configure and initialize the WS2812 LEDs. */

  ret = board_ws2812_initialize(0, WS2812_SPI, CONFIG_WS2812_LED_COUNT);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_ws2812_initialize() failed: %d\n", ret);
    }
#endif

  return ret;
}
