/****************************************************************************
 * boards/risc-v/esp32c3/esp32c3-devkit/src/esp32c3-devkit.h
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

#ifndef __BOARDS_RISCV_ESP32C3_ESP32C3_DEVKIT_SRC_ESP32C3_DEVKIT_H
#define __BOARDS_RISCV_ESP32C3_ESP32C3_DEVKIT_SRC_ESP32C3_DEVKIT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TIMERS */

#define TIMER0 0
#define TIMER1 1

/* ONESHOT */

#define ONESHOT_TIMER         1
#define ONESHOT_RESOLUTION_US 1

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_BOARDCTL=y :
 *     Called from the NSH library via board_app_initialize()
 *
 ****************************************************************************/

int esp32c3_bringup(void);

/****************************************************************************
 * Name: esp32c3_gpio_init
 ****************************************************************************/

#ifdef CONFIG_DEV_GPIO
int esp32c3_gpio_init(void);
#endif

/****************************************************************************
 * Name: board_wdt_init
 *
 * Description:
 *   Configure the timer driver.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_WATCHDOG
int board_wdt_init(void);
#endif

/****************************************************************************
 * Name: board_spidev_initialize
 *
 * Description:
 *   Initialize SPI driver and register the /dev/spi device.
 *
 * Input Parameters:
 *   bus - The SPI bus number, used to build the device path as /dev/spiN
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_DRIVER
int board_spidev_initialize(int bus);
#endif

/****************************************************************************
 * Name: board_spislavedev_initialize
 *
 * Description:
 *   Initialize SPI Slave driver and register the /dev/spislv device.
 *
 * Input Parameters:
 *   bus - The SPI bus number, used to build the device path as /dev/spislvN
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_SLAVE
int board_spislavedev_initialize(int bus);
#endif

/****************************************************************************
 * Name: board_i2c_init
 *
 * Description:
 *   Configure the I2C driver.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_DRIVER
int board_i2c_init(void);
#endif

/****************************************************************************
 * Name: board_oneshot_init
 *
 * Description:
 *   Configure the oneshot timer driver.
 *
 * Input Parameters:
 *   timer      - Timer instance to be used as oneshot timer.
 *   resolution - Oneshot timer resolution.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ONESHOT
int board_oneshot_init(int timer, uint16_t resolution);
#endif

/****************************************************************************
 * Name: board_bmp180_initialize
 *
 * Description:
 *   Initialize and register the BMP180 Pressure Sensor driver.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as /dev/pressN
 *   busno - The I2C bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMP180
int board_bmp180_initialize(int devno, int busno);
#endif

/****************************************************************************
 * Name: board_wlan_init
 *
 * Description:
 *   Configure the wireless subsystem.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_WIRELESS
int board_wlan_init(void);
#endif

/****************************************************************************
 * Name: board_spiflash_init
 *
 * Description:
 *   Initialize the SPIFLASH and register the MTD device.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_SPIFLASH
int board_spiflash_init(void);
#endif

/****************************************************************************
 * Name: esp32c3_spiflash_encrypt_test
 *
 * Description:
 *   Test ESP32-C3 SPI Flash driver read/write with encryption.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_SPIFLASH_ENCRYPTION_TEST
void esp32c3_spiflash_encrypt_test(void);
#endif

/****************************************************************************
 * Name: esp32c3_ledc_setup
 *
 * Description:
 *   Initialize LEDC PWM and register the PWM device.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_LEDC
int esp32c3_pwm_setup(void);
#endif

/****************************************************************************
 * Name: board_adc_init
 *
 * Description:
 *   Configure the ADC driver.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ADC
int board_adc_init(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_RISCV_ESP32C3_ESP32C3_DEVKIT_SRC_ESP32C3_DEVKIT_H */
