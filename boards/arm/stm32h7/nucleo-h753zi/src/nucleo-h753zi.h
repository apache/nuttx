/****************************************************************************
 * boards/arm/stm32h7/nucleo-h753zi/src/nucleo-h753zi.h
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

#ifndef __BOARDS_ARM_STM32H7_NUCLEO_H753ZI_SRC_NUCLEO_H753ZI_H
#define __BOARDS_ARM_STM32H7_NUCLEO_H753ZI_SRC_NUCLEO_H753ZI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * SECTION 1: FEATURE CONFIGURATION
 ****************************************************************************/

/* Core System Features */

#define HAVE_PROC            1
#define HAVE_USBDEV          1
#define HAVE_USBHOST         1
#define HAVE_USBMONITOR      1
#define HAVE_MTDCONFIG       1
#define HAVE_PROGMEM_CHARDEV 1
#define HAVE_RTC_DRIVER      1

/* USB Feature Dependencies */

#ifndef CONFIG_STM32H7_OTGFS
#  undef HAVE_USBDEV
#  undef HAVE_USBHOST
#endif

#ifndef CONFIG_USBDEV
#  undef HAVE_USBDEV
#endif

#ifndef CONFIG_USBHOST
#  undef HAVE_USBHOST
#endif

#ifndef CONFIG_USBMONITOR
#  undef HAVE_USBMONITOR
#endif

#if !defined(HAVE_USBDEV)
#  undef CONFIG_USBDEV_TRACE
#endif

#if !defined(HAVE_USBHOST)
#  undef CONFIG_USBHOST_TRACE
#endif

#if !defined(CONFIG_USBDEV_TRACE) && !defined(CONFIG_USBHOST_TRACE)
#  undef HAVE_USBMONITOR
#endif

/* MTD Feature Dependencies */

#if !defined(CONFIG_STM32H7_PROGMEM) || !defined(CONFIG_MTD_PROGMEM)
#  undef HAVE_PROGMEM_CHARDEV
#endif

/* RTC Feature Dependencies */

#if !defined(CONFIG_RTC) || !defined(CONFIG_RTC_DRIVER)
#  undef HAVE_RTC_DRIVER
#endif

/* Flash-based Parameters */

#if defined(CONFIG_MMCSD)
#  define FLASH_BASED_PARAMS
#endif

/****************************************************************************
 * SECTION 2: DEVICE DRIVER PATHS
 ****************************************************************************/

/* LED Driver */

#define LED_DRIVER_PATH      "/dev/userleds"

/* Button Driver */

#define BUTTONS_DRIVER_PATH  "/dev/buttons"

/* RTC Driver */

#define RTC_DRIVER_PATH      "/dev/rtc0"

/* CAN Driver */

#define CAN0_DRIVER_PATH     "/dev/can0"

/* Sensor Drivers */

#define MFRC522_DEVPATH      "/dev/rfid0"

/* Display Drivers */

#define ST7796_FB_PATH       "/dev/fb0"

/* Filesystem Paths */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define STM32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* MTD Devices */

#define PROGMEM_MTD_MINOR    0

/****************************************************************************
 * SECTION 3: GPIO HARDWARE DEFINITIONS
 ****************************************************************************/

/****************************************************************************
 * Board GPIO - LEDs
 ****************************************************************************/

/* LED GPIO Definitions */

#define GPIO_LD1         (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                          GPIO_OUTPUT_CLEAR | GPIO_PORTB | GPIO_PIN0)
#define GPIO_LD2         (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                          GPIO_OUTPUT_CLEAR | GPIO_PORTE | GPIO_PIN1)
#define GPIO_LD3         (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                          GPIO_OUTPUT_CLEAR | GPIO_PORTB | GPIO_PIN14)

/* LED Logical Name Aliases */

#define GPIO_LED_GREEN       GPIO_LD1
#define GPIO_LED_ORANGE      GPIO_LD2
#define GPIO_LED_RED         GPIO_LD3

/****************************************************************************
 * Board GPIO - Buttons
 ****************************************************************************/

#if defined(CONFIG_NUCLEO_H753ZI_BUTTON_SUPPORT) || \
    defined(CONFIG_NUCLEO_H753ZI_GPIO_DRIVER)
#  define GPIO_BTN_BUILT_IN    (GPIO_INPUT | GPIO_PULLDOWN | GPIO_EXTI | \
                                GPIO_PORTC | GPIO_PIN13)
#endif

/****************************************************************************
 * USB GPIO
 ****************************************************************************/

#define GPIO_OTGFS_VBUS      (GPIO_INPUT | GPIO_FLOAT | GPIO_SPEED_100MHz | \
                              GPIO_OPENDRAIN | GPIO_PORTA | GPIO_PIN9)
#define GPIO_OTGFS_PWRON     (GPIO_OUTPUT | GPIO_FLOAT | GPIO_SPEED_100MHz | \
                              GPIO_PUSHPULL | GPIO_PORTG | GPIO_PIN6)

#ifdef CONFIG_USBHOST
#  define GPIO_OTGFS_OVER    (GPIO_INPUT | GPIO_EXTI | GPIO_FLOAT | \
                              GPIO_SPEED_100MHz | GPIO_PUSHPULL |   \
                              GPIO_PORTG | GPIO_PIN7)
#else
#  define GPIO_OTGFS_OVER    (GPIO_INPUT | GPIO_FLOAT | GPIO_SPEED_100MHz | \
                              GPIO_PUSHPULL | GPIO_PORTG | GPIO_PIN7)
#endif

/****************************************************************************
 * Generic GPIO Examples
 ****************************************************************************/

/* GPIO Subsystem Definitions */

#define BOARD_NGPIOIN        1
#define BOARD_NGPIOOUT       3
#define BOARD_NGPIOINT       1

/* Placeholder - example for in, out and interrupt */

#define GPIO_IN1             (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTE | \
                              GPIO_PIN2)
#define GPIO_OUT1            (GPIO_OUTPUT | GPIO_PUSHPULL | \
                              GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | \
                              GPIO_PORTE | GPIO_PIN4)
#define GPIO_INT1            (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTE | \
                              GPIO_PIN5)

/****************************************************************************
 * SECTION 4: PERIPHERAL DEVICE CONFIGURATIONS
 ****************************************************************************/

/****************************************************************************
 * Core Communication Buses - I2C Pin Configurations
 ****************************************************************************/

/* I2C1 Pin Configurations */

#ifdef CONFIG_NUCLEO_H753ZI_I2C1_ENABLE

#  ifdef CONFIG_NUCLEO_H753ZI_I2C1_PINSET_1
     /* AF4: I2C1 on PB6/PB7 (Arduino D10/D9) */
#    define GPIO_I2C1_SCL  GPIO_I2C1_SCL_1  /* PB6 - AF4 */
#    define GPIO_I2C1_SDA  GPIO_I2C1_SDA_1  /* PB7 - AF4 */

#  elif defined(CONFIG_NUCLEO_H753ZI_I2C1_PINSET_2)
     /* AF4: I2C1 on PB8/PB9 (Morpho) */
#    define GPIO_I2C1_SCL  GPIO_I2C1_SCL_2  /* PB8 - AF4 */
#    define GPIO_I2C1_SDA  GPIO_I2C1_SDA_2  /* PB9 - AF4 */
#  endif

#  define I2C1_FREQUENCY  CONFIG_NUCLEO_H753ZI_I2C1_DEFAULT_FREQUENCY

#endif /* CONFIG_NUCLEO_H753ZI_I2C1_ENABLE */

/* I2C2 Pin Configurations */

#ifdef CONFIG_NUCLEO_H753ZI_I2C2_ENABLE

#  ifdef CONFIG_NUCLEO_H753ZI_I2C2_PINSET_1
     /* AF4: I2C2 on PB10/PB11 */
#    define GPIO_I2C2_SCL  GPIO_I2C2_SCL_1  /* PB10 - AF4 */
#    define GPIO_I2C2_SDA  GPIO_I2C2_SDA_1  /* PB11 - AF4 */

#  elif defined(CONFIG_NUCLEO_H753ZI_I2C2_PINSET_2)
     /* AF4: I2C2 on PF1/PF0 */
#    define GPIO_I2C2_SCL  GPIO_I2C2_SCL_2  /* PF1 - AF4 */
#    define GPIO_I2C2_SDA  GPIO_I2C2_SDA_2  /* PF0 - AF4 */

#  elif defined(CONFIG_NUCLEO_H753ZI_I2C2_PINSET_3)
     /* AF4: I2C2 on PH4/PH5 */
#    define GPIO_I2C2_SCL  GPIO_I2C2_SCL_3  /* PH4 - AF4 */
#    define GPIO_I2C2_SDA  GPIO_I2C2_SDA_3  /* PH5 - AF4 */
#  endif

#  define I2C2_FREQUENCY  CONFIG_NUCLEO_H753ZI_I2C2_DEFAULT_FREQUENCY

#endif /* CONFIG_NUCLEO_H753ZI_I2C2_ENABLE */

/* I2C3 Pin Configurations */

#ifdef CONFIG_NUCLEO_H753ZI_I2C3_ENABLE

#  ifdef CONFIG_NUCLEO_H753ZI_I2C3_PINSET_1
     /* AF4: I2C3 on PA8/PC9 */
#    define GPIO_I2C3_SCL  GPIO_I2C3_SCL_1  /* PA8 - AF4 */
#    define GPIO_I2C3_SDA  GPIO_I2C3_SDA_1  /* PC9 - AF4 */

#  elif defined(CONFIG_NUCLEO_H753ZI_I2C3_PINSET_2)
     /* AF4: I2C3 on PH7/PH8 */
#    define GPIO_I2C3_SCL  GPIO_I2C3_SCL_2  /* PH7 - AF4 */
#    define GPIO_I2C3_SDA  GPIO_I2C3_SDA_2  /* PH8 - AF4 */
#  endif

#  define I2C3_FREQUENCY  CONFIG_NUCLEO_H753ZI_I2C3_DEFAULT_FREQUENCY

#endif /* CONFIG_NUCLEO_H753ZI_I2C3_ENABLE */

/* I2C4 Pin Configurations */

#ifdef CONFIG_NUCLEO_H753ZI_I2C4_ENABLE

#  ifdef CONFIG_NUCLEO_H753ZI_I2C4_PINSET_1
     /* AF4: I2C4 on PD12/PD13 */
#    define GPIO_I2C4_SCL  GPIO_I2C4_SCL_1  /* PD12 - AF4 */
#    define GPIO_I2C4_SDA  GPIO_I2C4_SDA_1  /* PD13 - AF4 */

#  elif defined(CONFIG_NUCLEO_H753ZI_I2C4_PINSET_2)
     /* AF4: I2C4 on PF14/PF15 */
#    define GPIO_I2C4_SCL  GPIO_I2C4_SCL_2  /* PF14 - AF4 */
#    define GPIO_I2C4_SDA  GPIO_I2C4_SDA_2  /* PF15 - AF4 */

#  elif defined(CONFIG_NUCLEO_H753ZI_I2C4_PINSET_3)
     /* AF4: I2C4 on PH11/PH12 */
#    define GPIO_I2C4_SCL  GPIO_I2C4_SCL_3  /* PH11 - AF4 */
#    define GPIO_I2C4_SDA  GPIO_I2C4_SDA_3  /* PH12 - AF4 */

#  elif defined(CONFIG_NUCLEO_H753ZI_I2C4_PINSET_4)
     /* AF6: I2C4 on PB6/PB7 (shared with I2C1!) */
#    define GPIO_I2C4_SCL  GPIO_I2C4_SCL_4  /* PB6 - AF6 */
#    define GPIO_I2C4_SDA  GPIO_I2C4_SDA_4  /* PB7 - AF6 */

#  elif defined(CONFIG_NUCLEO_H753ZI_I2C4_PINSET_5)
     /* AF6: I2C4 on PB8/PB9 (shared with I2C1!) */
#    define GPIO_I2C4_SCL  GPIO_I2C4_SCL_5  /* PB8 - AF6 */
#    define GPIO_I2C4_SDA  GPIO_I2C4_SDA_5  /* PB9 - AF6 */
#  endif

#  define I2C4_FREQUENCY  CONFIG_NUCLEO_H753ZI_I2C4_DEFAULT_FREQUENCY

#endif /* CONFIG_NUCLEO_H753ZI_I2C4_ENABLE */

/****************************************************************************
 * Sensors
 ****************************************************************************/

/* MFRC522 - SPI RFID Reader */

#ifdef CONFIG_NUCLEO_H753ZI_MFRC522_ENABLE

/* Validate Kconfig */

#  ifndef CONFIG_NUCLEO_H753ZI_MFRC522_SPI_BUS
#    error "MFRC522 enabled but SPI bus not configured"
#  endif

#  ifndef CONFIG_NUCLEO_H753ZI_MFRC522_DEVID
#    error "MFRC522 enabled but device ID not configured"
#  endif

#  ifndef CONFIG_NUCLEO_H753ZI_MFRC522_CS_PIN
#    error "MFRC522 enabled but CS pin not configured"
#  endif

/* Device configuration (from Kconfig) */

#  define MFRC522_SPI_BUS            CONFIG_NUCLEO_H753ZI_MFRC522_SPI_BUS
#  define MFRC522_DEVICE_ID          CONFIG_NUCLEO_H753ZI_MFRC522_DEVID
#  define MFRC522_CS_PIN             CONFIG_NUCLEO_H753ZI_MFRC522_CS_PIN

/* CS Active Level */

#  if defined(CONFIG_NUCLEO_H753ZI_MFRC522_CS_ACTIVE_LOW)
#    define MFRC522_CS_ACTIVE_LOW    true
#  elif defined(CONFIG_NUCLEO_H753ZI_MFRC522_CS_ACTIVE_HIGH)
#    define MFRC522_CS_ACTIVE_LOW    false
#  else
     /* Default to active low if neither is explicitly set */
#    define MFRC522_CS_ACTIVE_LOW    true
#  endif

/* IRQ Configuration */

#  ifdef CONFIG_NUCLEO_H753ZI_MFRC522_IRQ_ENABLE
#    ifndef CONFIG_NUCLEO_H753ZI_MFRC522_IRQ_PIN
#      error "MFRC522 IRQ enabled but IRQ pin not configured"
#    endif
#    define MFRC522_IRQ_PIN          CONFIG_NUCLEO_H753ZI_MFRC522_IRQ_PIN
#    define MFRC522_IRQ_ENABLED      true
#  else
#    define MFRC522_IRQ_ENABLED      false
#  endif

#endif /* CONFIG_NUCLEO_H753ZI_MFRC522_ENABLE */

/* LPS22HB - I2C Pressure Sensor */

#ifdef CONFIG_SENSORS_LPS22HB
#  define GPIO_LPS22HB_INT1    (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTB | \
                                GPIO_PIN10)
#endif

/* LSM6DSL - I2C 6-axis IMU */

#ifdef CONFIG_SENSORS_LSM6DSL
#  define GPIO_LSM6DSL_INT1    (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTB | \
                                GPIO_PIN4)
#  define GPIO_LSM6DSL_INT2    (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTB | \
                                GPIO_PIN5)
#endif

/* LSM303AGR - I2C Magnetometer/Accelerometer */

/* TODO: Add LSM303AGR GPIO and I2C configurations when needed */

/* LSM9DS1 - I2C 9-axis IMU */

#ifdef CONFIG_SENSORS_LSM9DS1
#  define LMS9DS1_I2CBUS 1
#endif

/* TODO: Add new sensors here following this pattern:
 * - Validate required Kconfig settings
 * - Define GPIO pins (CS for SPI, INT pins, etc)
 * - Define bus configuration (SPI/I2C bus number)
 * - Add function prototype in SECTION 5
 * - Implement driver in src/stm32_yoursensor.c
 */

/****************************************************************************
 * Displays
 ****************************************************************************/

/* ST7796 - SPI LCD Display (480x320) */

#ifdef CONFIG_NUCLEO_H753ZI_ST7796_ENABLE

/* Validate Kconfig */

#  ifndef CONFIG_NUCLEO_H753ZI_ST7796_SPI_BUS
#    error "ST7796 enabled but SPI bus not configured"
#  endif

#  ifndef CONFIG_NUCLEO_H753ZI_ST7796_DEVID
#    error "ST7796 enabled but device ID not configured"
#  endif

#  ifndef CONFIG_NUCLEO_H753ZI_ST7796_CS_PIN
#    error "ST7796 enabled but CS pin not configured"
#  endif

#  ifndef CONFIG_NUCLEO_H753ZI_ST7796_DC_PIN
#    error "ST7796 enabled but DC pin not configured"
#  endif

#  ifndef CONFIG_NUCLEO_H753ZI_ST7796_RESET_PIN
#    error "ST7796 enabled but RESET pin not configured"
#  endif

#  ifndef CONFIG_NUCLEO_H753ZI_ST7796_LED_PIN
#    error "ST7796 enabled but LED pin not configured"
#  endif

/* Device configuration (from Kconfig) */

#  define ST7796_SPI_BUS           CONFIG_NUCLEO_H753ZI_ST7796_SPI_BUS
#  define ST7796_DEVICE_ID         CONFIG_NUCLEO_H753ZI_ST7796_DEVID
#  define ST7796_CS_PIN            CONFIG_NUCLEO_H753ZI_ST7796_CS_PIN
#  define ST7796_DC_PIN            CONFIG_NUCLEO_H753ZI_ST7796_DC_PIN
#  define ST7796_RESET_PIN         CONFIG_NUCLEO_H753ZI_ST7796_RESET_PIN
#  define ST7796_LED_PIN           CONFIG_NUCLEO_H753ZI_ST7796_LED_PIN

/* CS Active Level */

#  if defined(CONFIG_NUCLEO_H753ZI_ST7796_CS_ACTIVE_LOW)
#    define ST7796_CS_ACTIVE_LOW   true
#  elif defined(CONFIG_NUCLEO_H753ZI_ST7796_CS_ACTIVE_HIGH)
#    define ST7796_CS_ACTIVE_LOW   false
#  else
     /* Default to active low if neither is explicitly set */
#    define ST7796_CS_ACTIVE_LOW   true
#  endif

#endif /* CONFIG_NUCLEO_H753ZI_ST7796_ENABLE */

/* SSD1306 - I2C OLED Display (128x64/128x32) */

#ifdef CONFIG_NUCLEO_H753ZI_SSD1306_ENABLE

/* Hardware configuration from Kconfig */

#  define NUCLEO_SSD1306_I2C_BUS       CONFIG_NUCLEO_H753ZI_SSD1306_I2C_BUS
#  define NUCLEO_SSD1306_I2C_ADDR      CONFIG_NUCLEO_H753ZI_SSD1306_I2C_ADDR
#  define NUCLEO_SSD1306_I2C_FREQUENCY \
          CONFIG_NUCLEO_H753ZI_SSD1306_I2C_FREQUENCY
#  define NUCLEO_SSD1306_POWER_PERCENT \
          CONFIG_NUCLEO_H753ZI_SSD1306_POWER_PERCENT

/* System configuration - from Kconfig */

#  define NUCLEO_SSD1306_DEVPATH       CONFIG_NUCLEO_H753ZI_SSD1306_DEVPATH
#  define NUCLEO_SSD1306_DEVNO         CONFIG_NUCLEO_H753ZI_SSD1306_DEVNO

/* Device name for internal tracking */

#  define NUCLEO_SSD1306_DEVNAME       "ssd1306"

#endif /* CONFIG_NUCLEO_H753ZI_SSD1306_ENABLE */

/* TODO: Add new displays here following ST7796/SSD1306 pattern:
 * - Validate required Kconfig settings
 * - Define GPIO pins (CS, DC, RESET, etc)
 * - Define bus configuration (SPI/I2C)
 * - Add function prototypes in SECTION 5
 * - Implement driver in src/stm32_yourdisplay.c
 */

/****************************************************************************
 * Wireless Modules
 ****************************************************************************/

/* NRF24L01 - SPI 2.4GHz Transceiver */

#ifdef CONFIG_WL_NRF24L01
#  define GPIO_NRF24L01_CS     (GPIO_OUTPUT | GPIO_SPEED_50MHz | \
                                GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN4)
#  define GPIO_NRF24L01_CE     (GPIO_OUTPUT | GPIO_SPEED_50MHz | \
                                GPIO_OUTPUT_CLEAR | GPIO_PORTF | GPIO_PIN12)
#  define GPIO_NRF24L01_IRQ    (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTD | \
                                GPIO_PIN15)
#endif

/* TODO: Add new wireless modules here following NRF24L01 pattern:
 * - WiFi modules (ESP8266, ESP32-AT, etc)
 * - LoRa modules (SX1276, SX1278, etc)
 * - Bluetooth modules (HC-05, nRF52, etc)
 * - Validate Kconfig settings
 * - Define GPIO pins
 * - Add function prototypes in SECTION 5
 * - Implement driver in src/stm32_yourwireless.c
 */

/****************************************************************************
 * Storage Devices
 ****************************************************************************/

/* MMCSD - SPI SD Card */

#ifdef CONFIG_MMCSD_SPI
#  define GPIO_MMCSD_CS        (GPIO_OUTPUT | GPIO_PUSHPULL | \
                                GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | \
                                GPIO_PORTD | GPIO_PIN15)
#  define GPIO_MMCSD_NCD       (GPIO_INPUT | GPIO_PULLUP | GPIO_EXTI | \
                                GPIO_PORTF | GPIO_PIN12)
#endif

/* TODO: Add new storage devices here following MMCSD pattern:
 * - QSPI Flash (W25Q, MX25, etc)
 * - I2C EEPROM (AT24C, M24C, etc)
 * - SPI Flash (AT25, SST25, etc)
 * - Validate Kconfig settings
 * - Define GPIO pins (CS, WP, HOLD, etc)
 * - Add function prototypes in SECTION 5
 * - Implement driver in src/stm32_yourstorage.c
 */

/****************************************************************************
 * Actuators & LED Controllers
 ****************************************************************************/

/* PCA9635 - I2C LED Controller */

#ifdef CONFIG_PCA9635PW
#  define PCA9635_I2CBUS       1
#  define PCA9635_I2CADDR      0x40
#endif

/* PWM Configuration */

#define NUCLEOH753ZI_PWMTIMER 1

/* TODO: Add new actuators here:
 * - Servo controllers (PCA9685, etc)
 * - Motor drivers (DRV8833, L298N, etc)
 * - Relay modules
 * - Define GPIO pins
 * - Define bus configuration
 * - Add function prototypes in SECTION 5
 * - Implement driver in src/stm32_youractuator.c
 */

/****************************************************************************
 * SECTION 5: FUNCTION PROTOTYPES
 ****************************************************************************/

/****************************************************************************
 * Core System Initialization
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 * Perform architecture-specific initialization
 *
 * CONFIG_BOARD_LATE_INITIALIZE=y :
 * Called from board_late_initialize().
 *
 * CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y &&
 * CONFIG_NSH_ARCHINIT:
 * Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Communication Bus Drivers - SPI
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spi_initialize
 *
 * Description:
 *   Initialize SPI interfaces and CS pins.
 *
 * Returned Value:
 *   OK on success, negative errno on error
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_SPI
int stm32_spi_initialize(void);

/****************************************************************************
 * Name: stm32_spi_register_cs_device
 *
 * Description:
 *   Register a CS device for a specific SPI bus and device ID.
 *
 * Input Parameters:
 *   spi_bus     - SPI bus number (1-6)
 *   devid       - Device ID (0-15)
 *   cs_pin      - CS pin string (e.g., "PF1")
 *   active_low  - true if CS is active low, false if active high
 *
 * Returned Value:
 *   OK on success, negative errno on error
 *
 ****************************************************************************/

int stm32_spi_register_cs_device(int spi_bus, uint32_t devid,
                                  const char *cs_pin, bool active_low);

/****************************************************************************
 * Name: stm32_spi_unregister_cs_device
 *
 * Description:
 *   Unregister a CS device.
 *
 * Input Parameters:
 *   spi_bus - SPI bus number (1-6)
 *   devid   - Device ID
 *
 * Returned Value:
 *   OK on success, negative errno on error
 *
 ****************************************************************************/

int stm32_spi_unregister_cs_device(int spi_bus, uint32_t devid);

#ifdef CONFIG_SPI_CMDDATA

/****************************************************************************
 * Name: stm32_spi_register_dc_pin
 *
 * Description:
 *   Register CMD/DATA pin for SPI devices (e.g., displays).
 *
 * Input Parameters:
 *   spi_bus - SPI bus number (1-6)
 *   devid   - Device ID
 *   dc_pin  - DC pin string (e.g., "PF2")
 *
 * Returned Value:
 *   OK on success, negative errno on error
 *
 ****************************************************************************/

int stm32_spi_register_dc_pin(int spi_bus, uint32_t devid,
                               const char *dc_pin);

#endif /* CONFIG_SPI_CMDDATA */
#endif /* CONFIG_STM32H7_SPI */

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Nucleo-H753ZI
 *   board.
 *
 ****************************************************************************/

#if defined(CONFIG_STM32H7_SPI1) || defined(CONFIG_STM32H7_SPI2) || \
    defined(CONFIG_STM32H7_SPI3) || defined(CONFIG_STM32H7_SPI4) || \
    defined(CONFIG_STM32H7_SPI5) || defined(CONFIG_STM32H7_SPI6)
void weak_function stm32_spidev_initialize(void);
#endif

#ifdef CONFIG_SPI_DRIVER

/****************************************************************************
 * Name: stm32_spidev_register_all
 *
 * Description:
 *   Register all SPI devices for userspace access.
 *
 * Returned Value:
 *   OK on success, negative errno on error
 *
 ****************************************************************************/

int stm32_spidev_register_all(void);

#endif

/****************************************************************************
 * Communication Bus Drivers - I2C
 ****************************************************************************/

#ifdef CONFIG_STM32H7_I2C

/****************************************************************************
 * Name: stm32_i2c_initialize
 *
 * Description:
 *   Initialize I2C buses based on Kconfig configuration.
 *
 * Returned Value:
 *   OK on success, negative errno on error
 *
 ****************************************************************************/

int stm32_i2c_initialize(void);

/****************************************************************************
 * Name: stm32_i2c_register_device
 *
 * Description:
 *   Register an I2C device with specific address, frequency, and name.
 *
 * Input Parameters:
 *   i2c_bus   - I2C bus number (1-4)
 *   addr      - I2C slave address (7-bit, 0x08-0x77)
 *   frequency - Bus frequency for this device (Hz)
 *   name      - Descriptive name for logging (can be NULL)
 *
 * Returned Value:
 *   OK on success, negative errno on error
 *
 ****************************************************************************/

int stm32_i2c_register_device(int i2c_bus, uint8_t addr,
                               uint32_t frequency, const char *name);

/****************************************************************************
 * Name: stm32_i2c_unregister_device
 *
 * Description:
 *   Unregister an I2C device by address.
 *
 * Input Parameters:
 *   i2c_bus - I2C bus number (1-4)
 *   addr    - I2C slave address to unregister
 *
 * Returned Value:
 *   OK on success, negative errno on error
 *
 ****************************************************************************/

int stm32_i2c_unregister_device(int i2c_bus, uint8_t addr);

/****************************************************************************
 * Name: stm32_i2c_get_master
 *
 * Description:
 *   Get I2C master interface for a specific bus.
 *
 * Input Parameters:
 *   i2c_bus - I2C bus number (1-4)
 *
 * Returned Value:
 *   Pointer to I2C master interface, NULL if invalid or not initialized
 *
 ****************************************************************************/

struct i2c_master_s *stm32_i2c_get_master(int i2c_bus);

#ifdef CONFIG_I2C_RESET

/****************************************************************************
 * Name: stm32_i2c_scan_bus
 *
 * Description:
 *   Scan an I2C bus for connected devices (debugging).
 *
 * Input Parameters:
 *   i2c_bus - I2C bus number (1-4)
 *
 * Returned Value:
 *   Number of devices found, negative errno on error
 *
 ****************************************************************************/

int stm32_i2c_scan_bus(int i2c_bus);

#endif

/****************************************************************************
 * Name: stm32_i2c_list_devices
 *
 * Description:
 *   List all registered I2C devices on a specific bus (debugging).
 *
 * Input Parameters:
 *   i2c_bus - I2C bus number (1-4), or 0 for all buses
 *
 * Returned Value:
 *   Number of registered devices
 *
 ****************************************************************************/

int stm32_i2c_list_devices(int i2c_bus);

#endif /* CONFIG_STM32H7_I2C */

/****************************************************************************
 * CAN/FDCAN Drivers
 ****************************************************************************/

#ifdef CONFIG_STM32H7_FDCAN

/****************************************************************************
 * Name: stm32_fdcansockinitialize
 *
 * Description:
 *   Initialize FDCAN socket interface.
 *
 * Input Parameters:
 *   intf - Interface number (0 for FDCAN1/can0, 1 for FDCAN2/can1)
 *
 * Returned Value:
 *   OK on success, negative errno on error
 *
 ****************************************************************************/

int stm32_fdcansockinitialize(int intf);

#endif

/****************************************************************************
 * USB Drivers
 ****************************************************************************/

#ifdef CONFIG_STM32H7_OTGFS
void weak_function stm32_usbinitialize(void);
#endif

#if defined(CONFIG_STM32H7_OTGFS) && defined(CONFIG_USBHOST)

/****************************************************************************
 * Name: stm32_usbhost_initialize
 *
 * Description:
 *   Initialize USB host controller.
 *
 * Returned Value:
 *   OK on success, negative errno on error
 *
 ****************************************************************************/

int stm32_usbhost_initialize(void);

#endif

/****************************************************************************
 * Sensor Drivers
 ****************************************************************************/

#ifdef CONFIG_NUCLEO_H753ZI_MFRC522_ENABLE

/****************************************************************************
 * Name: stm32_mfrc522initialize
 *
 * Description:
 *   Initialize MFRC522 RFID reader.
 *
 * Input Parameters:
 *   devpath - Device path (e.g., "/dev/rfid0")
 *
 * Returned Value:
 *   OK on success, negative errno on error
 *
 ****************************************************************************/

int stm32_mfrc522initialize(const char *devpath);

#endif

#ifdef CONFIG_SENSORS_LSM6DSL

/****************************************************************************
 * Name: stm32_lsm6dsl_initialize
 *
 * Description:
 *   Initialize LSM6DSL 6-axis IMU sensor.
 *
 * Input Parameters:
 *   devpath - Device path (e.g., "/dev/lsm6dsl0")
 *
 * Returned Value:
 *   OK on success, negative errno on error
 *
 ****************************************************************************/

int stm32_lsm6dsl_initialize(char *devpath);

#endif

#ifdef CONFIG_SENSORS_LSM303AGR

/****************************************************************************
 * Name: stm32_lsm303agr_initialize
 *
 * Description:
 *   Initialize LSM303AGR magnetometer/accelerometer.
 *
 * Input Parameters:
 *   devpath - Device path
 *
 * Returned Value:
 *   OK on success, negative errno on error
 *
 ****************************************************************************/

int stm32_lsm303agr_initialize(char *devpath);

#endif

#ifdef CONFIG_SENSORS_LSM9DS1

/****************************************************************************
 * Name: stm32_lsm9ds1_initialize
 *
 * Description:
 *   Initialize LSM9DS1 9-axis IMU sensor.
 *
 * Input Parameters:
 *   devpath - Device path
 *
 * Returned Value:
 *   OK on success, negative errno on error
 *
 ****************************************************************************/

int stm32_lsm9ds1_initialize(char *devpath);

#endif

/****************************************************************************
 * Display Drivers
 ****************************************************************************/

/****************************************************************************
 * ST7796 TFT Display Support
 ****************************************************************************/

#if defined(CONFIG_LCD_ST7796) && defined(CONFIG_NUCLEO_H753ZI_ST7796_ENABLE)

/****************************************************************************
 * Name: stm32_st7796initialize
 *
 * Description:
 *   Initialize and register the ST7796 LCD framebuffer driver.
 *
 * Input Parameters:
 *   devno - Device number (0 for /dev/fb0)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_st7796initialize(int devno);

/****************************************************************************
 * Name: stm32_st7796_flush_fb
 *
 * Description:
 *   Flush the entire framebuffer to the display. This is needed for SPI
 *   displays to make the splashscreen visible after fb_register().
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_st7796_flush_fb(void);

/****************************************************************************
 * Name: stm32_st7796_backlight
 *
 * Description:
 *   Control the ST7796 backlight LED.
 *
 * Input Parameters:
 *   on - true to turn on, false to turn off
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_st7796_backlight(bool on);

/****************************************************************************
 * Name: stm32_st7796_power
 *
 * Description:
 *   Control the ST7796 display power.
 *
 * Input Parameters:
 *   on - true to turn on, false to turn off
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_st7796_power(bool on);

/****************************************************************************
 * Name: stm32_st7796_reset_display
 *
 * Description:
 *   Perform hardware reset of the ST7796 display.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_st7796_reset_display(void);

/****************************************************************************
 * Name: stm32_st7796_cleanup
 *
 * Description:
 *   Cleanup ST7796 resources.
 *
 * Returned Value:
 *   OK on success, negative errno on error
 *
 ****************************************************************************/

int stm32_st7796_cleanup(void);

#endif /* CONFIG_LCD_ST7796 && CONFIG_NUCLEO_H753ZI_ST7796_ENABLE */

#ifdef CONFIG_NUCLEO_H753ZI_SSD1306_ENABLE
/****************************************************************************
 * Name: stm32_ssd1306_get_devpath
 *
 * Description:
 *   Get the configured device path for SSD1306.
 *
 * Returned Value:
 *   Pointer to device path string
 *
 ****************************************************************************/

const char *stm32_ssd1306_get_devpath(void);

/****************************************************************************
 * Name: stm32_ssd1306_set_power
 *
 * Description:
 *   Change SSD1306 display power at runtime.
 *
 * Input Parameters:
 *   percent - Power level 0-100% (0 = off, 1-100 = on with brightness)
 *
 * Returned Value:
 *   OK on success, negative errno on error
 *
 ****************************************************************************/

int stm32_ssd1306_set_power(int percent);

/****************************************************************************
 * Name: stm32_ssd1306_set_brightness
 *
 * Description:
 *   Change SSD1306 display brightness at runtime (display must be on).
 *
 * Input Parameters:
 *   percent - Brightness level 0-100%
 *
 * Returned Value:
 *   OK on success, negative errno on error
 *
 ****************************************************************************/

int stm32_ssd1306_set_brightness(int percent);

#endif /* CONFIG_NUCLEO_H753ZI_SSD1306_ENABLE */

/****************************************************************************
 * Wireless Drivers
 ****************************************************************************/

#ifdef CONFIG_WL_NRF24L01

/****************************************************************************
 * Name: stm32_wlinitialize
 *
 * Description:
 *   Initialize NRF24L01 wireless module.
 *
 * Returned Value:
 *   OK on success, negative errno on error
 *
 ****************************************************************************/

int stm32_wlinitialize(void);

#endif

/****************************************************************************
 * Storage Drivers
 ****************************************************************************/

#ifdef CONFIG_MMCSD_SPI

/****************************************************************************
 * Name: stm32_mmcsd_initialize
 *
 * Description:
 *   Initialize MMC/SD card over SPI.
 *
 * Input Parameters:
 *   minor - Device minor number
 *
 * Returned Value:
 *   OK on success, negative errno on error
 *
 ****************************************************************************/

int stm32_mmcsd_initialize(int minor);

#endif

#ifdef CONFIG_MTD
#ifdef HAVE_PROGMEM_CHARDEV

/****************************************************************************
 * Name: stm32_progmem_init
 *
 * Description:
 *   Initialize internal flash as MTD device.
 *
 * Returned Value:
 *   OK on success, negative errno on error
 *
 ****************************************************************************/

int stm32_progmem_init(void);

#endif /* HAVE_PROGMEM_CHARDEV */
#endif /* CONFIG_MTD */

/****************************************************************************
 * Actuator & LED Controller Drivers
 ****************************************************************************/

#ifdef CONFIG_PCA9635PW

/****************************************************************************
 * Name: stm32_pca9635_initialize
 *
 * Description:
 *   Initialize PCA9635 I2C LED controller.
 *
 * Returned Value:
 *   OK on success, negative errno on error
 *
 ****************************************************************************/

int stm32_pca9635_initialize(void);

#endif

#ifdef CONFIG_PWM

/****************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *   Initialize PWM outputs.
 *
 * Returned Value:
 *   OK on success, negative errno on error
 *
 ****************************************************************************/

int stm32_pwm_setup(void);

#endif

/****************************************************************************
 * GPIO & ADC Drivers
 ****************************************************************************/

#ifdef CONFIG_DEV_GPIO

/****************************************************************************
 * Name: stm32_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 * Returned Value:
 *   OK on success, negative errno on error
 *
 ****************************************************************************/

int stm32_gpio_initialize(void);

#endif

#ifdef CONFIG_ADC

/****************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 * Returned Value:
 *   OK on success, negative errno on error
 *
 ****************************************************************************/

int stm32_adc_setup(void);

#endif

#endif /* __BOARDS_ARM_STM32H7_NUCLEO_H753ZI_SRC_NUCLEO_H753ZI_H */
