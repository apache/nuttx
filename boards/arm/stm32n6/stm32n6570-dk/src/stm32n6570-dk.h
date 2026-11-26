/****************************************************************************
 * boards/arm/stm32n6/stm32n6570-dk/src/stm32n6570-dk.h
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

#ifndef __BOARDS_ARM_STM32N6_STM32N6570_DK_SRC_STM32N6570_DK_H
#define __BOARDS_ARM_STM32N6_STM32N6570_DK_SRC_STM32N6570_DK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include "stm32_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LED definitions **********************************************************/

/* LD1 is active high; LD2 is active low.  Start with both LEDs off. */

#define GPIO_LD1       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHZ | \
                        GPIO_OUTPUT_CLEAR | GPIO_PORTO | GPIO_PIN1)
#define GPIO_LD2       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHZ | \
                        GPIO_OUTPUT_SET | GPIO_PORTG | GPIO_PIN10)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* USER1 (B2) on PC13 and TAMP (B4) on PE0, high when pressed */

#define GPIO_BTN_USER  (GPIO_INPUT | GPIO_PULLDOWN | GPIO_EXTI | \
                        GPIO_PORTC | GPIO_PIN13)
#define GPIO_BTN_TAMP  (GPIO_INPUT | GPIO_PULLDOWN | GPIO_EXTI | \
                        GPIO_PORTE | GPIO_PIN0)

/* SPI5 chip select on PA3 (Arduino D10), active low */

#define GPIO_SPI5_CS   (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHZ | \
                        GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN3)

/* LCD control: LCD_ON/OFF (PQ3) and backlight (PQ6) are active high,
 * LCD_NRST (PE1, shared with the GT911 touch controller) is active low and
 * starts asserted.
 */

#define GPIO_LCD_DISP  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHZ | \
                        GPIO_OUTPUT_CLEAR | GPIO_PORTQ | GPIO_PIN3)
#define GPIO_LCD_BL    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHZ | \
                        GPIO_OUTPUT_CLEAR | GPIO_PORTQ | GPIO_PIN6)
#define GPIO_LCD_NRST  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHZ | \
                        GPIO_OUTPUT_CLEAR | GPIO_PORTE | GPIO_PIN1)

/* GT911 touch controller on I2C2 (0x5d): interrupt on PQ4, rising edge.
 * Its reset is LCD_NRST (PE1).
 */

#define GPIO_TOUCH_INT (GPIO_INPUT | GPIO_PULLDOWN | GPIO_EXTI | \
                        GPIO_PORTQ | GPIO_PIN4)
#define TOUCH_I2C_BUS  2

/* Camera connector (MB1854 module): IMX335 NRST on PC8 (active low) and
 * module enable on PD2 (active high).  The module devices are on I2C1.
 */

#define GPIO_CAM_NRST  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHZ | \
                        GPIO_OUTPUT_CLEAR | GPIO_PORTC | GPIO_PIN8)
#define GPIO_CAM_EN    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHZ | \
                        GPIO_OUTPUT_CLEAR | GPIO_PORTD | GPIO_PIN2)
#define CAMERA_I2C_BUS 1

/* ISM330DLC IMU on the camera module, I2C1 (0x6a), polled.  It is register
 * compatible with the LSM6DS3TR-C and uses that driver.
 */

#define IMU_I2C_ADDR   0x6a

/* VL53L5CX time-of-flight sensor on the camera module, I2C1 (0x29):
 * INT on PQ0 (open drain, active low, EXTI0 like the TAMP button) and LPn
 * on PQ5 (low disables the I2C interface).
 */

#define GPIO_TOF_INT   (GPIO_INPUT | GPIO_PULLUP | GPIO_EXTI | \
                        GPIO_PORTQ | GPIO_PIN0)
#define GPIO_TOF_LPN   (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHZ | \
                        GPIO_OUTPUT_CLEAR | GPIO_PORTQ | GPIO_PIN5)

#define TOF_I2C_BUS    1
#define TOF_I2C_ADDR   0x29
#define TOF_DEVPATH    "/dev/video1"

/* microSD slot: card detect on PN12 (low when a card is inserted) and
 * card power enable on PQ7 (active high).
 */

#define GPIO_SDMMC2_NCD    (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | \
                            GPIO_PORTN | GPIO_PIN12)
#define GPIO_SDMMC2_PWR_EN (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHZ | \
                            GPIO_OUTPUT_CLEAR | GPIO_PORTQ | GPIO_PIN7)

#define SDIO_SLOTNO    0
#define SDIO_MINOR     0

/* USB1 Type-C connector (CN18): the TCPP03 port protection switches VBUS.
 * It is enabled by PA4 (active high) and set up over I2C2 (0x34).
 */

#define GPIO_TCPP03_EN (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHZ | \
                        GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN4)
#define TCPP03_I2C_BUS  2
#define TCPP03_I2C_ADDR 0x34

/* USB2 Type-A connector: VBUS power switch enabled by PB9 (active high) */

#define GPIO_USB2_PWR_EN (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHZ | \
                          GPIO_OUTPUT_CLEAR | GPIO_PORTB | GPIO_PIN9)

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
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

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Initialize the SPI buses and their chip select GPIOs.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_SPI
void stm32_spiinitialize(void);
#endif

/****************************************************************************
 * Name: stm32_spiregister
 *
 * Description:
 *   Register the SPI character drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_DRIVER
void stm32_spiregister(void);
#endif

/****************************************************************************
 * Name: stm32_camera_power
 *
 * Description:
 *   Power the camera module with the IMX335 held in reset.
 *
 ****************************************************************************/

#if defined(CONFIG_VIDEO_VL53L5CX) || defined(CONFIG_STM32_DCMIPP) || \
    defined(CONFIG_SENSORS_LSM6DS3TRC)
void stm32_camera_power(void);
#endif

/****************************************************************************
 * Name: stm32_imu_initialize
 *
 * Description:
 *   Power the camera module and register its IMU as
 *   /dev/uorb/sensor_accel0 and /dev/uorb/sensor_gyro0.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_LSM6DS3TRC
int stm32_imu_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_camera_initialize
 *
 * Description:
 *   Power up the camera module and register the IMX335 sensor, the
 *   DCMIPP capture interface and /dev/video.
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_DCMIPP) && defined(CONFIG_VIDEO_IMX335)
int stm32_camera_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_tof_initialize
 *
 * Description:
 *   Power the VL53L5CX and register it as a V4L2 depth camera.
 *
 ****************************************************************************/

#ifdef CONFIG_VIDEO_VL53L5CX
int stm32_tof_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_sdio_initialize
 *
 * Description:
 *   Initialize the microSD slot and register /dev/mmcsd0.
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_SDMMC2) && defined(CONFIG_MMCSD_SDIO)
int stm32_sdio_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_usbhost_initialize
 *
 * Description:
 *   Start the USB OTG HS host on the USB1 Type-C or the USB2 Type-A
 *   connector and wait for devices.
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_OTGHS) && defined(CONFIG_USBHOST)
int stm32_usbhost_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32N6_STM32N6570_DK_SRC_STM32N6570_DK_H */
