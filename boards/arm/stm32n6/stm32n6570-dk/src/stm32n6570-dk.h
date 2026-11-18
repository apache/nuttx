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

#ifdef CONFIG_VIDEO_VL53L5CX
void stm32_camera_power(void);
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

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32N6_STM32N6570_DK_SRC_STM32N6570_DK_H */
