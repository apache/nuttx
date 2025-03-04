/****************************************************************************
 * boards/arm/stm32h7/nucleo-h745zi/src/nucleo-h745zi.h
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

#ifndef __BOARDS_ARM_STM32H7_NUCLEO_H745ZI_SRC_NUCLEO_H745ZI_H
#define __BOARDS_ARM_STM32H7_NUCLEO_H745ZI_SRC_NUCLEO_H745ZI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define HAVE_PROC            1
#define HAVE_USBDEV          1
#define HAVE_USBHOST         1
#define HAVE_USBMONITOR      1
#define HAVE_MTDCONFIG       1
#define HAVE_PROGMEM_CHARDEV 1

/* Can't support USB host or device features if USB OTG FS is not enabled */

#ifndef CONFIG_STM32H7_OTGFS
#  undef HAVE_USBDEV
#  undef HAVE_USBHOST
#endif

/* Can't support USB device if USB device is not enabled */

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

#if !defined(CONFIG_STM32H7_PROGMEM) || !defined(CONFIG_MTD_PROGMEM)
#  undef HAVE_PROGMEM_CHARDEV
#endif

/* This is the on-chip progmem memory driver minor number */

#define PROGMEM_MTD_MINOR 0

/* flash  */
#if defined(CONFIG_MMCSD)
#  define FLASH_BASED_PARAMS
#endif

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define STM32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* Check if we can support the RTC driver */

#define HAVE_RTC_DRIVER 1
#if !defined(CONFIG_RTC) || !defined(CONFIG_RTC_DRIVER)
#  undef HAVE_RTC_DRIVER
#endif

/* LED
 *
 * The Nucleo-144 board has numerous LEDs but only three, LD1 a Green LED,
 * LD2 a Blue LED and LD3 a Red LED, that can be controlled by software. The
 * following definitions assume the default Solder Bridges are installed.
 */

#define GPIO_LD1       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                        GPIO_OUTPUT_CLEAR | GPIO_PORTB | GPIO_PIN0)
#define GPIO_LD2       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                        GPIO_OUTPUT_CLEAR | GPIO_PORTB | GPIO_PIN7)
#define GPIO_LD3       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                        GPIO_OUTPUT_CLEAR | GPIO_PORTB | GPIO_PIN14)

#define GPIO_LED_GREEN GPIO_LD1
#define GPIO_LED_BLUE  GPIO_LD2
#define GPIO_LED_RED   GPIO_LD3

#define LED_DRIVER_PATH "/dev/userleds"

/* BUTTONS
 *
 * The Blue pushbutton B1, labeled "User", is connected to GPIO PC13.
 * A high value will be sensed when the button is depressed.
 * Note:
 *    1) That the EXTI is included in the definition to enable an interrupt
 *       on this IO.
 *    2) The following definitions assume the default Solder Bridges are
 *       installed.
 */

#define GPIO_BTN_USER  (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | GPIO_PORTC | GPIO_PIN13)

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing (also connected to the green LED)
 * PG6  OTG_FS_PowerSwitchOn
 * PG7  OTG_FS_Overcurrent
 */

#define GPIO_OTGFS_VBUS   (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz| \
                           GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)

# define GPIO_OTGFS_PWRON  (GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|  \
                           GPIO_PUSHPULL|GPIO_PORTG|GPIO_PIN6)

#ifdef CONFIG_USBHOST
#  define GPIO_OTGFS_OVER (GPIO_INPUT|GPIO_EXTI|GPIO_FLOAT| \
                           GPIO_SPEED_100MHz|GPIO_PUSHPULL| \
                           GPIO_PORTG|GPIO_PIN7)
#else
#  define GPIO_OTGFS_OVER (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz| \
                           GPIO_PUSHPULL|GPIO_PORTG|GPIO_PIN7)
#endif

/* GPIO pins used by the GPIO Subsystem */

#define BOARD_NGPIOIN     4 /* Amount of GPIO Input pins */
#define BOARD_NGPIOOUT    8 /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT    1 /* Amount of GPIO Input w/ Interruption pins */

/* Example, used free Ports on the board */

#define GPIO_IN1          (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTE | GPIO_PIN7)
#define GPIO_IN2          (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTE | GPIO_PIN12)
#define GPIO_IN3          (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTE | GPIO_PIN14)
#define GPIO_IN4          (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTE | GPIO_PIN15)

#define GPIO_OUT1         (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                           GPIO_OUTPUT_SET | GPIO_PORTE | GPIO_PIN4)
#define GPIO_OUT2         (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                           GPIO_OUTPUT_SET | GPIO_PORTE | GPIO_PIN5)
#define GPIO_OUT3         (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                           GPIO_OUTPUT_SET | GPIO_PORTE | GPIO_PIN6)
#define GPIO_OUT4         (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                           GPIO_OUTPUT_SET | GPIO_PORTA| GPIO_PIN5)
#define GPIO_OUT5         (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                           GPIO_OUTPUT_SET | GPIO_PORTF | GPIO_PIN3)

#define GPIO_INT1         (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTE | GPIO_PIN5)

/* X-NUCLEO IKS01A2 */

#define GPIO_LPS22HB_INT1 (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN10)
#define GPIO_LSM6DSL_INT1 (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN4)
#define GPIO_LSM6DSL_INT2 (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN5)

/* NRF24L01
 * CS  - PA4
 * CE  - PF12 (D8)
 * IRQ - PD15 (D9)
 */

#define GPIO_NRF24L01_CS   (GPIO_OUTPUT | GPIO_SPEED_50MHz| \
                            GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN4)
#define GPIO_NRF24L01_CE   (GPIO_OUTPUT | GPIO_SPEED_50MHz| \
                            GPIO_OUTPUT_CLEAR | GPIO_PORTF | GPIO_PIN12)
#define GPIO_NRF24L01_IRQ  (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTD | GPIO_PIN15)

/* LMS9DS1 configuration */

#define LMS9DS1_I2CBUS 1

/* PCA9635 configuration */

#define PCA9635_I2CBUS  1
#define PCA9635_I2CADDR 0x40

/* Oled configuration */

#define OLED_I2C_PORT   2

/* PWM */

#if defined(CONFIG_STM32H7_TIM1_PWM)
#define NUCLEOH745ZI_PWMTIMER 1
#else
#define NUCLEOH745ZI_PWMTIMER 3
#endif

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
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y &&
 *   CONFIG_NSH_ARCHINIT:
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

#ifdef CONFIG_STM32H7_OTGFS
void weak_function stm32_usbinitialize(void);
#endif

/****************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

#ifdef CONFIG_ADC
int stm32_adc_setup(int adcno);
#endif

/****************************************************************************
 * Name: stm32_gpio_initialize
 *
 * Description:
 *   Initialize GPIO-Driver.
 *
 ****************************************************************************/

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)
int stm32_gpio_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host
 *   functionality. This function will start a thread that will monitor for
 *   device connection/disconnection events.
 *
 ****************************************************************************/

#if defined(CONFIG_STM32H7_OTGFS) && defined(CONFIG_USBHOST)
int stm32_usbhost_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

#ifdef CONFIG_PWM
int stm32_pwm_setup(void);
#endif

/****************************************************************************
 * Name: stm32_qencoder_initialize
 *
 * Description:
 *   Initialize and register a qencoder
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_QENCODER
int stm32_qencoder_initialize(const char *devpath, int timer);
#endif

#endif /* __BOARDS_ARM_STM32H7_NUCLEO_H745ZI_SRC_NUCLEO_H745ZI_H */
