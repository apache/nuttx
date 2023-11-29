/****************************************************************************
 * boards/arm/stm32h7/h747ai/src/h747ai.h
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

#ifndef __BOARDS_ARM_STM32H7_H747AI_SRC_H747AI_H
#define __BOARDS_ARM_STM32H7_H747AI_SRC_H747AI_H

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
 */

#define GPIO_LD1       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | \
                        GPIO_OUTPUT_CLEAR | GPIO_PORTA | GPIO_PIN4)
#define GPIO_LD2       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | \
                        GPIO_OUTPUT_CLEAR | GPIO_PORTA | GPIO_PIN3)
#define GPIO_LD3       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | \
                        GPIO_OUTPUT_CLEAR | GPIO_PORTB | GPIO_PIN0)
#define GPIO_LD4       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | \
                        GPIO_OUTPUT_CLEAR | GPIO_PORTE | GPIO_PIN9)
#define GPIO_LD5       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | \
                        GPIO_OUTPUT_CLEAR | GPIO_PORTF | GPIO_PIN14)
#define GPIO_LD6       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | \
                        GPIO_OUTPUT_CLEAR | GPIO_PORTE | GPIO_PIN11)

#define GPIO_LED1_GREEN GPIO_LD1
#define GPIO_LED1_BLUE  GPIO_LD2
#define GPIO_LED1_RED   GPIO_LD3
#define GPIO_LED2_GREEN GPIO_LD4
#define GPIO_LED2_BLUE  GPIO_LD5
#define GPIO_LED2_RED   GPIO_LD6

#define LED_DRIVER_PATH "/dev/userleds"

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

#define BOARD_NGPIOIN     1 /* Amount of GPIO Input pins */
#define BOARD_NGPIOOUT    1 /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT    1 /* Amount of GPIO Input w/ Interruption pins */

/* Example, used free Ports on the board */

#define GPIO_IN1          (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTE | GPIO_PIN2)
#define GPIO_OUT1         (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                           GPIO_OUTPUT_SET | GPIO_PORTE | GPIO_PIN4)
#define GPIO_INT1         (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTE | GPIO_PIN5)

/* PWM */

#define H747AI_PWMTIMER 1

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
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

#endif /* __BOARDS_ARM_STM32H7_H747AI_SRC_H747AI_H */
