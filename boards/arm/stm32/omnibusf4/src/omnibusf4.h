/****************************************************************************
 * boards/arm/stm32/omnibusf4/src/omnibusf4.h
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

#ifndef __BOARDS_ARM_STM32_OMNIBUSF4_SRC_OMNIBUSF4_H
#define __BOARDS_ARM_STM32_OMNIBUSF4_SRC_OMNIBUSF4_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <arch/stm32/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define STM32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* OMNIBUSF4 GPIOs **********************************************************/

#define SPIPORT_MPU6000   1
#define SPIMINOR_MPU6000  0
#define GPIO_CS_MPU6000   (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz |  \
                           GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN4)
#define GPIO_EXTI_MPU6000 (GPIO_INPUT | GPIO_FLOAT | GPIO_SPEED_50MHz | \
                           GPIO_OPENDRAIN | GPIO_PORTC | GPIO_PIN4)
#define DEVNODE_MPU6000   "/dev/imu0"

#define SPIPORT_MAX7456   3
#define SPIMINOR_MAX7456  0
#define GPIO_CS_MAX7456   (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz |  \
                           GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN15)
#define DEVNODE_MAX7456   "/dev/osd0"

/* USB OTG FS ***************************************************************/

/* PC5  OTG_FS_VBUS VBUS sensing */

#define GPIO_OTGFS_VBUS   (GPIO_INPUT | GPIO_FLOAT | GPIO_SPEED_100MHz |\
                           GPIO_OPENDRAIN | GPIO_PORTC | GPIO_PIN5)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_INITIALIZE=y :
 *     Called from board_initialize().
 *
 *   CONFIG_BOARD_INITIALIZE=y && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the omnibusf4
 *   board.
 *
 ****************************************************************************/

void weak_function stm32_spidev_initialize(void);

/****************************************************************************
 * Name: stm32_mmcsdinitialize
 *
 * Description:
 *   Sets up MMC/SD interface.
 *
 ****************************************************************************/

int stm32_mmcsd_initialize(int port, int minor);

/****************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called from stm32_usbinitialize very early in initialization to setup
 *   USB-related GPIO pins for the OMNIBUSF4 board.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_OTGFS
void weak_function stm32_usbinitialize(void);
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

#if defined(CONFIG_STM32_OTGFS) && defined(CONFIG_USBHOST)
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
int weak_function stm32_pwm_setup(void);
#endif

/****************************************************************************
 * Name: stm32_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_CAN_CHARDRIVER
int stm32_can_setup(void);
#endif

/****************************************************************************
 * Name: stm32_timer_driver_setup
 *
 * Description:
 *   Configure the timer driver.
 *
 * Input Parameters:
 *   devpath - The full path to the timer device.  This should be of the
 *             form /dev/timer0
 *   timer   - The timer's number.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_TIMER
int stm32_timer_driver_setup(const char *devpath, int timer);
#endif

/****************************************************************************
 * Name: stm32_mpu6000_initialize
 *
 * Description:
 *  Initialize the MPU6000 device.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_MPU60X0
int stm32_mpu6000_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_max7456_initialize
 *
 * Description:
 *  Initialize the MAX7456 OSD device.
 *
 ****************************************************************************/

#ifdef CONFIG_VIDEO_MAX7456
int stm32_max7456_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32_OMNIBUSF4_SRC_OMNIBUSF4_H */
