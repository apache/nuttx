/****************************************************************************
 * boards/arm/stm32/stm32f411e-disco/src/stm32f411e-disco.h
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

#ifndef __BOARDS_ARM_STM32_STM32F411E_DISCO_SRC_STM32F411E_DISCO_H
#define __BOARDS_ARM_STM32_STM32F411E_DISCO_SRC_STM32F411E_DISCO_H

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

/* LED.  User LD2: the green LED is a user LED connected to Arduino signal
 * D13 corresponding to MCU I/O PA5 (pin 21) or PB13 (pin 34) depending on
 * the STM32 target.
 *
 * - When the I/O is HIGH value, the LED is on.
 * - When the I/O is LOW, the LED is off.
 */

#define GPIO_LD2 \
  (GPIO_PORTD | GPIO_PIN12 | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_PULLUP | \
   GPIO_SPEED_50MHz)

/* Buttons
 *
 * B1 USER: the user button is connected to the I/O PA0 of the STM32
 * microcontroller.
 */

#define MIN_IRQBUTTON   BUTTON_USER
#define MAX_IRQBUTTON   BUTTON_USER
#define NUM_IRQBUTTONS  1

#define GPIO_BTN_USER \
  (GPIO_INPUT |GPIO_FLOAT |GPIO_EXTI | GPIO_PORTA | GPIO_PIN0)

/* SPI1 off */

#define GPIO_SPI1_MOSI_OFF (GPIO_INPUT | GPIO_PULLDOWN | \
                            GPIO_PORTA | GPIO_PIN7)
#define GPIO_SPI1_MISO_OFF (GPIO_INPUT | GPIO_PULLDOWN | \
                            GPIO_PORTA | GPIO_PIN6)
#define GPIO_SPI1_SCK_OFF  (GPIO_INPUT | GPIO_PULLDOWN | \
                            GPIO_PORTA | GPIO_PIN5)

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing (also connected to the green LED)
 * PC0  OTG_FS_PowerSwitchOn
 * PD5  OTG_FS_Overcurrent
 */

#define GPIO_OTGFS_VBUS   (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|\
                           GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)
#define GPIO_OTGFS_PWRON  (GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|\
                           GPIO_PUSHPULL|GPIO_PORTC|GPIO_PIN0)

#ifdef CONFIG_USBHOST
#  define GPIO_OTGFS_OVER (GPIO_INPUT|GPIO_EXTI|GPIO_FLOAT|\
                           GPIO_SPEED_100MHz|GPIO_PUSHPULL|\
                           GPIO_PORTD|GPIO_PIN5)

#else
#  define GPIO_OTGFS_OVER (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|\
                           GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN5)
#endif

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define STM32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Global driver instances */

#ifdef CONFIG_STM32_SPI1
extern struct spi_dev_s *g_spi1;
#endif
#ifdef CONFIG_STM32_SPI2
extern struct spi_dev_s *g_spi2;
#endif

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins.
 *
 ****************************************************************************/

void stm32_spidev_initialize(void);

/****************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called from stm32_boardinitialize very early in initialization to setup
 *   USB-related GPIO pins for the STM32F4Discovery board.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_OTGFS
void stm32_usbinitialize(void);
#endif

/****************************************************************************
 * Name: stm32_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host
 *   functionality.  This function will start a thread that will monitor for
 *   device connection/disconnection events.
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_OTGFS) && defined(CONFIG_USBHOST)
int stm32_usbhost_initialize(void);
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
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

#endif /* __BOARDS_ARM_STM32_STM32F411E_DISCO_SRC_STM32F411E_DISCO_H */
