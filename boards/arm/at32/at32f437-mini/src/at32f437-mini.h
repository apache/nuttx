/****************************************************************************
 * boards/arm/at32/at32f437-mini/src/at32f437-mini.h
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

#ifndef __BOARDS_ARM_AT32_AT32F437_MINI_SRC_AT32F437_MINI_H
#define __BOARDS_ARM_AT32_AT32F437_MINI_SRC_AT32F437_MINI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <arch/at32/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Assume that we have everything */

#define HAVE_USBDEV     1
#define HAVE_USBHOST    1
#define HAVE_USBMONITOR 0
#define HAVE_SDIO       1
#define HAVE_RTC_DRIVER 1
#define HAVE_W25        1

/* Can't support USB host or device features if USB OTG FS is not enabled */

#ifndef CONFIG_AT32_OTGFS
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

/* Can't support MMC/SD features if mountpoints are disabled or if SDIO
 * support is not enabled.
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_AT32_SDIO)
#  undef HAVE_SDIO
#endif

#undef  SDIO_MINOR     /* Any minor number, default 0 */
#define SDIO_SLOTNO 0  /* Only one slot */

#ifdef HAVE_SDIO
#  if !defined(CONFIG_NSH_MMCSDSLOTNO)
#    define CONFIG_NSH_MMCSDSLOTNO SDIO_SLOTNO
#  elif CONFIG_NSH_MMCSDSLOTNO != 0
#    warning "Only one MMC/SD slot, slot 0"
#    undef CONFIG_NSH_MMCSDSLOTNO
#    define CONFIG_NSH_MMCSDSLOTNO SDIO_SLOTNO
#  endif

#  if defined(CONFIG_NSH_MMCSDMINOR)
#    define SDIO_MINOR CONFIG_NSH_MMCSDMINOR
#  else
#    define SDIO_MINOR 0
#  endif
#endif

/* Check if we can support the RTC driver */

#if !defined(CONFIG_RTC) || !defined(CONFIG_RTC_DRIVER)
#  undef HAVE_RTC_DRIVER
#endif

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define AT32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define AT32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* Check if we have the prerequisites for an HCI UART */

#if !defined(CONFIG_AT32_HCIUART) || !defined(CONFIG_BLUETOOTH_UART)
#  undef HAVE_HCIUART
#elif defined(CONFIG_AT32_USART1_HCIUART)
#  define HCIUART_SERDEV HCIUART1
#elif defined(CONFIG_AT32_USART2_HCIUART)
#  define HCIUART_SERDEV HCIUART2
#elif defined(CONFIG_AT32_USART3_HCIUART)
#  define HCIUART_SERDEV HCIUART3
#elif defined(CONFIG_AT32_USART6_HCIUART)
#  define HCIUART_SERDEV HCIUART6
#elif defined(CONFIG_AT32_UART7_HCIUART)
#  define HCIUART_SERDEV HCIUART7
#elif defined(CONFIG_AT32_UART8_HCIUART)
#  define HCIUART_SERDEV HCIUART8
#else
#  error No HCI UART specifified
#endif

/* AT32F4 Mini GPIOs ********************************************************/

#define BOARD_NGPIOIN   0
#define BOARD_NGPIOOUT  1
#define BOARD_NGPIOINT  0

#define GPIO_OUT1         (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_DRV_MODETATE|\
                          GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN2)
#define GPIO_IN1
#define GPIO_INT1

/* LEDs */

#define GPIO_LED1       (GPIO_OUTPUT|GPIO_PUSHPULL|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN2)
#define GPIO_LED2       (GPIO_OUTPUT|GPIO_PUSHPULL|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN3)

/* SPI1 W25QXX */

#define W25QXX_FLASH_MINOR  0

#define FLASH_SPI1_CS (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_DRV_MODETATE | \
                        GPIO_OUTPUT_SET | GPIO_PORTD | GPIO_PIN7)

/* ETH */

#  define GPIO_EMAC_NINT  (GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI|\
                        GPIO_PORTA|GPIO_PIN1)
#  define GPIO_EMAC_NRST  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_DRV_MODETATE|\
                        GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN3)

/* PWM */

#define AT32F437_PWMTIMER   20
#define AT32F437_PWMCHANNEL 1

/* TIMER */

#define AT32F437_MINI_TIMER     3

/* I2C */

#define AT24_I2C_BUS   3 /* AT24Cxx connected to I2C3 */
#define AT24_MINOR     0

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
 * Name: at32_bringup
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

int at32_bringup(void);

/****************************************************************************
 * Name: at32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the at32f437-mini
 *   board.
 *
 ****************************************************************************/

void weak_function at32_spidev_initialize(void);

/****************************************************************************
 * Name: at32_i2sdev_initialize
 *
 * Description:
 *   Called to configure I2S chip select GPIO pins for the at32f437-mini
 *   board.
 *
 ****************************************************************************/

void weak_function at32_i2sdev_initialize(void);

/****************************************************************************
 * Name: at32_usbinitialize
 *
 * Description:
 *   Called from at32_usbinitialize very early in initialization to setup
 *   USB-related GPIO pins for the at32f437-mini board.
 *
 ****************************************************************************/

#ifdef CONFIG_AT32_OTGFS
void weak_function at32_usbinitialize(void);
#endif

/****************************************************************************
 * Name: at32_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host
 *   functionality. This function will start a thread that will monitor for
 *   device connection/disconnection events.
 *
 ****************************************************************************/

#if defined(CONFIG_AT32_OTGFS) && defined(CONFIG_USBHOST)
int at32_usbhost_initialize(void);
#endif

/****************************************************************************
 * Name: at32_netinitialize
 *
 * Description:
 *   Configure board resources to support networking.
 *
 ****************************************************************************/

#ifdef CONFIG_AT32_ETHMAC
void weak_function at32_netinitialize(void);
#endif

/****************************************************************************
 * Name: at32_gpio_initialize
 *
 * Description:
 *   Called to configure GPIO pins for the at32f437-mini
 *   board.
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_GPIO
int at32_gpio_initialize(void);
#endif

/****************************************************************************
 * Name: at32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the at32f437-mini
 *   board.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI
void at32_spidev_initialize(void);
#endif

/****************************************************************************
 * Name: at32_w25initialize
 *
 * Description:
 *   Called to configure W25 for the at32f437-mini
 *   board.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_W25
int at32_w25initialize(int minor);
#endif

/****************************************************************************
 * Name: at32_can_setup
 *
 * Description:
 *   Called to configure can chardriver for the at32f437-mini
 *   board.
 *
 ****************************************************************************/

#ifdef CONFIG_AT32_CAN_CHARDRIVER
int at32_can_setup(int port);
#endif

/****************************************************************************
 * Name: at32_cansock_setup
 *
 * Description:
 *   Called to configure can socket for the at32f437-mini
 *   board.
 *
 ****************************************************************************/

#ifdef CONFIG_AT32_CAN_SOCKET
int at32_cansock_setup(int port);
#endif

/****************************************************************************
 * Name: at32_netinitialize
 *
 * Description:
 *   Called to configure eth net for the at32f437-mini
 *   board.
 *
 ****************************************************************************/

#ifdef CONFIG_AT32_ETHMAC
void at32_netinitialize(void);
#endif

/****************************************************************************
 * Name: at32_sdinitialize
 *
 * Description:
 *   Called to configure sd card for the at32f437-mini
 *   board.
 *
 ****************************************************************************/

#ifdef CONFIG_AT32_SDIO
int at32_sdinitialize(int minor);
#endif

/****************************************************************************
 * Name: at32_pwm_setup
 *
 * Description:
 *   Called to configure pwm for the at32f437-mini
 *   board.
 *
 ****************************************************************************/

#ifdef CONFIG_AT32_PWM
int at32_pwm_setup(void);
#endif

/****************************************************************************
 * Name: at32_timer_driver_setup
 *
 * Description:
 *   Called to configure timer for the at32f437-mini
 *   board.
 *
 ****************************************************************************/

#ifdef CONFIG_AT32_TIM
int at32_timer_driver_setup(const char *devpath, int timer);
#endif

/****************************************************************************
 * Name: at32_adc_setup
 *
 * Description:
 *   Called to configure adc for the at32f437-mini
 *   board.
 *
 ****************************************************************************/

#ifdef CONFIG_AT32_ADC
int at32_adc_setup(void);
#endif

/****************************************************************************
 * Name: at32_at24_automount
 *
 * Description:
 *   Called to configure at24cxx for the at32f437-mini
 *   board.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_AT24XX
int at32_at24_automount(int minor);
#endif

#ifdef CONFIG_USERLED

/****************************************************************************
 * Name: board_userled_initialize
 *
 * Description:
 *   Called to configure user led for the at32f437-mini
 *   board.
 *
 ****************************************************************************/

uint32_t board_userled_initialize(void);

#ifdef CONFIG_PM

/****************************************************************************
 * Name: at32_led_pminitialize
 *
 * Description:
 *   Called to configure led pm for the at32f437-mini
 *   board.
 *
 ****************************************************************************/

void at32_led_pminitialize(void);
#endif

#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_AT32_AT32F437_MINI_SRC_AT32F437_MINI_H */
