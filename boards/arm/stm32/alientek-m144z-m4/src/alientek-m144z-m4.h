/****************************************************************************
 * boards/arm/stm32/alientek-m144z-m4/src/alientek-m144z-m4.h
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

#ifndef __BOARDS_ARM_STM32_ALIENTEK_M144Z_M4_SRC_ALIENTEK_M144Z_M4_H
#define __BOARDS_ARM_STM32_ALIENTEK_M144Z_M4_SRC_ALIENTEK_M144Z_M4_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <arch/stm32/chip.h>

#include "stm32.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration from stm32f4discovery */

/* Assume that we have everything */

#define HAVE_USBDEV     1
#define HAVE_USBHOST    1
#define HAVE_USBMONITOR 1
#define HAVE_SDIO       1
#define HAVE_RTC_DRIVER 1
#define HAVE_NETMONITOR 1

/* Can't support USB host or device features if USB OTG FS is not enabled */

#ifndef CONFIG_STM32_OTGFS
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

/* NSH Network monitor  */

#if !defined(CONFIG_NET) || !defined(CONFIG_STM32_EMACMAC)
#  undef HAVE_NETMONITOR
#endif

#if !defined(CONFIG_NSH_NETINIT_THREAD) || !defined(CONFIG_ARCH_PHY_INTERRUPT) || \
    !defined(CONFIG_NETDEV_PHY_IOCTL) || !defined(CONFIG_NET_UDP)
#  undef HAVE_NETMONITOR
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
 * Pre-processor Definitions
 ****************************************************************************/

/* ALIENTEK M144Z-M4 (STM32F407 minimum system board) board GPIOs ***********/

/* LEDs - 2x user LEDs, active LOW (board has 510R series resistor to 3V3)
 *
 *   LED0 (red)   = PF9
 *   LED1 (green) = PF10
 *
 * GPIO_OUTPUT_SET (start HIGH) keeps the LED off at power-on.
 */

#define GPIO_LED0 \
  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET| \
   GPIO_PORTF|GPIO_PIN9)
#define GPIO_LED1 \
  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET| \
   GPIO_PORTF|GPIO_PIN10)

/* Keep GPIO_LED_STATUS alias for legacy autoled code (= LED0 / red). */

#define GPIO_LED_STATUS   GPIO_LED0

/* BUTTONS
 *
 *   KEY0  = PE4 - active LOW (pull-up; pressed pulls to GND).
 *                 Shared with BOOT0 via a BAT54C diode for UART ISP entry.
 *   WK_UP = PA0 - active HIGH (pull-down; pressed pulls to 3.3 V).
 *                 Doubles as STM32 standby-wakeup source.
 *
 * EXTI interrupts are enabled on both pins.
 */

#define GPIO_BTN_KEY0     (GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI| \
                           GPIO_PORTE|GPIO_PIN4)
#define GPIO_BTN_WKUP     (GPIO_INPUT|GPIO_PULLDOWN|GPIO_EXTI| \
                           GPIO_PORTA|GPIO_PIN0)

#define MIN_IRQBUTTON     BUTTON_KEY0
#define MAX_IRQBUTTON     BUTTON_WKUP
#define NUM_IRQBUTTONS    (MAX_IRQBUTTON - MIN_IRQBUTTON + 1)

/* On-board SPI Flash (W25Q128, 16 MiB) chip select - GPIO PB14. */

#define GPIO_W25_CS \
  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET| \
   GPIO_PORTB|GPIO_PIN14)

/* USB OTG-FS - Type-C connector USB2 (no VBUS sensing / power switch on
 * this board; the connector is wired directly to PA11/PA12 + VBUS).
 *
 * Leave only the pin assignment hints; stm32_usb.c uses internal AF pins.
 */

/* No on-board USB-OTG-HS, no Ethernet PHY, no MRF24J40 radio. */

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

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Configure board-side SPI chip-select GPIOs.  Called once from
 *   stm32_boardinitialize() during early bring-up, before any SPI driver
 *   has a chance to touch its slave.  The SPI bus pins themselves
 *   (SCK/MISO/MOSI) are configured by stm32_spibus_initialize() in the
 *   common STM32 layer.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_SPI1
void stm32_spidev_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called from stm32_usbinitialize very early in initialization to setup
 *   USB-related GPIO pins (PA11/PA12, OTG-FS on the on-board USB2 Type-C
 *   connector).
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_OTGFS) || defined(CONFIG_STM32_OTGHS)
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
int stm32_adc_setup(void);
#endif

/****************************************************************************
 * Name: stm32_sdio_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support
 *
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_STM32_SDIO)
int stm32_sdio_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device
 *
 ****************************************************************************/

#ifdef CONFIG_CAN
int stm32_can_setup(void);
#endif

/****************************************************************************
 * Name: stm32_dac_setup
 *
 * Description:
 *   Initialize and register the DAC0 of the microcontroller.
 *
 * Input parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/dac0"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_DAC)
int stm32_dac_setup(void);
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
 * Returned Values:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_TIMER
int stm32_timer_driver_setup(const char *devpath, int timer);
#endif

/****************************************************************************
 * Name: stm32_mrf24j40_initialize
 *
 * Description:
 *   Initialize the MRF24J40 device.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_IEEE802154_MRF24J40
int stm32_mrf24j40_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32_ALIENTEK_M144Z_M4_SRC_ALIENTEK_M144Z_M4_H */
