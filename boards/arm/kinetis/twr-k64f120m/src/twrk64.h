/****************************************************************************
 * boards/arm/kinetis/twr-k64f120m/src/twrk64.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Marc Rechte <marc4@rechte.fr>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * This header file is only accessible from the src directory.
 * For /arch/arm/src accessibility use ../include/board.h instead.
 ****************************************************************************/

#ifndef __BOARDS_ARM_KINETIS_TWR_K64F120M_SRC_TWRK64_H
#define __BOARDS_ARM_KINETIS_TWR_K64F120M_SRC_TWRK64_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <arch/kinetis/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Assume we have everything */

#define HAVE_PROC        1
#define HAVE_MMCSD       1
#define HAVE_AUTOMOUNTER 1
#define HAVE_USBDEV      1

#if defined(CONFIG_KINETIS_RTC)
#define HAVE_RTC_DRIVER  1
#endif

/* Automount procfs */

#if !defined(CONFIG_FS_PROCFS)
#  undef HAVE_PROC
#endif

#if defined(HAVE_PROC) && defined(CONFIG_DISABLE_MOUNTPOINT)
#  warning Mountpoints disabled.  No procfs support
#  undef HAVE_PROC
#endif

#if defined(CONFIG_NSH_PROC_MOUNTPOINT)
#  define PROCFS_MOUNTPOUNT CONFIG_NSH_PROC_MOUNTPOINT
#else
#  define PROCFS_MOUNTPOUNT "/proc"
#endif

/* SD card support */

#define MMCSD_SLOTNO 0

/* Can't support MMC/SD features if mountpoints are disabled or if SDHC
 * support is not enabled.
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_KINETIS_SDHC)
#  undef HAVE_MMCSD
#endif

#ifdef HAVE_MMCSD
#  if defined(CONFIG_NSH_MMCSDSLOTNO) && CONFIG_NSH_MMCSDSLOTNO != 0
#    error Only one MMC/SD slot, slot 0
#  endif

#  ifdef CONFIG_NSH_MMCSDMINOR
#    define MMSCD_MINOR CONFIG_NSH_MMCSDMINOR
#  else
#    define MMSCD_MINOR 0
#  endif

/* We expect to receive GPIO interrupts for card insertion events */

#  ifndef CONFIG_KINETIS_GPIOIRQ
#    error "CONFIG_KINETIS_GPIOIRQ required for card detect interrupt"
#  endif

#  ifndef CONFIG_KINETIS_PORTBINTS
#    error "CONFIG_KINETIS_PORTBINTS required for card detect interrupt"
#  endif

#endif

/* Automounter */

#if !defined(CONFIG_FS_AUTOMOUNTER) || !defined(HAVE_MMCSD)
#  undef HAVE_AUTOMOUNTER
#  undef CONFIG_TWR_K64F120M_SDHC_AUTOMOUNT
#endif

#ifndef CONFIG_TWR_K64F120M_SDHC_AUTOMOUNT
#  undef HAVE_AUTOMOUNTER
#endif

/* Automounter defaults */

#ifdef HAVE_AUTOMOUNTER

#  ifndef CONFIG_TWR_K64F120M_SDHC_AUTOMOUNT_FSTYPE
#    define CONFIG_TWR_K64F120M_SDHC_AUTOMOUNT_FSTYPE "vfat"
#  endif

#  ifndef CONFIG_TWR_K64F120M_SDHC_AUTOMOUNT_BLKDEV
#    define CONFIG_TWR_K64F120M_SDHC_AUTOMOUNT_BLKDEV "/dev/mmcds0"
#  endif

#  ifndef CONFIG_TWR_K64F120M_SDHC_AUTOMOUNT_MOUNTPOINT
#    define CONFIG_TWR_K64F120M_SDHC_AUTOMOUNT_MOUNTPOINT "/mnt/sdcard"
#  endif

#  ifndef CONFIG_TWR_K64F120M_SDHC_AUTOMOUNT_DDELAY
#    define CONFIG_TWR_K64F120M_SDHC_AUTOMOUNT_DDELAY 1000
#  endif

#  ifndef CONFIG_TWR_K64F120M_SDHC_AUTOMOUNT_UDELAY
#    define CONFIG_TWR_K64F120M_SDHC_AUTOMOUNT_UDELAY 2000
#  endif
#endif /* HAVE_AUTOMOUNTER */

/* Can't support USB features if USB is not enabled */

#ifndef CONFIG_USBDEV
#  undef HAVE_USBDEV
#endif

/* How many SPI modules does this chip support? The LM3S6918 supports 2 SPI
 * modules (others may support more -- in such case, the following must be
 * expanded).
 */

#if KINETIS_NSPI < 1
#  undef CONFIG_KINETIS_SPI1
#  undef CONFIG_KINETIS_SPI2
#elif KINETIS_NSPI < 2
#  undef CONFIG_KINETIS_SPI2
#endif

/* Button definitions *******************************************************/

/* The TWR-K64F120M has 2 user buttons (plus a reset button):
 *
 * 1. SW1 (IRQ?)   PTC6
 * 2. SW3 (IRQ?)   PTA4
 */

#define BUTTON_SW1        0
#define BUTTON_SW3        1

#define BUTTON_SW1_BIT    (1 << BUTTON_SW1)
#define BUTTON_SW3_BIT    (1 << BUTTON_SW3)

/* Alternative pin resolution ***********************************************/

/* If there are alternative configurations for various pins in the
 * kinetis_k64pinmux.h header file, those alternative pins will be labeled
 * with a suffix like _1, _2, etc.
 * The logic in this file must select the correct pin configuration for the
 * board by defining a pin configuration (with no suffix) that maps to the
 * correct alternative.
 * Please refer to board README for pin explanation.
 */

#if 0
#define PIN_I2C0_SDA  PIN_I2C0_SDA_3
#define PIN_I2C0_SCL  PIN_I2C0_SCL_3

/* Connections via the General Purpose Tower Plug-in (TWRPI) Socket
 * TODO See README
 */

#define PIN_SPI2_SIN   PIN_SPI2_SIN_2
#define PIN_SPI2_SOUT  PIN_SPI2_SOUT_2
#define PIN_SPI2_SCK   PIN_SPI2_SCK_2

/* Connections via the Tower Primary Connector Side A
 * TODO See README
 */

/* PTE 26/27 */

#define PIN_UART3_RX   PIN_UART3_RX_2
#define PIN_UART3_TX   PIN_UART3_TX_2

/* PTE 24/25 */

#define PIN_UART4_RX   PIN_UART4_RX_2
#define PIN_UART4_TX   PIN_UART4_TX_2

/* Connections via the Tower Primary Connector Side B
 * TODO See README
 */
#endif

/* SDHC
 * important notice: on TWR-K64F120M, R521 (close to the SD card holder) is
 * not placed, hence WRPROTEC is always ON. Either place a 4.7KOhm resistor
 * or change PIN config to PULLDOWN, losing Write Protect function
 */

#define GPIO_SD_CARDDETECT (GPIO_PULLUP | PIN_INT_BOTH | PIN_PORTB | PIN20)
#define GPIO_SD_WRPROTECT  (GPIO_PULLUP | PIN_PORTB | PIN21)

/* SW */

#define GPIO_SW1           (GPIO_PULLUP | PIN_INT_BOTH | PIN_PORTC | PIN6)
#define GPIO_SW3           (GPIO_PULLUP | PIN_INT_BOTH | PIN_PORTA | PIN4)

/* LEDs. Note that LED1-3 are used by system, LED4 is for user defined apps. */

#define GPIO_LED1          (GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO | PIN_PORTE | PIN6)
#define GPIO_LED2          (GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO | PIN_PORTE | PIN7)
#define GPIO_LED3          (GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO | PIN_PORTE | PIN8)
#define GPIO_LED4          (GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO | PIN_PORTE | PIN9)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: k64_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the TWR-K64F120M board.
 *
 ****************************************************************************/

void weak_function k64_spidev_initialize(void);

/****************************************************************************
 * Name: k64_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins for the TWR-K64F120M board.
 *
 ****************************************************************************/

void weak_function k64_usbinitialize(void);

/****************************************************************************
 * Name: k64_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

#if defined(CONFIG_LIB_BOARDCTL) || defined(CONFIG_BOARD_LATE_INITIALIZE)
int k64_bringup(void);
#endif

/****************************************************************************
 * Name: k64_sdhc_initialize
 *
 * Description:
 *   Inititialize the SDHC SD card slot
 *
 ****************************************************************************/

#ifdef HAVE_MMCSD
int k64_sdhc_initialize(void);
#else
#  define k64_sdhc_initialize() (OK)
#endif

/****************************************************************************
 * Name: k64_cardinserted
 *
 * Description:
 *   Check if a card is inserted into the SDHC slot
 *
 ****************************************************************************/

#ifdef HAVE_AUTOMOUNTER
bool k64_cardinserted(void);
#else
#  define k64_cardinserted() (false)
#endif

/****************************************************************************
 * Name: k64_writeprotected
 *
 * Description:
 *   Check if the card in the MMC/SD slot is write protected
 *
 ****************************************************************************/

#ifdef HAVE_AUTOMOUNTER
bool k64_writeprotected(void);
#else
#  define k64_writeprotected() (false)
#endif

/****************************************************************************
 * Name:  k64_automount_initialize
 *
 * Description:
 *   Configure auto-mounter for the configured SDHC slot
 *
 * Input Parameters:
 *   None
 *
 *  Returned Value:
 *    None
 *
 ****************************************************************************/

#ifdef HAVE_AUTOMOUNTER
void k64_automount_initialize(void);
#endif

/****************************************************************************
 * Name:  k64_automount_event
 *
 * Description:
 *   The SDHC card detection logic has detected an insertion or removal event.
 *   It has already scheduled the MMC/SD block driver operations. Now we need
 *   to schedule the auto-mount event which will occur with a substantial
 *   delay to make sure that everything has settle down.
 *
 * Input Parameters:
 *   inserted - True if the card is inserted in the slot.  False otherwise.
 *
 *  Returned Value:
 *    None
 *
 *  Assumptions:
 *    Interrupts are disabled.
 *
 ****************************************************************************/

#ifdef HAVE_AUTOMOUNTER
void k64_automount_event(bool inserted);
#endif

/****************************************************************************
 * Name: k64_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

#ifdef CONFIG_PWM
int k64_pwm_setup(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_KINETIS_TWR_K64F120M_SRC_TWRK64_H */
