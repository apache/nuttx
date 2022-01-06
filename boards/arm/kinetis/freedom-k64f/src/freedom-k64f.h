/****************************************************************************
 * boards/arm/kinetis/freedom-k64f/src/freedom-k64f.h
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

#ifndef __BOARDS_ARM_KINETIS_FREEDOM_K64F_SRC_FREEDOM_K64F_H
#define __BOARDS_ARM_KINETIS_FREEDOM_K64F_SRC_FREEDOM_K64F_H

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

/* Application Configuration ************************************************/

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

#  ifndef CONFIG_KINETIS_PORTEINTS
#    error "CONFIG_KINETIS_PORTEINTS required for card detect interrupt"
#  endif

#endif

/* Automounter */

#if !defined(CONFIG_FS_AUTOMOUNTER) || !defined(HAVE_MMCSD)
#  undef HAVE_AUTOMOUNTER
#  undef CONFIG_FRDMK64F_SDHC_AUTOMOUNT
#endif

#ifndef CONFIG_FRDMK64F_SDHC_AUTOMOUNT
#  undef HAVE_AUTOMOUNTER
#endif

/* Automounter defaults */

#ifdef HAVE_AUTOMOUNTER

#  ifndef CONFIG_FRDMK64F_SDHC_AUTOMOUNT_FSTYPE
#    define CONFIG_FRDMK64F_SDHC_AUTOMOUNT_FSTYPE "vfat"
#  endif

#  ifndef CONFIG_FRDMK64F_SDHC_AUTOMOUNT_BLKDEV
#    define CONFIG_FRDMK64F_SDHC_AUTOMOUNT_BLKDEV "/dev/mmcds0"
#  endif

#  ifndef CONFIG_FRDMK64F_SDHC_AUTOMOUNT_MOUNTPOINT
#    define CONFIG_FRDMK64F_SDHC_AUTOMOUNT_MOUNTPOINT "/mnt/sdcard"
#  endif

#  ifndef CONFIG_FRDMK64F_SDHC_AUTOMOUNT_DDELAY
#    define CONFIG_FRDMK64F_SDHC_AUTOMOUNT_DDELAY 1000
#  endif

#  ifndef CONFIG_FRDMK64F_SDHC_AUTOMOUNT_UDELAY
#    define CONFIG_FRDMK64F_SDHC_AUTOMOUNT_UDELAY 2000
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

/* FREEDOM-K64F GPIOs *******************************************************/

/* A micro Secure Digital (SD) card slot is available on the FRDM-K64F
 * connected to the SD Host Controller (SDHC) signals of the MCU.
 * This slot will accept micro format SD memory cards.
 * The SD card detect pin (PTE6) is an open switch that shorts with VDD when
 * card is inserted.
 *
 *   ------------ ------------- --------
 *    SD Card Slot Board Signal  K64F Pin
 *    ------------ ------------- --------
 *    DAT0         SDHC0_D0      PTE0
 *    DAT1         SDHC0_D1      PTE1
 *    DAT2         SDHC0_D2      PTE5
 *    CD/DAT3      SDHC0_D3      PTE4
 *    CMD          SDHC0_CMD     PTE3
 *    CLK          SDHC0_DCLK    PTE2
 *    SWITCH       D_CARD_DETECT PTE6
 *    ------------ ------------- --------
 *
 * There is no Write Protect pin available to the K64F.
 */

#define GPIO_SD_CARDDETECT (GPIO_PULLUP | PIN_INT_BOTH | PIN_PORTE | PIN6)

/* Two push buttons, SW2 and SW3, are available on FRDM-K64F board,
 * where SW2 is connected to PTC6 and SW3 is connected to PTA4.
 * Besides the general purpose input/output functions,
 * SW2 and SW3 can be low-power wake up signal.
 * Also, only SW3 can be a non-maskable interrupt.
 *
 *   Switch GPIO Function
 *   ------ --------------------------------------------------------------
 *   SW2    PTC6/SPI0_SOUT/PD0_EXTRG/I2S0_RX_BCLK/FB_AD9/I2S0_MCLK/LLWU_P10
 *   SW3    PTA4/FTM0_CH1/NMI_b/LLWU_P3
 */

#define GPIO_SW2           (GPIO_PULLUP | PIN_INT_BOTH | PIN_PORTC | PIN6)
#define GPIO_SW3           (GPIO_PULLUP | PIN_INT_BOTH | PIN_PORTA | PIN4)

/* An RGB LED is connected through GPIO as shown below:
 *
 *   LED    K64
 *   ------ -------------------------------------------------------
 *   RED    PTB22/SPI2_SOUT/FB_AD29/CMP2_OUT
 *   GREEN  PTE26/ENET_1588_CLKIN/UART4_CTS_b/RTC_CLKOUT/USB0_CLKIN
 *   BLUE   PTB21/SPI2_SCK/FB_AD30/CMP1_OUT
 */

#define GPIO_LED_R         (GPIO_LOWDRIVE | GPIO_OUTPUT_ONE | PIN_PORTB | PIN22)
#define GPIO_LED_G         (GPIO_LOWDRIVE | GPIO_OUTPUT_ONE | PIN_PORTE | PIN26)
#define GPIO_LED_B         (GPIO_LOWDRIVE | GPIO_OUTPUT_ONE | PIN_PORTB | PIN21)

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: k64_i2cdev_initialize
 *
 * Description:
 *   Called to configure I2C chip select GPIO pins for the FRDM-K64F board.
 *
 ****************************************************************************/

int k64_i2cdev_initialize(void);

/****************************************************************************
 * Name: k64_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the FRDM-K64F board.
 *
 ****************************************************************************/

void weak_function k64_spidev_initialize(void);

/****************************************************************************
 * Name: k64_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins for the FREEDOM-K64F board.
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

#if defined(CONFIG_BOARDCTL) || defined(CONFIG_BOARD_LATE_INITIALIZE)
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
 *   The SDHC card detection logic has detected
 *   an insertion or removal event.
 *   It has already scheduled the MMC/SD block driver operations.
 *   Now we need to schedule the auto-mount event which will occur with a
 *   substantial delay to make sure that everything has settle down.
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
#endif /* __BOARDS_ARM_KINETIS_FREEDOM_K64F_SRC_FREEDOM_K64F_H */
