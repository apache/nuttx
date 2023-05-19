/****************************************************************************
 * boards/arm/sam34/sam4e-ek/src/sam4e-ek.h
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

#ifndef __BOARDS_ARM_SAM34_SAM4E_EK_SRC_SAM4E_EK_H
#define __BOARDS_ARM_SAM34_SAM4E_EK_SRC_SAM4E_EK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include <arch/irq.h>
#include <nuttx/irq.h>

#include "hardware/sam_pinmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define HAVE_HSMCI      1
#define HAVE_AT25       1
#define HAVE_USBDEV     1
#define HAVE_USBMONITOR 1
#define HAVE_NETWORK    1

/* HSMCI */

/* Can't support MMC/SD if the card interface is not enabled */

#if !defined(CONFIG_SAM34_HSMCI)
#  undef HAVE_HSMCI
#endif

/* Can't support MMC/SD features if mountpoints are disabled */

#if defined(HAVE_HSMCI) && defined(CONFIG_DISABLE_MOUNTPOINT)
#  warning Mountpoints disabled.  No MMC/SD support
#  undef HAVE_HSMCI
#endif

/* We need PIO interrupts on GPIOA to support card detect interrupts */

#if defined(HAVE_HSMCI) && !defined(CONFIG_SAM34_GPIOA_IRQ)
#  warning PIOA interrupts not enabled.  No MMC/SD support.
#  undef HAVE_HSMCI
#endif

/* AT25 Serial FLASH */

/* Can't support the AT25 device if it SPI0 or AT25 support are not enabled */

#if !defined(CONFIG_SAM34_SPI0) || !defined(CONFIG_MTD_AT25)
#  undef HAVE_AT25
#endif

/* Can't support AT25 features if mountpoints are disabled or if we were not
 * asked to mount the AT25 part
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_SAM4EEK_AT25_BLOCKMOUNT)
#  undef HAVE_AT25
#endif

/* If we are going to mount the AT25, then they user must also have told
 * us what to do with it by setting one of these.
 */

#ifndef CONFIG_FS_NXFFS
#  undef CONFIG_SAM4EEK_AT25_NXFFS
#endif

#if !defined(CONFIG_SAM4EEK_AT25_FTL) && !defined(CONFIG_SAM4EEK_AT25_NXFFS)
#  undef HAVE_AT25
#endif

#if defined(CONFIG_SAM4EEK_AT25_FTL) && defined(CONFIG_SAM4EEK_AT25_NXFFS)
#  warning Both CONFIG_SAM4EEK_AT25_FTL and CONFIG_SAM4EEK_AT25_NXFFS are set
#  warning Ignoring CONFIG_SAM4EEK_AT25_NXFFS
#  undef CONFIG_SAM4EEK_AT25_NXFFS
#endif

/* USB Device */

/* CONFIG_SAM34_UDP and CONFIG_USBDEV must be defined, or there is no USB
 * device.
 */

#if !defined(CONFIG_SAM34_UDP) || !defined(CONFIG_USBDEV)
#  undef HAVE_USBDEV
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

/* Networking */

#if !defined(CONFIG_NET) || !defined(CONFIG_SAM34_EMAC)
#  undef HAVE_NETWORK
#endif

/* SAM4E-EK GPIO Pin Definitions ********************************************/

/* LCD:
 *
 * The SAM4E-EK carries a TFT transmissive LCD module with touch panel,
 * FTM280C34D.
 * Its integrated driver IC is ILI9325. The LCD display area is 2.8 inches
 * diagonally measured, with a native resolution of 240 x 320 dots.
 *
 * The SAM4E16 communicates with the LCD through PIOC where an 8-bit parallel
 * "8080-like" protocol data bus has to be implemented in software.
 *
 *  ---- ----- --------- --------------------------------
 *  PIN  PIO   SIGNAL    NOTES
 *  ---- ----- --------- --------------------------------
 *    1                  VDD
 *    2  PC7   DB17
 *    3  PC6   DB16
 *    4  PC5   DB15
 *    5  PC4   DB14
 *    6  PC3   DB13
 *    7  PC2   DB12
 *    8  PC1   DB11
 *    9  PC0   DB10
 *   10        DB9       Pulled low
 *   11        DB8       Pulled low
 *   12        DB7       Pulled low
 *   13        DB6       Pulled low
 *   14        DB5       Pulled low
 *   15        DB4       Pulled low
 *   16        DB3       Pulled low
 *   17        DB2       Pulled low
 *   18        DB1       Pulled low
 *   19        DB0       Pulled low
 *  ---- ----- --------- --------------------------------
 *   20                  VDD
 *   21  PC11  RD
 *   22  PC8   WR
 *   23  PC19  RS
 *   24  PD18  CS        Via J8, pulled high.
 *   25        RESET     Connects to NSRST
 *   26        IM0       Pulled high
 *   27        IM1       Grounded
 *   28        GND
 *  ---- ----- --------- --------------------------------
 *   29 [PC13] LED-A     Backlight controls:  PC13 enables
 *   30 [PC13] LEDK1       AAT3155 charge pump that drives
 *   31 [PC13] LEDK2       the backlight LEDs
 *   32 [PC13] LEDK3
 *   33 [PC13] LEDK4
 *   34 [PC13] LEDK1
 *  ---- ----- --------- --------------------------------
 *   35        Y+        These go to the ADS7843
 *   36        Y-          touchscreen controller.
 *   37        X+
 *   38        X-
 *   39        NC
 *  ---- ----- --------- --------------------------------
 *
 * LCD backlight is made of 4 white chip LEDs in parallel, driven by an
 * AAT3155 charge pump, MN4.
 * The AAT3155 is controlled by the SAM3U4E through a single line
 * Simple Serial Control (S2Cwire) interface, which permits to enable,
 * disable, and set the LED drive current (LED brightness control) from
 * a 32-level logarithmic scale.
 * Four resistors R93/R94/R95/R96 are implemented for optional current
 * limitation.
 */

/* LCD Backlight pin definition. */

#define GPIO_LCD_BKL  (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                       GPIO_PORT_PIOC | GPIO_PIN13)

/* ADS7843 Touchscreen controller (TSC)
 *
 * The LCD module integrates a 4-wire touch screen panel controlled by
 * MN5, ADS7843, which is a slave device on the SAM3U4E SPI bus.
 * The ADS7843 touch ADC auxiliary inputs IN3/IN4 are connected to test
 * points for optional function extension.
 *
 *   ------ -------
 *   GPIO   PIN
 *   ------ -------
 *   PA11   /CS     (pulled high)
 *   PA12   DOUT
 *   PA13   DIN
 *   PA14   DCLK
 *   PA16   /PENIRQ
 *   PA17   BUSY
 *   ------ -------
 *
 * The IRQ is active low and pulled up.
 *
 * Pen Interrupt. Open anode output, requires 10kO to 100kO pull-up resistor
 * externally.  There is a 100KO pull-up on the SAM4E-EK board so no
 * additional pull-up should be required.
 *
 * BUSY is high impedance when CS is high (not selected).  When CS is
 * is low, BUSY is active high.  Since the pin is pulled up, it will appear
 * busy if CS is not selected (there is no pull-up onboard).
 */

#define GPIO_TCS_IRQ  (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_INT_BOTHEDGES | \
                       GPIO_PORT_PIOA | GPIO_PIN16)
#define GPIO_TCS_BUSY (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_PORT_PIOA | \
                       GPIO_PIN17)

#define SAM_TCS_IRQ   SAM_IRQ_PA16

/* Ethernet MAC.  The PHY interrupt is available on pin PD28 */

#define GPIO_PHY_IRQ  (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_CFG_DEGLITCH | \
                       GPIO_INT_FALLING | GPIO_PORT_PIOD | GPIO_PIN28)
#define SAM_PHY_IRQ   SAM_IRQ_PD28

/* LEDs
 *
 *   D2 PA0  Blue   Pulled high
 *   D3 PD20 Amber  Pulled high
 *   D4 PD21 Green  Pulled high
 */

#define GPIO_D3       (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_PORT_PIOD | \
                       GPIO_OUTPUT_CLEAR | GPIO_PIN20)
#define GPIO_D2       (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_PORT_PIOA | \
                       GPIO_OUTPUT_SET | GPIO_PIN0)
#define GPIO_D4       (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_PORT_PIOD | \
                       GPIO_OUTPUT_SET | GPIO_PIN21)

/* Buttons
 *
 * Four buttons for software inputs:
 *
 *   PA1  BUTTON_SCROLL-UP    Grounded
 *   PA2  BUTTON_SCROLL-DOWN  Grounded
 *   PA19 BUTTON_WAKU         Grounded
 *   PA20 BUTTON_TAMP         Grounded
 */

#define GPIO_SCROLLUP  (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_CFG_DEGLITCH | \
                        GPIO_INT_BOTHEDGES | GPIO_PORT_PIOA | GPIO_PIN1)
#define GPIO_SCROLLDWN (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_CFG_DEGLITCH | \
                        GPIO_INT_BOTHEDGES | GPIO_PORT_PIOA | GPIO_PIN2)
#define GPIO_WAKU      (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_CFG_DEGLITCH | \
                        GPIO_INT_BOTHEDGES | GPIO_PORT_PIOA | GPIO_PIN19)
#define GPIO_TAMP      (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_CFG_DEGLITCH | \
                        GPIO_INT_BOTHEDGES | GPIO_PORT_PIOA | GPIO_PIN20)

#define IRQ_SCROLLUP   SAM_IRQ_PA1
#define IRQ_SCROLLDWN  SAM_IRQ_PA2
#define IRQ_WAKU       SAM_IRQ_PA19
#define IRQ_TAMP       SAM_IRQ_PA20

/* USART1: To avoid any electrical conflict, the RS232 and RS485 transceiver
 * are isolated from the receiving line PA21.
 *
 * - Chose RS485 channel: Close 1-2 pins on JP11 and set PA23 to high level
 * - Chose RS232 channel: Close 2-3 pins on JP11 and set PA23 to low level
 */

#define GPIO_RS232_ENABLE (GPIO_OUTPUT | GPIO_CFG_DEFAULT | \
                           GPIO_OUTPUT_CLEAR | GPIO_PORT_PIOA | GPIO_PIN23)
#define GPIO_RS485_ENABLE (GPIO_OUTPUT | GPIO_CFG_DEFAULT | \
                           GPIO_OUTPUT_SET | GPIO_PORT_PIOA | GPIO_PIN23)

/* HSMCI SD Card Detect
 *
 *   PA26 DAT2
 *   PA27 DAT3
 *   PA28 CMD
 *   PA29 CLK
 *   PA30 DAT0
 *   PA31 DAT1
 *   PA6  CD        Pulled high
 */

#define GPIO_MCI_CD   (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_PORT_PIOA | GPIO_PIN6)
#define MCI_CD_IRQ    SAM_IRQ_PA6

/* SPI Chip Selects */

/* Touchscreen Controller:
 *
 *   ------ -------
 *   GPIO   PIN
 *   ------ -------
 *   PA11   /CS     (pulled high externally)
 *   PA12   DOUT
 *   PA13   DIN
 *   PA14   DCLK
 *   PA16   /PENIRQ
 *   PA17   BUSY
 *   ------ -------
 *
 * Chip select pin connected to the touchscreen controller and to the ZigBee
 * module connector.
 * Notice that the touchscreen chip select is implemented as a GPIO
 * OUTPUT that must be controlled by board-specific.
 * This is because the ADS7843E driver must be able to sample the device BUSY
 * GPIO input between SPI transfers.
 * However, the AD7843E will tri-state the BUSY input whenever the chip
 * select is de-asserted.
 * So the only option is to control the chip select manually and hold
 * it low throughout the SPI transfer.
 */

#define GPIO_TSC_CS   (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
                       GPIO_PORT_PIOA | GPIO_PIN11)
#define TSC_CSNUM     0

/* Serial FLASH (AT25DF321A)
 *
 *   ------ ------- ---------------
 *   GPIO   PIN     SAM4E FUNCTION
 *   ------ ------- ---------------
 *   PA13   SI      MOSI
 *   PA12   SO      MIS0
 *   PA14   SCK     SPCK
 *   PA5    /CS     NPCS3 (pulled high externally)
 *   ------ ------- ---------------
 */

#define GPIO_FLASH_CS (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
                       GPIO_PORT_PIOA | GPIO_PIN5)
#define FLASH_CSNUM   3

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
 * Name: sam_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the SAM4E-EK board.
 *
 ****************************************************************************/

void weak_function sam_spidev_initialize(void);

/****************************************************************************
 * Name: sam_hsmci_initialize
 *
 * Description:
 *   Initialize HSMCI support
 *
 ****************************************************************************/

#ifdef HAVE_HSMCI
int sam_hsmci_initialize(int minor);
#else
#  define sam_hsmci_initialize(minor) (-ENOSYS)
#endif

/****************************************************************************
 * Name: sam_netinitialize
 *
 * Description:
 *   Configure board resources to support networking.
 *
 ****************************************************************************/

#ifdef HAVE_NETWORK
void weak_function sam_netinitialize(void);
#endif

/****************************************************************************
 * Name: sam_cardinserted
 *
 * Description:
 *   Check if a card is inserted into the selected HSMCI slot
 *
 ****************************************************************************/

#ifdef HAVE_HSMCI
bool sam_cardinserted(int slotno);
#else
#  define sam_cardinserted(slotno) (false)
#endif

/****************************************************************************
 * Name: sam_writeprotected
 *
 * Description:
 *   Check if the card in the MMCSD slot is write protected
 *
 ****************************************************************************/

#ifdef HAVE_HSMCI
bool sam_writeprotected(int slotno);
#else
#  define sam_writeprotected(slotno) (false)
#endif

/****************************************************************************
 * Name: sam_at25_automount
 *
 * Description:
 *   Initialize, configure, and mount the AT25 serial FLASH.  The FLASH will
 *   be mounted at /dev/at25.
 *
 ****************************************************************************/

#ifdef HAVE_AT25
int sam_at25_automount(int minor);
#else
#  define sam_at25_automount(minor) (-ENOSYS)
#endif

/****************************************************************************
 * Name: sam_tsc_setup
 *
 * Description:
 *   This function is called by board-bringup logic to configure the
 *   touchscreen device.
 *  This function will register the driver as /dev/inputN where N is the
 *   minor device number.
 *
 * Input Parameters:
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_INPUT_ADS7843E
int sam_tsc_setup(int minor);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_SAM34_SAM4E_EK_SRC_SAM4E_EK_H */
