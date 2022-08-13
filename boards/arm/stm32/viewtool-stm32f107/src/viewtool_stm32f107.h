/****************************************************************************
 * boards/arm/stm32/viewtool-stm32f107/src/viewtool_stm32f107.h
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

#ifndef __BOARDS_ARM_STM32_VIEWTOOL_STM32F107_SRC_VIEWTOOL_STM32F107_H
#define __BOARDS_ARM_STM32_VIEWTOOL_STM32F107_SRC_VIEWTOOL_STM32F107_H

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

/* Assume that everything is supported */

#define HAVE_USBDEV     1
#define HAVE_MMCSD      1
#define HAVE_RTC_DRIVER 1

/* Handle chip differences */

#if defined(CONFIG_ARCH_CHIP_STM32F103VC)
#  undef CONFIG_STM32_OTGFS
#elif defined(CONFIG_ARCH_CHIP_STM32F107VC)
#  undef CONFIG_STM32_USB
#  undef CONFIG_STM32_SDIO
#else
#  error Unknown chip on Viewtool board
#  undef HAVE_USBDEV
#  undef HAVE_MMCSD
#endif

/* Check if USB is enabled */

#if !defined(CONFIG_STM32_OTGFS) && !defined(CONFIG_STM32_USB)
#  undef HAVE_USBDEV
#elif !defined(CONFIG_USBDEV)
#  warning CONFIG_STM32_OTGFS (F107) or CONFIG_STM32_USB (F103) is enabled but CONFIG_USBDEV is not
#  undef HAVE_USB
#endif

/* Can't support MMC/SD features if the SDIO peripheral is disabled */

#ifndef CONFIG_STM32_SDIO
#  undef HAVE_MMCSD
#endif

/* Can't support MMC/SD features if mountpoints are disabled */

#ifdef CONFIG_DISABLE_MOUNTPOINT
#  undef HAVE_MMCSD
#endif

/* Default MMC/SD slot number/device minor number */

#define VIEWTOOL_MMCSD_SLOTNO 0

/* Check if we can support the RTC driver */

#if !defined(CONFIG_RTC) || !defined(CONFIG_RTC_DRIVER)
#  undef HAVE_RTC_DRIVER
#endif

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define STM32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* GPIO Configuration *******************************************************/

/* LEDs
 *
 * There are four LEDs on the ViewTool STM32F103/F107 board that can be
 * controlled by software:
 * LED1 through LED4.  All pulled high and can be illuminated by driving the
 * output to low
 *
 *   LED1 PA6
 *   LED2 PA7
 *   LED3 PB12
 *   LED4 PB13
 */

#define GPIO_LED1       (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN6)
#define GPIO_LED2       (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN7)
#define GPIO_LED3       (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN12)
#define GPIO_LED4       (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN13)

/* Buttons ******************************************************************/

/* All pulled high and will be sensed low when depressed.
 *
 *   SW2 PC11  Needs J42 closed
 *   SW3 PC12  Needs J43 closed
 *   SW4 PA0   Needs J44 closed
 */

#define MIN_IRQBUTTON   BUTTON_SW2
#define MAX_IRQBUTTON   BUTTON_SW4
#define NUM_IRQBUTTONS  (BUTTON_SW4 - BUTTON_SW2 + 1)

#define GPIO_SW2        (GPIO_INPUT | GPIO_CNF_INFLOAT | GPIO_MODE_INPUT | \
                         GPIO_EXTI | GPIO_PORTC | GPIO_PIN11)
#define GPIO_SW3        (GPIO_INPUT | GPIO_CNF_INFLOAT | GPIO_MODE_INPUT | \
                         GPIO_EXTI | GPIO_PORTC | GPIO_PIN12)
#define GPIO_SW4        (GPIO_INPUT | GPIO_CNF_INFLOAT | GPIO_MODE_INPUT | \
                         GPIO_EXTI | GPIO_PORTA | GPIO_PIN10)

/* microSD Card Interface
 *
 * microSD Connector
 * -----------------
 *
 *   ----------------------------- ------------------------- ---------------
 *          Connector J17            GPIO CONFIGURATION(s)
 *   PIN SIGNAL        LEGEND          (no remapping)        DP83848C Board
 *   --- ------------- ----------- ------------------------- ---------------
 *   1   VDD 3.3       N/A         N/A                       3.3
 *   2   GND           N/A         N/A                       GND
 *   3   PC8           SDIO_D0     GPIO_SDIO_D0              D0
 *   4   PD2           SDIO_CMD    GPIO_SDIO_CMD             CMD
 *   5   PC12          SDIO_CLK    GPIO_SDIO_CK              CLK
 *   6   PC11          SDIO_D3     GPIO_SDIO_D3              D3
 *   7   PC10          SDIO_D2     GPIO_SDIO_D2              D2
 *   8   PC9           SDIO_D1     GPIO_SDIO_D1              D1
 *   9   PA8           CD          Board-specific GPIO input CD
 *   --- ------------- ----------- ------------------------- ----------------
 *
 *   NOTES:
 *   1. The STM32F107 does not support the SDIO/memory card interface.
 *      So the SD card cannot be used with the STM32F107 (unless the pin-out
 *      just happens to match up with an SPI-based card interface???)
 */

#ifdef CONFIG_ARCH_CHIP_STM32F103VC
#  define GPIO_SD_CD      (GPIO_INPUT | GPIO_CNF_INFLOAT | GPIO_EXTI| \
                           GPIO_PORTA | GPIO_PIN8)
#endif

/* USB
 *
 * The Viewtool base board has a USB Mini-B connector.
 * Only USB device can be supported with this connector.
 *
 *  ------------------------- ------------------------------------
 *        USB Connector
 *        J10 mini-USB       GPIO CONFIGURATION(s)
 * --- --------- ----------- ------------------------------------
 * Pin Signal
 * --- --------- ----------- ------------------------------------
 *  1  USB_VBUS  VDD_USB     (No sensing available)
 *  2  OTG_DM    PA11        GPIO_OTGFS_DM (F107) GPIO_USB_DM (F103)
 *  3  OTG_DP    PA12        GPIO_OTGFS_DP (F107) GPIO_USB_DP (F103)
 *  4  OTG_ID    PA10        GPIO_OTGFS_ID (F107)
 *  5  Shield    N/A         N/A
 *  6  Shield    N/A         N/A
 *  7  Shield    N/A         N/A
 *  8  Shield    N/A         N/A
 *  9  Shield    N/A         N/A
 *               PE11 USB_EN   GPIO controlled soft pull-up (if J51 closed)
 *
 *  NOTES:
 *  1. GPIO_OTGFS_VBUS (F107) should not be configured.  No VBUS sensing
 *  2. GPIO_OTGFS_SOF (F107) is not used
 *  3. The OTG FS module has is own, internal soft pull-up logic.  J51 should
 *     be open so that PE11 activity does effect USB.
 */

#ifdef CONFIG_ARCH_CHIP_STM32F103VC
#  define GPIO_USB_PULLUP (GPIO_OUTPUT | GPIO_CNF_OUTOD | GPIO_MODE_50MHz | \
                           GPIO_OUTPUT_SET | GPIO_PORTE | GPIO_PIN11)
#endif

/* LCD
 *
 * An LCD may be connected via J11.
 * Only the STM32F103 supports the FSMC signals needed to drive the LCD.
 *
 * The LCD features an (1) HY32D module with built-in SSD1289 LCD controller,
 * and (a) a XPT2046 touch screen controller.
 *
 * LCD Connector
 * -------------
 *
 *   ----------------------------- -------------------- ---------------------
 *          Connector J11          GPIO CONFIGURATION(s)
 *   PIN SIGNAL        LEGEND          (F103 only)            LCD Module
 *   --- ------------- ----------- -------------------- ---------------------
 *   1   VDD_5         NC          N/A                  5V      ---
 *   2   GND           GND         N/A                  GND     ---
 *   3   PD14          DATA0       GPIO_NPS_D0          D0      HY32D
 *   4   PD15          DATA1       GPIO_NPS_D1          D1      HY32D
 *   5   PD0           DATA2       GPIO_NPS_D2          D2      HY32D
 *   6   PD1           DATA3       GPIO_NPS_D3          D3      HY32D
 *   7   PE7           DATA4       GPIO_NPS_D4          D4      HY32D
 *   8   PE8           DATA5       GPIO_NPS_D5          D5      HY32D
 *   9   PE9           DATA6       GPIO_NPS_D6          D6      HY32D
 *   10  PE10          DATA7       GPIO_NPS_D7          D7      HY32D
 *   11  PE11          DATA8       GPIO_NPS_D8          D8      HY32D
 *   12  PE12          DATA9       GPIO_NPS_D9          D9      HY32D
 *   13  PE13          DATA10      GPIO_NPS_D10         D10     HY32D
 *   14  PE14          DATA11      GPIO_NPS_D11         D11     HY32D
 *   15  PE15          DATA12      GPIO_NPS_D12         D12     HY32D
 *   16  PD8           DATA13      GPIO_NPS_D13         D13     HY32D
 *   17  PD9           DATA14      GPIO_NPS_D14         D14     HY32D
 *   18  PD10          DATA15      GPIO_NPS_D15         D15     HY32D
 *   19  (3)           LCD_CS      GPIO_NPS_NE1         CS      HY32D
 *   20  PD11          LCD_RS      GPIO_NPS_A16         RS      HY32D
 *   21  PD5           LCD_R/W     GPIO_NPS_NWE         WR      HY32D
 *   22  PD4           LCD_RD      GPIO_NPS_NOE         RD      HY32D
 *   23  PB1           LCD_RESET   (GPIO)               RESET   HY32D
 *   24  N/C           NC          N/A                  TE      (unused?)
 *   25  VDD_3.3       BL_VCC      N/A                  BLVDD   CA6219
 *                                                              (Drives LCD
 *                                                               backlight)
 *   26  GND           BL_GND      N/A                  BLGND   CA6219
 *   27  PB0           BL_PWM      GPIO_TIM3_CH3OUT(2)  BL_CNT  CA6219
 *   28  PC5           LCDTP_IRQ   (GPIO)               TP_IRQ  XPT2046
 *   29  PC4           LCDTP_CS    (GPIO)               TP_CS   XPT2046
 *   30  PB13          LCDTP_CLK   GPIO_SPI2_SCK        TP_SCK  XPT2046
 *   31  PB15          LCDTP_DIN   GPIO_SPI2_MOSI       TP_SI   XPT2046
 *   32  PB14          LCDTP_DOUT  GPIO_SPI2_MISO       TP_SO   XPT2046
 *   33  VDD_3.3       VDD_3.3     N/A                  3.3V    ---
 *   34  GND           GND         N/A                   GND     ---
 *   --- ------------- ----------- -------------------- ---------------------
 *
 *   NOTES:
 *   1) Only the F103 version of the board supports the FSMC
 *   2) No remap
 *   3) LCD_CS is controlled by J13 JUMPER4 (under the LCD unfortunately):
 *
 *      1->2 : PD7 (GPIO_NPS_NE1) enables the multiplexor : 1E\ enable input
 *                                                           (active LOW)
 *      3->4 : PD13 provides 1A0 input (1A1 is grounded). : 1A0 address input
 *             So will chip enable to either LCD_CS or
 *             Flash_CS.
 *      5->6 : 1Y0 output to LCD_CS                     : 1Y0 address output
 *      7->8 : 1Y1 output to Flash_CE                   : 1Y1 address output
 *
 *      Truth Table:
 *      1E\ 1A0 1A1 1Y0 1Y1
 *      --- --- --- --- ---
 *      HI  N/A N/A HI  HI
 *      LO  LO  LO  LO  HI
 *      LO  HI  LO  HI  LO
 */

#define GPIO_LCD_RESET    (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz | \
                           GPIO_OUTPUT_SET | GPIO_PORTB| GPIO_PIN1)
#define GPIO_LCDTP_IRQ    (GPIO_INPUT | GPIO_CNF_INFLOAT | GPIO_MODE_INPUT | \
                           GPIO_EXTI | GPIO_PORTC | GPIO_PIN5)
#define GPIO_LCDTP_CS     (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz | \
                           GPIO_OUTPUT_SET | GPIO_PORTC | GPIO_PIN4)

/* Freescale MPL115A barometer (optional add-on)
 *
 * This board support logic includes support for a Freescale MPL115A
 * barometer using SPI3 with chip select on PB6.
 */

#define GPIO_MPL115A_CS   (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz | \
                           GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN6)

/* FT80x GUI Discrete I/O (See README.txt for details):
 *
 * ------ ----------- --------------------
 * NAME   VIEWTOOL    STM32
 * ------ ----------- --------------------
 * CS#    J8  Pin 12  PA4/NSS1  (For SPI1)
 * CS#    J8  Pin  6  PB12/NSS2 (For SPI2)
 * INT#   J18 Pin  8  PA1
 * PD#    J18 Pin  6  PC5
 */

#if defined(CONFIG_VIEWTOOL_FT80X_SPI1)
#  define FT80X_SPIBUS    1
#  define GPIO_FT80X_CS   (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz | \
                           GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN4)
#elif defined(CONFIG_VIEWTOOL_FT80X_SPI2)
#  define FT80X_SPIBUS    2
#  define GPIO_FT80X_CS   (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz | \
                           GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN12)
#endif
#define GPIO_FT80X_INT    (GPIO_INPUT | GPIO_CNF_INFLOAT | GPIO_MODE_INPUT | \
                           GPIO_EXTI | GPIO_PORTA | GPIO_PIN1)
#define GPIO_FT80_PD      (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz | \
                           GPIO_OUTPUT_CLEAR | GPIO_PORTC| GPIO_PIN5)

/* MAX3421E USB HOST Discrete I/O (See README.txt for details):
 *
 * ------ ----------- --------------------
 * NAME   VIEWTOOL    STM32
 * ------ ----------- --------------------
 * CS#    J8  Pin 12  PA4/NSS1  (For SPI1)
 * CS#    J8  Pin  6  PB12/NSS2 (For SPI2)
 * INT#   J18 Pin 10  PA0
 * RST#   J18 Pin  8  PA1
 * GPX    J18 Pin  6  PC5 (not used)
 */

#if defined(CONFIG_VIEWTOOL_MAX3421E_SPI1)
#  define MAX3421E_SPIBUS   1
#  define GPIO_MAX3421E_CS  (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz | \
                             GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN4)
#elif defined(CONFIG_VIEWTOOL_MAX3421E_SPI2)
#  define MAX3421E_SPIBUS   2
#  define GPIO_MAX3421E_CS  (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz | \
                             GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN12)
#endif
#define GPIO_MAX3421E_INT   (GPIO_INPUT | GPIO_CNF_INFLOAT | GPIO_MODE_INPUT | \
                             GPIO_EXTI | GPIO_PORTA | GPIO_PIN0)
#define GPIO_MAX3421E_RST   (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz | \
                             GPIO_OUTPUT_CLEAR | GPIO_PORTA | GPIO_PIN1)
#define GPIO_MAX3421E_GPX   (GPIO_INPUT | GPIO_CNF_INFLOAT | GPIO_MODE_INPUT | \
                             GPIO_PORTC | GPIO_PIN5)

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

#ifndef __ASSEMBLY__

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
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the M3 Wildfire board.
 *
 ****************************************************************************/

void weak_function stm32_spidev_initialize(void);

/****************************************************************************
 * Name: stm32_led_initialize
 *
 * Description:
 *   Configure LEDs.  LEDs are left in the OFF state.
 *
 ****************************************************************************/

void stm32_led_initialize(void);

/****************************************************************************
 * Name: stm32_usbdev_initialize
 *
 * Description:
 *   Called from stm32_usbdev_initialize very early in initialization to
 *   setup USB-related GPIO pins for the Viewtool STM32F107 board.
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_OTGFS) && defined(CONFIG_USBDEV)
void weak_function stm32_usbdev_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_tsc_setup
 *
 * Description:
 *   This function is called by board-bringup logic to configure the
 *   touchscreen device.  This function will register the driver as
 *   /dev/inputN where N is the minor device number.
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
int stm32_tsc_setup(int minor);
#endif

/****************************************************************************
 * Name: stm32_sdinitialize
 *
 * Description:
 *   Initialize the SPI-based SD card.  Requires CONFIG_DISABLE_MOUNTPOINT=n
 *   and CONFIG_STM32_SPI1=y
 *
 ****************************************************************************/

int stm32_sdinitialize(int minor);

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
 * Name: stm32_mpl115ainitialize
 *
 * Description:
 *   Initialize and register the MPL115A Pressure Sensor driver.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/press0"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_MPL115A) && defined(CONFIG_STM32_SPI3)
int stm32_mpl115ainitialize(const char *devpath);
#endif

/****************************************************************************
 * Name: stm32_ft80x_setup
 *
 * Description:
 *   This function is called by board-bringup logic to configure the
 *   FT80x GUI device.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#if defined(CONFIG_VIEWTOOL_FT80X_SPI1) || defined(CONFIG_VIEWTOOL_FT80X_SPI2)
int stm32_ft80x_setup(void);
#endif

/****************************************************************************
 * Name: stm32_max3421e_setup
 *
 * Description:
 *   This function is called by board-bringup logic to configure the
 *   MAX3421E USB host.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#if defined(CONFIG_VIEWTOOL_MAX3421E_SPI1) || defined(CONFIG_VIEWTOOL_MAX3421E_SPI2)
int stm32_max3421e_setup(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32_VIEWTOOL_STM32F107_SRC_VIEWTOOL_STM32F107_H */
