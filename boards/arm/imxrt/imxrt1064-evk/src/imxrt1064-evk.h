/****************************************************************************
 * boards/arm/imxrt/imxrt1064-evk/src/imxrt1064-evk.h
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

#ifndef __BOARDS_ARM_IMXRT_IMXRT1064_EVK_SRC_IMXRT1064_EVK_H
#define __BOARDS_ARM_IMXRT_IMXRT1064_EVK_SRC_IMXRT1064_EVK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <arch/irq.h>
#include <nuttx/irq.h>

#include "imxrt_gpio.h"
#include "imxrt_iomuxc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Touchscreen definitions **************************************************/

/* The IMXRT 1050/1060 have connectors for the LCD model RK043FN02H-CT.
 * It comes with the FT5336GQQ (FT5X06) touchscreen chip integrated.
 * FT5X06 is connected to the LPI2C1 bus.
 */

/* LPI2C address of the FT5336GQQ touchscreen chip */

#define FT5X06_I2C_ADDRESS  0x38

/* i.MX RT 1060 GPIO Pin Definitions ****************************************/

/* LEDs */

/* There are four LED status indicators located on the EVK Board.
 * The functions of these LEDs include:
 *
 *   - Main Power Supply(D3)
 *     Green: DC 5V main supply is normal.
 *     Red:   J2 input voltage is over 5.6V.
 *     Off:   The board is not powered.
 *   - Reset RED LED(D15)
 *   - OpenSDA LED(D16)
 *   - USER LED(D18)
 *
 * Only a single LED, D18, is under software control.
 */

#define GPIO_LED        (GPIO_OUTPUT | IOMUX_LED_DEFAULT | \
                         GPIO_OUTPUT_ZERO | GPIO_PORT1 | GPIO_PIN9)  /* AD_BO_09 */

#define LED_DRIVER_PATH "/dev/userleds"

/* Buttons ******************************************************************/

/* The IMXRT board has one external user button
 *
 * 1. SW8 (IRQ88)   GPIO5-00
 */

#define GPIO_SW8       (GPIO_INTERRUPT | GPIO_INTBOTH_EDGES | \
                        IOMUX_SW_DEFAULT | \
                        GPIO_PORT5 | GPIO_PIN0)    /* WAKEUP */

#define GPIO_SW8_INT   (_IMXRT_GPIO5_0_15_BASE+0)

/* LCD Backlight */

#define GPIO_LCD_BL     (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GPIO_PORT2 | \
                         GPIO_PIN31 | IOMUX_LCD_BL_DEFAULT)

/* Ethernet */

/* Ethernet Interrupt: GPIOAD_B0_10
 *
 * This pin has a week pull-up within the PHY, is open-drain, and requires
 * an external 1k ohm pull-up resistor (present on the EVK).  A falling
 * edge then indicates a change in state of the PHY.
 */

#define GPIO_ENET_INT   (IOMUX_ENET_INT_DEFAULT | \
                         GPIO_PORT1 | GPIO_PIN10)    /* AD_B0_10 */
#define GPIO_ENET_IRQ   IMXRT_IRQ_GPIO1_10

/* Ethernet Reset:  GPIOAD_B0_09
 *
 * The #RST uses inverted logic.  The initial value of zero will put the
 * PHY into the reset state.
 */

#define GPIO_ENET_RST   (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | \
                         GPIO_PORT1 | GPIO_PIN9 | IOMUX_ENET_RST_DEFAULT) /* AD_B0_09 */

#ifdef CONFIG_ETH0_PHY_KSZ8081
#  ifdef GPIO_LED
#    warning LED interferes with ETH reset unless R323 is removed.
#  endif
#endif

/* LPSPI1 CS:  GPIO_SD_B0_01 */

#define IOMUX_LPSPI1_CS (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | \
                         IOMUX_SPEED_MEDIUM | IOMUX_PULL_UP_100K | \
                         _IOMUX_PULL_ENABLE)
#define GPIO_LPSPI1_CS  (GPIO_OUTPUT | GPIO_OUTPUT_ONE | \
                         GPIO_PORT3 | GPIO_PIN13 | IOMUX_LPSPI1_CS)

/* LPSPI3 CS:  GPIO_AD_B0_03 */

#define IOMUX_LPSPI3_CS      (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | \
                              IOMUX_SPEED_MEDIUM | IOMUX_PULL_UP_100K | \
                              _IOMUX_PULL_ENABLE)
#define GPIO_LPSPI3_CS       (GPIO_OUTPUT | GPIO_OUTPUT_ONE | \
                              GPIO_PORT1 | GPIO_PIN3 | IOMUX_LPSPI3_CS) /* GPIO_AD_B0_03 */

/* MMC/SD */

#define IOMUX_MMCSD_EN       (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | \
                              IOMUX_SPEED_MEDIUM | IOMUX_PULL_UP_100K | \
                              _IOMUX_PULL_ENABLE)
#define GPIO_MMCSD_EN        (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | \
                              GPIO_PORT3 | GPIO_PIN2 | IOMUX_MMCSD_EN)

/* Touchscreen
 *
 * Interrupt line: GPIO_AD_B0_11
 *
 * The interrupt line coming from the touchscreen FT5336GQQ IC.
 * The touchscreen IC is integrated into the optional RK043FN02H-CT LCD panel
 * and it's connected to the LPI2C1 bus.
 *
 * Reset line: GPIO_AD_B0_02
 *
 * The reset line is active low.
 */

#define GPIO_FT5X06_INTR     IMXRT_IRQ_GPIO1_11

#define IOMUX_FT5X06_RST     (IOMUX_PULL_NONE | IOMUX_CMOS_OUTPUT | \
                              IOMUX_DRIVE_40OHM | IOMUX_SPEED_MEDIUM | \
                              IOMUX_SLEW_SLOW)                            /* AD_B0_11 */
#define GPIO_FT5X06_CTRST_N  (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | \
                              GPIO_PORT1 | GPIO_PIN2 | IOMUX_FT5X06_RST)  /* AD_B0_02 */

/* Test Pins ****************************************************************/

#define BOARD_NGPIOIN   0 /* Amount of GPIO Input pins */
#define BOARD_NGPIOOUT  4 /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT  0 /* Amount of GPIO Input w/ Interruption pins */

#define GPIO_GOUT1      (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | IOMUX_GOUT_DEFAULT | \
                         GPIO_PORT1 | GPIO_PIN19)

#define GPIO_GOUT2      (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | IOMUX_GOUT_DEFAULT | \
                         GPIO_PIN18 | GPIO_PORT1)

#define GPIO_GOUT3      (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | IOMUX_GOUT_DEFAULT | \
                         GPIO_PIN10 | GPIO_PORT1)

#define GPIO_GOUT4      (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | IOMUX_GOUT_DEFAULT | \
                         GPIO_PIN9 | GPIO_PORT1)

/* USB OTG ID Pin： GPIO_AD_B1_02 */

#define GPIO_USBOTG_ID  (GPIO_USB_OTG1_ID_1 | IOMUX_USBOTG_ID_DEFAULT)      /* AD_B1_02 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: imxrt_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

#if defined(CONFIG_BOARDCTL) || defined(CONFIG_BOARD_LATE_INITIALIZE)
int imxrt_bringup(void);
#endif

/****************************************************************************
 * Name: imxrt_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the i.MXRT1050 EVK.
 *
 ****************************************************************************/

void imxrt_spidev_initialize(void);

/****************************************************************************
 * Name: imxrt_mmcsd_spi_initialize
 *
 * Description:
 *   Initialize SPI-based SD card and card detect thread.
 *
 ****************************************************************************/

#ifdef CONFIG_MMCSD_SPI
int imxrt_mmcsd_spi_initialize(int minor)
#endif

/****************************************************************************
 * Name: imxrt_autoled_initialize
 *
 * Description:
 *   Initialize NuttX-controlled LED logic
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void imxrt_autoled_initialize(void);
#endif

/****************************************************************************
 * Name: imxrt_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_GPIO
int imxrt_gpio_initialize(void);
#endif

/****************************************************************************
 * Name: imxrt_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_FLEXCAN
int imxrt_can_setup(void);
#endif

/****************************************************************************
 * Name: imxrt_adc_initialize
 *
 * Description:
 *   Initialize ADC drivers
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_ADC
int imxrt_adc_initialize(void);
#endif

/****************************************************************************
 * Name: imxrt_ft5x06_register
 *
 * Description:
 *   Initialize ft5x06 IC touchscreen driver
 *
 ****************************************************************************/

#ifdef CONFIG_INPUT_FT5X06
int imxrt_ft5x06_register(void);
#endif

/****************************************************************************
 * Name: imxrt_backlight
 *
 * Description:
 *   Initialize the backlight pins of the LCD and turn it ON
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_LCD
void imxrt_lcd_initialize(void);
#endif

#if defined(CONFIG_IMXRT_USBOTG) || defined(CONFIG_USBHOST)
int imxrt_usbhost_initialize(void);
#endif

#ifdef CONFIG_IMXRT_FLEXSPI
int imxrt_flexspi_nor_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_IMXRT_IMXRT1064_EVK_SRC_IMXRT1064_EVK_H */
