/****************************************************************************
 * boards/arm/imxrt/imxrt1170-evk/src/imxrt1170-evk.h
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

/* Copyright 2022 NXP */

#ifndef __BOARDS_ARM_IMXRT_IMXRT1170_EVK_SRC_IMXRT1170_EVK_H
#define __BOARDS_ARM_IMXRT_IMXRT1170_EVK_SRC_IMXRT1170_EVK_H

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

/* The i.MX RT1170 EVK has a connector (J48) for RK055AHD091-CTG or
 * RK055IQH091-CTG LCDs.  The RK055AHD091-CTG comes with the GT911
 * touchscreen driver chip integrated.  GT911 is connected to the LPI2C5 bus.
 */

/* LPI2C address of the GT911 touchscreen driver chip */

#define GT911_I2C_ADDRESS  0x5d

/* i.MX RT 1176 GPIO Pin Definitions ****************************************/

/* LEDs */

/* There are five LED status indicators located on the EVK Board.
 * The functions of these LEDs include:
 *
 *   - Main Power Supply (D16)
 *     Green: DC 5V main supply is normal.
 *     Red:   J43 input voltage is over 5.6V.
 *     Off:   The board is not powered.
 *   - Reset LED - Red (D7)
 *   - OpenSDA LED - Red (D5)
 *   - USER LED 1 - Green (D6)
 *   - USER LED 2 - Red (D34)
 *
 * Only two LEDs, D6 & D34, are under software control.
 */

#define GPIO_LED1  (GPIO_OUTPUT | IOMUX_LED_DEFAULT | GPIO_OUTPUT_ZERO | \
                    GPIO_PORT3 | GPIO_PIN3)   /* GPIO_AD_04 */
#define GPIO_LED2  (GPIO_OUTPUT | IOMUX_LED_DEFAULT | GPIO_OUTPUT_ZERO | \
                    GPIO_PORT3 | GPIO_PIN25)  /* GPIO_AD_26 */

#define LED_DRIVER_PATH "/dev/userleds"

/* Buttons ******************************************************************/

/* The IMXRT1170-EVK board has seven (sets of) switches or buttons:
 *
 *   - External boot configuration switches (SW1)
 *   - External boot configuration switches (SW2)
 *   - MCU reset button (SW3)
 *   - System Reset Button / POR_BUTTON (SW4)
 *   - 5V power switch (SW5)
 *   - CPU ONOFF Button (SW6)
 *   - CPU Wakeup Button / USER_BUTTON (SW7)
 *
 * Only one button, SW7, can be used for user input.
 */

#define GPIO_SW7       (GPIO_INTERRUPT | GPIO_INTBOTH_EDGES | \
                        IOMUX_SW_DEFAULT | GPIO_PORT13 | \
                        GPIO_PIN0)  /* WAKEUP */

#define GPIO_SW7_INT   (_IMXRT_GPIO13_BASE+0)

/* LCD Backlight */

#define GPIO_LCD_BL    (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GPIO_PORT3 | \
                         GPIO_PIN30 | IOMUX_LCD_BL_DEFAULT)  /* GPIO_AD_30 */

/* 10/100 Mbps Ethernet & Gigabit Ethernet */

/* 10/100 Mbps Ethernet Interrupt: GPIO_AD_12
 * Gigabit Ethernet Interrupt: GPIO_DISP_B2_12
 *
 * This pin has a week pull-up within the PHY, is open-drain, and requires
 * an external 1k ohm pull-up resistor (present on the EVK).  A falling
 * edge then indicates a change in state of the PHY.
 */

#define GPIO_ENET_INT  (IOMUX_ENET_INT_DEFAULT | GPIO_OUTPUT | \
                        GPIO_PORT3 | GPIO_PIN11)  /* GPIO_AD_12 */
#define GPIO_ENET_IRQ  IMXRT_IRQ_GPIO3_0_15

#define GPIO_ENET1G_INT (IOMUX_ENET_INT_DEFAULT | \
                         GPIO_PORT5 | GPIO_PIN13)  /* GPIO_DISP_B2_12 */
#define GPIO_ENET1G_IRQ IMXRT_IRQ_GPIO5_13

/* 10/100 Mbps Ethernet Reset:  GPIO_LPSR_12
 * Gigabit Ethernet Reset: GPIO_DISP_B2_13
 *
 * The #RST uses inverted logic.  The initial value of zero will put the
 * PHY into the reset state.
 */

#define GPIO_ENET_RST   (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | \
                         GPIO_PORT6 | GPIO_PIN12 | \
                         IOMUX_ENET_RST_DEFAULT)  /* GPIO_LPSR_12 */

#define GPIO_ENET1G_RST (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | \
                         GPIO_PORT5 | GPIO_PIN14 | \
                         IOMUX_ENET_RST_DEFAULT)  /* GPIO_DISP_B2_13 */

/* LPSPI1 CS:  GPIO_AD_29 */

#define IOMUX_LPSPI1_CS (IOMUX_SLEW_FAST | IOMUX_DRIVE_HIGHSTRENGTH | \
                         IOMUX_PULL_UP)
#define GPIO_LPSPI1_CS  (GPIO_OUTPUT | GPIO_OUTPUT_ONE | \
                         GPIO_PORT3 | GPIO_PIN28 | \
                         IOMUX_LPSPI1_CS)  /* GPIO_AD_29 */

/* LPSPI6 CS:  GPIO_LPSR_09 */

#define IOMUX_LPSPI6_CS (IOMUX_SLEW_FAST | IOMUX_DRIVE_HIGHSTRENGTH | \
                         IOMUX_PULL_UP)
#define GPIO_LPSPI6_CS  (GPIO_OUTPUT | GPIO_OUTPUT_ONE | \
                         GPIO_PORT6 | GPIO_PIN9 | \
                         IOMUX_LPSPI6_CS)  /* GPIO_LPSR_09 */

/* Touchscreen
 *
 * Interrupt line: GPIO_AD_00
 *
 * The interrupt line coming from the GT911 touchscreen driver IC.
 * The touchscreen driver is integrated into the optional RK055AHD091-CTG LCD
 * panel and it's connected to the LPI2C5 bus.
 *
 * Reset line: GPIO_AD_01
 *
 * The reset line is active low.
 */

#define GPIO_GT911_INTR    IMXRT_IRQ_GPIO2_31  /* GPIO_AD_00 */

#define IOMUX_GT911_RST    (IOMUX_PULL_NONE | IOMUX_CMOS_OUTPUT | \
                            IOMUX_DRIVE_40OHM | IOMUX_SPEED_MEDIUM | \
                            IOMUX_SLEW_SLOW)
#define GPIO_GT911_CTRST_N (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | \
                            GPIO_PORT3 | GPIO_PIN0 | \
                            IOMUX_GT911_RST)  /* GPIO_AD_01 */

/* Test Pins ****************************************************************/

#define BOARD_NGPIOIN   0  /* Amount of GPIO Input pins */
#define BOARD_NGPIOOUT  4  /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT  0  /* Amount of GPIO Input w/ Interruption pins */

#define GPIO_GOUT1  (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | IOMUX_GOUT_DEFAULT | \
                     GPIO_PORT5 | GPIO_PIN12)  /* GPIO_DISP_B2_11 */

#define GPIO_GOUT2  (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | IOMUX_GOUT_DEFAULT | \
                     GPIO_PORT5 | GPIO_PIN11)  /* GPIO_DISP_B2_10 */

#define GPIO_GOUT3  (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | IOMUX_GOUT_DEFAULT | \
                     GPIO_PORT5 | GPIO_PIN13)  /* GPIO_DISP_B2_12 */

#define GPIO_GOUT4  (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | IOMUX_GOUT_DEFAULT | \
                     GPIO_PORT3 | GPIO_PIN3)   /* GPIO_AD_04 */

/* USB OTG1 ID Pin： GPIO_AD_09
 * USB OTG2 ID Pin： GPIO_AD_08
 */

#define GPIO_USBOTG1_ID  (GPIO_USB_OTG1_ID_1 | IOMUX_USBOTG_ID_DEFAULT)  /* GPIO_AD_09 */
#define GPIO_USBOTG2_ID  (GPIO_USB_OTG2_ID_1 | IOMUX_USBOTG_ID_DEFAULT)  /* GPIO_AD_08 */

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

#ifdef CONFIG_IMXRT1170_FLEXSPI_NOR
int imxrt_flexspi_nor_initialize(void);
#endif

#ifdef CONFIG_IMXRT1170_FLEXSPI_FRAM
int imxrt_flexspi_fram_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_IMXRT_IMXRT1170_EVK_SRC_IMXRT1170_EVK_H */
