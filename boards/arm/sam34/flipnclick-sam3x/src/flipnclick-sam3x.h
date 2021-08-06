/****************************************************************************
 * boards/arm/sam34/flipnclick-sam3x/src/flipnclick-sam3x.h
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

#ifndef __BOARDS_ARM_SAM34_FLIPNCLICK_SAM3X_SRC_FLIPNCLICK_SAM3X_H
#define __BOARDS_ARM_SAM34_FLIPNCLICK_SAM3X_SRC_FLIPNCLICK_SAM3X_H

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

#define HAVE_SSD1306 1

/* The SSD1306 LCD must be selected, installed on the Flip&Click, and must
 * be configured to use the SPI interface.
 */

#if !defined(CONFIG_FLIPNCLICK_SAM3X_SSD1306) || \
    !defined(CONFIG_LCD_SSD1306_SPI)
#  undef HAVE_SSD1306
#  undef CONFIG_FLIPNCLICK_SAM3X_SSD1306
#  undef CONFIG_FLIPNCLICK_SAM3X_SSD1306_MBA
#  undef CONFIG_FLIPNCLICK_SAM3X_SSD1306_MBB
#  undef CONFIG_FLIPNCLICK_SAM3X_SSD1306_MBC
#  undef CONFIG_FLIPNCLICK_SAM3X_SSD1306_MBD
#endif

/* There are four LEDs on the top, blue side of the board.  Only one can be
 * controlled by software:
 *
 *   LED L - PB27 (PWM13)
 *
 * There are also four LEDs on the back, white side of the board:
 *
 *   LED A - PC6
 *   LED B - PC5
 *   LED C - PC7
 *   LED D - PC8
 *
 * A high output value illuminates the LEDs.
 */

#define GPIO_LED_L   (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                      GPIO_PORT_PIOB | GPIO_PIN27)
#define GPIO_LED_A   (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                      GPIO_PORT_PIOC | GPIO_PIN6)
#define GPIO_LED_B   (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                      GPIO_PORT_PIOC | GPIO_PIN5)
#define GPIO_LED_C   (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                      GPIO_PORT_PIOC | GPIO_PIN7)
#define GPIO_LED_D   (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                      GPIO_PORT_PIOC | GPIO_PIN8)

/* SPI chip select pins.
 *
 * SPI0 is available on the Arduino compatible SPI connector (but no SPI is
 * available on pins D10-D13 of the main Arduino Shield connectors where
 * you might expect then).  The SPI connector is configured as follows:
 *
 *   Pin Board Signal SAM3X  Pin Board Signal SAM3X
 *   --- ------------ -----  --- ------------ -----
 *    1  SPI0_MISO    PA25    2  VCC-5V       N/A
 *    3  SPI0_SCK     PA27    4  SPI0_MOSI    PA26
 *    5  MRST         NRSTB   6  GND          N/A
 *
 * SPI0 is also available on each of the mikroBUS Click connectors (in
 * addition to 5V and GND).  The connectivity differs only in the chip
 * select pin:
 *
 *   MikroBUS A:              MikroBUS B:
 *   Pin  Board Signal SAM3X  Pin  Board Signal SAM3X
 *   ---- ------------ -----  ---- ------------ -----
 *   CS   SPI0_CS0     PA28   CS   PA29         PA29
 *   SCK  SPI0_SCK     PA27   SCK  SPI0_SCK     PA27
 *   MISO SPI0_MISO    PA25   MISO SPI0_MISO    PA25
 *   MOSI SPI0_MOSI    PA26   MOSI SPI0_MOSI    PA26
 *
 *   MikroBUS C:              MikroBUS D:
 *   Pin  Board Signal SAM3X  Pin  Board Signal SAM3X
 *   ---- ------------ -----  ---- ------------ -----
 *   CS   SPI0_CS2     PB21   CS   SPI0_CS3     PB23
 *   SCK  SPI0_SCK     PA27   SCK  SPI0_SCK     PA27
 *   MISO SPI0_MISO    PA25   MISO SPI0_MISO    PA25
 *   MOSI SPI0_MOSI    PA26   MOSI SPI0_MOSI    PA26
 */

#define GPIO_MBA_CS  (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
                      GPIO_PORT_PIOA | GPIO_PIN28)
#define MBA_CSNUM    0

#define GPIO_MBB_CS  (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
                      GPIO_PORT_PIOA | GPIO_PIN29)
#define MBB_CSNUM    1

#define GPIO_MBC_CS  (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
                      GPIO_PORT_PIOB | GPIO_PIN21)
#define MBC_CSNUM    2

#define GPIO_MBD_CS  (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
                      GPIO_PORT_PIOB | GPIO_PIN23)
#define MBD_CSNUM    3

/* SSD1306 OLED
 *
 * The HiletGo is a 128x64 OLED that can be driven either via SPI or I2C (SPI
 * is the default and is what is used here).  I have mounted the OLED on a
 * proto click board.  The OLED is connected as follows:
 *
 * OLED  ALIAS       DESCRIPTION   PROTO CLICK
 * ----- ----------- ------------- -----------------
 *  GND              Ground        GND
 *  VCC              Power Supply  5V  (3-5V)
 *  D0   SCL,CLK,SCK Clock         SCK
 *  D1   SDA,MOSI    Data          MOSI,SDI
 *  RES  RST,RESET   Reset         RST (GPIO OUTPUT)
 *  DC   AO          Data/Command  INT (GPIO OUTPUT)
 *  CS               Chip Select   CS  (GPIO OUTPUT)
 *
 * NOTE that this is a write-only display (MOSI only)!
 *
 *   MikroBUS A:              MikroBUS B:
 *   Pin  Board Signal SAM3X  Pin  Board Signal SAM3X
 *   ---- ------------ -----  ---- ------------ -------
 *   RST  RSTA         PC1    RST  RSTB         PC2
 *   DC   INTA         PD1    DC   INTB         PD2
 *
 *   MikroBUS C:              MikroBUS D:
 *   Pin  Board Signal SAM3X  Pin  Board Signal SAM3X
 *   ---- ------------ -----  ---- ------------ -------
 *   RST  RSTC         PC3    RST  RSTD         PC4
 *   DC   INTC         PD3    DC   INTD         PD6
 */

#if defined(CONFIG_FLIPNCLICK_SAM3X_SSD1306_MBA)
#  ifndef CONFIG_SAM34_SPI0
#    error "The OLED driver requires CONFIG_SAM34_SPI0 in the configuration"
#  endif

#  define SSD1306_SPI_BUS  0
#  define SSD1306_CSNUM    MBA_CSNUM
#  define GPIO_SSD1306_CS  GPIO_MBA_CS
#  define GPIO_SSD1306_RST (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                            GPIO_PORT_PIOC | GPIO_PIN1)
#  define GPIO_SSD1306_DC  (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                            GPIO_PORT_PIOD | GPIO_PIN1)

#elif defined(CONFIG_FLIPNCLICK_SAM3X_SSD1306_MBB)
#  ifndef CONFIG_SAM34_SPI0
#    error "The OLED driver requires CONFIG_SAM34_SPI0 in the configuration"
#  endif

#  define SSD1306_SPI_BUS  0
#  define SSD1306_CSNUM    MBB_CSNUM
#  define GPIO_SSD1306_CS  GPIO_MBB_CS
#  define GPIO_SSD1306_RST (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                            GPIO_PORT_PIOC | GPIO_PIN2)
#  define GPIO_SSD1306_DC  (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                            GPIO_PORT_PIOD | GPIO_PIN2)

#elif defined(CONFIG_FLIPNCLICK_SAM3X_SSD1306_MBC)
#  ifndef CONFIG_SAM34_SPI0
#    error "The OLED driver requires CONFIG_SAM34_SPI0 in the configuration"
#  endif

#  define SSD1306_SPI_BUS  0
#  define SSD1306_CSNUM    MBC_CSNUM
#  define GPIO_SSD1306_CS  GPIO_MBC_CS
#  define GPIO_SSD1306_RST (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                            GPIO_PORT_PIOC | GPIO_PIN3)
#  define GPIO_SSD1306_DC  (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                            GPIO_PORT_PIOD | GPIO_PIN3)

#elif defined(CONFIG_FLIPNCLICK_SAM3X_SSD1306_MBD)
#  ifndef CONFIG_SAM34_SPI0
#    error "The OLED driver requires CONFIG_SAM34_SPI0 in the configuration"
#  endif

#  define SSD1306_SPI_BUS  0
#  define SSD1306_CSNUM    MBD_CSNUM
#  define GPIO_SSD1306_CS  GPIO_MBD_CS
#  define GPIO_SSD1306_RST (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                            GPIO_PORT_PIOC | GPIO_PIN4)
#  define GPIO_SSD1306_DC  (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                            GPIO_PORT_PIOD | GPIO_PIN6)
#endif

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
 * Name: sam_bringup
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

int sam_bringup(void);

/****************************************************************************
 * Name: sam_graphics_setup
 *
 * Description:
 *   Called by either NX initialization logic (via board_graphics_setup) or
 *   directly from the board bring-up logic in order to configure the
 *   SSD1306 OLED.
 *
 ****************************************************************************/

#ifdef HAVE_SSD1306
struct lcd_dev_s;  /* Forward reference */
FAR struct lcd_dev_s *sam_graphics_setup(unsigned int devno);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_SAM34_FLIPNCLICK_SAM3X_SRC_FLIPNCLICK_SAM3X_H */
