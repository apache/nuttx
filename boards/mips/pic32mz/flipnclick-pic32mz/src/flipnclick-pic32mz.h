/****************************************************************************
 * boards/mips/pic32mz/flipnclick-pic32mz/src/flipnclick-pic32mz.h
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

#ifndef __BOARDS_MIPS_PIC32MZ_FLIPNCLICK_PIC32MZ_SRC_FLIPNCLICK_PIC32MZ_H
#define __BOARDS_MIPS_PIC32MZ_FLIPNCLICK_PIC32MZ_SRC_FLIPNCLICK_PIC32MZ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define HAVE_SSD1306 1

/* The SSD1306 LCD must be selected, installed on the Flip&Click, and must
 * be configured to use the SPI interface.
 */

#if !defined(CONFIG_FLIPNCLICK_PIC32MZ_SSD1306) || \
    !defined(CONFIG_LCD_SSD1306_SPI)
#  undef HAVE_SSD1306
#  undef CONFIG_FLIPNCLICK_PIC32MZ_SSD1306
#  undef CONFIG_FLIPNCLICK_PIC32MZ_SSD1306_MBA
#  undef CONFIG_FLIPNCLICK_PIC32MZ_SSD1306_MBB
#  undef CONFIG_FLIPNCLICK_PIC32MZ_SSD1306_MBC
#  undef CONFIG_FLIPNCLICK_PIC32MZ_SSD1306_MBD
#endif

/* LEDs *********************************************************************/

/* There are four LEDs on the top, red side of the board.  Only one can be
 * controlled by software:
 *
 *   LED L      - RB14 (SPI3_SCK)
 *
 * There are also four LEDs on the back, white side of the board:
 *
 *   LED A      - RA6
 *   LED B      - RA7
 *   LED C      - RE0
 *   LED D      - RE1
 *
 * A high output value illuminates the LEDs.
 */

#define GPIO_LED_L  (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTB | GPIO_PIN14)
#define GPIO_LED_A  (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTA | GPIO_PIN6)
#define GPIO_LED_B  (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTA | GPIO_PIN7)
#define GPIO_LED_C  (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTE | GPIO_PIN0)
#define GPIO_LED_D  (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTE | GPIO_PIN1)

/* The Flip&Click PIC32MZ has 2 user push buttons labeled T1 and T2 on the
 * white side of the board:
 *
 * PIN   LED  Notes
 * ----- ---- -------------------------
 * RD10  T1   Sensed low when closed
 * RD11  T2   Sensed low when closed
 *
 * The switches have external pull-up resistors. The switches are pulled high
 * (+3.3V) and grounded when pressed.
 */

#define GPIO_T1      (GPIO_INPUT | GPIO_INTERRUPT | GPIO_PORTD | GPIO_PIN10)
#define GPIO_T2      (GPIO_INPUT | GPIO_INTERRUPT | GPIO_PORTD | GPIO_PIN11)

/* SPI Chip Selects
 *
 * SPI3 is available on pins D10-D13 of the Arduino Shield connectors where
 * you would expect them.  The SPI connector is configured as follows:
 *
 *   Pin J1 Board Signal PIC32MZ
 *   --- -- ------------ -------
 *   D10 8  SPI3_SCK     RB14
 *   D10 7  SPI3_MISO    RB9
 *   D11 6  SPI3_MOSI    RB10
 *   D13 5  SPI3_SS      RB9
 */

#define GPIO_ARD_CS  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORTB | GPIO_PIN14)

/* SPI1 and SPI2 are also available on the mikroBUS Click connectors (in
 * addition to 5V and GND).  The connectivity between connectors A and B and
 * between C and D differs only in the chip select pin:
 *
 *   MikroBUS A:                 MikroBUS B:
 *   Pin  Board Signal PIC32MZ  Pin  Board Signal PIC32MZ
 *   ---- ------------ -------  ---- ------------ -------
 *   CS   SPI2_SS1     RA0      CS   SPI2_SS0     RE4
 *   SCK  SPI2_SCK     RG6      SCK  SPI2_SCK     RG6
 *   MISO SPI2_MISO    RC4      MISO SPI2_MISO    RC4
 *   MOSI SPI2_MOSI    RB5      MOSI SPI2_MOSI    RB5
 *
 *   MikroBUS C:                 MikroBUS D:
 *   Pin  Board Signal PIC32MZ  Pin  Board Signal PIC32MZ
 *   ---- ------------ -------  ---- ------------ -------
 *   CS   SPI1_SS0     RD12     CS   SPI1_SS1     RD13
 *   SCK  SPI1_SCK     RD1      SCK  SPI1_SCK     RD1
 *   MISO SPI1_MISO    RD2      MISO SPI1_MISO    RD2
 *   MOSI SPI1_MOSI    RD3      MOSI SPI1_MOSI    RD3
 */

#define GPIO_MBA_CS  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORTA | GPIO_PIN0)
#define GPIO_MBB_CS  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORTE | GPIO_PIN4)
#define GPIO_MBC_CS  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORTD | GPIO_PIN12)
#define GPIO_MBD_CS  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORTD | GPIO_PIN13)

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
 *   MikroBUS A:                 MikroBUS B:
 *   Pin  Board Signal PIC32MZ  Pin  Board Signal PIC32MZ
 *   ---- ------------ -------  ---- ------------ -------
 *   RST  RST4         RE2      RST  RST3         RG13
 *   DC   INT4         RD9      DC   INT3         RG1
 *
 *   MikroBUS C:                 MikroBUS D:
 *   Pin  Board Signal PIC32MZ  Pin  Board Signal PIC32MZ
 *   ---- ------------ -------  ---- ------------ -------
 *   RST  RST1         RG14     RST  RST2         RG12
 *   DC   INT1         RD5      DC   INT2         RD4
 */

#if defined(CONFIG_FLIPNCLICK_PIC32MZ_SSD1306_MBA)
#  ifndef CONFIG_PIC32MZ_SPI2
#    error "The OLED driver requires CONFIG_PIC32MZ_SPI2 in the configuration"
#  endif

#  define SSD1306_SPI_BUS  2
#  define GPIO_SSD1306_CS  GPIO_MBA_CS
#  define GPIO_SSD1306_RST (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTE | GPIO_PIN2)
#  define GPIO_SSD1306_DC  (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTD | GPIO_PIN9)

#elif defined(CONFIG_FLIPNCLICK_PIC32MZ_SSD1306_MBB)
#  ifndef CONFIG_PIC32MZ_SPI2
#    error "The OLED driver requires CONFIG_PIC32MZ_SPI2 in the configuration"
#  endif

#  define SSD1306_SPI_BUS  2
#  define GPIO_SSD1306_CS  GPIO_MBB_CS
#  define GPIO_SSD1306_RST (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTG | GPIO_PIN13)
#  define GPIO_SSD1306_DC  (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTG | GPIO_PIN1)

#elif defined(CONFIG_FLIPNCLICK_PIC32MZ_SSD1306_MBC)
#  ifndef CONFIG_PIC32MZ_SPI1
#    error "The OLED driver requires CONFIG_PIC32MZ_SPI1 in the configuration"
#  endif

#  define SSD1306_SPI_BUS  1
#  define GPIO_SSD1306_CS  GPIO_MBC_CS
#  define GPIO_SSD1306_RST (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTG | GPIO_PIN14)
#  define GPIO_SSD1306_DC  (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTD | GPIO_PIN5)

#elif defined(CONFIG_FLIPNCLICK_PIC32MZ_SSD1306_MBD)
#  ifndef CONFIG_PIC32MZ_SPI1
#    error "The OLED driver requires CONFIG_PIC32MZ_SPI1 in the configuration"
#  endif

#  define SSD1306_SPI_BUS  1
#  define GPIO_SSD1306_CS  GPIO_MBD_CS
#  define GPIO_SSD1306_RST (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTG | GPIO_PIN12)
#  define GPIO_SSD1306_DC  (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTD | GPIO_PIN4)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: pic32mz_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PCB Logic board.
 *
 ****************************************************************************/

#ifdef CONFIG_PIC32MZ_SPI
void weak_function pic32mz_spidev_initialize(void);
#endif

/****************************************************************************
 * Name: pic32mz_led_initialize
 *
 * Description:
 *   Configure on-board LEDs if LED support has been selected.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void pic32mz_led_initialize(void);
#endif

/****************************************************************************
 * Name: pic32mz_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int pic32mz_bringup(void);

/****************************************************************************
 * Name: pic32mz_graphics_setup
 *
 * Description:
 *   Called by either NX initialization logic (via board_graphics_setup) or
 *   directly from the board bring-up logic in order to configure the
 *   SSD1306 OLED.
 *
 ****************************************************************************/

#ifdef HAVE_SSD1306
struct lcd_dev_s;  /* Forward reference */
struct lcd_dev_s *pic32mz_graphics_setup(unsigned int devno);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_MIPS_PIC32MZ_FLIPNCLICK_PIC32MZ_SRC_FLIPNCLICK_PIC32MZ_H */
