/****************************************************************************
 * boards/mips/pic32mz/chipkit-wifire/src/chipkit-wifire.h
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

#ifndef __BOARDS_MIPS_PIC32MZ_CHIPKIT_WIFIRE_SRC_CHIPKIT_WIFIRE_H
#define __BOARDS_MIPS_PIC32MZ_CHIPKIT_WIFIRE_SRC_CHIPKIT_WIFIRE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* LEDs *********************************************************************/

/* There are four LEDs on the top side of the board:
 *
 *   LED LD1      - RG6
 *   LED LD2      - RD4
 *   LED LD3      - RB11
 *   LED LD4      - RG15
 *
 * A high output value illuminates the LEDs.
 */

#define GPIO_LED_LD1  (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTG | GPIO_PIN6)
#define GPIO_LED_LD2  (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTD | GPIO_PIN4)
#define GPIO_LED_LD3  (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTB | GPIO_PIN11)
#define GPIO_LED_LD4  (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTG | GPIO_PIN15)

/* The chipKIT Wi-Fire has 2 user push buttons labeled BTN1 and BTN2 on the
 * white side of the board:
 *
 * PIN   Button  Notes
 * ----- ----    -------------------------
 * RA5   BTN1    Sensed low when closed
 * RA4   BTN2    Sensed low when closed
 *
 * The switches have external pull-down resistors. The switches are
 * pulled down and pulled up to +3.3V when pressed.
 */

#define GPIO_BTN1      (GPIO_INPUT | GPIO_INTERRUPT | GPIO_PORTA | GPIO_PIN5)
#define GPIO_BTN2      (GPIO_INPUT | GPIO_INTERRUPT | GPIO_PORTA | GPIO_PIN4)

/* SPI Chip Selects
 *
 * SPI1 is available on pins D5,D7,D35,D36 of the Arduino Shield connectors
 * where you would expect then.  The SPI connector is configured as follows:
 *
 *   Pin J7&10 Board Signal PIC32MZ
 *   --- --    ------------ -------
 *   D5  11    SPI1_SCK     RD1
 *   D36 6     SPI1_MISO    RF1
 *   D35 4     SPI1_MOSI    RC1
 *   D7  15    SPI1_SS      RE9
 */

#define GPIO_ARD_SPI1_CS  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORTE | GPIO_PIN9)

/* SPI2 is available on pins D10-D13 of the Arduino Shield connectors where
 * you would expect then.  The SPI connector is configured as follows:
 *
 *   Pin J7&10 Board Signal PIC32MZ
 *   --- --    ------------ -------
 *   D13 11    SPI2_SCK     RG6
 *   D12 9     SPI2_MISO    RF0
 *   D11 7     SPI2_MOSI    RD11
 *   D10 5     SPI2_SS      RG9
 */

#define GPIO_ARD_SPI2_CS  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORTG | GPIO_PIN9)

/* SPI3 is available on microSD connector as follows:
 *
 *   Pin  Board Signal PIC32MZ
 *   ---- ------------ -------
 *   SCK  SPI3_SCK     RB14
 *   SDO  SPI3_MISO    RB10
 *   SDI  SPI3_MOSI    RC4
 *   CS   SPI3_SS      RC3
 */

#define GPIO_SD_SPI3_CS  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORTC | GPIO_PIN3)

/* SPI4 is connected to MRF24WG0MA WiFi module as follows:
 *
 *   Pin  Board Signal PIC32MZ
 *   ---- ------------ -------
 *   SCK  SPI4_SCK     RD10
 *   SDO  SPI4_MISO    RF5
 *   SDI  SPI4_MOSI    RG0
 *   CS   SPI4_SS      RD9
 */

#define GPIO_WIFI_SPI4_CS  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORTD | GPIO_PIN9)

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

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_MIPS_PIC32MZ_CHIPKIT_WIFIRE_SRC_CHIPKIT_WIFIRE_H */
