/****************************************************************************
 * boards/mips/pic32mz/chipkit-wifire/include/board.h
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

#ifndef __BOARDS_MIPS_PIC32MZ_CHIPKIT_WIFIRE_INCLUDE_BOARD_H
#define __BOARDS_MIPS_PIC32MZ_CHIPKIT_WIFIRE_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdbool.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* Crystal frequencies
 *
 * - A 24 MHz oscillator circuit (IC7) is connected to the on-board
 *   microcontroller. This oscillator circuit functions as the controller’s
 *   primary oscillator.
 * - The starter kit also has provisions for an external secondary 32 kHz
 *   oscillator (X2); however, this is not populated.
 */

#define BOARD_POSC_FREQ        24000000  /* Primary OSC XTAL frequency (IC7, 24MHz) */
#define BOARD_SOSC_FREQ        32000     /* Secondary OSC XTAL frequency (X2, 32KHz) */

/* Oscillator modes.
 *
 * - BOARD_POSC_ECMODE:  An external oscillator is connected to OSC1/OSC2
 * - BOARD_POSC_HSMODE:  An external crystal or resonator is connected to
 *                       OSC1/OSC2
 */

#define BOARD_FNOSC_SPLL       1         /* Use system PLL */
#define BOARD_POSC_ECMODE      1         /* External clock (EC) mode */
#define BOARD_POSC_SWITCH      1         /* Enable clock switching */
#undef  BOARD_POSC_FSCM                  /* Disable clock monitoring */

/* PLL configuration and resulting CPU clock.
 * CPU_CLOCK = ((POSC_FREQ / IDIV) * MULT) / ODIV
 */

#define BOARD_PLL_INPUT        BOARD_POSC_FREQ
#define BOARD_PLL_IDIV         3         /* PLL input divider */
#define BOARD_PLL_MULT         50        /* PLL multiplier */
#define BOARD_PLL_ODIV         2         /* PLL output divider */

#define BOARD_CPU_CLOCK        200000000 /* CPU clock: 200MHz = (24MHz / 3) * 50 / 2) */

/* Peripheral clocks */

/* PBCLK1
 *   Peripherals: OSC2 pin
 *
 * NOTES:
 *   - PBCLK1 is used by system modules and cannot be turned off
 *   - PBCLK1 divided by 2 is available on the OSC2 pin in certain clock
 *     modes.
 */

#define BOARD_PB1DIV           5         /* Divider = 5 */
#define BOARD_PBCLK1           40000000  /* PBCLK1 frequency = 200MHz/5 = 40MHz */

/* PBCLK2
 *   Peripherals: PMP, I2C, UART, SPI
 */

#define BOARD_PBCLK2_ENABLE    1         /* Enable PBCLK2 */
#define BOARD_PB2DIV           2         /* Divider = 2 */
#define BOARD_PBCLK2           100000000 /* PBCLK2 frequency = 200MHz/2 = 100MHz */

/* PBCLK3
 *   Peripherals: ADC, Comparator, Timers, Output Compare, Input Compare
 *
 * NOTES:
 *   - Timer 1 uses SOSC
 */

#define BOARD_PBCLK3_ENABLE    1         /* Enable PBCLK3 */
#define BOARD_PB3DIV           4         /* Divider = 4 */
#define BOARD_PBCLK3           50000000  /* PBCLK3 frequency = 200MHz/4 = 50MHz */

/* PBCLK4
 *   Peripherals: Ports
 */

#define BOARD_PBCLK4_ENABLE    1         /* Enable PBCLK4 */
#define BOARD_PB4DIV           2         /* Divider = 2 */
#define BOARD_PBCLK4           100000000 /* PBCLK4 frequency = 200MHz/2 = 100MHz */

/* PBCLK5
 *   Peripherals: Flash, Crypto, RND, USB, Ethernet, SQI
 *
 * NOTES:
 *   - PBCLK5 is used to fetch data from/to the Flash Controller, while the
 *     FRC clock is used for programming
 */

#define BOARD_PBCLK5_ENABLE    1         /* Enable PBCLK5 */
#define BOARD_PB5DIV           2         /* Divider = 2 */
#define BOARD_PBCLK5           100000000 /* PBCLK5 frequency = 200MHz/2 = 100MHz */

/* PBCLK6
 *   Peripherals:
 */

#undef BOARD_PBCLK6_ENABLE

/* PBCLK7
 *   Peripherals:  CPU, Deadman timer
 */

#undef BOARD_PBCLK7_ENABLE

/* PBCLK8
 *   Peripherals: EBI
 */

#undef BOARD_PBCLK8_ENABLE

/* Watchdog pre-scaler (re-visit) */

#define BOARD_WD_PRESCALER     1048576   /* Watchdog pre-scaler */

/* Ethernet MII clocking.
 *
 * The clock divider used to create the MII Management Clock (MDC).  The MIIM
 * module uses the PBCLK5 as an input clock.  According to the IEEE 802.3
 * Specification this should be no faster than 2.5 MHz. However, some PHYs
 * support clock rates up to 12.5 MHz.
 */

#define BOARD_EMAC_MIIM_DIV    40        /* Ideal: 100MHz/40 = 2.5MHz */

/* LED definitions **********************************************************/

/* There are four LEDs on the top side of the board:
 * controlled by software:
 *
 *   LED LD1      - RG6
 *   LED LD2      - RD4
 *   LED LD3      - RB11
 *   LED LD4      - RG15
 *
 * A high output value illuminates the LEDs.
 */

/* LED index values for use with board_userled(): */

#  define BOARD_LED_LD1   0
#  define BOARD_LED_LD2   1
#  define BOARD_LED_LD3   2
#  define BOARD_LED_LD4   3
#  define BOARD_NLEDS     4

/* LED bits for use with board_userled_all() */

#  define BOARD_LED_LD1_BIT (1 << BOARD_LED_LD1)
#  define BOARD_LED_LD2_BIT (1 << BOARD_LED_LD2)
#  define BOARD_LED_LD3_BIT (1 << BOARD_LED_LD3)
#  define BOARD_LED_LD4_BIT (1 << BOARD_LED_LD4)

/* These LEDs are available to the application and are all available to the
 * application unless CONFIG_ARCH_LEDS is defined.  In that case, the usage
 * by the board port is defined in include/board.h and
 * src/pic32mz_autoleds.c. The LEDs are used to encode OS-related events
 * as follows:
 *
 *      SYMBOL                MEANING                    LED STATE
 *                                                     A   B   C   D
 *      ----------------      ----------------------- --- --- --- ---
 */

#define LED_STARTED      0 /* NuttX has been started  ON  OFF OFF OFF */
#define LED_HEAPALLOCATE 1 /* Heap has been allocated OFF ON  OFF OFF */
#define LED_IRQSENABLED  2 /* Interrupts enabled      OFF OFF ON  OFF */
#define LED_STACKCREATED 3 /* Idle stack created      OFF OFF OFF ON  */
#define LED_INIRQ        4 /* In an interrupt         ON  ON  ON  ON  */
#define LED_SIGNAL       4 /* In a signal handler     ON  ON  ON  ON  */
#define LED_ASSERTION    4 /* An assertion failed     ON  ON  ON  ON  */
#define LED_PANIC        4 /* The system has crashed  ON  ON  ON  ON  */
#undef  LED_IDLE           /* MCU is is sleep mode    ---- Not used - */

/* Switch definitions *******************************************************/

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

#define BUTTON_BTN1            0
#define BUTTON_BTN2            1
#define NUM_BUTTONS            2

#define BUTTON_BTN1_BIT        (1 << BUTTON_BTN1)
#define BUTTON_BTN2_BIT        (1 << BUTTON_BTN2)

/* UARTS ********************************************************************/

#define BOARD_U4RX_PPS  U4RXR_RPF2
#define BOARD_U4TX_PPS  U4TX_RPF8R

/* SPI **********************************************************************/

/* SPI1 is available on pins D5,D7,D35,D36 of the Arduino Shield connectors
 * where you would expect then.  The SPI connector is configured as follows:
 *
 *   Pin J7&10 Board Signal PIC32MZ
 *   --- --    ------------ -------
 *   D5  11    SPI1_SCK     RD1
 *   D36 6     SPI1_MISO    RF1
 *   D35 4     SPI1_MOSI    RC1
 *   D7  15    SPI1_SS      RE9
 *
 * SPI2 is available on pins D10-D13 of the Arduino Shield connectors where
 * you would expect then.  The SPI connector is configured as follows:
 *
 *   Pin J7&10 Board Signal PIC32MZ
 *   --- --    ------------ -------
 *   D13 11    SPI2_SCK     RG6
 *   D12 9     SPI2_MISO    RF0
 *   D11 7     SPI2_MOSI    RD11
 *   D10 5     SPI2_SS      RG9
 *
 * SPI3 is available on microSD connector as follows:
 *
 *   Pin  Board Signal PIC32MZ
 *   ---- ------------ -------
 *   SCK  SPI3_SCK     RB14
 *   SDO  SPI3_MISO    RB10
 *   SDI  SPI3_MOSI    RC4
 *   CS   SPI3_SS      RC3
 *
 * SPI4 is connected to MRF24WG0MA WiFi module as follows:
 *
 *   Pin  Board Signal PIC32MZ
 *   ---- ------------ -------
 *   SCK  SPI4_SCK     RD10
 *   SDO  SPI4_MISO    RF5
 *   SDI  SPI4_MOSI    RG0
 *   CS   SPI4_SS      RD9
 *
 * Chip select pin definitions are provided in
 * boards/mips/chipkit-wifire/src/chipkit-wifire.h.
 *
 * CLK (output) pins have no alternative pin configurations.
 */

#define BOARD_SDI1_PPS  SDI1R_RPF1
#define BOARD_SDO1_PPS  SDO1_RPC1R

#define BOARD_SDI2_PPS  SDI2R_RPF0
#define BOARD_SDO2_PPS  SDO2_RPD11R

#define BOARD_SDI3_PPS  SDI3R_RPB10
#define BOARD_SDO3_PPS  SDO3_RPC4R

#define BOARD_SDI4_PPS  SDI4R_RPF5
#define BOARD_SDO4_PPS  SDO4_RPG0R

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_MIPS_PIC32MZ_CHIPKIT_WIFIRE_INCLUDE_BOARD_H */
