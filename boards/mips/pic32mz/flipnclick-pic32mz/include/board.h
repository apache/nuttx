/****************************************************************************
 * boards/mips/pic32mz/flipnclick-pic32mz/include/board.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************/

#ifndef __BOARDS_MIPS_PIC32MZ_FLIPNCLICK_PIC32MZ_INCLUDE_BOARD_H
#define __BOARDS_MIPS_PIC32MZ_FLIPNCLICK_PIC32MZ_INCLUDE_BOARD_H

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

/* REVISIT:  The PIC32MZ2048EFH100 is capable of operating at 252MHz */

/* Crystal frequencies
 *
 * - A 24 MHz oscillator circuit (Y4) is connected to the on-board
 *   microcontroller. This oscillator circuit functions as the controller’s
 *   primary oscillator. Depending on which is populated on the starter kit
 *   board, a 24 MHz crystal (Y1) may be used instead of Y4.
 * - The starter kit also has provisions for an external secondary 32 kHz
 *   oscillator (Y2); however, this is not populated.
 */

#define BOARD_POSC_FREQ        24000000  /* Primary OSC XTAL frequency (Y4, 24MHz) */
#define BOARD_SOSC_FREQ        32000     /* Secondary OSC XTAL frequency (Y2, 32KHz) */

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
 *   Peripherals: Flash, Crypto, RND, USB, CAN, Ethernet, SQI
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

#ifdef CONFIG_ARCH_LEDS

/* LED index values for use with board_userled(): */

#  define BOARD_LED_A     0
#  define BOARD_LED_B     1
#  define BOARD_LED_C     2
#  define BOARD_LED_D     3
#  define BOARD_NLEDS     4

/* LED bits for use with board_userled_all() */

#  define BOARD_LED_A_BIT (1 << BOARD_LED_A)
#  define BOARD_LED_B_BIT (1 << BOARD_LED_B)
#  define BOARD_LED_C_BIT (1 << BOARD_LED_C)
#  define BOARD_LED_D_BIT (1 << BOARD_LED_D)
#else

/* LED index values for use with board_userled(): */

#  define BOARD_LED_L     0
#  define BOARD_LED_A     1
#  define BOARD_LED_B     2
#  define BOARD_LED_C     3
#  define BOARD_LED_D     4
#  define BOARD_NLEDS     5

/* LED bits for use with board_userled_all() */

#  define BOARD_LED_L_BIT (1 << BOARD_LED_L)
#  define BOARD_LED_A_BIT (1 << BOARD_LED_A)
#  define BOARD_LED_B_BIT (1 << BOARD_LED_B)
#  define BOARD_LED_C_BIT (1 << BOARD_LED_C)
#  define BOARD_LED_D_BIT (1 << BOARD_LED_D)
#endif

/* These LEDs are available to the application and are all available to the
 * application unless CONFIG_ARCH_LEDS is defined.  In that case, the usage
 * by the board port is defined in include/board.h and src/sam_autoleds.c.
 * The LEDs are used to encode OS-related events as follows:
 *
 *      SYMBOL                MEANING                        LED STATE
 *                                                     L   A   B   C   D
 *      ----------------      ----------------------- --- --- --- --- ---
 */

#define LED_STARTED      0 /* NuttX has been started  OFF ON  OFF OFF OFF */
#define LED_HEAPALLOCATE 1 /* Heap has been allocated OFF OFF ON  OFF OFF */
#define LED_IRQSENABLED  2 /* Interrupts enabled      OFF OFF OFF ON  OFF */
#define LED_STACKCREATED 3 /* Idle stack created      OFF OFF OFF OFF ON  */
#define LED_INIRQ        4 /* In an interrupt         GLO N/C N/C N/C N/C */
#define LED_SIGNAL       4 /* In a signal handler     GLO N/C N/C N/C N/C */
#define LED_ASSERTION    4 /* An assertion failed     GLO N/C N/C N/C N/C */
#define LED_PANIC        4 /* The system has crashed  2Hz N/C N/C N/C N/C */
#undef  LED_IDLE           /* MCU is is sleep mode    ---- Not used ----- */

/* Thus if LED L is faintly glowing and all other LEDs are off (except LED D
 * which was left on but is no longer controlled by NuttX and so may be
 * in any state),
 * NuttX has successfully booted and is, apparently, running normally and
 * taking interrupts.
 * If any of LEDs A-D are statically set, then NuttX failed to boot
 * and the LED indicates the initialization phase where the failure occurred.
 * If LED L is flashing at approximately 2Hz, then a fatal error has been
 * detected and the system has halted.
 *
 * NOTE: After booting, LEDs A-D are no longer used by the system and may be
 * controlled the application.
 */

/* Switch definitions *******************************************************/

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

#define BUTTON_T1              0
#define BUTTON_T2              1
#define NUM_BUTTONS            2

#define BUTTON_T1_BIT          (1 << BUTTON_T1)
#define BUTTON_T2_BIT          (1 << BUTTON_T2)

/* UARTS ********************************************************************/

/* Convenient U[S]ARTs that may be used as the Serial console include:
 *
 * 1) An Arduino Serial Shield.  The RX and TX pins are available on the
 *    Arduino connector D0 and D1 pins, respectively.  These are connected
 *    to UART5, UART5_RX and UART5_TX which are RD14 and RD15, respectively.
 *
 * 2) Mikroe Click Serial Shield.  There are four Click bus connectors with
 *    serial ports available as follows:
 *
 *    Click A:  UART4 UART4_RX and UART4_TX which are RG9 and RE3, respectively.
 *    Click B:  UART3 UART3_RX and UART3_TX which are RF0 and RF1, respectively.
 *    Click C:  UART1 UART1_RX and UART1_TX which are RC1 and RE5, respectively.
 *    Click D:  UART2 UART2_RX and UART2_TX which are RC3 and RC2, respectively.
 */

#define BOARD_U1RX_PPS  U1RXR_RPC1
#define BOARD_U1TX_PPS  U1TX_RPE5R

#define BOARD_U2RX_PPS  U2RXR_RPC3
#define BOARD_U2TX_PPS  U2TX_RPC2R

#define BOARD_U3RX_PPS  U3RXR_RPF0
#define BOARD_U3TX_PPS  U3TX_RPF1R

#define BOARD_U4RX_PPS  U4RXR_RPG9
#define BOARD_U4TX_PPS  U4TX_RPE3R

#define BOARD_U5RX_PPS  U5RXR_RPD14
#define BOARD_U5TX_PPS  U5TX_RPD15R

/* SPI **********************************************************************/

/* SPI3 is available on pins D10-D13 of the Arduino Shield connectors where
 * you would expect then.  The SPI connector is configured as follows:
 *
 *   Pin J1 Board Signal PIC32MZ
 *   --- -- ------------ -------
 *   D10 8  SPI3_SCK     RB14
 *   D11 7  SPI3_MISO    RB9
 *   D12 6  SPI3_MOSI    RB10
 *   D13 5  SPI3_SS      RB9
 *
 * SPI1 and SPI2 are also available on the mikroBUS Click connectors (in
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
 *
 * Chip select pin definitions are provided in
 * boards/mips/flipnclick-pic32mz/src/flipnclick-pic32mz.h.
 *
 * CLK (output) pins have no alternative pin configurations.
 */

#define BOARD_SDI1_PPS  SDI1R_RPD2
#define BOARD_SDO1_PPS  SDO1_RPD3R

#define BOARD_SDI2_PPS  SDI2R_RPC4
#define BOARD_SDO2_PPS  SDO2_RPB5R

#define BOARD_SDI3_PPS  SDI3R_RPB9
#define BOARD_SDO3_PPS  SDO3_RPB9R

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
#endif /* __BOARDS_MIPS_PIC32MZ_FLIPNCLICK_PIC32MZ_INCLUDE_BOARD_H */
