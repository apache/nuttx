/****************************************************************************
 * boards/arm/sam34/flipnclick-sam3x/include/board.h
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

#ifndef __BOARDS_ARM_SAM34_FLIPNCLICK_SAM3X_INCLUDE_BOARD_H
#define __BOARDS_ARM_SAM34_FLIPNCLICK_SAM3X_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#  ifdef CONFIG_SAM34_GPIO_IRQ
#    include <arch/irq.h>
#  endif
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* After power-on reset, the SAM3X device is running on a 4MHz internal RC.
 *  These definitions will configure clocking
 *
 * MAINOSC:  Frequency = 12MHz (crystal)
 * PLLA: PLL Divider = 1, Multiplier = 14 to generate PLLACK = 168MHz
 * Master Clock (MCK): Source = PLLACK, Prescalar = 1 to generate MCK = 84MHz
 * CPU clock: 84MHz
 */

#define BOARD_32KOSC_FREQUENCY     (32768)
#define BOARD_SCLK_FREQUENCY       (BOARD_32KOSC_FREQUENCY)
#define BOARD_MAINOSC_FREQUENCY    (12000000)  /* MAINOSC: 12MHz crystal on-board */

/* Main oscillator register settings.
 *
 *   The start up time should be should be:
 *   Start Up Time = 8 * MOSCXTST / SLCK = 56 Slow Clock Cycles.
 */

#define BOARD_CKGR_MOR_MOSCXTST    (62 << PMC_CKGR_MOR_MOSCXTST_SHIFT) /* Start-up Time */

/* PLLA configuration.
 *
 *   Divider = 1
 *   Multiplier = 14
 */

#define BOARD_CKGR_PLLAR_MUL       (13 << PMC_CKGR_PLLAR_MUL_SHIFT)
#define BOARD_CKGR_PLLAR_COUNT     (63 << PMC_CKGR_PLLAR_COUNT_SHIFT)
#define BOARD_CKGR_PLLAR_DIV       PMC_CKGR_PLLAR_DIV_BYPASS

/* PMC master clock register settings.
 *
 *  Source = PLLA
 *  Divider = 2
 */

#define BOARD_PMC_MCKR_CSS         PMC_MCKR_CSS_PLLA
#define BOARD_PMC_MCKR_PRES        PMC_MCKR_PRES_DIV2

/* USB UTMI PLL start-up time */

#define BOARD_CKGR_UCKR_UPLLCOUNT  (3 << PMC_CKGR_UCKR_UPLLCOUNT_SHIFT)

/* Resulting frequencies */

#define BOARD_PLLA_FREQUENCY       (168000000) /* PLLACK:  14 * 12Mhz / 1 */
#define BOARD_MCK_FREQUENCY        (84000000)  /* MCK:     PLLACK / 2 */
#define BOARD_CPU_FREQUENCY        (84000000)  /* CPU:     MCK */

/* HSMCI clocking
 *
 * Multimedia Card Interface clock (MCCK or MCI_CK) is Master Clock (MCK)
 * divided by (2*(CLKDIV+1)).
 *
 *   MCI_SPEED = MCCK / (2*(CLKDIV+1))
 *   CLKDIV = MCCK / MCI_SPEED / 2 - 1
 *
 * Where CLKDIV has a range of 0-255.
 */

/* MCK = 84MHz, CLKDIV = 104, MCI_SPEED = 84MHz / 2 * (104+1) = 400 KHz */

#define HSMCI_INIT_CLKDIV          (104 << HSMCI_MR_CLKDIV_SHIFT)

/* MCK = 84MHz, CLKDIV = 2, MCI_SPEED = 84MHz / 2 * (2+1) = 14 MHz */

#define HSMCI_MMCXFR_CLKDIV        (1 << HSMCI_MR_CLKDIV_SHIFT)

/* MCK = 84MHz, CLKDIV = 1, MCI_SPEED = 84MHz / 2 * (1+1) = 21 MHz */

#define HSMCI_SDXFR_CLKDIV         (1 << HSMCI_MR_CLKDIV_SHIFT)
#define HSMCI_SDWIDEXFR_CLKDIV     HSMCI_SDXFR_CLKDIV

/* FLASH wait states
 *
 * FWS MAX FREQUENCY
 *     1.62V 1.8V
 * --- ----- ------
 *  0  17MHz 19MHz
 *  1  45MHz 50MHz
 *  2  58MHz 64MHz
 *  3  70MHz 80MHz
 *  4  78MHz 90MHz
 */

#define BOARD_FWS                  4

/* LED definitions **********************************************************/

/* There are four LEDs on the top, blue side of the board.  Only one can be
 * controlled by software:
 *
 *   LED L      - PB27 (PWM13)
 *
 * There are also four LEDs on the back, white side of the board:
 *
 *   LED A      - PC6
 *   LED B      - PC5
 *   LED C      - PC7
 *   LED D      - PC8
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
 * application unless CONFIG_ARCH_LEDS is defined.
 * In that case, the usage by the board port is defined in include/board.h
 * and src/sam_autoleds.c.
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

/* Thus if LED L is faintly glowing and all other LEDs are off
 * (except LED D which was left on but is no longer controlled by NuttX and
 * so may be in any state),
 * NuttX has successfully booted and is, apparently, running normally and
 * taking interrupts.
 * If any of LEDs A-D are statically set, then NuttX failed to boot and the
 * LED indicates the initialization phase where the failure occurred.
 * If LED L is flashing at approximately 2Hz, then a fatal error has been
 * detected and the system has halted.
 *
 * NOTE: After booting, LEDs A-D are no longer used by the system and may be
 * controlled the application.
 */

/* Button definitions *******************************************************/

/*   There are no buttons on the Arduino Due board. */

/* GPIO pin configurations **************************************************/

/* Universal Asynchronous Receiver Transceiver (UART)
 *
 *   The SAM3X has a UART and 4 USARTS.  The Programming port uses a USB-to-
 *   serial chip connected to the first of the MCU (RX0 and TX0 on PA8 and
 *   PA9, respectively).
 *   The output from that port is visible using the Arduino tool.
 *
 *   Any of UART and USART0-3 may be used as a serial console.
 *   By default, UART0 is used as the serial console in all configurations.
 *
 * There are no alternatives for these pins.
 */

/* Universal Synchronous Asynchronous Receiver Transmitter (USART)
 *
 *   The RX and TX pins are available on the Arduino connector D0 and D1
 *   pins, respectively.
 *   These are connected to USART0, RXD0 and TXD0 which are PA10 and PA11,
 *   respectively.
 *
 *   There are four Click bus connectors with serial ports available as
 *   follows:
 *
 *   Click A:  USART0 RXD0 and TXD0 which are, again, PA10 and PA11.
 *   Click B:  USART1 RXD1 and TXD1 which are PA12 and PA13, respectively.
 *   Click C:  USART3 RXD3 and TXD3 which are PD5 and PD4, respectively.
 *   Click D:  USART3 RXD3 and TXD3 which are, again, PD5 and PD4.
 *
 * There are no alternatives for these pins.
 */

/* SPI:
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
 *
 * Chip select pin definitions are provided in
 * boards/arm/sam34/flipnclick-sam3x/src/flipnclick-3x.h.
 *
 * There are no alternative pin selections for SPI0_MISO and SPIO_MOSI.
 */

#define GPIO_SPI0_SPCK   GPIO_SPI0_SPCK_1

/* I2C (aka TWI):
 *
 * I2C0 is available on pins D16-D17 of the Arduino Shield connectors where
 * you would expect then.  The SPI connector is configured as follows:
 *
 *   Pin Label J1 Board Signal SAM3X
 *   --- ----- -- ------------ -----
 *   D16 SCL1  8  I2C0_SCL     PA17
 *   D17 SDA1  7  I2C0_SDA     PA18
 *
 * I2C0 and I2C1 are also available on the mikroBUS Click connectors (in
 * addition to 5V and GND).  The connectors A and B share I2C0 with the
 * Arduino shield connector.  Connectors C and D both connect to I2C1:
 *
 *   MikroBUS A:              MikroBUS B:
 *   Pin  Board Signal SAM3X  Pin  Board Signal SAM3X
 *   ---- ------------ -----  ---- ------------ -------
 *   SCL  I2C0_SCL     PA18   SCL  I2C0_SCL     PA18
 *   SDA  I2C0_SDA     PA17   SDA  I2C0_SDA     PA17
 *
 *   MikroBUS C:              MikroBUS D:
 *   Pin  Board Signal SAM3X  Pin  Board Signal SAM3X
 *   ---- ------------ -----  ---- ------------ -------
 *   SCL  I2C1_SCL     PB13   SCL  I2C1_SCL     PB13
 *   SDA  I2C1_SDA     PB12   SDA  I2C1_SDA     PB12
 *
 * There are no alternative pin selections for TWI0 and TWI1.
 */

#endif /* __BOARDS_ARM_SAM34_FLIPNCLICK_SAM3X_INCLUDE_BOARD_H */
