/****************************************************************************
 * boards/arm/sama5/sama5d3-xplained/include/board.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#ifndef __BOARDS_ARM_SAMA5_SAMA5D3_XPLAINED_INCLUDE_BOARD_H
#define __BOARDS_ARM_SAMA5_SAMA5D3_XPLAINED_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

/*****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* After power-on reset, the SAMA5 device is running on a 12MHz internal RC.
 * These definitions will configure operational clocking.
 */

/* On-board crystal frequencies */

#define BOARD_MAINOSC_FREQUENCY    (12000000)  /* MAINOSC: 12MHz crystal on-board */
#define BOARD_SLOWCLK_FREQUENCY    (32768)     /* Slow Clock: 32.768KHz */

#if defined(CONFIG_SAMA5_BOOT_SDRAM)

/* When booting from SDRAM, NuttX is loaded in SDRAM by an intermediate
 * bootloader.
 * That bootloader had to have already configured the PLL and SDRAM for
 * proper operation.
 *
 * In this case, we don not reconfigure the clocking.
 * Rather, we need to query the register settings to determine the clock
 * frequencies.
 * We can only assume that the Main clock source is the on-board 12MHz
 * crystal.
 */

#  include <arch/board/board_sdram.h>

#elif defined(CONFIG_SAMA5D3XPLAINED_384MHZ)

/* OHCI Only.
 * This is an alternative slower configuration that will produce a 48MHz
 * USB clock with the required accuracy using only PLLA.
 * When PPLA is used to clock OHCI, an additional requirement is the
 * PLLACK be a multiple of 48MHz.
 * This setup results in a CPU clock of 384MHz.
 *
 * This case is only interesting for experimentation.
 */

#  include <arch/board/board_384mhz.h>

#elif defined(CONFIG_SAMA5D3XPLAINED_528MHZ)

/* This is the configuration results in a CPU clock of 528MHz.
 *
 * In this configuration, UPLL is the source of the UHPHS clock (if enabled).
 */

#  include <arch/board/board_528mhz.h>

#else /* #elif defined(CONFIG_SAMA5D3XPLAINED_396MHZ) */

/* This is the configuration provided in the Atmel example code.
 * This setup results in a CPU clock of 396MHz.
 *
 * In this configuration, UPLL is the source of the UHPHS clock (if enabled).
 */

#  include <arch/board/board_396mhz.h>

#endif

/* LED definitions **********************************************************/

/* There are two LEDs on the SAMA5D3 series-CM board that can be controlled
 * by software.  A  blue LED is controlled via PIO pins.  A red LED normally
 * provides an indication that power is supplied to the board but can also
 * be controlled via software.
 *
 *   PE23.  This blue LED is pulled high and is illuminated by pulling PE23
 *   low.
 *
 *   PE24.  The red LED is also pulled high but is driven by a transistor so
 *   that it is illuminated when power is applied even if PE24 is not
 *   configured as an output.  If PE24 is configured as an output, then the
 *   LCD is illuminated by a high output.
 */

/* LED index values for use with board_userled() */

#define BOARD_BLUE        0
#define BOARD_RED         1
#define BOARD_NLEDS       2

/* LED bits for use with board_userled_all() */

#define BOARD_BLUE_BIT    (1 << BOARD_BLUE)
#define BOARD_RED_BIT     (1 << BOARD_RED)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/sam_leds.c. The LEDs are used to encode OS-related
 * events as follows:
 *
 *      SYMBOL            Val    Meaning                     LED state
 *                                                         Blue     Red
 *      ----------------- ---   -----------------------  -------- --------
 */

#define LED_STARTED       0  /* NuttX has been started     OFF      OFF      */
#define LED_HEAPALLOCATE  0  /* Heap has been allocated    OFF      OFF      */
#define LED_IRQSENABLED   0  /* Interrupts enabled         OFF      OFF      */
#define LED_STACKCREATED  1  /* Idle stack created         ON       OFF      */
#define LED_INIRQ         2  /* In an interrupt              No change       */
#define LED_SIGNAL        2  /* In a signal handler          No change       */
#define LED_ASSERTION     2  /* An assertion failed          No change       */
#define LED_PANIC         3  /* The system has crashed     OFF      Blinking */
#undef  LED_IDLE             /* MCU is is sleep mode         Not used        */

/* Thus if the blue LED is statically on, NuttX has successfully booted and
 * is, apparently, running normmally.  If the red is flashing at
 * approximately 2Hz, then a fatal error has been detected and the system
 * has halted.
 */

/* Button definitions *******************************************************/

/* The following push buttons switches are available:
 *
 *    1. One board reset button (BP2). When pressed and released, this push
 *       button causes a power-on reset of the whole board.
 *
 *    2. One wakeup pushbutton that brings the processor out of Low-power mode
 *       (BP1)
 *
 *    3. One user pushbutton (BP3)
 *
 *  Only the user push button (BP3) is controllable by software:
 *
 *    - PE29.  Pressing the switch connect PE29 to ground.  Therefore, PE29
 *      must be pulled high internally.  When the button is pressed the SAMA5
 *      will sense "0" is on PE29.
 */

#define BUTTON_USER       0
#define NUM_BUTTONS       1

#define BUTTON_USER_BIT   (1 << BUTTON_USER)

/* NAND *********************************************************************/

/* Address for transferring command bytes to the nandflash, CLE A22 */

#define BOARD_EBICS3_NAND_CMDADDR   0x60400000

/* Address for transferring address bytes to the nandflash, ALE A21 */

#define BOARD_EBICS3_NAND_ADDRADDR  0x60200000

/* Address for transferring data bytes to the nandflash. */

#define BOARD_EBICS3_NAND_DATAADDR  0x60000000

/* PIO configuration ********************************************************/

/* PWM.
 * There are no dedicated PWM output pins available to the user for PWM
 * testing.
 * Care must be taken because all PWM output pins conflict with some other
 * usage of the pin by other devices.
 * Furthermore, many of these pins have not been brought out to an external
 * connector:
 *
 *    -----+---+---+----+------+----------------
 *     PWM  PIN PER PIO   I/O   CONFLICTS
 *    -----+---+---+----+------+----------------
 *     PWM0 FI   B  PC28 J2.30  SPI1, ISI
 *          H    B  PB0   ---   GMAC
 *               B  PA20 J1.14  LCDC, ISI
 *          L    B  PB1   ---   GMAC
 *               B  PA21 J1.16  LCDC, ISI
 *    -----+---+---+----+------+----------------
 *     PWM1 FI   B  PC31 J2.36  HDMI
 *          H    B  PB4   ---   GMAC
 *               B  PA22 J1.18  LCDC, ISI
 *          L    B  PB5   ---   GMAC
 *               B  PE31 J3.20  ISI, HDMI
 *               B  PA23 J1.20  LCDC, ISI
 *    -----+---+---+----+------+----------------
 *     PWM2 FI   B  PC29 J2.29  UART0, ISI, HDMI
 *          H    C  PD5   ---   HSMCI0
 *               B  PB8   ---   GMAC
 *          L    C  PD6   ---   HSMCI0
 *               B  PB9   ---   GMAC
 *    -----+---+---+----+------+----------------
 *     PWM3 FI   C  PD16  ---  SPI0, Audio
 *          H    C  PD7   ---  HSMCI0
 *               B  PB12 J3.7  GMAC
 *          L    C  PD8   ---  HSMCI0
 *               B  PB13  ---  GMAC
 *    -----+---+---+----+------+----------------
 */

/* PWM channel 0:
 *
 * PA20 and PA21 can be used if the LCDC or ISI are not selected.
 * These outputs are available on J1, pins 14 and 16, respectively.
 *
 * If the GMAC is not selected, then PB0 and PB1 could also be used.
 * However, these pins are not available at the I/O expansion connectors.
 */

#if !defined(CONFIG_SAMA5_LCDC) && !defined(CONFIG_SAMA5_ISI)
#  define PIO_PWM0_H  PIO_PWM0_H_2
#  define PIO_PWM0_L  PIO_PWM0_L_2
#elif !defined(CONFIG_SAMA5_GMAC)
#  define PIO_PWM0_H  PIO_PWM0_H_1
#  define PIO_PWM0_L  PIO_PWM0_L_1
#endif

/* PWM channel 1:
 *
 * PA22 and PA23 can be used if the LCDC or ISI are not selected.
 * These outputs are available on J1, pins 18 and 20, respectively.
 *
 * PE31 can be used if the ISI is not selected
 * (and the HDMI is not being used).
 * That signal is available at J3 pin 20.
 *
 * If the GMAC is not selected, then PB4 and PB5 could also be used.
 * However, these pins are not available at the I/O expansion connectors.
 */

#if !defined(CONFIG_SAMA5_LCDC) && !defined(CONFIG_SAMA5_ISI)
#  define PIO_PWM1_H  PIO_PWM1_H_2
#elif !defined(CONFIG_SAMA5_GMAC)
#  define PIO_PWM1_H  PIO_PWM1_H_1
#endif

#if !defined(CONFIG_SAMA5_LCDC) && !defined(CONFIG_SAMA5_ISI)
#  define PIO_PWM1_L  PIO_PWM1_L_3
#elif !defined(CONFIG_SAMA5_ISI)
#  define PIO_PWM1_L  PIO_PWM1_L_2
#elif !defined(CONFIG_SAMA5_GMAC)
#  define PIO_PWM1_L  PIO_PWM1_L_1
#endif

/* PWM channel 2:
 *
 * None of the output pin options are available at any of the I/O expansion
 * connectors for PWM channel 2
 */

#if !defined(CONFIG_SAMA5_HSMCI0)
#  define PIO_PWM2_H  PIO_PWM2_H_1
#  define PIO_PWM2_L  PIO_PWM2_L_1
#elif !defined(CONFIG_SAMA5_GMAC)
#  define PIO_PWM2_H  PIO_PWM2_H_2
#  define PIO_PWM2_L  PIO_PWM2_L_2
#endif

/* PWM channel 3:
 *
 * If the GMAC is not selected,
 * then PB12 can used and is available at J3 pin 7.
 * None of the other output pins are accessible at the
 * I/O expansion connectors.
 */

#if !defined(CONFIG_SAMA5_GMAC)
#  define PIO_PWM3_H  PIO_PWM3_H_2
#  define PIO_PWM3_L  PIO_PWM3_L_2
#elif !defined(CONFIG_SAMA5_HSMCI0)
#  define PIO_PWM3_H  PIO_PWM3_H_1
#  define PIO_PWM3_L  PIO_PWM3_L_1
#endif

/****************************************************************************
 * Assembly Language Macros
 ****************************************************************************/

#ifdef __ASSEMBLY__
  .macro config_sdram
  .endm
#endif /* __ASSEMBLY__ */

#endif /* __BOARDS_ARM_SAMA5_SAMA5D3_XPLAINED_INCLUDE_BOARD_H */
