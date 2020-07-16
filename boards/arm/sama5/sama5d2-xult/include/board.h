/****************************************************************************
 * boards/arm/sama5/sama5d2-xult/include/board.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#ifndef __BOARDS_ARM_SAMA5_SAMA5D2_XULT_INCLUDE_BOARD_H
#define __BOARDS_ARM_SAMA5_SAMA5D2_XULT_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdbool.h>
#  include <nuttx/irq.h>
#endif

/****************************************************************************
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

#elif defined(CONFIG_SAMA5D2XULT_384MHZ)

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

#elif defined(CONFIG_SAMA5D2XULT_498MHZ)

/* This is the configuration results in a CPU clock of 498MHz.
 *
 * In this configuration, UPLL is the source of the UHPHS clock (if enabled).
 */

#  include <arch/board/board_498mhz.h>

#elif defined(CONFIG_SAMA5D2XULT_528MHZ)

/* This is the configuration results in a CPU clock of 528MHz.
 *
 * In this configuration, UPLL is the source of the UHPHS clock (if enabled).
 */

#  include <arch/board/board_528mhz.h>

#else /* #elif defined(CONFIG_SAMA5D2XULT_396MHZ) */

/* This is the configuration provided in the Atmel example code.
 * This setup results in a CPU clock of 396MHz.
 *
 * In this configuration, UPLL is the source of the UHPHS clock (if enabled).
 */

#  include <arch/board/board_396mhz.h>

#endif

/* LED definitions **********************************************************/

/* There is an RGB LED on board the SAMA5D2-XULT.
 * The RED component is driven by the SDHC_CD pin (PA13) and so will not
 * be used.  The LEDs are provided VDD_LED and so bringing the LED low will
 * will illuminated the LED.
 *
 *   ------------------------------ ------------------- ---------------------
 *   SAMA5D2 PIO                    SIGNAL              USAGE
 *   ------------------------------ ------------------- ---------------------
 *   PA13                           SDHC_CD_PA13        Red LED
 *   PB5                            LED_GREEN_PB5       Green LED
 *   PB0                            LED_BLUE_PB0        Blue LED
 *   ------------------------------ ------------------- ---------------------
 */

#ifndef CONFIG_ARCH_LEDS

/* LED index values for use with board_userled() */

#define BOARD_GREEN       0
#define BOARD_BLUE        1
#define BOARD_NLEDS       2

/* LED bits for use with board_userled_all() */

#define BOARD_GREEN_BIT  (1 << BOARD_GREEN)
#define BOARD_BLUE_BIT   (1 << BOARD_BLUE)

#else

/* LED index values for use with board_userled() */

#define BOARD_BLUE        0
#define BOARD_NLEDS       1

/* LED bits for use with board_userled_all() */

#define BOARD_BLUE_BIT   (1 << BOARD_BLUE)
#endif

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/sam_leds.c. The LEDs are used to encode OS-related
 * events as follows.  Note that only the GREEN LED is used in this case
 *
 *      SYMBOL            Val    Meaning                   Green LED
 *      ----------------- ---   -----------------------  -----------
 */

#define LED_STARTED       0  /* NuttX has been started     OFF       */
#define LED_HEAPALLOCATE  0  /* Heap has been allocated    OFF       */
#define LED_IRQSENABLED   0  /* Interrupts enabled         OFF       */
#define LED_STACKCREATED  1  /* Idle stack created         ON        */
#define LED_INIRQ         2  /* In an interrupt            N/C       */
#define LED_SIGNAL        2  /* In a signal handler        N/C       */
#define LED_ASSERTION     2  /* An assertion failed        N/C       */
#define LED_PANIC         3  /* The system has crashed     Flash     */
#undef  LED_IDLE             /* MCU is is sleep mode       Not used  */

/* Thus if the Green LED is statically on, NuttX has successfully  booted
 * and is, apparently, running normally.
 * If LED is flashing at approximately 2Hz, then a fatal error has been
 * detected and the system has halted.
 */

/* Button definitions *******************************************************/

/* A single button, PB_USER (PB6), is available on the SAMA5D2-XULT
 *
 *  ------------------------------ ------------------- ----------------------
 *  SAMA5D2 PIO                    SIGNAL              USAGE
 *  ------------------------------ ------------------- ----------------------
 *  PB6                            USER_PB_PB6         PB_USER push button
 *  ------------------------------ ------------------- ----------------------
 *
 *  Closing PB_USER will bring PB6 to ground so 1) PB6 should have a weak
 * pull-up, and 2) when PB_USER is pressed, a low value will be senses.
 */

#define BUTTON_USER       0
#define NUM_BUTTONS       1

#define BUTTON_USER_BIT   (1 << BUTTON_USER)

/* Pin disambiguation *******************************************************/

/* Alternative pin selections are provided with a numeric suffix like _1, _2,
 * etc. Drivers, however, will use the pin selection without the numeric
 * suffix.
 * Additional definitions are required in this board.h file.
 * For example, if we wanted the PCK0on PB26, then the following definition
 * should appear in the board.h header file for that board:
 *
 *   #define PIO_PMC_PCK0 PIO_PMC_PCK0_1
 *
 * The PCK logic will then automatically configure PB26 as the PCK0 pin.
 */

/* DEBUG / DBGU Port (J1).  There is a TTL serial connection available on
 * pins 2 and 3 of the DEBUG connector.  This may be driven by UART1,
 * depending upon the setting of JP2 (DBGU_PE on the schematic, DEBUG_DIS
 * on the board):
 *
 *   ---- ------------------------ -------------
 *   J1   SCHEMATIC                   SAMA5D2
 *   PIN  NAME(s)                  PIO  FUNCTION
 *   ---- ------------------------ -------------
 *    2   DBGU_TXD  DBGU_UTXD1_PD3 PD3  UTXD1
 *    3   DBGU_RXD  DBGU_URXD1_PD2 PD2  URXD1
 *   ---- ------------------------ -------------
 */

#define PIO_UART1_RXD     PIO_UART1_RXD_1
#define PIO_UART1_TXD     PIO_UART1_TXD_1

/* Standard UART on Arduino connector (J22) is UART2.
 *
 *   ---- ------- -------------
 *   J22  BOARD      SAMA5D2
 *   PIN  NAME    PIO  FUNCTION
 *   ---- ------- -------------
 *    7   URXD2   PD4 UART2 URXD2
 *    8   UTXD2   PD5 UART2 UTXD2
 *   ---- ------- -------------
 */

#define PIO_UART2_RXD     PIO_UART2_RXD_2
#define PIO_UART2_TXD     PIO_UART2_TXD_2

/* Standard UART on Arduino connector (J17) is UART3.
 *
 *   ---- ------- -------------
 *   J17  BOARD      SAMA5D2
 *   PIN  NAME    PIO  FUNCTION
 *   ---- ------- -------------
 *    27   URXD3  PB11 UART3 URXD3
 *    28   UTXD3  PB12 UART3 UTXD3
 *   ---- ------- -------------
 */

#define PIO_UART3_RXD     PIO_UART3_RXD_1
#define PIO_UART3_TXD     PIO_UART3_TXD_1

/* Standard UART on Arduino connector (J21) is FLEXCOM4.
 *
 *   ---- ------- -------------
 *   J21  BOARD      SAMA5D2
 *   PIN  NAME    PIO  FUNCTION
 *   ---- ------- -------------
 *    7   F4_TXD  PD12 FLEXCOM4
 *    8   F4_RXD  PD13 FLEXCOM4
 *   ---- ------- -------------
 */

#define PIO_FLEXCOM4_IO0  PIO_FLEXCOM4_IO0_2
#define PIO_FLEXCOM4_IO1  PIO_FLEXCOM4_IO1_2

/* Other USARTs are available on J22:
 *
 *   ---- ------- -------------
 *   J22  BOARD      SAMA5D2
 *   PIN  NAME    PIO  FUNCTION
 *   ---- ------- -------------
 *    3   F0_TXD  PB28 FLEXCOM0
 *    4   F0_RXD  PB29 FLEXCOM0
 *    5   F3_TXD  PB23 FLEXCOM3
 *    6   F3_RXD  PB22 FLEXCOM3
 *   ---- ------- -------------
 */

#define PIO_FLEXCOM3_IO0  PIO_FLEXCOM3_IO0_2
#define PIO_FLEXCOM3_IO1  PIO_FLEXCOM3_IO1_2

/* UARTs available on EXT1
 *
 *   ---- ------- -------------
 *   EXT1 BOARD      SAMA5D2
 *   PIN  NAME    PIO  FUNCTION
 *   ---- ------- -------------
 *    13  UART_RX PA23 FLEXCOM1
 *    14  UART_TX PA24 FLEXCOM1
 *   ---- ------- ---- --------
 */

/* UARTs available on EXT2
 *
 *   ---- ------- -------------
 *   EXT2 BOARD      SAMA5D2
 *   PIN  NAME    PIO  FUNCTION
 *   ---- ------- -------------
 *    13  UART_RX PB29 FLEXCOM0
 *    14  UART_TX PB28 FLEXCOM0
 *   ---- ------- ---- --------
 */

/* SDIO - Used for both Port 0 & 1 ******************************************/

/* 386 KHz for initial inquiry stuff */

#define BOARD_SDMMC_IDMODE_PRESCALER    SDMMC_SYSCTL_SDCLKFS_DIV256
#define BOARD_SDMMC_IDMODE_DIVISOR      SDMMC_SYSCTL_DVS_DIV(2)

/* 24.8MHz for other modes */

#define BOARD_SDMMC_MMCMODE_PRESCALER   SDMMC_SYSCTL_SDCLKFS_DIV8
#define BOARD_SDMMC_MMCMODE_DIVISOR     SDMMC_SYSCTL_DVS_DIV(1)

#define BOARD_SDMMC_SD1MODE_PRESCALER   SDMMC_SYSCTL_SDCLKFS_DIV8
#define BOARD_SDMMC_SD1MODE_DIVISOR     SDMMC_SYSCTL_DVS_DIV(1)

#define BOARD_SDMMC_SD4MODE_PRESCALER   SDMMC_SYSCTL_SDCLKFS_DIV8
#define BOARD_SDMMC_SD4MODE_DIVISOR     SDMMC_SYSCTL_DVS_DIV(1)

/****************************************************************************
 * Assembly Language Macros
 ****************************************************************************/

#ifdef __ASSEMBLY__
  .macro config_sdram
  .endm
#endif /* __ASSEMBLY__ */

#endif /* __BOARDS_ARM_SAMA5_SAMA5D2_XULT_INCLUDE_BOARD_H */
