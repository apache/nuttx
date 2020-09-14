/****************************************************************************
 * boards/arm/sama5/giant-board/include/board.h
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

#ifndef __BOARDS_ARM_SAMA5_GIANT_BOARD_INCLUDE_BOARD_H
#define __BOARDS_ARM_SAMA5_GIANT_BOARD_INCLUDE_BOARD_H

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

#define BOARD_MAINOSC_FREQUENCY    (24000000)  /* MAINOSC: 12MHz crystal on-board */
#define BOARD_SLOWCLK_FREQUENCY    (32768)     /* Slow Clock: 32.768KHz */

#if defined(CONFIG_SAMA5_BOOT_SDRAM)
/* When booting from SDRAM, NuttX is loaded in SDRAM by an intermediate
 * bootloader.
 * That bootloader had to have already configured the PLL and SDRAM for
 * proper operation.
 *
 * In this case, we do not reconfigure the clocking.
 * Rather, we need to query the register settings to determine the clock
 * frequencies.
 * We can only assume that the Main clock source is the on-board 24MHz
 * crystal.
 */

#  include <arch/board/board_sdram.h>

#elif defined(CONFIG_GIANT_BOARD_492MHZ)

/* This configuration results in a CPU clock of 492MHz.
 *
 * In this configuration, UPLL is the source of the UHPHS clock (if enabled).
 */

#  include <arch/board/board_492mhz.h>

#endif

/* LED definitions **********************************************************/

/* There is a status LED on board the Giant Board.
 *
 *   ------------------------------ ------------------- ---------------------
 *   SAMA5D27 PIO                   SIGNAL              USAGE
 *   ------------------------------ ------------------- ---------------------
 *   PA6                            STATUS_LED          Orange LED
 *   ------------------------------ ------------------- ---------------------
 */

/* LED index values for use with board_userled() */

#define BOARD_ORANGE      0
#define BOARD_NLEDS       1

/* LED bits for use with board_userled_all() */

#define BOARD_ORANGE_BIT  (1 << BOARD_ORANGE)

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

/* Thus if the Orange LED is statically on, NuttX has successfully  booted
 * and is, apparently, running normally.
 * If LED is flashing at approximately 2Hz, then a fatal error has been
 * detected and the system has halted.
 */

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

/* UART1. There is a TTL serial connection available on
 * pins RX and TX.  This is be driven by UART1.
 *
 *   ---- ------------------------ -------------
 *   J1/2  SCHEMATIC                   SAMA5D2
 *   PIN   NAME(s)                  PIO  FUNCTION
 *   ---- ------------------------ -------------
 *    14   UART1_RX                 PD3  UTXD1
 *    15   UART1_TX                 PD2  URXD1
 *   ---- ------------------------ -------------
 */

#define PIO_UART1_RXD     PIO_UART1_RXD_1
#define PIO_UART1_TXD     PIO_UART1_TXD_1

/* FLEXCOM4 is available as a UART.
 *
 *   ----- ------- -------------
 *   J1/2  BOARD      SAMA5D2
 *   PIN   NAME    PIO  FUNCTION
 *   ----- ------- -------------
 *    6    AD2     PD21 FLEXCOM4
 *    9    AD3     PD22 FLEXCOM4
 *   ----- ------- -------------
 */

#define PIO_FLEXCOM4_IO0  PIO_FLEXCOM4_IO0_2
#define PIO_FLEXCOM4_IO1  PIO_FLEXCOM4_IO1_2

/* PIO pins */

#define PIO_TX      IO_

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

#endif /* __BOARDS_ARM_SAMA5_GIANT_BOARD_INCLUDE_BOARD_H */
