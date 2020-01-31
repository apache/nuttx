/****************************************************************************
 * boards/arm/am335x/beaglebone-black/include/board.h
 *
 *   Copyright (C) 2018 Petro Karashchenko. All rights reserved.
 *   Author: Petro Karashchenko <petro.karashchenko@gmail.com>
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

#ifndef __BOARDS_ARM_AM335X_BEAGLEBONE_BLACK_INCLUDE_BOARD_H
#define __BOARDS_ARM_AM335X_BEAGLEBONE_BLACK_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/am335x_pinmux.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* Since NuttX is booted from a loader on the AM335X, clocking should already
 * be setup when NuttX starts.
 */

/* LED definitions **********************************************************/

/* The beaglebone black has four user LEDs; all four can be controlled from
 * software.
 * All are tied to ground and, hence, illuminated by driving the output pins
 * to a high value:
 *
 *  1. LED0 GPMC_A5  GPMC_A5/GMII2_TXD0/RGMII2_TD0/RMII2_TXD0/GPMC_A21/
 *                   PR1_MII1_RXD3/eQEP1B_IN/GPIO1_21
 *  2. LED1 GPMC_A6  GPMC_A6/GMII2_TXCLK/RGMII2_TCLK/MMC2_DAT4/GPMC_A22/
 *                   PR1_MII1_RXD2/eQEP1_INDEX/GPIO1_22
 *  3. LED2 GPMC_A7  GPMC_A7/GMII2_RXCLK/RGMII2_RCLK/MMC2_DAT5/GPMC_A23/
 *                   PR1_MII1_RXD1/eQEP1_STROBE/GPIO1_23
 *  4. LED3 GPMC_A8  GPMC_A8/GMII2_RXD3/RGMII2_RD3/MMC2_DAT6/GPMC_A24/
 *                   PR1_MII1_RXD0/MCASP0_ACLKX/GPIO1_24
 */

/* LED index values for use with board_userled() */

#define USER_LED0         0
#define USER_LED1         1
#define USER_LED2         2
#define USER_LED3         3
#define BOARD_NLEDS       4

/* LED bits for use with board_userled_all() */

#define BOARD_LED0_BIT    (1 << USER_LED0)
#define BOARD_LED1_BIT    (1 << USER_LED1)
#define BOARD_LED2_BIT    (1 << USER_LED2)
#define BOARD_LED3_BIT    (1 << USER_LED3)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/am335x_leds.c.
 * The LEDs are used to encode OS-related events as follows:
 *
 *      SYMBOL            Value Meaning                       LED state
 *                                                          LED0 LED1 LED2
 *      ----------------- ----- -----------------------  ---- ---- ---------*
 */

#define LED_STARTED         0   /* NuttX has been started   ON   OFF  OFF */
#define LED_HEAPALLOCATE    1   /* Heap has been allocated  OFF  ON   OFF */
#define LED_IRQSENABLED     2   /* Interrupts enabled       ON   ON   OFF */
#define LED_STACKCREATED    2   /* Idle stack created       ON   ON   OFF */
#define LED_INIRQ           3   /* In an interrupt          N/C  N/C  Soft glow */
#define LED_SIGNAL          3   /* In a signal handler      N/C  N/C  Soft glow */
#define LED_ASSERTION       3   /* An assertion failed      N/C  N/C  Soft glow */
#define LED_PANIC           3   /* The system has crashed   N/C  N/C  2Hz Flashing */

/*      LED_IDLE           ---  /* MCU is is sleep mode         Not used
 *
 * After booting, LED0 and 1 are not longer used by the system and can be
 * used for other purposes by the application (Of course, all LEDs are
 * available to the application if CONFIG_ARCH_LEDS is not defined.
 */

/* NAND *********************************************************************/

/* GPIO pin disambiguation **************************************************/

/* UARTs ********************************************************************/

/* One UART connections is available:
 *
 * 1. UART0 is available on FT2232H USB to Serial Adapter.
 *
 *    FT2232H BDBUS1 Pin     UART0-RXD  UART0_RXD/SPI1_CS0/DCAN0_TX/I2C2_SDA/
 *                                      eCAP2_IN_PWM2_OUT/PR1_PRU1_PRU_R30_14/
 *                                      PR1_PRU1_PRU_R31_14/GPIO1_10
 *    FT2232H BDBUS0 Pin     UART0-TXD  UART0_TXD/SPI1_CS1/DCAN0_RX/I2C2_SCL/
 *                                      eCAP1_IN_PWM1_OUT/PR1_PRU1_PRU_R30_15/
 *                                      PR1_PRU1_PRU_R31_15/GPIO1_11
 */

/* I2Cs *********************************************************************/

#define GPIO_I2C1_SCL       GPIO_I2C1_SCL_2
#define GPIO_I2C1_SCL       GPIO_I2C1_SDA_2

#define GPIO_I2C2_SCL       GPIO_I2C2_SCL_1
#define GPIO_I2C2_SCL       GPIO_I2C2_SDA_1

/* CANs *********************************************************************/

#define GPIO_DCAN0_RX       GPIO_DCAN0_RX_3
#define GPIO_DCAN0_TX       GPIO_DCAN0_TX_3

#define GPIO_DCAN1_RX       GPIO_DCAN1_RX_3
#define GPIO_DCAN1_TX       GPIO_DCAN1_TX_3

/****************************************************************************
 * Assembly Language Macros
 ****************************************************************************/

#ifdef __ASSEMBLY__
  .macro  config_sdram
  .endm
#endif /* __ASSEMBLY__ */

#endif /* __BOARDS_ARM_AM335X_BEAGLEBONE_BLACK_INCLUDE_BOARD_H */
