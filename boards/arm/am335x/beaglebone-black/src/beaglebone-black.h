/****************************************************************************
 * boards/arm/am335x/beaglebone-black/src/beaglebone-black.h
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

#ifndef __BOARDS_ARM_BEAGLEBONE_BLACK_SRC_BEAGLEBONE_BLACK_H
#define __BOARDS_ARM_BEAGLEBONE_BLACK_SRC_BEAGLEBONE_BLACK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include <arch/irq.h>
#include <nuttx/irq.h>

#include "am335x_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LCD **********************************************************************/

#define HAVE_LCD      1
#define HAVE_TDA19988 1

#if !defined(CONFIG_AM335X_LCDC) || !defined(CONFIG_VIDEO_EDID)
#  undef HAVE_LCD
#  undef HAVE_TDA19988
#elif !defined(CONFIG_LCD_TDA19988) || !defined(CONFIG_AM335X_I2C2)
#  undef HAVE_TDA19988
#  if !defined(CONFIG_BEAGLEBONE_VIDEOMODE)
#    define CONFIG_BEAGLEBONE_VIDEOMODE "640x480x60"
#  endif
#endif

#if defined(HAVE_LCD)
#  define TDA19988_I2CBUS        2
#  define TDA19988_HDMI_I2CADDR  0x70
#  define TDA19988_CEC_I2CADDR   0x34
#  define TDA19988_I2CFREQUENCY  400000
#endif

/* LEDs *****************************************************************************/

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

#define GPIO_LED0 (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GPIO_PORT1 | GPIO_PIN21)
#define GPIO_LED1 (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GPIO_PORT1 | GPIO_PIN22)
#define GPIO_LED2 (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GPIO_PORT1 | GPIO_PIN23)
#define GPIO_LED3 (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GPIO_PORT1 | GPIO_PIN24)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/am335x_leds.c.
 * The LEDs are used to encode OS-related events as follows:
 *
 *   SYMBOL            Meaning                      LED state
 *                                              LED1 LED3 LED4
 *   ----------------- -----------------------  ---- ---- ---- ------------
 *   LED_STARTED       NuttX has been started   ON   OFF  OFF
 *   LED_HEAPALLOCATE  Heap has been allocated  OFF  ON   OFF
 *   LED_IRQSENABLED   Interrupts enabled       ON   ON   OFF
 *   LED_STACKCREATED  Idle stack created       ON   ON   OFF
 *   LED_INIRQ         In an interrupt          N/C  N/C  Soft glow
 *   LED_SIGNAL        In a signal handler      N/C  N/C  Soft glow
 *   LED_ASSERTION     An assertion failed      N/C  N/C  Soft glow
 *   LED_PANIC         The system has crashed   N/C  N/C  2Hz Flashing
 *   LED_IDLE          MCU is is sleep mode         Not used
 *
 * After booting, LED1 and 3 are not longer used by the system and can be
 * used for other purposes by the application (Of course, all LEDs are
 * available to the application if CONFIG_ARCH_LEDS is not defined.
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am335x_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int am335x_bringup(void);

/****************************************************************************
 * Name: am335x_led_initialize
 *
 * Description:
 *   Configure LEDs.  LEDs are left in the OFF state.
 *
 ****************************************************************************/

void am335x_led_initialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_BEAGLEBONE_BLACK_SRC_BEAGLEBONE_BLACK_H */
