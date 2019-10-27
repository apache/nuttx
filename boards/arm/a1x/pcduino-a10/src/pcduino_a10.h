/****************************************************************************
 * boards/arm/a1x/pcduino-a10/src/pcduino-a10.h
 *
 *   Copyright (C) 2013, 2019 Gregory Nutt. All rights reserved.
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

#ifndef __BOARDS_ARM_A1X_PCDUINO_A10_PCDUINO_A10_H
#define __BOARDS_ARM_A1X_PCDUINO_A10_PCDUINO_A10_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include <arch/irq.h>
#include <nuttx/irq.h>

#include "a1x_pio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* LEDs *********************************************************************/

/* The pcDuino v1 has four green LEDs; three can be controlled from software.
 * Two are tied to ground and, hence, illuminated by driving the output pins
 * to a high value:
 *
 *  1. LED1 SPI0_CLK  SPI0_CLK/UART5_RX/EINT23/PI11
 *  2. LED5 IPSOUT    From the PMU (not controllable by software)
 */

#define PIO_LED1 (PIO_OUTPUT | PIO_PULL_NONE | \
                  PIO_DRIVE_MEDLOW | PIO_INT_NONE | \
                  PIO_OUTPUT_CLEAR | PIO_PORT_PIOI | PIO_PIN11)

/* And two are pull high and, hence, illuminated by grounding the output:
 *
 *   3. LED3 RX_LED    LCD1_D16/ATAD12/KP_IN6/SMC_DET/EINT16/CSI1_D16/PH16
 *   4. LED4 TX_LED    LCD1_D15/ATAD11/KP_IN5/SMC_VPPPP/EINT15/CSI1_D15/PH15
 */

#define PIO_LED3 (PIO_OUTPUT | PIO_PULL_NONE | \
                  PIO_DRIVE_MEDLOW | PIO_INT_NONE | \
                  PIO_OUTPUT_SET | PIO_PORT_PIOH | PIO_PIN16)

#define PIO_LED4 (PIO_OUTPUT | PIO_PULL_NONE | \
                  PIO_DRIVE_MEDLOW | PIO_INT_NONE | \
                  PIO_OUTPUT_SET | PIO_PORT_PIOH | PIO_PIN15)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/a1x_leds.c.
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
 * After booting, LED1 and 3 are not longer used by the system and can be used
 * for other purposes by the application (Of course, all LEDs are available to
 * the application if CONFIG_ARCH_LEDS is not defined.
 */

/* Buttons ******************************************************************/

/* There are a total of five switches on-board.  All pulled high and, hence,
 * will be sensed as low when closed.
 *
 *   SW1 Reset     (not available to software)
 *   SW2 UBOOT     UBOOT_SEL (?)
 *   SW3 Key_Back  LCD1_D17/ATAD13/KP_IN7/SMC_VCCEN/EINT17/CSI1_D17/PH17
 *   SW4 Key_Home  LCD1_D18/ATAD14/KP_OUT0/SMC_SLK/EINT18/CSI1_D18/PH18
 *   SW5 Key_Menu  LCD1_D19/ATAD15/KP_OUT1/SMC_SDA/EINT19/CSI1_D19/PH19
 */

#ifdef CONFIG_A1X_PIO_IRQ
#  define PIO_KEY_BACK (PIO_EINT | PIO_PULL_NONE | PIO_DRIVE_NONE | \
                        PIO_INT_BOTHEDGES | IO_PORT_PIOH | PIO_PIN17)
#  define PIO_KEY_HOME (PIO_EINT | PIO_PULL_NONE | PIO_DRIVE_NONE | \
                        PIO_INT_BOTHEDGES | IO_PORT_PIOH | PIO_PIN18)
#  define PIO_KEY_MENU (PIO_EINT | PIO_PULL_NONE | PIO_DRIVE_NONE | \
                        PIO_INT_BOTHEDGES | IO_PORT_PIOH | PIO_PIN19)
#else
#  define PIO_KEY_BACK (PIO_INPUT | PIO_PULL_NONE | PIO_DRIVE_NONE | \
                        PIO_INT_NONE | IO_PORT_PIOH | PIO_PIN17)
#  define PIO_KEY_HOME (PIO_INPUT | PIO_PULL_NONE | PIO_DRIVE_NONE | \
                        PIO_INT_NONE | IO_PORT_PIOH | PIO_PIN18)
#  define PIO_KEY_MENU (PIO_INPUT | PIO_PULL_NONE | PIO_DRIVE_NONE | \
                        PIO_INT_NONE | IO_PORT_PIOH | PIO_PIN19)
#endif

/* SPI Chip Selects *********************************************************/

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
 * Name: a1x_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int a1x_bringup(void);

/****************************************************************************
 * Name: a1x_led_initialize
 *
 * Description:
 *   Configure LEDs.  LEDs are left in the OFF state.
 *
 ****************************************************************************/

void a1x_led_initialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_A1X_PCDUINO_A10_PCDUINO_A10_H */
