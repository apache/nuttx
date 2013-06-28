/************************************************************************************
 * configs/arduino-due/src/arduino-due.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __CONFIGS_ARDUINO_DUE_SRC_ARDUNO_DUE_H
#define __CONFIGS_ARDUINO_DUE_SRC_ARDUNO_DUE_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include <arch/irq.h>
#include <nuttx/irq.h>

#include "chip/sam_pinmap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/*  There are three user-controllable LEDs on board the Arduino Due board:
 *
 *     LED              GPIO
 *     ---------------- -----
 *     L   Amber LED    PB27
 *     TX  Yellow LED   PA21
 *     RX  Yellow LED   PC30
 *
 * LED L is connected to ground and can be illuminated by driving the PB27
 * output high. The TX and RX LEDs are pulled high and can be illuminated by
 * driving the corresponding
 * GPIO output to low.
 *
 * These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/sam_leds.c. The LEDs are used to encode OS-related
 * events as follows:
 *
 *   SYMBOL                MEANING                         LED STATE
 *                                                   L         TX       RX
 *   -------------------  -----------------------  -------- -------- --------
 *   LED_STARTED          NuttX has been started     OFF      OFF      OFF
 *   LED_HEAPALLOCATE     Heap has been allocated    OFF      OFF      OFF
 *   LED_IRQSENABLED      Interrupts enabled         OFF      OFF      OFF
 *   LED_STACKCREATED     Idle stack created         ON       OFF      OFF
 *   LED_INIRQ            In an interrupt            N/C      GLOW     OFF
 *   LED_SIGNAL           In a signal handler        N/C      GLOW     OFF
 *   LED_ASSERTION        An assertion failed        N/C      GLOW     OFF
 *   LED_PANIC            The system has crashed     N/C      N/C      Blinking
 *   LED_IDLE             MCU is is sleep mode       ------ Not used --------
 *
 * Thus if LED L is statically on, NuttX has successfully booted and is,
 * apparently, running normmally.  If LED RX is glowing, then NuttX is
 * handling interupts (and also signals and assertions).  If TX is flashing
 * at approximately 2Hz, then a fatal error has been detected and the system
 */

#define GPIO_LED_L   (GPIO_OUTPUT | GPIO_CFG_PULLUP | GPIO_OUTPUT_CLEAR | \
                      GPIO_PORT_PIOB | GPIO_PIN27)
#define GPIO_LED_RX  (GPIO_OUTPUT | GPIO_CFG_PULLUP | GPIO_OUTPUT_SET | \
                      GPIO_PORT_PIOC | GPIO_PIN30)
#define GPIO_LED_TX  (GPIO_OUTPUT | GPIO_CFG_PULLUP | GPIO_OUTPUT_SET | \
                      GPIO_PORT_PIOA | GPIO_PIN21)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public data
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: sam_sram_initialize
 *
 * Description:
 *   Configure and enable SRAM on board the SAM4S Xplained
 *
 ************************************************************************************/

#ifdef CONFIG_ARCH_EXTSRAM0
void sam_sram_initialize(void);
#endif

/************************************************************************************
 * Name: up_ledinit
 ************************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void up_ledinit(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_ARDUINO_DUE_SRC_ARDUNO_DUE_H */

