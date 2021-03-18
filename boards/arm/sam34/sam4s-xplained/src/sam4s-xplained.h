/****************************************************************************
 * boards/arm/sam34/sam4s-xplained/src/sam4s-xplained.h
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

#ifndef __BOARDS_ARM_SAM4S_XPLAINED_SRC_SAM4S_XPLAINED_H
#define __BOARDS_ARM_SAM4S_XPLAINED_SRC_SAM4S_XPLAINED_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include <arch/irq.h>
#include <nuttx/irq.h>

#include "hardware/sam_pinmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* There are four LEDs on board the SAM4S Xplained board, two of these can be
 * controlled by software in the SAM4S:
 *
 *   LED              GPIO
 *   ---------------- -----
 *   D9  Yellow LED   PC10
 *   D10 Yellow LED   PC17
 *
 * Both can be illuminated by driving the GPIO output to ground (low).
 *
 * These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/up_leds.c. The LEDs are used to encode OS-related
 * events as follows:
 *
 *   SYMBOL                Meaning                     LED state
 *                                                   D9     D10
 *   -------------------  -----------------------  -------- --------
 *   LED_STARTED          NuttX has been started     OFF      OFF
 *   LED_HEAPALLOCATE     Heap has been allocated    OFF      OFF
 *   LED_IRQSENABLED      Interrupts enabled         OFF      OFF
 *   LED_STACKCREATED     Idle stack created         ON       OFF
 *   LED_INIRQ            In an interrupt              No change
 *   LED_SIGNAL           In a signal handler          No change
 *   LED_ASSERTION        An assertion failed          No change
 *   LED_PANIC            The system has crashed     OFF      Blinking
 *   LED_IDLE             MCU is is sleep mode         Not used
 *
 * Thus if D9 is statically on, NuttX has successfully booted and is,
 * apparently, running normmally.  If D10 is flashing at approximately
 * 2Hz, then a fatal error has been detected and the system has halted.
 */

#define GPIO_D9      (GPIO_OUTPUT | GPIO_CFG_PULLUP | GPIO_OUTPUT_SET | \
                      GPIO_PORT_PIOC | GPIO_PIN10)
#define GPIO_D10     (GPIO_OUTPUT | GPIO_CFG_PULLUP | GPIO_OUTPUT_SET | \
                      GPIO_PORT_PIOC | GPIO_PIN17)

/* Mechanical buttons:
 *
 * The SAM4S Xplained has two mechanical buttons.
 * One button is the RESET button connected to the SAM4S reset line and the
 * other is a generic user configurable button labeled BP2.
 * When a button is pressed it will drive the I/O line to GND.
 *
 *   PA5 BP2
 */

#define GPIO_BP2     (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_CFG_DEGLITCH | \
                      GPIO_INT_BOTHEDGES | GPIO_PORT_PIOA | GPIO_PIN5)
#define IRQ_BP2      SAM_IRQ_PA5

/* SRAM Enable.  EBI_NLB PC16.  Active=low; Standby=high */

#define GPIO_EBI_NLB (GPIO_OUTPUT | GPIO_CFG_PULLUP | GPIO_OUTPUT_SET | \
                      GPIO_PORT_PIOC | GPIO_PIN16)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_sram_initialize
 *
 * Description:
 *   Configure and enable SRAM on board the SAM4S Xplained
 *
 ****************************************************************************/

#ifdef CONFIG_SAM34_EXTSRAM0
void sam_sram_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_SAM34_SAM4S_XPLAINED_SRC_SAM4S_XPLAINED_H */
