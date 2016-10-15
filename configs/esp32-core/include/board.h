/****************************************************************************
 * configs/esp32-core/include/board.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#ifndef __CONFIGS_ESP32_CORE_INCLUDE_BOARD_H
#define __CONFIGS_ESP32_CORE_INCLUDE_BOARD_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* A single LED labelled D1 is available.
 *
 *  When CONFIG_ARCH_LEDS is defined in the NuttX configuration, NuttX will
 *  control the LED as follows:
 *
 *    SYMBOL              Value Meaning                 LED
 *    ------------------- ----- ----------------------- ------ */
#define LED_STARTED        0   /* NuttX has been started  OFF */
#define LED_HEAPALLOCATE   0   /* Heap has been allocated OFF */
#define LED_IRQSENABLED    0   /* Interrupts enabled      OFF */
#define LED_STACKCREATED   1   /* Idle stack created      ON */
#define LED_INIRQ          2   /* In an interrupt         N/C */
#define LED_SIGNAL         2   /* In a signal handler     N/C */
#define LED_ASSERTION      2   /* An assertion failed     N/C */
#define LED_PANIC          3   /* The system has crashed  FLASH */

/* Thus if the LED is statically on, NuttX has successfully  booted and is,
 * apparently, running normally.  If LED is flashing at approximately
 * 2Hz, then a fatal error has been detected and the system has halted.
 */

#endif /* __CONFIGS_ESP32_CORE_INCLUDE_BOARD_H */
