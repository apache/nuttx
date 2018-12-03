/************************************************************************************
 * arch/arm/include/tiva/cc13x2_cc26x2_v1_irq.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_INCLUDE_TIVA_CC13X2_CC26x2_V2_IRQ_H
#define __ARCH_ARM_INCLUDE_TIVA_CC13X2_CC26x2_V2_IRQ_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/* IRQ numbers.  The IRQ number corresponds vector number and hence map directly to
 * bits in the NVIC.  This does, however, waste several words of memory in the IRQ
 * to handle mapping tables.
 */

/* External interrupts (vectors >= 16) */

#define TIVA_IRQ_INTERRUPTS          (16) /* Vector number of the first external
                                           * interrupt */

#define TIVA_IRQ_AON_GPIO_EDGE       (16) /* Edge detect event from IOC */
#define TIVA_IRQ_I2C                 (17) /* Interrupt event from I2C */
#define TIVA_IRQ_RFC_CPE_1           (18) /* Combined Interrupt for CPE
                                           * Generated events */
#define TIVA_IRQ_PKA                 (19) /* PKA Interrupt event */
#define TIVA_IRQ_AON_RTC_COMB        (20) /* Event from AON_RTC */
#define TIVA_IRQ_UART0_COMB          (21) /* UART0 combined interrupt */
#define TIVA_IRQ_AUX_SWEV0           (22) /* AUX software event 0 */
#define TIVA_IRQ_SSI0_COMB           (23) /* SSI0 combined interrupt */
#define TIVA_IRQ_SSI1_COMB           (24) /* SSI1 combined interrupt */
#define TIVA_IRQ_RFC_CPE_0           (25) /* Combined Interrupt for CPE
                                           * Generated events */
#define TIVA_IRQ_RFC_HW_COMB         (26) /* Combined RFC hardware interrupt */
#define TIVA_IRQ_RFC_CMD_ACK         (27) /* RFC Doorbell Command
                                           * Acknowledgement Interrupt */
#define TIVA_IRQ_I2S                 (28) /* Interrupt event from I2S */
#define TIVA_IRQ_AUX_SWEV1           (29) /* AUX software event 1 */
#define TIVA_IRQ_WDT                 (30) /* Watchdog interrupt event */
#define TIVA_IRQ_GPT0A               (31) /* GPT0A interrupt event */
#define TIVA_IRQ_GPT0B               (32) /* GPT0B interrupt event */
#define TIVA_IRQ_GPT1A               (33) /* GPT1A interrupt event */
#define TIVA_IRQ_GPT1B               (34) /* GPT1B interrupt event */
#define TIVA_IRQ_GPT2A               (35) /* GPT2A interrupt event */
#define TIVA_IRQ_GPT2B               (36) /* GPT2B interrupt event */
#define TIVA_IRQ_GPT3A               (37) /* GPT3A interrupt event */
#define TIVA_IRQ_GPT3B               (38) /* GPT3B interrupt event */
#define TIVA_IRQ_CRYPTO_RESULT_AVAIL (39) /* CRYPTO result available interrupt
                                           * event */
#define TIVA_IRQ_DMA_DONE_COMB       (40) /* Combined DMA done */
#define TIVA_IRQ_DMA_ERR             (41) /* DMA bus error */
#define TIVA_IRQ_FLASH               (42) /* FLASH controller error event */
#define TIVA_IRQ_SWEV0               (43) /* Software event 0 */
#define TIVA_IRQ_AUX_COMB            (44) /* AUX combined event */
#define TIVA_IRQ_AON_PROG0           (45) /* AON programmable event 0 */
#define TIVA_IRQ_PROG0               (46) /* Programmable Interrupt 0 */
#define TIVA_IRQ_AUX_COMPA           (47) /* AUX Compare A event */
#define TIVA_IRQ_AUX_ADC             (48) /* AUX ADC interrupt event */
#define TIVA_IRQ_TRNG                (49) /* TRNG Interrupt event */
#define TIVA_IRQ_OSC_COMB            (50) /* Combined event from Oscillator
                                           * control */
#define TIVA_IRQ_AUX_TIMER2_EV0      (51) /* AUX Timer2 event 0 */
#define TIVA_IRQ_UART1_COMB          (52) /* UART1 combined interrupt */
#define TIVA_IRQ_BATMON_COMB         (53) /* Combined event from battery
                                           * monitor */

#define NR_IRQS                      (54) /* Number of interrupt vectors */

#endif /* __ARCH_ARM_INCLUDE_TIVA_CC13X2_CC26x2_V2_IRQ_H */
