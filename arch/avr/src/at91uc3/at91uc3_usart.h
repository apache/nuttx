/************************************************************************************
 * arch/avr/src/at91uc3/at91uc3_usart.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __ARCH_AVR_SRC_AT91UC3_AT91UC3_USART_H
#define __ARCH_AVR_SRC_AT91UC3_AT91UC3_USART_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define AVR32_USART_CR_OFFSET      0x0000 /* Control Register */
#define AVR32_USART_MR_OFFSET      0x0004 /* Mode Register */
#define AVR32_USART_IER_OFFSET     0x0008 /* Interrupt Enable Register */
#define AVR32_USART_IDR_OFFSET     0x000c /* Interrupt Disable Register */
#define AVR32_USART_IMR_OFFSET     0x0010 /* Interrupt Mask Register */
#define AVR32_USART_CSR_OFFSET     0x0014 /* Channel Status Register */
#define AVR32_USART_RHR_OFFSET     0x0018 /* Receiver Holding Register */
#define AVR32_USART_THR_OFFSET     0x001c /* Transmitter Holding Register */
#define AVR32_USART_BRGR_OFFSET    0x0020 /* Baud Rate Generator Register */
#define AVR32_USART_RTOR_OFFSET    0x0024 /* Receiver Time-out Register */
#define AVR32_USART_TTGR_OFFSET    0x0028 /* Transmitter Timeguard Register */
#define AVR32_USART_FIDI_OFFSET    0x0040 /* FI DI Ratio Register */
#define AVR32_USART_NER_OFFSET     0x0044 /* Number of Errors Register */
#define AVR32_USART_IFR_OFFSET     0x004c /* IrDA Filter Register */
#define AVR32_USART_MAN_OFFSET     0x0050 /* Manchester Encoder Decoder Register */
#define AVR32_USART_VERSION_OFFSET 0x00fc /* Version Register */

/* Register Addresses ***************************************************************/

#define AVR32_USART0_CR            (AVR32_USART0_BASE+AVR32_USART_CR_OFFSET)
#define AVR32_USART0_MR            (AVR32_USART0_BASE+AVR32_USART_MR_OFFSET)
#define AVR32_USART0_IER           (AVR32_USART0_BASE+AVR32_USART_IER_OFFSET)
#define AVR32_USART0_IDR           (AVR32_USART0_BASE+AVR32_USART_IDR_OFFSET)
#define AVR32_USART0_IMR           (AVR32_USART0_BASE+AVR32_USART_IMR_OFFSET)
#define AVR32_USART0_CSR           (AVR32_USART0_BASE+AVR32_USART_CSR_OFFSET)
#define AVR32_USART0_RHR           (AVR32_USART0_BASE+AVR32_USART_RHR_OFFSET)
#define AVR32_USART0_THR           (AVR32_USART0_BASE+AVR32_USART_THR_OFFSET)
#define AVR32_USART0_BRGR          (AVR32_USART0_BASE+AVR32_USART_BRGR_OFFSET)
#define AVR32_USART0_RTOR          (AVR32_USART0_BASE+AVR32_USART_RTOR_OFFSET)
#define AVR32_USART0_TTGR          (AVR32_USART0_BASE+AVR32_USART_TTGR_OFFSET)
#define AVR32_USART0_FIDI          (AVR32_USART0_BASE+AVR32_USART_FIDI_OFFSET)
#define AVR32_USART0_NER           (AVR32_USART0_BASE+AVR32_USART_NER_OFFSET)
#define AVR32_USART0_IFR           (AVR32_USART0_BASE+AVR32_USART_IFR_OFFSET)
#define AVR32_USART0_MAN           (AVR32_USART0_BASE+AVR32_USART_MAN_OFFSET)
#define AVR32_USART0_VERSION       (AVR32_USART0_BASE+AVR32_USART_VERSION_OFFSET)

/* Register Bit-field Definitions ***************************************************/

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_AVR_SRC_AT91UC3_AT91UC3_USART_H */

