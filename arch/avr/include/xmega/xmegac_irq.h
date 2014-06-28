/****************************************************************************
 * arch/avr/include/xmega/xmegac_irq.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

/* This file should never be included directed but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_AVR_INCLUDE_XMEGA_XMEGAC_IRQ_H
#define __ARCH_AVR_INCLUDE_XMEGA_XMEGAC_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>
#include <arch/avr/avr.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* The ATmegaXXXc has 64 interrupt vectors including vector 0, the reset
 * vector.  Many of these are unused.  The remaining, valid interrupt vectors
 * are assigned IRQ numbers here:
 */

#define XMEGA_IRQ_OSCF     xx /* 0x0002 Crystal oscillator failure interrupt vector (NMI) */
#define XMEGA_IRQ_PORTC    xx /* 0x0004 Port C interrupt base */
#define XMEGA_IRQ_PORTR    xx /* 0x0008 Port R interrupt base */
#define XMEGA_IRQ_DMA      xx /* 0x000c DMA controller interrupt base */
#define XMEGA_IRQ_RTC      xx /* 0x0014 Real Time Counter Interrupt base */
#define XMEGA_IRQ_TWIC     xx /* 0x0018 Two-Wire Interface on Port C Interrupt base */
#define XMEGA_IRQ_TCC0     xx /* 0x001c Timer/Counter 0 on port C Interrupt base */
#define XMEGA_IRQ_TCC1     xx /* 0x0028 Timer/Counter 1 on port C Interrupt base */
#define XMEGA_IRQ_SPIC     xx /* 0x0030 SPI on port C Interrupt vector */
#define XMEGA_IRQ_USARTC0  xx /* 0x0032 USART 0 on port C Interrupt base */
#define XMEGA_IRQ_AES      xx /* 0x003e AES Interrupt vector */
#define XMEGA_IRQ_NVM      xx /* 0x0040 Non-Volatile Memory Interrupt base */
#define XMEGA_IRQ_PORTB    xx /* 0x0044 Port B Interrupt base */
#define XMEGA_IRQ_PORTE    xx /* 0x0056 Port E INT base */
#define XMEGA_IRQ_TWIE     xx /* 0x005a Two-Wire Interface on Port E Interrupt base */
#define XMEGA_IRQ_TCE0     xx /* 0x005e Timer/Counter 0 on port E Interrupt base */
#define XMEGA_IRQ_USARTE0  xx /* 0x0074 USART 0 on port E Interrupt base */
#define XMEGA_IRQ_PORTD    xx /* 0x0080 Port D Interrupt base */
#define XMEGA_IRQ_PORTA    xx /* 0x0084 Port A Interrupt base */
#define XMEGA_IRQ_ACA      xx /* 0x0088 Analog Comparator on Port A Interrupt base */
#define XMEGA_IRQ_ADCA     xx /* 0x008e Analog to Digital Converter on Port A Interrupt base */
#define XMEGA_IRQ_TCD0     xx /* 0x009a Timer/Counter 0 on port D Interrupt base */
#define XMEGA_IRQ_SPID     xx /* 0x00ae SPI D Interrupt vector */
#define XMEGA_IRQ_         xx /* 0x00b0 USARTD0 USART 0 on port D Interrupt base */
#define XMEGA_IRQ_USARTD1  xx /* 0x00b6 USART 1 on port D Interrupt base */
#define XMEGA_IRQ_PORTF    xx /* 0x00d0 Port F Interrupt base */
#define XMEGA_IRQ_TCF0     xx /* 0x00d8 Timer/Counter 0 on port F Interrupt base */
#define XMEGA_IRQ_USB      xx /* 0x00fa USB on port D Interrupt base */

#define NR_IRQS            xx

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifndef __ASSEMBLY__
#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_AVR_INCLUDE_XMEGA_XMEGAC_IRQ_H */

