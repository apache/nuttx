/****************************************************************************
 * arch/avr/include/xmega/xmegac_irq.h
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

/* This file should never be included directly but, rather, only indirectly
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
 * Pre-processor Definitions
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
#define AVR_PC_SIZE        xx

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifndef __ASSEMBLY__
#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_AVR_INCLUDE_XMEGA_XMEGAC_IRQ_H */
