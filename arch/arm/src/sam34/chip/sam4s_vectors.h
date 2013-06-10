/************************************************************************************************
 * arch/arm/src/sam34/chip/sam42_vectors.h
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
 ************************************************************************************************/

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/
/* This file is included by sam_vectors.S.  It provides the macro VECTOR that
 * supplies ach SAM3U vector in terms of a (lower-case) ISR label and an
 * (upper-case) IRQ number as defined in arch/arm/include/sam/sam3u_irq.h.
 * sam_vectors.S will defined the VECTOR in different ways in order to generate
 * the interrupt vectors and handlers in their final form.
 */

/* If the common ARMv7-M vector handling is used, then all it needs is the following
 * definition that provides the number of supported vectors.
 */

#ifdef CONFIG_ARMV7M_CMNVECTOR

/* Reserve 35 interrupt table entries for I/O interrupts. */

# define ARMV7M_PERIPHERAL_INTERRUPTS 35

#else
  VECTOR(sam_SUPC, SAM_IRQ_SUPC)      /* Vector 16+0: Supply Controller */
  VECTOR(sam_RSTC, SAM_IRQ_RSTC)      /* Vector 16+1: Reset Controller */
  VECTOR(sam_RTC, SAM_IRQ_RTC)        /* Vector 16+2: Real Time Clock */
  VECTOR(sam_RTT, SAM_IRQ_RTT)        /* Vector 16+3: Real Time Timer */
  VECTOR(sam_WDT, SAM_IRQ_WDT)        /* Vector 16+4: Watchdog Timer */
  VECTOR(sam_PMC, SAM_IRQ_PMC)        /* Vector 16+5: Power Management Controller */
  VECTOR(sam_EEFC0, SAM_IRQ_EEFC0)    /* Vector 16+6: Enhanced Embedded Flash Controller 0 */
  VECTOR(sam_EEFC1, SAM_IRQ_EEFC1)    /* Vector 16+7: Enhanced Embedded Flash Controller 1 */
  VECTOR(sam_UART0, SAM_IRQ_UART0)    /* Vector 16+8: Universal Asynchronous Receiver Transmitter 0 */
  VECTOR(sam_UART1, SAM_IRQ_UART1)    /* Vector 16+9: Universal Asynchronous Receiver Transmitter 1 */
  VECTOR(sam_SMC, SAM_IRQ_SMC)        /* Vector 16+10: Static Memory Controller */
  VECTOR(sam_PIOA, SAM_IRQ_PIOA)      /* Vector 16+11: Parallel I/O Controller A */
  VECTOR(sam_PIOB, SAM_IRQ_PIOB)      /* Vector 16+12: Parallel I/O Controller B */
  VECTOR(sam_PIOB, SAM_IRQ_PIOB)      /* Vector 16+13: Parallel I/O Controller C */
  VECTOR(sam_USART0, SAM_IRQ_USART0)  /* Vector 16+14: USART 0 */
  VECTOR(sam_USART1, SAM_IRQ_USART0)  /* Vector 16+15: USART 1 */
  UNUSED(SAM_IRQ_RESERVED_16)         /* Vector 16+16: Reserved */
  UNUSED(SAM_IRQ_RESERVED_17)         /* Vector 16+17: Reserved */
  VECTOR(sam_HSMCI, SAM_IRQ_HSMCI)    /* Vector 16+18: High Speed Multimedia Card Interface */
  VECTOR(sam_TWI0, SAM_IRQ_TWI0)      /* Vector 16+19: Two-Wire Interface 0 */
  VECTOR(sam_TWI1, SAM_IRQ_TWI1)      /* Vector 16+20: Two-Wire Interface 1 */
  VECTOR(sam_SSC, SAM_IRQ_SSC)        /* Vector 16+22: Synchronous Serial Controller */
  VECTOR(sam_TC0, SAM_IRQ_TC0)        /* Vector 16+23: Timer Counter 0 */
  VECTOR(sam_TC1, SAM_IRQ_TC1)        /* Vector 16+24: Timer Counter 1 */
  VECTOR(sam_TC2, SAM_IRQ_TC2)        /* Vector 16+25: Timer Counter 2 */
  VECTOR(sam_TC3, SAM_IRQ_TC3)        /* Vector 16+26: Timer Counter 3 */
  VECTOR(sam_TC4, SAM_IRQ_TC4)        /* Vector 16+27: Timer Counter 4 */
  VECTOR(sam_TC5, SAM_IRQ_TC5)        /* Vector 16+28: Timer Counter 5 */
  VECTOR(sam_ADC, SAM_IRQ_ADC)        /* Vector 16+29: Analog To Digital Converter */
  VECTOR(sam_DACC, SAM_IRQ_DACC)      /* Vector 16+30: Digital To Analog Converter */
  VECTOR(sam_PWM, SAM_IRQ_PWM)        /* Vector 16+31: Pulse Width Modulation */
  VECTOR(sam_CRCCU, SAM_IRQ_CRCCU)    /* Vector 16+32: CRC Calculation Unit */
  VECTOR(sam_ACC, SAM_IRQ_ACC)        /* Vector 16+33: Analog Comparator */
  VECTOR(sam_UDP, SAM_IRQ_UDP)        /* Vector 16+34: USB Device Port */
#endif
