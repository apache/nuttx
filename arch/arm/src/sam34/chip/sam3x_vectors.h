/************************************************************************************************
 * arch/arm/src/sam34/chip/sam3x_vectors.h
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

/* Reserve 45 interrupt table entries for I/O interrupts. */

#  define ARMV7M_PERIPHERAL_INTERRUPTS 45

#else
  VECTOR(sam_supc, SAM_IRQ_SUPC)      /* Vector 16+0:  Supply Controller */
  VECTOR(sam_rstc, SAM_IRQ_RSTC)      /* Vector 16+1:  Reset Controller */
  VECTOR(sam_rtc, SAM_IRQ_RTC)        /* Vector 16+2:  Real Time Clock */
  VECTOR(sam_rtt, SAM_IRQ_RTT)        /* Vector 16+3:  Real Time Timer */
  VECTOR(sam_wdt, SAM_IRQ_WDT)        /* Vector 16+4:  Watchdog Timer */
  VECTOR(sam_pmc, SAM_IRQ_PMC)        /* Vector 16+5:  Power Management Controller */
  VECTOR(sam_eefc0, SAM_IRQ_EEFC0)    /* Vector 16+6:  Enhanced Embedded Flash Controller 0 */
  VECTOR(sam_eefc1, SAM_IRQ_EEFC1)    /* Vector 16+7:  Enhanced Embedded Flash Controller 1 */
  VECTOR(sam_uart0, SAM_IRQ_UART0)    /* Vector 16+8:  Universal Asynchronous Receiver Transmitter */
  VECTOR(sam_smc, SAM_IRQ_SMC)        /* Vector 16+9:  Static Memory Controller */
  VECTOR(sam_sdramc, SAM_IRQ_SDRAMC)  /* Vector 16+10: Synchronous Dynamic RAM Controller */
  VECTOR(sam_pioa, SAM_IRQ_PIOA)      /* Vector 16+11: Parallel I/O Controller A */
  VECTOR(sam_piob, SAM_IRQ_PIOB)      /* Vector 16+12: Parallel I/O Controller B */
  VECTOR(sam_pioc, SAM_IRQ_PIOC)      /* Vector 16+13: Parallel I/O Controller C */
  VECTOR(sam_piod, SAM_IRQ_PIOD)      /* Vector 16+14: Parallel I/O Controller D */
  VECTOR(sam_pioe, SAM_IRQ_PIOE)      /* Vector 16+15: Parallel I/O Controller E */
  VECTOR(sam_piof, SAM_IRQ_PIOF)      /* Vector 16+16: Parallel I/O Controller F */
  VECTOR(sam_usart0, SAM_IRQ_USART0)  /* Vector 16+17: USART 0 */
  VECTOR(sam_usart1, SAM_IRQ_USART1)  /* Vector 16+18: USART 1 */
  VECTOR(sam_usart2, SAM_IRQ_USART2)  /* Vector 16+19: USART 2 */
  VECTOR(sam_usart3, SAM_IRQ_USART3)  /* Vector 16+20: USART 3 */
  VECTOR(sam_hsmci, SAM_IRQ_HSMCI)    /* Vector 16+21: High Speed Multimedia Card Interface */
  VECTOR(sam_twi0, SAM_IRQ_TWI0)      /* Vector 16+22: Two-Wire Interface 0 */
  VECTOR(sam_twi1, SAM_IRQ_TWI1)      /* Vector 16+23: Two-Wire Interface 1 */
  VECTOR(sam_spi0, SAM_IRQ_SPI0)      /* Vector 16+24: Serial Peripheral Interface 0 */
  VECTOR(sam_spi1, SAM_IRQ_SPI1)      /* Vector 16+25: Serial Peripheral Interface 1 */
  VECTOR(sam_ssc, SAM_IRQ_SSC)        /* Vector 16+26: Synchronous Serial Controller */
  VECTOR(sam_tc0, SAM_IRQ_TC0)        /* Vector 16+27: Timer Counter 0 */
  VECTOR(sam_tc1, SAM_IRQ_TC1)        /* Vector 16+28: Timer Counter 1 */
  VECTOR(sam_tc2, SAM_IRQ_TC2)        /* Vector 16+29: Timer Counter 2 */
  VECTOR(sam_tc3, SAM_IRQ_TC3)        /* Vector 16+30: Timer Counter 3 */
  VECTOR(sam_tc4, SAM_IRQ_TC4)        /* Vector 16+31: Timer Counter 4 */
  VECTOR(sam_tc5, SAM_IRQ_TC5)        /* Vector 16+32: Timer Counter 5 */
  VECTOR(sam_tc6, SAM_IRQ_TC6)        /* Vector 16+33: Timer Counter 6 */
  VECTOR(sam_tc7, SAM_IRQ_TC7)        /* Vector 16+34: Timer Counter 7 */
  VECTOR(sam_tc8, SAM_IRQ_TC8)        /* Vector 16+35: Timer Counter 8 */
  VECTOR(sam_pwm, SAM_IRQ_PWM)        /* Vector 16+36: Pulse Width Modulation Controller */
  VECTOR(sam_adc, SAM_IRQ_ADC)        /* Vector 16+37: ADC Controller */
  VECTOR(sam_dacc, SAM_IRQ_DACC)      /* Vector 16+38: DAC Controller */
  VECTOR(sam_dmac, SAM_IRQ_DMAC)      /* Vector 16+39: DMA Controller */
  VECTOR(sam_uotghs, SAM_IRQ_UOTGHS)  /* Vector 16+40: USB OTG High Speed */
  VECTOR(sam_trng, SAM_IRQ_TRNG)      /* Vector 16+41: True Random Number Generator */
  VECTOR(sam_emac, SAM_IRQ_EMAC)      /* Vector 16+42: Ethernet MAC */
  VECTOR(sam_can0, SAM_IRQ_CAN0)      /* Vector 16+43: CAN Controller 0 */
  VECTOR(sam_can1, SAM_IRQ_CAN1)      /* Vector 16+44: CAN Controller 1 */
#endif
