/************************************************************************************************
 * arch/arm/src/sam34/chip/sam4cm_vectors.h
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
  VECTOR(sam_supc, SAM_IRQ_SUPC)      /* Vector 16+0: Supply Controller */
  VECTOR(sam_rstc, SAM_IRQ_RSTC)      /* Vector 16+1: Reset Controller */
  VECTOR(sam_rtc, SAM_IRQ_RTC)        /* Vector 16+2: Real Time Clock */
  VECTOR(sam_rtt, SAM_IRQ_RTT)        /* Vector 16+3: Real Time Timer */
  VECTOR(sam_wdt, SAM_IRQ_WDT)        /* Vector 16+4: Watchdog Timer */
  VECTOR(sam_pmc, SAM_IRQ_PMC)        /* Vector 16+5: Power Management Controller */
  VECTOR(sam_eefc0, SAM_IRQ_EEFC0)    /* Vector 16+6: Enhanced Embedded Flash Controller 0 */
  UNUSED(SAM_IRQ_RESERVED_7)          /* Vector 16+7: Reserved */
  VECTOR(sam_uart0, SAM_IRQ_UART0)    /* Vector 16+8: Universal Asynchronous Receiver Transmitter 0 */
  UNUSED(SAM_IRQ_RESERVED_9)          /* Vector 16+9: Reserved */
  UNUSED(SAM_IRQ_RESERVED_10)         /* Vector 16+10: Unused */
  VECTOR(sam_pioa, SAM_IRQ_PIOA)      /* Vector 16+11: Parallel I/O Controller A */
  VECTOR(sam_piob, SAM_IRQ_PIOB)      /* Vector 16+12: Parallel I/O Controller B */
  UNUSED(SAM_IRQ_RESERVED_13)         /* Vector 16+13: Reserved */
  VECTOR(sam_usart0, SAM_IRQ_USART0)  /* Vector 16+14: USART 0 */
  VECTOR(sam_usart1, SAM_IRQ_USART1)  /* Vector 16+15: USART 1 */
  VECTOR(sam_usart2, SAM_IRQ_USART2)  /* Vector 16+16: USART 2 */
  VECTOR(sam_usart3, SAM_IRQ_USART3)  /* Vector 16+17: USART 3 */
  UNUSED(SAM_IRQ_RESERVED_18)         /* Vector 16+18: Reserved */
  VECTOR(sam_twi0, SAM_IRQ_TWI0)      /* Vector 16+19: Two-Wire Interface 0 */
  VECTOR(sam_twi1, SAM_IRQ_TWI1)      /* Vector 16+20: Two-Wire Interface 1 */
  VECTOR(sam_spi0, SAM_IRQ_SPI0)      /* Vector 16+21: Serial Peripheral Interface */
  UNUSED(SAM_IRQ_RESERVED_22)         /* Vector 16+22: Reserved */
  VECTOR(sam_tc0, SAM_IRQ_TC0)        /* Vector 16+23: Timer Counter 0 */
  VECTOR(sam_tc1, SAM_IRQ_TC1)        /* Vector 16+24: Timer Counter 1 */
  VECTOR(sam_tc2, SAM_IRQ_TC2)        /* Vector 16+25: Timer Counter 2 */
  VECTOR(sam_tc3, SAM_IRQ_TC3)        /* Vector 16+26: Timer Counter 3 */
  VECTOR(sam_tc4, SAM_IRQ_TC4)        /* Vector 16+27: Timer Counter 4 */
  VECTOR(sam_tc5, SAM_IRQ_TC5)        /* Vector 16+28: Timer Counter 5 */
  VECTOR(sam_adc, SAM_IRQ_ADC)        /* Vector 16+29: Analog To Digital Converter */
  VECTOR(sam_arm, SAM_IRQ_ARM)        /* Vector 16+30: FPU signals (only on CM4P1 core): FPIXC, FPOFC, FPUFC, FPIOC, FPDZC, FPIDC, FPIXC */
  VECTOR(sam_ipc0, SAM_IRQ_IPC0)      /* Vector 16+31: Interprocessor communication 0 */
  VECTOR(sam_slcdc, SAM_IRQ_SLCDC)    /* Vector 16+32: Segment LCD Controller */
  VECTOR(sam_trng, SAM_IRQ_TRNG)      /* Vector 16+33: True Random Generator */
  VECTOR(sam_icm, SAM_IRQ_ICM)        /* Vector 16+34: Integrity Check Module */
  VECTOR(sam_cpkcc, SAM_IRQ_CPKCC)    /* Vector 16+35: Classical Public Key Cryptography Controller */
  VECTOR(sam_aes, SAM_IRQ_AES)        /* Vector 16+36: Advanced Enhanced Standard */
  VECTOR(sam_pioc, SAM_IRQ_PIOC)      /* Vector 16+37: Parallel I/O Controller C */
  VECTOR(sam_uart1, SAM_IRQ_UART1)    /* Vector 16+38: Universal Asynchronous Receiver Transmitter 1 */
  VECTOR(sam_ipc1, SAM_IRQ_IPC1)      /* Vector 16+39: Interprocessor communication 1 */
  UNUSED(SAM_IRQ_RESERVED_40)         /* Vector 16+40: Reserved */
  VECTOR(sam_pwm, SAM_IRQ_PWM)        /* Vector 16+41: Pulse Width Modulation */
//VECTOR(sam_sram, SAM_IRQ_SRAM)      /* Vector 16+42: SRAM1 (I/D Code bus of CM4P1), SRAM2 (Systembus of CM4P1) */
//VECTOR(sam_smc1, SAM_IRQ_SMC1)      /* Vector 16+43: Static Memory Controller 1 */
#endif