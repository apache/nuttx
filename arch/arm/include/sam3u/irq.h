/****************************************************************************************
 * arch/arm/include/sam3u/irq.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************/

/* This file should never be included directed but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_SAM3U_IRQ_H
#define __ARCH_ARM_INCLUDE_SAM3U_IRQ_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

/****************************************************************************************
 * Definitions
 ****************************************************************************************/

/* SAM3U Peripheral Identifiers */

#define SAM3U_PID_SUPC           (0)  /* Supply Controller */
#define SAM3U_PID_RSTC           (1)  /* Reset Controller */
#define SAM3U_PID_RTC            (2)  /* Real Time Clock */
#define SAM3U_PID_RTT            (3)  /* Real Time Timer */
#define SAM3U_PID_WDT            (4)  /* Watchdog Timer */
#define SAM3U_PID_PMC            (5)  /* Power Management Controller */
#define SAM3U_PID_EEFC0          (6)  /* Enhanced Embedded Flash Controller 0 */
#define SAM3U_PID_EEFC1          (7)  /* Enhanced Embedded Flash Controller 1 */
#define SAM3U_PID_UART           (8)  /* Universal Asynchronous Receiver Transmitter */
#define SAM3U_PID_SMC            (9)  /* Static Memory Controller */
#define SAM3U_PID_PIOA          (10)  /* Parallel I/O Controller A */
#define SAM3U_PID_PIOB          (11)  /* Parallel I/O Controller B */
#define SAM3U_PID_PIOC          (12)  /* Parallel I/O Controller C */
#define SAM3U_PID_USART0        (13)  /* USART 0 */
#define SAM3U_PID_USART1        (14)  /* USART 1 */
#define SAM3U_PID_USART2        (15)  /* USART 2 */
#define SAM3U_PID_USART3        (16)  /* USART 3 */
#define SAM3U_PID_HSMCI         (17)  /* High Speed Multimedia Card Interface */
#define SAM3U_PID_TWI0          (18)  /* Two-Wire Interface 0 */
#define SAM3U_PID_TWI1          (19)  /* Two-Wire Interface 1 */
#define SAM3U_PID_SPI           (20)  /* Serial Peripheral Interface */
#define SAM3U_PID_SSC           (21)  /* Synchronous Serial Controller */
#define SAM3U_PID_TC0           (22)  /* Timer Counter 0 */
#define SAM3U_PID_TC1           (23)  /* Timer Counter 1 */
#define SAM3U_PID_TC2           (24)  /* Timer Counter 2 */
#define SAM3U_PID_PWM           (25)  /* Pulse Width Modulation Controller */
#define SAM3U_PID_ADC12B        (26)  /* 12-bit ADC Controller */
#define SAM3U_PID_ADC           (27)  /* 10-bit ADC Controller */
#define SAM3U_PID_DMAC          (28)  /* DMA Controller */
#define SAM3U_PID_UDPHS         (29)  /* USB Device High Speed */
#define NR_PIDS                 (30)  /* Number of peripheral identifiers */

/* IRQ numbers.  The IRQ number corresponds vector number and hence map directly to
 * bits in the NVIC.  This does, however, waste several words of memory in the IRQ
 * to handle mapping tables.
 */

/* Processor Exceptions (vectors 0-15) */

#define SAM3U_IRQ_RESERVED       (0) /* Reserved vector (only used with CONFIG_DEBUG) */
                                     /* Vector  0: Reset stack pointer value */
                                     /* Vector  1: Reset (not handler as an IRQ) */
#define SAM3U_IRQ_NMI            (2) /* Vector  2: Non-Maskable Interrupt (NMI) */
#define SAM3U_IRQ_HARDFAULT      (3) /* Vector  3: Hard fault */
#define SAM3U_IRQ_MPU            (4) /* Vector  4: Memory management (MPU) */
#define SAM3U_IRQ_BUSFAULT       (5) /* Vector  5: Bus fault */
#define SAM3U_IRQ_USAGEFAULT     (6) /* Vector  6: Usage fault */
#define SAM3U_IRQ_SVCALL        (11) /* Vector 11: SVC call */
#define SAM3U_IRQ_DBGMONITOR    (12) /* Vector 12: Debug Monitor */
                                     /* Vector 13: Reserved */
#define SAM3U_IRQ_PENDSV        (14) /* Vector 14: Pendable system service request */
#define SAM3U_IRQ_SYSTICK       (15) /* Vector 15: System tick */

/* External interrupts (vectors >= 16) */

#define SAM3U_IRQ_EXTINT        (16)
#define SAM3U_IRQ_SUPC          (SAM3U_IRQ_EXTINT+SAM3U_PID_SUPC)   /* Supply Controller */
#define SAM3U_IRQ_RSTC          (SAM3U_IRQ_EXTINT+SAM3U_PID_RSTC)   /* Reset Controller */
#define SAM3U_IRQ_RTC           (SAM3U_IRQ_EXTINT+SAM3U_PID_RTC)    /* Real Time Clock */
#define SAM3U_IRQ_RTT           (SAM3U_IRQ_EXTINT+SAM3U_PID_RTT)    /* Real Time Timer */
#define SAM3U_IRQ_WDT           (SAM3U_IRQ_EXTINT+SAM3U_PID_WDT)    /* Watchdog Timer */
#define SAM3U_IRQ_PMC           (SAM3U_IRQ_EXTINT+SAM3U_PID_PMC)    /* Power Management Controller */
#define SAM3U_IRQ_EEFC0         (SAM3U_IRQ_EXTINT+SAM3U_PID_EEFC0)  /* Enhanced Embedded Flash Controller 0 */
#define SAM3U_IRQ_EEFC1         (SAM3U_IRQ_EXTINT+SAM3U_PID_EEFC1)  /* Enhanced Embedded Flash Controller 1 */
#define SAM3U_IRQ_UART          (SAM3U_IRQ_EXTINT+SAM3U_PID_UART)   /* Universal Asynchronous Receiver Transmitter */
#define SAM3U_IRQ_SMC           (SAM3U_IRQ_EXTINT+SAM3U_PID_SMC)    /* Static Memory Controller */
#define SAM3U_IRQ_PIOA          (SAM3U_IRQ_EXTINT+SAM3U_PID_PIOA)   /* Parallel I/O Controller A */
#define SAM3U_IRQ_PIOB          (SAM3U_IRQ_EXTINT+SAM3U_PID_PIOB)   /* Parallel I/O Controller B */
#define SAM3U_IRQ_PIOC          (SAM3U_IRQ_EXTINT+SAM3U_PID_PIOC)   /* Parallel I/O Controller C */
#define SAM3U_IRQ_USART0        (SAM3U_IRQ_EXTINT+SAM3U_PID_USART0) /* USART 0 */
#define SAM3U_IRQ_USART1        (SAM3U_IRQ_EXTINT+SAM3U_PID_USART1) /* USART 1 */
#define SAM3U_IRQ_USART2        (SAM3U_IRQ_EXTINT+SAM3U_PID_USART2) /* USART 2 */
#define SAM3U_IRQ_USART3        (SAM3U_IRQ_EXTINT+SAM3U_PID_USART3) /* USART 3 */
#define SAM3U_IRQ_HSMCI         (SAM3U_IRQ_EXTINT+SAM3U_PID_HSMCI)  /* High Speed Multimedia Card Interface */
#define SAM3U_IRQ_TWI0          (SAM3U_IRQ_EXTINT+SAM3U_PID_TWI0)   /* Two-Wire Interface 0 */
#define SAM3U_IRQ_TWI1          (SAM3U_IRQ_EXTINT+SAM3U_PID_TWI1)   /* Two-Wire Interface 1 */
#define SAM3U_IRQ_SPI           (SAM3U_IRQ_EXTINT+SAM3U_PID_SPI)    /* Serial Peripheral Interface */
#define SAM3U_IRQ_SSC           (SAM3U_IRQ_EXTINT+SAM3U_PID_SSC)    /* Synchronous Serial Controller */
#define SAM3U_IRQ_TC0           (SAM3U_IRQ_EXTINT+SAM3U_PID_TC0)    /* Timer Counter 0 */
#define SAM3U_IRQ_TC1           (SAM3U_IRQ_EXTINT+SAM3U_PID_TC1)    /* Timer Counter 1 */
#define SAM3U_IRQ_TC2           (SAM3U_IRQ_EXTINT+SAM3U_PID_TC2)    /* Timer Counter 2 */
#define SAM3U_IRQ_PWM           (SAM3U_IRQ_EXTINT+SAM3U_PID_PWM)    /* Pulse Width Modulation Controller */
#define SAM3U_IRQ_ADC12B        (SAM3U_IRQ_EXTINT+SAM3U_PID_ADC12B) /* 12-bit ADC Controller */
#define SAM3U_IRQ_ADC           (SAM3U_IRQ_EXTINT+SAM3U_PID_ADC)    /* 10-bit ADC Controller */
#define SAM3U_IRQ_DMAC          (SAM3U_IRQ_EXTINT+SAM3U_PID_DMAC)   /* DMA Controller */
#define SAM3U_IRQ_UDPHS         (SAM3U_IRQ_EXTINT+SAM3U_PID_UDPHS)  /* USB Device High Speed */
#define NR_IRQS                 (SAM3U_IRQ_EXTINT+NR_PIDS)          /* Total number of IRQ numbers */

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Inline functions
 ****************************************************************************************/

/****************************************************************************************
 * Public Variables
 ****************************************************************************************/

/****************************************************************************************
 * Public Function Prototypes
 ****************************************************************************************/

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

#endif /* __ARCH_ARM_INCLUDE_SAM3U_IRQ_H */

