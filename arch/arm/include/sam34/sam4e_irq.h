/****************************************************************************************
 * arch/arm/include/sam34/sam4e_irq.h
 *
 *   Copyright (C) 2013-2014 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************/

/* This file should never be included directed but, rather, only indirectly through
 * nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_SAM34_SAM4E_IRQ_H
#define __ARCH_ARM_INCLUDE_SAM34_SAM4E_IRQ_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

/****************************************************************************************
 * Definitions
 ****************************************************************************************/

/* SAM4E Peripheral Identifiers */

#define SAM_PID_SUPC           (0)  /* Supply Controller */
#define SAM_PID_RSTC           (1)  /* Reset Controller */
#define SAM_PID_RTC            (2)  /* Real Time Clock */
#define SAM_PID_RTT            (3)  /* Real Time Timer */
#define SAM_PID_WDT            (4)  /* Watchdog Timer */
#define SAM_PID_PMC            (5)  /* Power Management Controller */
#define SAM_PID_EEFC0          (6)  /* Enhanced Embedded Flash Controller */
#define SAM_PID_UART0          (7)  /* Universal Asynchronous Receiver Transmitter 0 */
#define SAM_PID_SMC            (8)  /* Static Memory Controller */
#define SAM_PID_PIOA           (9)  /* Parallel I/O Controller A */
#define SAM_PID_PIOB          (10)  /* Parallel I/O Controller B */
#define SAM_PID_PIOC          (11)  /* Parallel I/O Controller C */
#define SAM_PID_PIOD          (12)  /* Parallel I/O Controller D */
#define SAM_PID_PIOE          (13)  /* Parallel I/O Controller E */
#define SAM_PID_USART0        (14)  /* USART 0 */
#define SAM_PID_USART1        (15)  /* USART 1 */
#define SAM_PID_HSMCI         (16)  /* High Speed Multimedia Card Interface */
#define SAM_PID_TWI0          (17)  /* Two-Wire Interface 0 */
#define SAM_PID_TWI1          (18)  /* Two-Wire Interface 1 */
#define SAM_PID_SPI0          (19)  /* Serial Peripheral Interface */
#define SAM_PID_DMAC          (20)  /* DMAC */
#define SAM_PID_TC0           (21)  /* Timer Counter 0 */
#define SAM_PID_TC1           (22)  /* Timer Counter 1 */
#define SAM_PID_TC2           (23)  /* Timer Counter 2 */
#define SAM_PID_TC3           (24)  /* Timer Counter 3 */
#define SAM_PID_TC4           (25)  /* Timer Counter 4 */
#define SAM_PID_TC5           (26)  /* Timer Counter 5 */
#define SAM_PID_TC6           (27)  /* Timer Counter 6 */
#define SAM_PID_TC7           (28)  /* Timer Counter 7 */
#define SAM_PID_TC8           (29)  /* Timer Counter 8 */
#define SAM_PID_AFEC0         (30)  /* Analog Front End 0 */
#define SAM_PID_AFEC1         (31)  /* Analog Front End 1 */
#define SAM_PID_DACC          (32)  /* Digital to Analog Converter */
#define SAM_PID_ACC           (33)  /* Analog Comparator */
#define SAM_PID_ARM           (34)  /* FPU signals: FPIXC, FPOFC, FPUFC, FPIOC, FPDZC,FPIDC, FPIXC */
#define SAM_PID_UDP           (35)  /* USB Device */
#define SAM_PID_PWM           (36)  /* PWM */
#define SAM_PID_CAN0          (37)  /* CAN0 */
#define SAM_PID_CAN1          (38)  /* CAN1 */
#define SAM_PID_AES           (39)  /* AES */
                                    /* 40-43: Reserved */
#define SAM_PID_EMAC          (44)  /* EMAC */
#define SAM_PID_UART1         (45)  /* UART */
                                    /* 46: Reserved */
#define NR_PIDS               (47)  /* Number of peripheral identifiers */

/* External interrupts (priority levels >= 256*/

#define SAM_IRQ_SUPC          (SAM_IRQ_EXTINT+SAM_PID_SUPC)   /* Supply Controller */
#define SAM_IRQ_RSTC          (SAM_IRQ_EXTINT+SAM_PID_RSTC)   /* Reset Controller */
#define SAM_IRQ_RTC           (SAM_IRQ_EXTINT+SAM_PID_RTC)    /* Real Time Clock */
#define SAM_IRQ_RTT           (SAM_IRQ_EXTINT+SAM_PID_RTT)    /* Real Time Timer */
#define SAM_IRQ_WDT           (SAM_IRQ_EXTINT+SAM_PID_WDT)    /* Watchdog Timer */
#define SAM_IRQ_PMC           (SAM_IRQ_EXTINT+SAM_PID_PMC)    /* Power Management Controller */
#define SAM_IRQ_EEFC0         (SAM_IRQ_EXTINT+SAM_PID_EEFC0)  /* Enhanced Embedded Flash Controller */
#define SAM_IRQ_UART0         (SAM_IRQ_EXTINT+SAM_PID_UART0)  /* Universal Asynchronous Receiver Transmitter 0 */
#define SAM_IRQ_PIOA          (SAM_IRQ_EXTINT+SAM_PID_PIOA)   /* Parallel I/O Controller A */
#define SAM_IRQ_PIOB          (SAM_IRQ_EXTINT+SAM_PID_PIOB)   /* Parallel I/O Controller B */

#define SAM_IRQ_PIOC          (SAM_IRQ_EXTINT+SAM_PID_PIOC)   /* Parallel I/O Controller C */
#define SAM_IRQ_PIOD          (SAM_IRQ_EXTINT+SAM_PID_PIOD)   /* Parallel I/O Controller D */
#define SAM_IRQ_PIOE          (SAM_IRQ_EXTINT+SAM_PID_PIOE)   /* Parallel I/O Controller E */
#define SAM_IRQ_USART0        (SAM_IRQ_EXTINT+SAM_PID_USART0) /* USART 0 */
#define SAM_IRQ_USART1        (SAM_IRQ_EXTINT+SAM_PID_USART1) /* USART 1 */
#define SAM_IRQ_HSMCI         (SAM_IRQ_EXTINT+SAM_PID_HSMCI)  /* High Speed Multimedia Card Interface */
#define SAM_IRQ_TWI0          (SAM_IRQ_EXTINT+SAM_PID_TWI0)   /* Two-Wire Interface 0 */
#define SAM_IRQ_TWI1          (SAM_IRQ_EXTINT+SAM_PID_TWI1)   /* Two-Wire Interface 1 */
#define SAM_IRQ_SPI0          (SAM_IRQ_EXTINT+SAM_PID_SPI0)   /* Serial Peripheral Interface */
#define SAM_IRQ_DMAC          (SAM_IRQ_EXTINT+SAM_PID_DMAC)   /* DMAC */
#define SAM_IRQ_TC0           (SAM_IRQ_EXTINT+SAM_PID_TC0)    /* Timer Counter 0 */
#define SAM_IRQ_TC1           (SAM_IRQ_EXTINT+SAM_PID_TC1)    /* Timer Counter 1 */
#define SAM_IRQ_TC2           (SAM_IRQ_EXTINT+SAM_PID_TC2)    /* Timer Counter 2 */
#define SAM_IRQ_TC3           (SAM_IRQ_EXTINT+SAM_PID_TC3)    /* Timer Counter 3 */
#define SAM_IRQ_TC4           (SAM_IRQ_EXTINT+SAM_PID_TC4)    /* Timer Counter 4 */
#define SAM_IRQ_TC5           (SAM_IRQ_EXTINT+SAM_PID_TC5)    /* Timer Counter 5 */
#define SAM_IRQ_TC6           (SAM_IRQ_EXTINT+SAM_PID_TC6)    /* Timer Counter 6 */
#define SAM_IRQ_TC7           (SAM_IRQ_EXTINT+SAM_PID_TC7)    /* Timer Counter 7 */
#define SAM_IRQ_TC8           (SAM_IRQ_EXTINT+SAM_PID_TC8)    /* Timer Counter 8 */
#define SAM_IRQ_AFEC0         (SAM_IRQ_EXTINT+SAM_PID_AFEC0)  /* Analog Front End 0 */
#define SAM_IRQ_AFEC1         (SAM_IRQ_EXTINT+SAM_PID_AFEC1)  /* Analog Front End 1 */
#define SAM_IRQ_DACC          (SAM_IRQ_EXTINT+SAM_PID_DACC)   /* Digital to Analog Converter */
#define SAM_IRQ_ACC           (SAM_IRQ_EXTINT+SAM_PID_ACC)    /* Analog Comparator */
#define SAM_IRQ_ARM           (SAM_IRQ_EXTINT+SAM_PID_ARM)    /* FPU signals: FPIXC, FPOFC, FPUFC, FPIOC, FPDZC,FPIDC, FPIXC */
#define SAM_IRQ_UDP           (SAM_IRQ_EXTINT+SAM_PID_UDP)    /* USB Device */
#define SAM_IRQ_PWM           (SAM_IRQ_EXTINT+SAM_PID_PWM)    /* PWM */
#define SAM_IRQ_CAN0          (SAM_IRQ_EXTINT+SAM_PID_CAN0)   /* CAN0 */
#define SAM_IRQ_CAN1          (SAM_IRQ_EXTINT+SAM_PID_CAN1)   /* CAN1 */
#define SAM_IRQ_AES           (SAM_IRQ_EXTINT+SAM_PID_AES)    /* AES */
#define SAM_IRQ_EMAC          (SAM_IRQ_EXTINT+SAM_PID_EMAC)   /* EMAC */
#define SAM_IRQ_UART1         (SAM_IRQ_EXTINT+SAM_PID_UART1)  /* UART1 */

#define SAM_IRQ_NEXTINT       NR_PIDS                         /* Total number of external interrupt numbers */
#define SAM_IRQ_NIRQS         (SAM_IRQ_EXTINT+NR_PIDS)        /* The number of real IRQs */

/* GPIO interrupts (derived from SAM_IRQ_PIOA/B/C) */

#ifdef CONFIG_SAM34_GPIOA_IRQ
#  define SAM_IRQ_GPIOA_PINS  (SAM_IRQ_EXTINT + SAM_IRQ_NEXTINT)
#  define SAM_IRQ_PA0         (SAM_IRQ_GPIOA_PINS+0)          /* GPIOA, PIN 0 */
#  define SAM_IRQ_PA1         (SAM_IRQ_GPIOA_PINS+1)          /* GPIOA, PIN 1 */
#  define SAM_IRQ_PA2         (SAM_IRQ_GPIOA_PINS+2)          /* GPIOA, PIN 2 */
#  define SAM_IRQ_PA3         (SAM_IRQ_GPIOA_PINS+3)          /* GPIOA, PIN 3 */
#  define SAM_IRQ_PA4         (SAM_IRQ_GPIOA_PINS+4)          /* GPIOA, PIN 4 */
#  define SAM_IRQ_PA5         (SAM_IRQ_GPIOA_PINS+5)          /* GPIOA, PIN 5 */
#  define SAM_IRQ_PA6         (SAM_IRQ_GPIOA_PINS+6)          /* GPIOA, PIN 6 */
#  define SAM_IRQ_PA7         (SAM_IRQ_GPIOA_PINS+7)          /* GPIOA, PIN 7 */
#  define SAM_IRQ_PA8         (SAM_IRQ_GPIOA_PINS+8)          /* GPIOA, PIN 8 */
#  define SAM_IRQ_PA9         (SAM_IRQ_GPIOA_PINS+9)          /* GPIOA, PIN 9 */
#  define SAM_IRQ_PA10        (SAM_IRQ_GPIOA_PINS+10)         /* GPIOA, PIN 10 */
#  define SAM_IRQ_PA11        (SAM_IRQ_GPIOA_PINS+11)         /* GPIOA, PIN 11 */
#  define SAM_IRQ_PA12        (SAM_IRQ_GPIOA_PINS+12)         /* GPIOA, PIN 12 */
#  define SAM_IRQ_PA13        (SAM_IRQ_GPIOA_PINS+13)         /* GPIOA, PIN 13 */
#  define SAM_IRQ_PA14        (SAM_IRQ_GPIOA_PINS+14)         /* GPIOA, PIN 14 */
#  define SAM_IRQ_PA15        (SAM_IRQ_GPIOA_PINS+15)         /* GPIOA, PIN 15 */
#  define SAM_IRQ_PA16        (SAM_IRQ_GPIOA_PINS+16)         /* GPIOA, PIN 16 */
#  define SAM_IRQ_PA17        (SAM_IRQ_GPIOA_PINS+17)         /* GPIOA, PIN 17 */
#  define SAM_IRQ_PA18        (SAM_IRQ_GPIOA_PINS+18)         /* GPIOA, PIN 18 */
#  define SAM_IRQ_PA19        (SAM_IRQ_GPIOA_PINS+19)         /* GPIOA, PIN 19 */
#  define SAM_IRQ_PA20        (SAM_IRQ_GPIOA_PINS+20)         /* GPIOA, PIN 20 */
#  define SAM_IRQ_PA21        (SAM_IRQ_GPIOA_PINS+21)         /* GPIOA, PIN 21 */
#  define SAM_IRQ_PA22        (SAM_IRQ_GPIOA_PINS+22)         /* GPIOA, PIN 22 */
#  define SAM_IRQ_PA23        (SAM_IRQ_GPIOA_PINS+23)         /* GPIOA, PIN 23 */
#  define SAM_IRQ_PA24        (SAM_IRQ_GPIOA_PINS+24)         /* GPIOA, PIN 24 */
#  define SAM_IRQ_PA25        (SAM_IRQ_GPIOA_PINS+25)         /* GPIOA, PIN 25 */
#  define SAM_IRQ_PA26        (SAM_IRQ_GPIOA_PINS+26)         /* GPIOA, PIN 26 */
#  define SAM_IRQ_PA27        (SAM_IRQ_GPIOA_PINS+27)         /* GPIOA, PIN 27 */
#  define SAM_IRQ_PA28        (SAM_IRQ_GPIOA_PINS+28)         /* GPIOA, PIN 28 */
#  define SAM_IRQ_PA29        (SAM_IRQ_GPIOA_PINS+29)         /* GPIOA, PIN 29 */
#  define SAM_IRQ_PA30        (SAM_IRQ_GPIOA_PINS+30)         /* GPIOA, PIN 30 */
#  define SAM_IRQ_PA31        (SAM_IRQ_GPIOA_PINS+31)         /* GPIOA, PIN 31 */
#  define SAM_NGPIOAIRQS      32
#else
#  define SAM_NGPIOAIRQS      0
#endif

#ifdef CONFIG_SAM34_GPIOB_IRQ
#  define SAM_IRQ_GPIOB_PINS  (SAM_IRQ_EXTINT + SAM_IRQ_NEXTINT + SAM_NGPIOAIRQS)
#  define SAM_IRQ_PB0         (SAM_IRQ_GPIOB_PINS+0)          /* GPIOB, PIN 0 */
#  define SAM_IRQ_PB1         (SAM_IRQ_GPIOB_PINS+1)          /* GPIOB, PIN 1 */
#  define SAM_IRQ_PB2         (SAM_IRQ_GPIOB_PINS+2)          /* GPIOB, PIN 2 */
#  define SAM_IRQ_PB3         (SAM_IRQ_GPIOB_PINS+3)          /* GPIOB, PIN 3 */
#  define SAM_IRQ_PB4         (SAM_IRQ_GPIOB_PINS+4)          /* GPIOB, PIN 4 */
#  define SAM_IRQ_PB5         (SAM_IRQ_GPIOB_PINS+5)          /* GPIOB, PIN 5 */
#  define SAM_IRQ_PB6         (SAM_IRQ_GPIOB_PINS+6)          /* GPIOB, PIN 6 */
#  define SAM_IRQ_PB7         (SAM_IRQ_GPIOB_PINS+7)          /* GPIOB, PIN 7 */
#  define SAM_IRQ_PB8         (SAM_IRQ_GPIOB_PINS+8)          /* GPIOB, PIN 8 */
#  define SAM_IRQ_PB9         (SAM_IRQ_GPIOB_PINS+9)          /* GPIOB, PIN 9 */
#  define SAM_IRQ_PB10        (SAM_IRQ_GPIOB_PINS+10)         /* GPIOB, PIN 10 */
#  define SAM_IRQ_PB11        (SAM_IRQ_GPIOB_PINS+11)         /* GPIOB, PIN 11 */
#  define SAM_IRQ_PB12        (SAM_IRQ_GPIOB_PINS+12)         /* GPIOB, PIN 12 */
#  define SAM_IRQ_PB13        (SAM_IRQ_GPIOB_PINS+13)         /* GPIOB, PIN 13 */
#  define SAM_IRQ_PB14        (SAM_IRQ_GPIOB_PINS+14)         /* GPIOB, PIN 14 */
#  define SAM_NGPIOBIRQS      15
#else
#  define SAM_NGPIOBIRQS      0
#endif

#ifdef CONFIG_SAM34_GPIOC_IRQ
#  define SAM_IRQ_GPIOC_PINS  (SAM_IRQ_EXTINT + SAM_IRQ_NEXTINT + SAM_NGPIOAIRQS + SAM_NGPIOBIRQS)
#  define SAM_IRQ_PC0         (SAM_IRQ_GPIOC_PINS+0)          /* GPIOC, PIN 0 */
#  define SAM_IRQ_PC1         (SAM_IRQ_GPIOC_PINS+1)          /* GPIOC, PIN 1 */
#  define SAM_IRQ_PC2         (SAM_IRQ_GPIOC_PINS+2)          /* GPIOC, PIN 2 */
#  define SAM_IRQ_PC3         (SAM_IRQ_GPIOC_PINS+3)          /* GPIOC, PIN 3 */
#  define SAM_IRQ_PC4         (SAM_IRQ_GPIOC_PINS+4)          /* GPIOC, PIN 4 */
#  define SAM_IRQ_PC5         (SAM_IRQ_GPIOC_PINS+5)          /* GPIOC, PIN 5 */
#  define SAM_IRQ_PC6         (SAM_IRQ_GPIOC_PINS+6)          /* GPIOC, PIN 6 */
#  define SAM_IRQ_PC7         (SAM_IRQ_GPIOC_PINS+7)          /* GPIOC, PIN 7 */
#  define SAM_IRQ_PC8         (SAM_IRQ_GPIOC_PINS+8)          /* GPIOC, PIN 8 */
#  define SAM_IRQ_PC9         (SAM_IRQ_GPIOC_PINS+9)          /* GPIOC, PIN 9 */
#  define SAM_IRQ_PC10        (SAM_IRQ_GPIOC_PINS+10)         /* GPIOC, PIN 10 */
#  define SAM_IRQ_PC11        (SAM_IRQ_GPIOC_PINS+11)         /* GPIOC, PIN 11 */
#  define SAM_IRQ_PC12        (SAM_IRQ_GPIOC_PINS+12)         /* GPIOC, PIN 12 */
#  define SAM_IRQ_PC13        (SAM_IRQ_GPIOC_PINS+13)         /* GPIOC, PIN 13 */
#  define SAM_IRQ_PC14        (SAM_IRQ_GPIOC_PINS+14)         /* GPIOC, PIN 14 */
#  define SAM_IRQ_PC15        (SAM_IRQ_GPIOC_PINS+15)         /* GPIOC, PIN 15 */
#  define SAM_IRQ_PC16        (SAM_IRQ_GPIOC_PINS+16)         /* GPIOC, PIN 16 */
#  define SAM_IRQ_PC17        (SAM_IRQ_GPIOC_PINS+17)         /* GPIOC, PIN 17 */
#  define SAM_IRQ_PC18        (SAM_IRQ_GPIOC_PINS+18)         /* GPIOC, PIN 18 */
#  define SAM_IRQ_PC19        (SAM_IRQ_GPIOC_PINS+19)         /* GPIOC, PIN 19 */
#  define SAM_IRQ_PC20        (SAM_IRQ_GPIOC_PINS+20)         /* GPIOC, PIN 20 */
#  define SAM_IRQ_PC21        (SAM_IRQ_GPIOC_PINS+21)         /* GPIOC, PIN 21 */
#  define SAM_IRQ_PC22        (SAM_IRQ_GPIOC_PINS+22)         /* GPIOC, PIN 22 */
#  define SAM_IRQ_PC23        (SAM_IRQ_GPIOC_PINS+23)         /* GPIOC, PIN 23 */
#  define SAM_IRQ_PC24        (SAM_IRQ_GPIOC_PINS+24)         /* GPIOC, PIN 24 */
#  define SAM_IRQ_PC25        (SAM_IRQ_GPIOC_PINS+25)         /* GPIOC, PIN 25 */
#  define SAM_IRQ_PC26        (SAM_IRQ_GPIOC_PINS+26)         /* GPIOC, PIN 26 */
#  define SAM_IRQ_PC27        (SAM_IRQ_GPIOC_PINS+27)         /* GPIOC, PIN 27 */
#  define SAM_IRQ_PC28        (SAM_IRQ_GPIOC_PINS+28)         /* GPIOC, PIN 28 */
#  define SAM_IRQ_PC29        (SAM_IRQ_GPIOC_PINS+29)         /* GPIOC, PIN 29 */
#  define SAM_IRQ_PC30        (SAM_IRQ_GPIOC_PINS+30)         /* GPIOC, PIN 30 */
#  define SAM_IRQ_PC31        (SAM_IRQ_GPIOC_PINS+31)         /* GPIOC, PIN 31 */
#  define SAM_NGPIOCIRQS      32
#else
#  define SAM_NGPIOCIRQS      0
#endif

#ifdef CONFIG_SAM34_GPIOD_IRQ
#  define SAM_IRQ_GPIOD_PINS  (SAM_IRQ_EXTINT + SAM_IRQ_NEXTINT + SAM_NGPIOAIRQS + SAM_NGPIOBIRQS)
#  define SAM_IRQ_PD0         (SAM_IRQ_GPIOD_PINS+0)          /* GPIOD, PIN 0 */
#  define SAM_IRQ_PD1         (SAM_IRQ_GPIOD_PINS+1)          /* GPIOD, PIN 1 */
#  define SAM_IRQ_PD2         (SAM_IRQ_GPIOD_PINS+2)          /* GPIOD, PIN 2 */
#  define SAM_IRQ_PD3         (SAM_IRQ_GPIOD_PINS+3)          /* GPIOD, PIN 3 */
#  define SAM_IRQ_PD4         (SAM_IRQ_GPIOD_PINS+4)          /* GPIOD, PIN 4 */
#  define SAM_IRQ_PD5         (SAM_IRQ_GPIOD_PINS+5)          /* GPIOD, PIN 5 */
#  define SAM_IRQ_PD6         (SAM_IRQ_GPIOD_PINS+6)          /* GPIOD, PIN 6 */
#  define SAM_IRQ_PD7         (SAM_IRQ_GPIOD_PINS+7)          /* GPIOD, PIN 7 */
#  define SAM_IRQ_PD8         (SAM_IRQ_GPIOD_PINS+8)          /* GPIOD, PIN 8 */
#  define SAM_IRQ_PD9         (SAM_IRQ_GPIOD_PINS+9)          /* GPIOD, PIN 9 */
#  define SAM_IRQ_PD10        (SAM_IRQ_GPIOD_PINS+10)         /* GPIOD, PIN 10 */
#  define SAM_IRQ_PD11        (SAM_IRQ_GPIOD_PINS+11)         /* GPIOD, PIN 11 */
#  define SAM_IRQ_PD12        (SAM_IRQ_GPIOD_PINS+12)         /* GPIOD, PIN 12 */
#  define SAM_IRQ_PD13        (SAM_IRQ_GPIOD_PINS+13)         /* GPIOD, PIN 13 */
#  define SAM_IRQ_PD14        (SAM_IRQ_GPIOD_PINS+14)         /* GPIOD, PIN 14 */
#  define SAM_IRQ_PD15        (SAM_IRQ_GPIOD_PINS+15)         /* GPIOD, PIN 15 */
#  define SAM_IRQ_PD16        (SAM_IRQ_GPIOD_PINS+16)         /* GPIOD, PIN 16 */
#  define SAM_IRQ_PD17        (SAM_IRQ_GPIOD_PINS+17)         /* GPIOD, PIN 17 */
#  define SAM_IRQ_PD18        (SAM_IRQ_GPIOD_PINS+18)         /* GPIOD, PIN 18 */
#  define SAM_IRQ_PD19        (SAM_IRQ_GPIOD_PINS+19)         /* GPIOD, PIN 19 */
#  define SAM_IRQ_PD20        (SAM_IRQ_GPIOD_PINS+20)         /* GPIOD, PIN 20 */
#  define SAM_IRQ_PD21        (SAM_IRQ_GPIOD_PINS+21)         /* GPIOD, PIN 21 */
#  define SAM_IRQ_PD22        (SAM_IRQ_GPIOD_PINS+22)         /* GPIOD, PIN 22 */
#  define SAM_IRQ_PD23        (SAM_IRQ_GPIOD_PINS+23)         /* GPIOD, PIN 23 */
#  define SAM_IRQ_PD24        (SAM_IRQ_GPIOD_PINS+24)         /* GPIOD, PIN 24 */
#  define SAM_IRQ_PD25        (SAM_IRQ_GPIOD_PINS+25)         /* GPIOD, PIN 25 */
#  define SAM_IRQ_PD26        (SAM_IRQ_GPIOD_PINS+26)         /* GPIOD, PIN 26 */
#  define SAM_IRQ_PD27        (SAM_IRQ_GPIOD_PINS+27)         /* GPIOD, PIN 27 */
#  define SAM_IRQ_PD28        (SAM_IRQ_GPIOD_PINS+28)         /* GPIOD, PIN 28 */
#  define SAM_IRQ_PD29        (SAM_IRQ_GPIOD_PINS+29)         /* GPIOD, PIN 29 */
#  define SAM_IRQ_PD30        (SAM_IRQ_GPIOD_PINS+30)         /* GPIOD, PIN 30 */
#  define SAM_IRQ_PD31        (SAM_IRQ_GPIOD_PINS+31)         /* GPIOD, PIN 31 */
#  define SAM_NGPIODIRQS      32
#else
#  define SAM_NGPIODIRQS      0
#endif

#ifdef CONFIG_SAM34_GPIOE_IRQ
#  define SAM_IRQ_GPIOE_PINS  (SAM_IRQ_EXTINT + SAM_IRQ_NEXTINT + SAM_NGPIOAIRQS + SAM_NGPIOBIRQS)
#  define SAM_IRQ_PE0         (SAM_IRQ_GPIOE_PINS+0)          /* GPIOE, PIN 0 */
#  define SAM_IRQ_PE1         (SAM_IRQ_GPIOE_PINS+1)          /* GPIOE, PIN 1 */
#  define SAM_IRQ_PE2         (SAM_IRQ_GPIOE_PINS+2)          /* GPIOE, PIN 2 */
#  define SAM_IRQ_PE3         (SAM_IRQ_GPIOE_PINS+3)          /* GPIOE, PIN 3 */
#  define SAM_IRQ_PE4         (SAM_IRQ_GPIOE_PINS+4)          /* GPIOE, PIN 4 */
#  define SAM_IRQ_PE5         (SAM_IRQ_GPIOE_PINS+5)          /* GPIOE, PIN 5 */
#  define SAM_NGPIOEIRQS      6
#else
#  define SAM_NGPIOEIRQS      0
#endif

/* Total number of IRQ numbers */

#define NR_VECTORS            SAM_IRQ_NIRQS
#define NR_IRQS               (SAM_IRQ_EXTINT + SAM_IRQ_NEXTINT + \
                               SAM_NGPIOAIRQS + SAM_NGPIOBIRQS + SAM_NGPIOCIRQS + \
                               SAM_NGPIODIRQS + SAM_NGPIOEIRQS)

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

#endif /* __ARCH_ARM_INCLUDE_SAM34_SAM4E_IRQ_H */
