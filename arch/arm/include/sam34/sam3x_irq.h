/****************************************************************************
 * arch/arm/include/sam34/sam3x_irq.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_SAM34_SAM3X_IRQ_H
#define __ARCH_ARM_INCLUDE_SAM34_SAM3X_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* SAM3X Peripheral Identifiers */

#define SAM_PID_SUPC           (0)  /* Supply Controller */
#define SAM_PID_RSTC           (1)  /* Reset Controller */
#define SAM_PID_RTC            (2)  /* Real Time Clock */
#define SAM_PID_RTT            (3)  /* Real Time Timer */
#define SAM_PID_WDT            (4)  /* Watchdog Timer */
#define SAM_PID_PMC            (5)  /* Power Management Controller */
#define SAM_PID_EEFC0          (6)  /* Enhanced Embedded Flash Controller 0 */
#define SAM_PID_EEFC1          (7)  /* Enhanced Embedded Flash Controller 1 */
#define SAM_PID_UART0          (8)  /* Universal Asynchronous Receiver Transmitter 0 */
#define SAM_PID_SMC            (9)  /* Static Memory Controller */
#define SAM_PID_SDRAMC        (10)  /* Synchronous Dynamic RAM Controller */
#define SAM_PID_PIOA          (11)  /* Parallel I/O Controller A */
#define SAM_PID_PIOB          (12)  /* Parallel I/O Controller B */
#define SAM_PID_PIOC          (13)  /* Parallel I/O Controller C */
#define SAM_PID_PIOD          (14)  /* Parallel I/O Controller D */
#define SAM_PID_PIOE          (15)  /* Parallel I/O Controller E */
#define SAM_PID_PIOF          (16)  /* Parallel I/O Controller F */
#define SAM_PID_USART0        (17)  /* USART 0 */
#define SAM_PID_USART1        (18)  /* USART 1 */
#define SAM_PID_USART2        (19)  /* USART 2 */
#define SAM_PID_USART3        (20)  /* USART 3 */
#define SAM_PID_HSMCI         (21)  /* High Speed Multimedia Card Interface */
#define SAM_PID_TWI0          (22)  /* Two-Wire Interface 0 */
#define SAM_PID_TWI1          (23)  /* Two-Wire Interface 1 */
#define SAM_PID_SPI0          (24)  /* Serial Peripheral Interface 0 */
#define SAM_PID_SPI1          (25)  /* Serial Peripheral Interface 2 */
#define SAM_PID_SSC           (26)  /* Synchronous Serial Controller */
#define SAM_PID_TC0           (27)  /* Timer Counter 0 */
#define SAM_PID_TC1           (28)  /* Timer Counter 1 */
#define SAM_PID_TC2           (29)  /* Timer Counter 2 */
#define SAM_PID_TC3           (30)  /* Timer Counter 3 */
#define SAM_PID_TC4           (31)  /* Timer Counter 4 */
#define SAM_PID_TC5           (32)  /* Timer Counter 5 */
#define SAM_PID_TC6           (33)  /* Timer Counter 6 */
#define SAM_PID_TC7           (34)  /* Timer Counter 7 */
#define SAM_PID_TC8           (35)  /* Timer Counter 8 */
#define SAM_PID_PWM           (36)  /* Pulse Width Modulation Controller */
#define SAM_PID_ADC           (37)  /* ADC Controller */
#define SAM_PID_DACC          (38)  /* DAC Controller */
#define SAM_PID_DMAC          (39)  /* DMA Controller */
#define SAM_PID_UOTGHS        (40)  /* USB OTG High Speed */
#define SAM_PID_TRNG          (41)  /* True Random Number Generator */
#define SAM_PID_EMAC          (42)  /* Ethernet MAC */
#define SAM_PID_CAN0          (43)  /* CAN Controller 0 */
#define SAM_PID_CAN1          (44)  /* CAN Controller 1 */

#define NR_PIDS               (45)  /* Number of peripheral identifiers */

/* External interrupts (vectors >= 16) */

#define SAM_IRQ_SUPC          (SAM_IRQ_EXTINT+SAM_PID_SUPC)   /* Supply Controller */
#define SAM_IRQ_RSTC          (SAM_IRQ_EXTINT+SAM_PID_RSTC)   /* Reset Controller */
#define SAM_IRQ_RTC           (SAM_IRQ_EXTINT+SAM_PID_RTC)    /* Real Time Clock */
#define SAM_IRQ_RTT           (SAM_IRQ_EXTINT+SAM_PID_RTT)    /* Real Time Timer */
#define SAM_IRQ_WDT           (SAM_IRQ_EXTINT+SAM_PID_WDT)    /* Watchdog Timer */
#define SAM_IRQ_PMC           (SAM_IRQ_EXTINT+SAM_PID_PMC)    /* Power Management Controller */
#define SAM_IRQ_EEFC0         (SAM_IRQ_EXTINT+SAM_PID_EEFC0)  /* Enhanced Embedded Flash Controller 0 */
#define SAM_IRQ_EEFC1         (SAM_IRQ_EXTINT+SAM_PID_EEFC1)  /* Enhanced Embedded Flash Controller 1 */
#define SAM_IRQ_UART0         (SAM_IRQ_EXTINT+SAM_PID_UART0)  /* Universal Asynchronous Receiver Transmitter */
#define SAM_IRQ_SMC           (SAM_IRQ_EXTINT+SAM_PID_SMC)    /* Static Memory Controller */
#define SAM_IRQ_SDRAMC        (SAM_IRQ_EXTINT+SAM_PID_SDRAMC) /* Synchronous Dynamic RAM Controller */
#define SAM_IRQ_PIOA          (SAM_IRQ_EXTINT+SAM_PID_PIOA)   /* Parallel I/O Controller A */
#define SAM_IRQ_PIOB          (SAM_IRQ_EXTINT+SAM_PID_PIOB)   /* Parallel I/O Controller B */
#define SAM_IRQ_PIOC          (SAM_IRQ_EXTINT+SAM_PID_PIOC)   /* Parallel I/O Controller C */
#define SAM_IRQ_PIOD          (SAM_IRQ_EXTINT+SAM_PID_PIOD)   /* Parallel I/O Controller D */
#define SAM_IRQ_PIOE          (SAM_IRQ_EXTINT+SAM_PID_PIOE)   /* Parallel I/O Controller E */
#define SAM_IRQ_PIOF          (SAM_IRQ_EXTINT+SAM_PID_PIOF)   /* Parallel I/O Controller F */
#define SAM_IRQ_USART0        (SAM_IRQ_EXTINT+SAM_PID_USART0) /* USART 0 */
#define SAM_IRQ_USART1        (SAM_IRQ_EXTINT+SAM_PID_USART1) /* USART 1 */
#define SAM_IRQ_USART2        (SAM_IRQ_EXTINT+SAM_PID_USART2) /* USART 2 */
#define SAM_IRQ_USART3        (SAM_IRQ_EXTINT+SAM_PID_USART3) /* USART 3 */
#define SAM_IRQ_HSMCI         (SAM_IRQ_EXTINT+SAM_PID_HSMCI)  /* High Speed Multimedia Card Interface */
#define SAM_IRQ_TWI0          (SAM_IRQ_EXTINT+SAM_PID_TWI0)   /* Two-Wire Interface 0 */
#define SAM_IRQ_TWI1          (SAM_IRQ_EXTINT+SAM_PID_TWI1)   /* Two-Wire Interface 1 */
#define SAM_IRQ_SPI0          (SAM_IRQ_EXTINT+SAM_PID_SPI0)   /* Serial Peripheral Interface 0 */
#define SAM_IRQ_SPI1          (SAM_IRQ_EXTINT+SAM_PID_SPI1)   /* Serial Peripheral Interface 1 */
#define SAM_IRQ_SSC           (SAM_IRQ_EXTINT+SAM_PID_SSC)    /* Synchronous Serial Controller */
#define SAM_IRQ_TC0           (SAM_IRQ_EXTINT+SAM_PID_TC0)    /* Timer Counter 0 */
#define SAM_IRQ_TC1           (SAM_IRQ_EXTINT+SAM_PID_TC1)    /* Timer Counter 1 */
#define SAM_IRQ_TC2           (SAM_IRQ_EXTINT+SAM_PID_TC2)    /* Timer Counter 2 */
#define SAM_IRQ_TC3           (SAM_IRQ_EXTINT+SAM_PID_TC3)    /* Timer Counter 3 */
#define SAM_IRQ_TC4           (SAM_IRQ_EXTINT+SAM_PID_TC4)    /* Timer Counter 4 */
#define SAM_IRQ_TC5           (SAM_IRQ_EXTINT+SAM_PID_TC5)    /* Timer Counter 5 */
#define SAM_IRQ_TC6           (SAM_IRQ_EXTINT+SAM_PID_TC6)    /* Timer Counter 6 */
#define SAM_IRQ_TC7           (SAM_IRQ_EXTINT+SAM_PID_TC7)    /* Timer Counter 7 */
#define SAM_IRQ_TC8           (SAM_IRQ_EXTINT+SAM_PID_TC8)    /* Timer Counter 8 */
#define SAM_IRQ_PWM           (SAM_IRQ_EXTINT+SAM_PID_PWM)    /* Pulse Width Modulation Controller */
#define SAM_IRQ_ADC           (SAM_IRQ_EXTINT+SAM_PID_ADC)    /* ADC Controller */
#define SAM_IRQ_DACC          (SAM_IRQ_EXTINT+SAM_PID_DACC)   /* DAC Controller */
#define SAM_IRQ_DMAC          (SAM_IRQ_EXTINT+SAM_PID_DMAC)   /* DMA Controller */
#define SAM_IRQ_UOTGHS        (SAM_IRQ_EXTINT+SAM_PID_UOTGHS) /* USB OTG High Speed */
#define SAM_IRQ_TRNG          (SAM_IRQ_EXTINT+SAM_PID_TRNG)   /* True Random Number Generator */
#define SAM_IRQ_EMAC          (SAM_IRQ_EXTINT+SAM_PID_EMAC)   /* Ethernet MAC */
#define SAM_IRQ_CAN0          (SAM_IRQ_EXTINT+SAM_PID_CAN0)   /* CAN Controller 0 */
#define SAM_IRQ_CAN1          (SAM_IRQ_EXTINT+SAM_PID_CAN1)   /* CAN Controller 1 */

#define SAM_IRQ_NEXTINT       NR_PIDS                         /* Number of external interrupts */
#define SAM_IRQ_NIRQS         (SAM_IRQ_EXTINT+NR_PIDS)        /* The number of real IRQs */

/* GPIO interrupts (derived from SAM_IRQ_PIOA/B/C/D/E/F) */

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
#  define SAM_IRQ_PB15        (SAM_IRQ_GPIOB_PINS+15)         /* GPIOB, PIN 15 */
#  define SAM_IRQ_PB16        (SAM_IRQ_GPIOB_PINS+16)         /* GPIOB, PIN 16 */
#  define SAM_IRQ_PB17        (SAM_IRQ_GPIOB_PINS+17)         /* GPIOB, PIN 17 */
#  define SAM_IRQ_PB18        (SAM_IRQ_GPIOB_PINS+18)         /* GPIOB, PIN 18 */
#  define SAM_IRQ_PB19        (SAM_IRQ_GPIOB_PINS+19)         /* GPIOB, PIN 19 */
#  define SAM_IRQ_PB20        (SAM_IRQ_GPIOB_PINS+20)         /* GPIOB, PIN 20 */
#  define SAM_IRQ_PB21        (SAM_IRQ_GPIOB_PINS+21)         /* GPIOB, PIN 21 */
#  define SAM_IRQ_PB22        (SAM_IRQ_GPIOB_PINS+22)         /* GPIOB, PIN 22 */
#  define SAM_IRQ_PB23        (SAM_IRQ_GPIOB_PINS+23)         /* GPIOB, PIN 23 */
#  define SAM_IRQ_PB24        (SAM_IRQ_GPIOB_PINS+24)         /* GPIOB, PIN 24 */
#  define SAM_IRQ_PB25        (SAM_IRQ_GPIOB_PINS+25)         /* GPIOB, PIN 25 */
#  define SAM_IRQ_PB26        (SAM_IRQ_GPIOB_PINS+26)         /* GPIOB, PIN 26 */
#  define SAM_IRQ_PB27        (SAM_IRQ_GPIOB_PINS+27)         /* GPIOB, PIN 27 */
#  define SAM_IRQ_PB28        (SAM_IRQ_GPIOB_PINS+28)         /* GPIOB, PIN 28 */
#  define SAM_IRQ_PB29        (SAM_IRQ_GPIOB_PINS+29)         /* GPIOB, PIN 29 */
#  define SAM_IRQ_PB30        (SAM_IRQ_GPIOB_PINS+30)         /* GPIOB, PIN 30 */
#  define SAM_IRQ_PB31        (SAM_IRQ_GPIOB_PINS+31)         /* GPIOB, PIN 31 */
#  define SAM_NGPIOBIRQS      32
#else
#  define SAM_NGPIOBIRQS      0
#endif

#ifdef CONFIG_SAM34_GPIOC_IRQ
#  define SAM_IRQ_GPIOC_PINS  (SAM_IRQ_EXTINT + SAM_IRQ_NEXTINT + SAM_NGPIOAIRQS + \
                               SAM_NGPIOBIRQS)
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
#  define SAM_IRQ_GPIOD_PINS  (SAM_IRQ_EXTINT + SAM_IRQ_NEXTINT + SAM_NGPIOAIRQS + \
                               SAM_NGPIOBIRQS + SAM_NGPIOCIRQS)
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
#  define SAM_IRQ_GPIOE_PINS  (SAM_IRQ_EXTINT + SAM_IRQ_NEXTINT + SAM_NGPIOAIRQS + \
                               SAM_NGPIOBIRQS + SAM_NGPIOCIRQS + SAM_NGPIODIRQS)
#  define SAM_IRQ_PE0         (SAM_IRQ_GPIOE_PINS+0)          /* GPIOE, PIN 0 */
#  define SAM_IRQ_PE1         (SAM_IRQ_GPIOE_PINS+1)          /* GPIOE, PIN 1 */
#  define SAM_IRQ_PE2         (SAM_IRQ_GPIOE_PINS+2)          /* GPIOE, PIN 2 */
#  define SAM_IRQ_PE3         (SAM_IRQ_GPIOE_PINS+3)          /* GPIOE, PIN 3 */
#  define SAM_IRQ_PE4         (SAM_IRQ_GPIOE_PINS+4)          /* GPIOE, PIN 4 */
#  define SAM_IRQ_PE5         (SAM_IRQ_GPIOE_PINS+5)          /* GPIOE, PIN 5 */
#  define SAM_IRQ_PE6         (SAM_IRQ_GPIOE_PINS+6)          /* GPIOE, PIN 6 */
#  define SAM_IRQ_PE7         (SAM_IRQ_GPIOE_PINS+7)          /* GPIOE, PIN 7 */
#  define SAM_IRQ_PE8         (SAM_IRQ_GPIOE_PINS+8)          /* GPIOE, PIN 8 */
#  define SAM_IRQ_PE9         (SAM_IRQ_GPIOE_PINS+9)          /* GPIOE, PIN 9 */
#  define SAM_IRQ_PE10        (SAM_IRQ_GPIOE_PINS+10)         /* GPIOE, PIN 10 */
#  define SAM_IRQ_PE11        (SAM_IRQ_GPIOE_PINS+11)         /* GPIOE, PIN 11 */
#  define SAM_IRQ_PE12        (SAM_IRQ_GPIOE_PINS+12)         /* GPIOE, PIN 12 */
#  define SAM_IRQ_PE13        (SAM_IRQ_GPIOE_PINS+13)         /* GPIOE, PIN 13 */
#  define SAM_IRQ_PE14        (SAM_IRQ_GPIOE_PINS+14)         /* GPIOE, PIN 14 */
#  define SAM_IRQ_PE15        (SAM_IRQ_GPIOE_PINS+15)         /* GPIOE, PIN 15 */
#  define SAM_IRQ_PE16        (SAM_IRQ_GPIOE_PINS+16)         /* GPIOE, PIN 16 */
#  define SAM_IRQ_PE17        (SAM_IRQ_GPIOE_PINS+17)         /* GPIOE, PIN 17 */
#  define SAM_IRQ_PE18        (SAM_IRQ_GPIOE_PINS+18)         /* GPIOE, PIN 18 */
#  define SAM_IRQ_PE19        (SAM_IRQ_GPIOE_PINS+19)         /* GPIOE, PIN 19 */
#  define SAM_IRQ_PE20        (SAM_IRQ_GPIOE_PINS+20)         /* GPIOE, PIN 20 */
#  define SAM_IRQ_PE21        (SAM_IRQ_GPIOE_PINS+21)         /* GPIOE, PIN 21 */
#  define SAM_IRQ_PE22        (SAM_IRQ_GPIOE_PINS+22)         /* GPIOE, PIN 22 */
#  define SAM_IRQ_PE23        (SAM_IRQ_GPIOE_PINS+23)         /* GPIOE, PIN 23 */
#  define SAM_IRQ_PE24        (SAM_IRQ_GPIOE_PINS+24)         /* GPIOE, PIN 24 */
#  define SAM_IRQ_PE25        (SAM_IRQ_GPIOE_PINS+25)         /* GPIOE, PIN 25 */
#  define SAM_IRQ_PE26        (SAM_IRQ_GPIOE_PINS+26)         /* GPIOE, PIN 26 */
#  define SAM_IRQ_PE27        (SAM_IRQ_GPIOE_PINS+27)         /* GPIOE, PIN 27 */
#  define SAM_IRQ_PE28        (SAM_IRQ_GPIOE_PINS+28)         /* GPIOE, PIN 28 */
#  define SAM_IRQ_PE29        (SAM_IRQ_GPIOE_PINS+29)         /* GPIOE, PIN 29 */
#  define SAM_IRQ_PE30        (SAM_IRQ_GPIOE_PINS+30)         /* GPIOE, PIN 30 */
#  define SAM_IRQ_PE31        (SAM_IRQ_GPIOE_PINS+31)         /* GPIOE, PIN 31 */
#  define SAM_NGPIOEIRQS      32
#else
#  define SAM_NGPIOEIRQS      0
#endif

#ifdef CONFIG_SAM34_GPIOF_IRQ
#  define SAM_IRQ_GPIOF_PINS  (SAM_IRQ_EXTINT + SAM_IRQ_NEXTINT + SAM_NGPIOAIRQS + \
                               SAM_NGPIOBIRQS + SAM_NGPIOCIRQS + SAM_NGPIODIRQS + \
                               SAM_NGPIOEIRQS)
#  define SAM_IRQ_PF0         (SAM_IRQ_GPIOF_PINS+0)          /* GPIOF, PIN 0 */
#  define SAM_IRQ_PF1         (SAM_IRQ_GPIOF_PINS+1)          /* GPIOF, PIN 1 */
#  define SAM_IRQ_PF2         (SAM_IRQ_GPIOF_PINS+2)          /* GPIOF, PIN 2 */
#  define SAM_IRQ_PF3         (SAM_IRQ_GPIOF_PINS+3)          /* GPIOF, PIN 3 */
#  define SAM_IRQ_PF4         (SAM_IRQ_GPIOF_PINS+4)          /* GPIOF, PIN 4 */
#  define SAM_IRQ_PF5         (SAM_IRQ_GPIOF_PINS+5)          /* GPIOF, PIN 5 */
#  define SAM_IRQ_PF6         (SAM_IRQ_GPIOF_PINS+6)          /* GPIOF, PIN 6 */
#  define SAM_IRQ_PF7         (SAM_IRQ_GPIOF_PINS+7)          /* GPIOF, PIN 7 */
#  define SAM_IRQ_PF8         (SAM_IRQ_GPIOF_PINS+8)          /* GPIOF, PIN 8 */
#  define SAM_IRQ_PF9         (SAM_IRQ_GPIOF_PINS+9)          /* GPIOF, PIN 9 */
#  define SAM_IRQ_PF10        (SAM_IRQ_GPIOF_PINS+10)         /* GPIOF, PIN 10 */
#  define SAM_IRQ_PF11        (SAM_IRQ_GPIOF_PINS+11)         /* GPIOF, PIN 11 */
#  define SAM_IRQ_PF12        (SAM_IRQ_GPIOF_PINS+12)         /* GPIOF, PIN 12 */
#  define SAM_IRQ_PF13        (SAM_IRQ_GPIOF_PINS+13)         /* GPIOF, PIN 13 */
#  define SAM_IRQ_PF14        (SAM_IRQ_GPIOF_PINS+14)         /* GPIOF, PIN 14 */
#  define SAM_IRQ_PF15        (SAM_IRQ_GPIOF_PINS+15)         /* GPIOF, PIN 15 */
#  define SAM_IRQ_PF16        (SAM_IRQ_GPIOF_PINS+16)         /* GPIOF, PIN 16 */
#  define SAM_IRQ_PF17        (SAM_IRQ_GPIOF_PINS+17)         /* GPIOF, PIN 17 */
#  define SAM_IRQ_PF18        (SAM_IRQ_GPIOF_PINS+18)         /* GPIOF, PIN 18 */
#  define SAM_IRQ_PF19        (SAM_IRQ_GPIOF_PINS+19)         /* GPIOF, PIN 19 */
#  define SAM_IRQ_PF20        (SAM_IRQ_GPIOF_PINS+20)         /* GPIOF, PIN 20 */
#  define SAM_IRQ_PF21        (SAM_IRQ_GPIOF_PINS+21)         /* GPIOF, PIN 21 */
#  define SAM_IRQ_PF22        (SAM_IRQ_GPIOF_PINS+22)         /* GPIOF, PIN 22 */
#  define SAM_IRQ_PF23        (SAM_IRQ_GPIOF_PINS+23)         /* GPIOF, PIN 23 */
#  define SAM_IRQ_PF24        (SAM_IRQ_GPIOF_PINS+24)         /* GPIOF, PIN 24 */
#  define SAM_IRQ_PF25        (SAM_IRQ_GPIOF_PINS+25)         /* GPIOF, PIN 25 */
#  define SAM_IRQ_PF26        (SAM_IRQ_GPIOF_PINS+26)         /* GPIOF, PIN 26 */
#  define SAM_IRQ_PF27        (SAM_IRQ_GPIOF_PINS+27)         /* GPIOF, PIN 27 */
#  define SAM_IRQ_PF28        (SAM_IRQ_GPIOF_PINS+28)         /* GPIOF, PIN 28 */
#  define SAM_IRQ_PF29        (SAM_IRQ_GPIOF_PINS+29)         /* GPIOF, PIN 29 */
#  define SAM_IRQ_PF30        (SAM_IRQ_GPIOF_PINS+30)         /* GPIOF, PIN 30 */
#  define SAM_IRQ_PF31        (SAM_IRQ_GPIOF_PINS+31)         /* GPIOF, PIN 31 */
#  define SAM_NGPIOFIRQS      32
#else
#  define SAM_NGPIOFIRQS      0
#endif

/* Total number of IRQ numbers */

#define NR_IRQS               (SAM_IRQ_EXTINT + SAM_IRQ_NEXTINT + \
                               SAM_NGPIOAIRQS + SAM_NGPIOBIRQS + SAM_NGPIOCIRQS + \
                               SAM_NGPIODIRQS + SAM_NGPIOEIRQS + SAM_NGPIOFIRQS)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

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

#endif /* __ARCH_ARM_INCLUDE_SAM34_SAM3X_IRQ_H */
