/****************************************************************************
 * arch/arm/include/sama5/sama5d3_irq.h
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

#ifndef __ARCH_ARM_INCLUDE_SAMA5_SAMA5D3_IRQ_H
#define __ARCH_ARM_INCLUDE_SAMA5_SAMA5D3_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* SAMA5D3 Peripheral Identifiers */

#define SAM_PID_FIQ            (0)  /* Advanced Interrupt Controller FIQ */
#define SAM_PID_SYS            (1)  /* System Controller Interrupt PMC */
#define SAM_PID_DBGU           (2)  /* Debug Unit Interrupt */
#define SAM_PID_PIT            (3)  /* Periodic Interval Timer Interrupt */
#define SAM_PID_WDT            (4)  /* Watchdog timer Interrupt */
#define SAM_PID_HSMC           (5)  /* Multi-bit ECC Interrupt */
#define SAM_PID_PIOA           (6)  /* Parallel I/O Controller A */
#define SAM_PID_PIOB           (7)  /* Parallel I/O Controller B */
#define SAM_PID_PIOC           (8)  /* Parallel I/O Controller C */
#define SAM_PID_PIOD           (9)  /* Parallel I/O Controller D */
#define SAM_PID_PIOE           (10) /* Parallel I/O Controller E */
#define SAM_PID_SMD            (11) /* SMD Soft Modem */
#define SAM_PID_USART0         (12) /* USART 0 */
#define SAM_PID_USART1         (13) /* USART 1 */
#define SAM_PID_USART2         (14) /* USART 2 */
#define SAM_PID_USART3         (15) /* USART 3 */
#define SAM_PID_UART0          (16) /* UART 0 */
#define SAM_PID_UART1          (17) /* UART 1 */
#define SAM_PID_TWI0           (18) /* Two-Wire Interface 0 */
#define SAM_PID_TWI1           (19) /* Two-Wire Interface 1 */
#define SAM_PID_TWI2           (20) /* Two-Wire Interface 2 */
#define SAM_PID_HSMCI0         (21) /* High Speed Multimedia Card Interface 0 */
#define SAM_PID_HSMCI1         (22) /* High Speed Multimedia Card Interface 1 */
#define SAM_PID_HSMCI2         (23) /* High Speed Multimedia Card Interface 2 */
#define SAM_PID_SPI0           (24) /* Serial Peripheral Interface 0 */
#define SAM_PID_SPI1           (25) /* Serial Peripheral Interface 1 */
#define SAM_PID_TC0            (26) /* Timer Counter 0 (ch. 0, 1, 2) */
#define SAM_PID_TC1            (27) /* Timer Counter 1 (ch. 3, 4, 5) */
#define SAM_PID_PWM            (28) /* Pulse Width Modulation Controller */
#define SAM_PID_ADC            (29) /* Touch Screen ADC Controller */
#define SAM_PID_DMAC0          (30) /* DMA Controller 0 */
#define SAM_PID_DMAC1          (31) /* DMA Controller 1 */
#define SAM_PID_UHPHS          (32) /* USB Host High Speed */
#define SAM_PID_UDPHS          (33) /* USB Device High Speed */
#define SAM_PID_GMAC           (34) /* Gigabit Ethernet MAC */
#define SAM_PID_EMAC           (35) /* Ethernet MAC */
#define SAM_PID_LCDC           (36) /* LCD Controller */
#define SAM_PID_ISI            (37) /* Image Sensor Interface */
#define SAM_PID_SSC0           (38) /* Synchronous Serial Controller 0 */
#define SAM_PID_SSC1           (39) /* Synchronous Serial Controller 1 */
#define SAM_PID_CAN0           (40) /* CAN controller 0 */
#define SAM_PID_CAN1           (41) /* CAN controller 1 */
#define SAM_PID_SHA            (42) /* Secure Hash Algorithm */
#define SAM_PID_AES            (43) /* Advanced Encryption Standard */
#define SAM_PID_TDES           (44) /* Triple Data Encryption Standard */
#define SAM_PID_TRNG           (45) /* True Random Number Generator */
#define SAM_PID_ARM            (46) /* Performance Monitor Unit */
#define SAM_PID_AIC            (47) /* Advanced Interrupt Controller IRQ */
#define SAM_PID_FUSE           (48) /* Fuse Controller */
#define SAM_PID_MPDDRC         (49) /* MPDDR controller */
                                    /* 50-63 Reserved */

/* External interrupts vectors numbers (same as peripheral ID) */

#define SAM_IRQ_FIQ            SAM_PID_FIQ    /* Advanced Interrupt Controller FIQ */
#define SAM_IRQ_SYS            SAM_PID_SYS    /* System Controller Interrupt PMC */
#define SAM_IRQ_DBGU           SAM_PID_DBGU   /* Debug Unit Interrupt */
#define SAM_IRQ_PIT            SAM_PID_PIT    /* Periodic Interval Timer Interrupt */
#define SAM_IRQ_WDT            SAM_PID_WDT    /* Watchdog timer Interrupt */
#define SAM_IRQ_HSMC           SAM_PID_HSMC   /* Multi-bit ECC Interrupt */
#define SAM_IRQ_PIOA           SAM_PID_PIOA   /* Parallel I/O Controller A */
#define SAM_IRQ_PIOB           SAM_PID_PIOB   /* Parallel I/O Controller B */
#define SAM_IRQ_PIOC           SAM_PID_PIOC   /* Parallel I/O Controller C */
#define SAM_IRQ_PIOD           SAM_PID_PIOD   /* Parallel I/O Controller D */
#define SAM_IRQ_PIOE           SAM_PID_PIOE   /* Parallel I/O Controller E */
#define SAM_IRQ_SMD            SAM_PID_SMD    /* SMD Soft Modem */
#define SAM_IRQ_USART0         SAM_PID_USART0 /* USART 0 */
#define SAM_IRQ_USART1         SAM_PID_USART1 /* USART 1 */
#define SAM_IRQ_USART2         SAM_PID_USART2 /* USART 2 */
#define SAM_IRQ_USART3         SAM_PID_USART3 /* USART 3 */
#define SAM_IRQ_UART0          SAM_PID_UART0  /* UART 0 */
#define SAM_IRQ_UART1          SAM_PID_UART1  /* UART 1 */
#define SAM_IRQ_TWI0           SAM_PID_TWI0   /* Two-Wire Interface 0 */
#define SAM_IRQ_TWI1           SAM_PID_TWI1   /* Two-Wire Interface 1 */
#define SAM_IRQ_TWI2           SAM_PID_TWI2   /* Two-Wire Interface 2 */
#define SAM_IRQ_HSMCI0         SAM_PID_HSMCI0 /* High Speed Multimedia Card Interface 0 */
#define SAM_IRQ_HSMCI1         SAM_PID_HSMCI1 /* High Speed Multimedia Card Interface 1 */
#define SAM_IRQ_HSMCI2         SAM_PID_HSMCI2 /* High Speed Multimedia Card Interface 2 */
#define SAM_IRQ_SPI0           SAM_PID_SPI0   /* Serial Peripheral Interface 0 */
#define SAM_IRQ_SPI1           SAM_PID_SPI1   /* Serial Peripheral Interface 1 */
#define SAM_IRQ_TC0            SAM_PID_TC0    /* Timer Counter 0 (ch. 0, 1, 2) */
#define SAM_IRQ_TC1            SAM_PID_TC1    /* Timer Counter 1 (ch. 3, 4, 5) */
#define SAM_IRQ_PWM            SAM_PID_PWM    /* Pulse Width Modulation Controller */
#define SAM_IRQ_ADC            SAM_PID_ADC    /* Touch Screen ADC Controller */
#define SAM_IRQ_DMAC0          SAM_PID_DMAC0  /* DMA Controller 0 */
#define SAM_IRQ_DMAC1          SAM_PID_DMAC1  /* DMA Controller 1 */
#define SAM_IRQ_UHPHS          SAM_PID_UHPHS  /* USB Host High Speed */
#define SAM_IRQ_UDPHS          SAM_PID_UDPHS  /* USB Device High Speed */
#define SAM_IRQ_GMAC           SAM_PID_GMAC   /* Gigabit Ethernet MAC */
#define SAM_IRQ_EMAC           SAM_PID_EMAC   /* Ethernet MAC */
#define SAM_IRQ_LCDC           SAM_PID_LCDC   /* LCD Controller */
#define SAM_IRQ_ISI            SAM_PID_ISI    /* Image Sensor Interface */
#define SAM_IRQ_SSC0           SAM_PID_SSC0   /* Synchronous Serial Controller 0 */
#define SAM_IRQ_SSC1           SAM_PID_SSC1   /* Synchronous Serial Controller 1 */
#define SAM_IRQ_CAN0           SAM_PID_CAN0   /* CAN controller 0 */
#define SAM_IRQ_CAN1           SAM_PID_CAN1   /* CAN controller 1 */
#define SAM_IRQ_SHA            SAM_PID_SHA    /* Secure Hash Algorithm */
#define SAM_IRQ_AES            SAM_PID_AES    /* Advanced Encryption Standard */
#define SAM_IRQ_TDES           SAM_PID_TDES   /* Triple Data Encryption Standard */
#define SAM_IRQ_TRNG           SAM_PID_TRNG   /* True Random Number Generator */
#define SAM_IRQ_ARM            SAM_PID_ARM    /* Performance Monitor Unit */
#define SAM_IRQ_AIC            SAM_PID_AIC    /* Advanced Interrupt Controller IRQ */
#define SAM_IRQ_FUSE           SAM_PID_FUSE   /* Fuse Controller */
#define SAM_IRQ_MPDDRC         SAM_PID_MPDDRC /* MPDDR controller */

#define SAM_IRQ_NINT           (SAM_PID_MPDDRC + 1)

/* PIO interrupts (derived from SAM_IRQ_PIOA/B/C/D/E/F) */

#ifdef CONFIG_SAMA5_PIOA_IRQ
#  define SAM_IRQ_PIOA_PINS    (SAM_IRQ_NINT)
#  define SAM_IRQ_PA0          (SAM_IRQ_PIOA_PINS+0)  /* PIOA, PIN 0 */
#  define SAM_IRQ_PA1          (SAM_IRQ_PIOA_PINS+1)  /* PIOA, PIN 1 */
#  define SAM_IRQ_PA2          (SAM_IRQ_PIOA_PINS+2)  /* PIOA, PIN 2 */
#  define SAM_IRQ_PA3          (SAM_IRQ_PIOA_PINS+3)  /* PIOA, PIN 3 */
#  define SAM_IRQ_PA4          (SAM_IRQ_PIOA_PINS+4)  /* PIOA, PIN 4 */
#  define SAM_IRQ_PA5          (SAM_IRQ_PIOA_PINS+5)  /* PIOA, PIN 5 */
#  define SAM_IRQ_PA6          (SAM_IRQ_PIOA_PINS+6)  /* PIOA, PIN 6 */
#  define SAM_IRQ_PA7          (SAM_IRQ_PIOA_PINS+7)  /* PIOA, PIN 7 */
#  define SAM_IRQ_PA8          (SAM_IRQ_PIOA_PINS+8)  /* PIOA, PIN 8 */
#  define SAM_IRQ_PA9          (SAM_IRQ_PIOA_PINS+9)  /* PIOA, PIN 9 */
#  define SAM_IRQ_PA10         (SAM_IRQ_PIOA_PINS+10) /* PIOA, PIN 10 */
#  define SAM_IRQ_PA11         (SAM_IRQ_PIOA_PINS+11) /* PIOA, PIN 11 */
#  define SAM_IRQ_PA12         (SAM_IRQ_PIOA_PINS+12) /* PIOA, PIN 12 */
#  define SAM_IRQ_PA13         (SAM_IRQ_PIOA_PINS+13) /* PIOA, PIN 13 */
#  define SAM_IRQ_PA14         (SAM_IRQ_PIOA_PINS+14) /* PIOA, PIN 14 */
#  define SAM_IRQ_PA15         (SAM_IRQ_PIOA_PINS+15) /* PIOA, PIN 15 */
#  define SAM_IRQ_PA16         (SAM_IRQ_PIOA_PINS+16) /* PIOA, PIN 16 */
#  define SAM_IRQ_PA17         (SAM_IRQ_PIOA_PINS+17) /* PIOA, PIN 17 */
#  define SAM_IRQ_PA18         (SAM_IRQ_PIOA_PINS+18) /* PIOA, PIN 18 */
#  define SAM_IRQ_PA19         (SAM_IRQ_PIOA_PINS+19) /* PIOA, PIN 19 */
#  define SAM_IRQ_PA20         (SAM_IRQ_PIOA_PINS+20) /* PIOA, PIN 20 */
#  define SAM_IRQ_PA21         (SAM_IRQ_PIOA_PINS+21) /* PIOA, PIN 21 */
#  define SAM_IRQ_PA22         (SAM_IRQ_PIOA_PINS+22) /* PIOA, PIN 22 */
#  define SAM_IRQ_PA23         (SAM_IRQ_PIOA_PINS+23) /* PIOA, PIN 23 */
#  define SAM_IRQ_PA24         (SAM_IRQ_PIOA_PINS+24) /* PIOA, PIN 24 */
#  define SAM_IRQ_PA25         (SAM_IRQ_PIOA_PINS+25) /* PIOA, PIN 25 */
#  define SAM_IRQ_PA26         (SAM_IRQ_PIOA_PINS+26) /* PIOA, PIN 26 */
#  define SAM_IRQ_PA27         (SAM_IRQ_PIOA_PINS+27) /* PIOA, PIN 27 */
#  define SAM_IRQ_PA28         (SAM_IRQ_PIOA_PINS+28) /* PIOA, PIN 28 */
#  define SAM_IRQ_PA29         (SAM_IRQ_PIOA_PINS+29) /* PIOA, PIN 29 */
#  define SAM_IRQ_PA30         (SAM_IRQ_PIOA_PINS+30) /* PIOA, PIN 30 */
#  define SAM_IRQ_PA31         (SAM_IRQ_PIOA_PINS+31) /* PIOA, PIN 31 */
#  define SAM_NPIOAIRQS        32
#else
#  define SAM_NPIOAIRQS        0
#endif

#ifdef CONFIG_SAMA5_PIOB_IRQ
#  define SAM_IRQ_PIOB_PINS    (SAM_IRQ_NINT + SAM_NPIOAIRQS)
#  define SAM_IRQ_PB0          (SAM_IRQ_PIOB_PINS+0)  /* PIOB, PIN 0 */
#  define SAM_IRQ_PB1          (SAM_IRQ_PIOB_PINS+1)  /* PIOB, PIN 1 */
#  define SAM_IRQ_PB2          (SAM_IRQ_PIOB_PINS+2)  /* PIOB, PIN 2 */
#  define SAM_IRQ_PB3          (SAM_IRQ_PIOB_PINS+3)  /* PIOB, PIN 3 */
#  define SAM_IRQ_PB4          (SAM_IRQ_PIOB_PINS+4)  /* PIOB, PIN 4 */
#  define SAM_IRQ_PB5          (SAM_IRQ_PIOB_PINS+5)  /* PIOB, PIN 5 */
#  define SAM_IRQ_PB6          (SAM_IRQ_PIOB_PINS+6)  /* PIOB, PIN 6 */
#  define SAM_IRQ_PB7          (SAM_IRQ_PIOB_PINS+7)  /* PIOB, PIN 7 */
#  define SAM_IRQ_PB8          (SAM_IRQ_PIOB_PINS+8)  /* PIOB, PIN 8 */
#  define SAM_IRQ_PB9          (SAM_IRQ_PIOB_PINS+9)  /* PIOB, PIN 9 */
#  define SAM_IRQ_PB10         (SAM_IRQ_PIOB_PINS+10) /* PIOB, PIN 10 */
#  define SAM_IRQ_PB11         (SAM_IRQ_PIOB_PINS+11) /* PIOB, PIN 11 */
#  define SAM_IRQ_PB12         (SAM_IRQ_PIOB_PINS+12) /* PIOB, PIN 12 */
#  define SAM_IRQ_PB13         (SAM_IRQ_PIOB_PINS+13) /* PIOB, PIN 13 */
#  define SAM_IRQ_PB14         (SAM_IRQ_PIOB_PINS+14) /* PIOB, PIN 14 */
#  define SAM_IRQ_PB15         (SAM_IRQ_PIOB_PINS+15) /* PIOB, PIN 15 */
#  define SAM_IRQ_PB16         (SAM_IRQ_PIOB_PINS+16) /* PIOB, PIN 16 */
#  define SAM_IRQ_PB17         (SAM_IRQ_PIOB_PINS+17) /* PIOB, PIN 17 */
#  define SAM_IRQ_PB18         (SAM_IRQ_PIOB_PINS+18) /* PIOB, PIN 18 */
#  define SAM_IRQ_PB19         (SAM_IRQ_PIOB_PINS+19) /* PIOB, PIN 19 */
#  define SAM_IRQ_PB20         (SAM_IRQ_PIOB_PINS+20) /* PIOB, PIN 20 */
#  define SAM_IRQ_PB21         (SAM_IRQ_PIOB_PINS+21) /* PIOB, PIN 21 */
#  define SAM_IRQ_PB22         (SAM_IRQ_PIOB_PINS+22) /* PIOB, PIN 22 */
#  define SAM_IRQ_PB23         (SAM_IRQ_PIOB_PINS+23) /* PIOB, PIN 23 */
#  define SAM_IRQ_PB24         (SAM_IRQ_PIOB_PINS+24) /* PIOB, PIN 24 */
#  define SAM_IRQ_PB25         (SAM_IRQ_PIOB_PINS+25) /* PIOB, PIN 25 */
#  define SAM_IRQ_PB26         (SAM_IRQ_PIOB_PINS+26) /* PIOB, PIN 26 */
#  define SAM_IRQ_PB27         (SAM_IRQ_PIOB_PINS+27) /* PIOB, PIN 27 */
#  define SAM_IRQ_PB28         (SAM_IRQ_PIOB_PINS+28) /* PIOB, PIN 28 */
#  define SAM_IRQ_PB29         (SAM_IRQ_PIOB_PINS+29) /* PIOB, PIN 29 */
#  define SAM_IRQ_PB30         (SAM_IRQ_PIOB_PINS+30) /* PIOB, PIN 30 */
#  define SAM_IRQ_PB31         (SAM_IRQ_PIOB_PINS+31) /* PIOB, PIN 31 */
#  define SAM_NPIOBIRQS        32
#else
#  define SAM_NPIOBIRQS        0
#endif

#ifdef CONFIG_SAMA5_PIOC_IRQ
#  define SAM_IRQ_PIOC_PINS    (SAM_IRQ_NINT + SAM_NPIOAIRQS + SAM_NPIOBIRQS)
#  define SAM_IRQ_PC0          (SAM_IRQ_PIOC_PINS+0)  /* PIOC, PIN 0 */
#  define SAM_IRQ_PC1          (SAM_IRQ_PIOC_PINS+1)  /* PIOC, PIN 1 */
#  define SAM_IRQ_PC2          (SAM_IRQ_PIOC_PINS+2)  /* PIOC, PIN 2 */
#  define SAM_IRQ_PC3          (SAM_IRQ_PIOC_PINS+3)  /* PIOC, PIN 3 */
#  define SAM_IRQ_PC4          (SAM_IRQ_PIOC_PINS+4)  /* PIOC, PIN 4 */
#  define SAM_IRQ_PC5          (SAM_IRQ_PIOC_PINS+5)  /* PIOC, PIN 5 */
#  define SAM_IRQ_PC6          (SAM_IRQ_PIOC_PINS+6)  /* PIOC, PIN 6 */
#  define SAM_IRQ_PC7          (SAM_IRQ_PIOC_PINS+7)  /* PIOC, PIN 7 */
#  define SAM_IRQ_PC8          (SAM_IRQ_PIOC_PINS+8)  /* PIOC, PIN 8 */
#  define SAM_IRQ_PC9          (SAM_IRQ_PIOC_PINS+9)  /* PIOC, PIN 9 */
#  define SAM_IRQ_PC10         (SAM_IRQ_PIOC_PINS+10) /* PIOC, PIN 10 */
#  define SAM_IRQ_PC11         (SAM_IRQ_PIOC_PINS+11) /* PIOC, PIN 11 */
#  define SAM_IRQ_PC12         (SAM_IRQ_PIOC_PINS+12) /* PIOC, PIN 12 */
#  define SAM_IRQ_PC13         (SAM_IRQ_PIOC_PINS+13) /* PIOC, PIN 13 */
#  define SAM_IRQ_PC14         (SAM_IRQ_PIOC_PINS+14) /* PIOC, PIN 14 */
#  define SAM_IRQ_PC15         (SAM_IRQ_PIOC_PINS+15) /* PIOC, PIN 15 */
#  define SAM_IRQ_PC16         (SAM_IRQ_PIOC_PINS+16) /* PIOC, PIN 16 */
#  define SAM_IRQ_PC17         (SAM_IRQ_PIOC_PINS+17) /* PIOC, PIN 17 */
#  define SAM_IRQ_PC18         (SAM_IRQ_PIOC_PINS+18) /* PIOC, PIN 18 */
#  define SAM_IRQ_PC19         (SAM_IRQ_PIOC_PINS+19) /* PIOC, PIN 19 */
#  define SAM_IRQ_PC20         (SAM_IRQ_PIOC_PINS+20) /* PIOC, PIN 20 */
#  define SAM_IRQ_PC21         (SAM_IRQ_PIOC_PINS+21) /* PIOC, PIN 21 */
#  define SAM_IRQ_PC22         (SAM_IRQ_PIOC_PINS+22) /* PIOC, PIN 22 */
#  define SAM_IRQ_PC23         (SAM_IRQ_PIOC_PINS+23) /* PIOC, PIN 23 */
#  define SAM_IRQ_PC24         (SAM_IRQ_PIOC_PINS+24) /* PIOC, PIN 24 */
#  define SAM_IRQ_PC25         (SAM_IRQ_PIOC_PINS+25) /* PIOC, PIN 25 */
#  define SAM_IRQ_PC26         (SAM_IRQ_PIOC_PINS+26) /* PIOC, PIN 26 */
#  define SAM_IRQ_PC27         (SAM_IRQ_PIOC_PINS+27) /* PIOC, PIN 27 */
#  define SAM_IRQ_PC28         (SAM_IRQ_PIOC_PINS+28) /* PIOC, PIN 28 */
#  define SAM_IRQ_PC29         (SAM_IRQ_PIOC_PINS+29) /* PIOC, PIN 29 */
#  define SAM_IRQ_PC30         (SAM_IRQ_PIOC_PINS+30) /* PIOC, PIN 30 */
#  define SAM_IRQ_PC31         (SAM_IRQ_PIOC_PINS+31) /* PIOC, PIN 31 */
#  define SAM_NPIOCIRQS        32
#else
#  define SAM_NPIOCIRQS        0
#endif

#ifdef CONFIG_SAMA5_PIOD_IRQ
#  define SAM_IRQ_PIOD_PINS    (SAM_IRQ_NINT + SAM_NPIOAIRQS + SAM_NPIOBIRQS + \
                                SAM_NPIOCIRQS)
#  define SAM_IRQ_PD0          (SAM_IRQ_PIOD_PINS+0)  /* PIOD, PIN 0 */
#  define SAM_IRQ_PD1          (SAM_IRQ_PIOD_PINS+1)  /* PIOD, PIN 1 */
#  define SAM_IRQ_PD2          (SAM_IRQ_PIOD_PINS+2)  /* PIOD, PIN 2 */
#  define SAM_IRQ_PD3          (SAM_IRQ_PIOD_PINS+3)  /* PIOD, PIN 3 */
#  define SAM_IRQ_PD4          (SAM_IRQ_PIOD_PINS+4)  /* PIOD, PIN 4 */
#  define SAM_IRQ_PD5          (SAM_IRQ_PIOD_PINS+5)  /* PIOD, PIN 5 */
#  define SAM_IRQ_PD6          (SAM_IRQ_PIOD_PINS+6)  /* PIOD, PIN 6 */
#  define SAM_IRQ_PD7          (SAM_IRQ_PIOD_PINS+7)  /* PIOD, PIN 7 */
#  define SAM_IRQ_PD8          (SAM_IRQ_PIOD_PINS+8)  /* PIOD, PIN 8 */
#  define SAM_IRQ_PD9          (SAM_IRQ_PIOD_PINS+9)  /* PIOD, PIN 9 */
#  define SAM_IRQ_PD10         (SAM_IRQ_PIOD_PINS+10) /* PIOD, PIN 10 */
#  define SAM_IRQ_PD11         (SAM_IRQ_PIOD_PINS+11) /* PIOD, PIN 11 */
#  define SAM_IRQ_PD12         (SAM_IRQ_PIOD_PINS+12) /* PIOD, PIN 12 */
#  define SAM_IRQ_PD13         (SAM_IRQ_PIOD_PINS+13) /* PIOD, PIN 13 */
#  define SAM_IRQ_PD14         (SAM_IRQ_PIOD_PINS+14) /* PIOD, PIN 14 */
#  define SAM_IRQ_PD15         (SAM_IRQ_PIOD_PINS+15) /* PIOD, PIN 15 */
#  define SAM_IRQ_PD16         (SAM_IRQ_PIOD_PINS+16) /* PIOD, PIN 16 */
#  define SAM_IRQ_PD17         (SAM_IRQ_PIOD_PINS+17) /* PIOD, PIN 17 */
#  define SAM_IRQ_PD18         (SAM_IRQ_PIOD_PINS+18) /* PIOD, PIN 18 */
#  define SAM_IRQ_PD19         (SAM_IRQ_PIOD_PINS+19) /* PIOD, PIN 19 */
#  define SAM_IRQ_PD20         (SAM_IRQ_PIOD_PINS+20) /* PIOD, PIN 20 */
#  define SAM_IRQ_PD21         (SAM_IRQ_PIOD_PINS+21) /* PIOD, PIN 21 */
#  define SAM_IRQ_PD22         (SAM_IRQ_PIOD_PINS+22) /* PIOD, PIN 22 */
#  define SAM_IRQ_PD23         (SAM_IRQ_PIOD_PINS+23) /* PIOD, PIN 23 */
#  define SAM_IRQ_PD24         (SAM_IRQ_PIOD_PINS+24) /* PIOD, PIN 24 */
#  define SAM_IRQ_PD25         (SAM_IRQ_PIOD_PINS+25) /* PIOD, PIN 25 */
#  define SAM_IRQ_PD26         (SAM_IRQ_PIOD_PINS+26) /* PIOD, PIN 26 */
#  define SAM_IRQ_PD27         (SAM_IRQ_PIOD_PINS+27) /* PIOD, PIN 27 */
#  define SAM_IRQ_PD28         (SAM_IRQ_PIOD_PINS+28) /* PIOD, PIN 28 */
#  define SAM_IRQ_PD29         (SAM_IRQ_PIOD_PINS+29) /* PIOD, PIN 29 */
#  define SAM_IRQ_PD30         (SAM_IRQ_PIOD_PINS+30) /* PIOD, PIN 30 */
#  define SAM_IRQ_PD31         (SAM_IRQ_PIOD_PINS+31) /* PIOD, PIN 31 */
#  define SAM_NPIODIRQS        32
#else
#  define SAM_NPIODIRQS        0
#endif

#ifdef CONFIG_SAMA5_PIOE_IRQ
#  define SAM_IRQ_PIOE_PINS    (SAM_IRQ_NINT + SAM_NPIOAIRQS + \
                                SAM_NPIOBIRQS + SAM_NPIOCIRQS + SAM_NPIODIRQS)
#  define SAM_IRQ_PE0          (SAM_IRQ_PIOE_PINS+0)  /* PIOE, PIN 0 */
#  define SAM_IRQ_PE1          (SAM_IRQ_PIOE_PINS+1)  /* PIOE, PIN 1 */
#  define SAM_IRQ_PE2          (SAM_IRQ_PIOE_PINS+2)  /* PIOE, PIN 2 */
#  define SAM_IRQ_PE3          (SAM_IRQ_PIOE_PINS+3)  /* PIOE, PIN 3 */
#  define SAM_IRQ_PE4          (SAM_IRQ_PIOE_PINS+4)  /* PIOE, PIN 4 */
#  define SAM_IRQ_PE5          (SAM_IRQ_PIOE_PINS+5)  /* PIOE, PIN 5 */
#  define SAM_IRQ_PE6          (SAM_IRQ_PIOE_PINS+6)  /* PIOE, PIN 6 */
#  define SAM_IRQ_PE7          (SAM_IRQ_PIOE_PINS+7)  /* PIOE, PIN 7 */
#  define SAM_IRQ_PE8          (SAM_IRQ_PIOE_PINS+8)  /* PIOE, PIN 8 */
#  define SAM_IRQ_PE9          (SAM_IRQ_PIOE_PINS+9)  /* PIOE, PIN 9 */
#  define SAM_IRQ_PE10         (SAM_IRQ_PIOE_PINS+10) /* PIOE, PIN 10 */
#  define SAM_IRQ_PE11         (SAM_IRQ_PIOE_PINS+11) /* PIOE, PIN 11 */
#  define SAM_IRQ_PE12         (SAM_IRQ_PIOE_PINS+12) /* PIOE, PIN 12 */
#  define SAM_IRQ_PE13         (SAM_IRQ_PIOE_PINS+13) /* PIOE, PIN 13 */
#  define SAM_IRQ_PE14         (SAM_IRQ_PIOE_PINS+14) /* PIOE, PIN 14 */
#  define SAM_IRQ_PE15         (SAM_IRQ_PIOE_PINS+15) /* PIOE, PIN 15 */
#  define SAM_IRQ_PE16         (SAM_IRQ_PIOE_PINS+16) /* PIOE, PIN 16 */
#  define SAM_IRQ_PE17         (SAM_IRQ_PIOE_PINS+17) /* PIOE, PIN 17 */
#  define SAM_IRQ_PE18         (SAM_IRQ_PIOE_PINS+18) /* PIOE, PIN 18 */
#  define SAM_IRQ_PE19         (SAM_IRQ_PIOE_PINS+19) /* PIOE, PIN 19 */
#  define SAM_IRQ_PE20         (SAM_IRQ_PIOE_PINS+20) /* PIOE, PIN 20 */
#  define SAM_IRQ_PE21         (SAM_IRQ_PIOE_PINS+21) /* PIOE, PIN 21 */
#  define SAM_IRQ_PE22         (SAM_IRQ_PIOE_PINS+22) /* PIOE, PIN 22 */
#  define SAM_IRQ_PE23         (SAM_IRQ_PIOE_PINS+23) /* PIOE, PIN 23 */
#  define SAM_IRQ_PE24         (SAM_IRQ_PIOE_PINS+24) /* PIOE, PIN 24 */
#  define SAM_IRQ_PE25         (SAM_IRQ_PIOE_PINS+25) /* PIOE, PIN 25 */
#  define SAM_IRQ_PE26         (SAM_IRQ_PIOE_PINS+26) /* PIOE, PIN 26 */
#  define SAM_IRQ_PE27         (SAM_IRQ_PIOE_PINS+27) /* PIOE, PIN 27 */
#  define SAM_IRQ_PE28         (SAM_IRQ_PIOE_PINS+28) /* PIOE, PIN 28 */
#  define SAM_IRQ_PE29         (SAM_IRQ_PIOE_PINS+29) /* PIOE, PIN 29 */
#  define SAM_IRQ_PE30         (SAM_IRQ_PIOE_PINS+30) /* PIOE, PIN 30 */
#  define SAM_IRQ_PE31         (SAM_IRQ_PIOE_PINS+31) /* PIOE, PIN 31 */
#  define SAM_NPIOEIRQS        32
#else
#  define SAM_NPIOEIRQS        0
#endif

/* Total number of IRQ numbers */

#define NR_IRQS               (SAM_IRQ_NINT + \
                               SAM_NPIOAIRQS + SAM_NPIOBIRQS + SAM_NPIOCIRQS + \
                               SAM_NPIODIRQS + SAM_NPIOEIRQS )

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

#endif /* __ARCH_ARM_INCLUDE_SAMA5_SAMA5D3_IRQ_H */
