/****************************************************************************************
 * arch/arm/include/sama5/sama5d2_irq.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_INCLUDE_SAMA5_SAMA5D2_IRQ_H
#define __ARCH_ARM_INCLUDE_SAMA5_SAMA5D2_IRQ_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* SAMA5D3 Peripheral Identifiers */

#define SAM_PID_FIQ            (0)  /* Advanced Interrupt Controller FIQ */
                                    /* 2 Reserved */
#define SAM_PID_ARM            (2)  /* Performance Monitor Unit */
#define SAM_PID_PIT            (3)  /* Periodic Interval Timer Interrupt */
#define SAM_PID_WDT            (4)  /* Watchdog timer Interrupt */
#define SAM_PID_EMAC0          (5)  /* Ethernet MAC 0 */
#define SAM_PID_XDMAC0         (6)  /* DMA Controller 0 */
#define SAM_PID_XDMAC1         (7)  /* DMA Controller 1 */
#define SAM_PID_ICM            (8)  /* Integrity Check Monitor */
#define SAM_PID_AES            (9)  /* Advanced Encryption Standard */

#define SAM_PID_AESB           (10) /* AES bridge */
#define SAM_PID_TDES           (11) /* Triple Data Encryption Standard */
#define SAM_PID_SHA            (12) /* Secure Hash Algorithm */
#define SAM_PID_MPDDRC         (13) /* MPDDR controller */
#define SAM_PID_MATRIX1        (14) /* H32MX, 32-bit AHB Matrix */
#define SAM_PID_MATRIX0        (15) /* H64MX, 64-bit AHB Matrix */
#define SAM_PID_SECUMOD        (16) /* Secure Module */
#define SAM_PID_HSMC           (17) /* Multi-bit ECC Interrupt */
#define SAM_PID_PIO            (18) /* Parallel I/O Controller */
#define SAM_PID_FLEXCOM0       (19) /* FLEXCOM 0 */

#define SAM_PID_FLEXCOM1       (20) /* FLEXCOM 1 */
#define SAM_PID_FLEXCOM2       (21) /* FLEXCOM 2 */
#define SAM_PID_FLEXCOM3       (22) /* FLEXCOM 3 */
#define SAM_PID_FLEXCOM4       (23) /* FLEXCOM 4 */
#define SAM_PID_UART0          (24) /* UART 0 */
#define SAM_PID_UART1          (25) /* UART 1 */
#define SAM_PID_UART2          (26) /* UART 2 */
#define SAM_PID_UART3          (27) /* UART 3 */
#define SAM_PID_UART4          (28) /* UART 4 */
#define SAM_PID_TWI0           (29) /* Two-Wire Interface 0 */

#define SAM_PID_TWI1           (30) /* Two-Wire Interface 1 */
#define SAM_PID_SDMMC0         (31) /* Secure Data Memory Card Controller 0 */
#define SAM_PID_SDMMC1         (32) /* Secure Data Memory Card Controller 1 */
#define SAM_PID_SPI0           (33) /* Serial Peripheral Interface 0 */
#define SAM_PID_SPI1           (34) /* Serial Peripheral Interface 1 */
#define SAM_PID_TC0            (35) /* Timer Counter 0 (ch. 0, 1, 2) */
#define SAM_PID_TC1            (36) /* Timer Counter 1 (ch. 3, 4, 5) */
                                    /* 37 Reserved */
#define SAM_PID_PWM            (38) /* Pulse Width Modulation Controller */
                                    /* 39 Reserved */

#define SAM_PID_ADC            (40) /* Touch Screen ADC Controller */
#define SAM_PID_UHPHS          (41) /* USB Host High Speed */
#define SAM_PID_UDPHS          (42) /* USB Device High Speed */
#define SAM_PID_SSC0           (43) /* Synchronous Serial Controller 0 */
#define SAM_PID_SSC1           (44) /* Synchronous Serial Controller 1 */
#define SAM_PID_LCDC           (45) /* LCD Controller */
#define SAM_PID_ISC            (46) /* Image Sensor Controller */
#define SAM_PID_TRNG           (47) /* True Random Number Generator */
#define SAM_PID_PDMIC          (48) /* Pulse Density Modulation Interface Controller */
#define SAM_PID_IRQID          (49) /* IRQ Interrupt ID */

#define SAM_PID_SFC            (50) /* Fuse Controller */
#define SAM_PID_SECURAM        (51) /* Secured RAM */
#define SAM_PID_QSPI0          (52) /* QuadSPI 0 */
#define SAM_PID_QSPI1          (53) /* QuadSPI 1 */
#define SAM_PID_I2SC0          (54) /* Inter-IC Sound Controller 0 */
#define SAM_PID_I2SC1          (55) /* Inter-IC Sound Controller 1 */
#define SAM_PID_MCAN00         (56) /* MCAN controller 0, Interrupt 0 */
#define SAM_PID_MCAN10         (57) /* MCAN controller 1, Interrupt 0 */
                                    /* 58 Reserved */
#define SAM_PID_CLASSD         (59) /* Audio Class D Amplifier */

#define SAM_PID_SFR            (60) /* Special Function Register */
#define SAM_PID_SAIC           (61) /* Secured Advanced Interrupt Controller */
#define SAM_PID_AIC            (62) /* Advanced Interrupt Controller */
#define SAM_PID_L2CC           (63) /* L2 Cache Controller */
#define SAM_PID_MCAN01         (64) /* MCAN controller 0, Interrupt 1 */
#define SAM_PID_MCAN11         (65) /* MCAN controller 1, Interrupt 1 */
#define SAM_PID_EMACQ1         (66) /* EMAC Queue 1 Interrupt */
#define SAM_PID_EMACQ2         (67) /* EMAC Queue 2 Interrupt */
#define SAM_PID_PIOB           (68) /* Parallel I/O Controller B */
#define SAM_PID_PIOC           (69) /* Parallel I/O Controller C */

#define SAM_PID_PIOD           (70) /* Parallel I/O Controller D */
#define SAM_PID_SDMMC0T        (71) /* Secure Data Memory Card Controller 0 */
#define SAM_PID_SDMMC1T        (72) /* Secure Data Memory Card Controller 1 */
                                    /* 73 Reserved */
#define SAM_PID_SYS            (74) /* System Controller Interrupt PMC, RTC, RSTC */
#define SAM_PID_ACC            (75) /* Analog Comparator */
#define SAM_PID_RXLP           (76) /* UART Low-Power */
#define SAM_PID_SFRBU          (77) /* Special Function Register BackUp */
#define SAM_PID_CHIPID         (78) /* Chip ID */

#define SAM_NPIDS              (79)

/* External interrupts vectors numbers (same as peripheral ID) */

#define SAM_IRQ_FIQ            SAM_PID_FIQ       /* Advanced Interrupt Controller FIQ */
#define SAM_IRQ_ARM            SAM_PID_ARM       /* Performance Monitor Unit */
#define SAM_IRQ_PIT            SAM_PID_PIT       /* Periodic Interval Timer Interrupt */
#define SAM_IRQ_WDT            SAM_PID_WDT       /* Watchdog timer Interrupt */
#define SAM_IRQ_EMAC0          SAM_PID_EMAC0     /* Ethernet MAC 0 */
#define SAM_IRQ_XDMAC0         SAM_PID_XDMAC0    /* DMA Controller 0 */
#define SAM_IRQ_XDMAC1         SAM_PID_XDMAC1    /* DMA Controller 1 */
#define SAM_IRQ_ICM            SAM_PID_ICM       /* Integrity Check Monitor */
#define SAM_IRQ_AES            SAM_PID_AES       /* Advanced Encryption Standard */

#define SAM_IRQ_AESB           SAM_PID_AESB      /* AES bridge */
#define SAM_IRQ_TDES           SAM_PID_TDES      /* Triple Data Encryption Standard */
#define SAM_IRQ_SHA            SAM_PID_SHA       /* Secure Hash Algorithm */
#define SAM_IRQ_MPDDRC         SAM_PID_MPDDRC    /* MPDDR controller */
#define SAM_IRQ_MATRIX1        SAM_PID_MATRIX1   /* H32MX, 32-bit AHB Matrix */
#define SAM_IRQ_MATRIX0        SAM_PID_MATRIX0   /* H64MX, 64-bit AHB Matrix */
#define SAM_IRQ_SECUMOD        SAM_PID_SECUMOD   /* Secure Module */
#define SAM_IRQ_HSMC           SAM_PID_HSMC      /* Multi-bit ECC Interrupt */
#define SAM_IRQ_PIO            SAM_PID_PIO       /* Parallel I/O Controller */
#define SAM_IRQ_FLEXCOM0       SAM_PID_FLEXCOM0  /* FLEXCOM 0 */

#define SAM_IRQ_FLEXCOM1       SAM_PID_FLEXCOM1  /* FLEXCOM 1 */
#define SAM_IRQ_FLEXCOM2       SAM_PID_FLEXCOM2  /* FLEXCOM 2 */
#define SAM_IRQ_FLEXCOM3       SAM_PID_FLEXCOM3  /* FLEXCOM 3 */
#define SAM_IRQ_FLEXCOM4       SAM_PID_FLEXCOM4  /* FLEXCOM 4 */
#define SAM_IRQ_UART0          SAM_PID_UART0     /* UART 0 */
#define SAM_IRQ_UART1          SAM_PID_UART1     /* UART 1 */
#define SAM_IRQ_UART2          SAM_PID_UART2     /* UART 2 */
#define SAM_IRQ_UART3          SAM_PID_UART3     /* UART 3 */
#define SAM_IRQ_UART4          SAM_PID_UART4     /* UART 4 */
#define SAM_IRQ_TWI0           SAM_PID_TWI0      /* Two-Wire Interface 0 */

#define SAM_IRQ_TWI1           SAM_PID_TWI1      /* Two-Wire Interface 1 */
#define SAM_IRQ_SDMMC0         SAM_PID_SDMMC0    /* Secure Data Memory Card Controller 0 */
#define SAM_IRQ_SDMMC1         SAM_PID_SDMMC1    /* Secure Data Memory Card Controller 1 */
#define SAM_IRQ_SPI0           SAM_PID_SPI0      /* Serial Peripheral Interface 0 */
#define SAM_IRQ_SPI1           SAM_PID_SPI1      /* Serial Peripheral Interface 1 */
#define SAM_IRQ_TC0            SAM_PID_TC0       /* Timer Counter 0 (ch. 0, 1, 2) */
#define SAM_IRQ_TC1            SAM_PID_TC1       /* Timer Counter 1 (ch. 3, 4, 5) */
#define SAM_IRQ_PWM            SAM_PID_PWM       /* Pulse Width Modulation Controller */

#define SAM_IRQ_ADC            SAM_PID_ADC       /* Touch Screen ADC Controller */
#define SAM_IRQ_UHPHS          SAM_PID_UHPHS     /* USB Host High Speed */
#define SAM_IRQ_UDPHS          SAM_PID_UDPHS     /* USB Device High Speed */
#define SAM_IRQ_SSC0           SAM_PID_SSC0      /* Synchronous Serial Controller 0 */
#define SAM_IRQ_SSC1           SAM_PID_SSC1      /* Synchronous Serial Controller 1 */
#define SAM_IRQ_LCDC           SAM_PID_LCDC      /* LCD Controller */
#define SAM_IRQ_ISC            SAM_PID_ISC       /* Image Sensor Controller */
#define SAM_IRQ_TRNG           SAM_PID_TRNG      /* True Random Number Generator */
#define SAM_IRQ_PDMIC          SAM_PID_PDMIC     /* Pulse Density Modulation Interface Controller */
#define SAM_IRQ_IRQID          SAM_PID_IRQID     /* IRQ Interrupt ID */

#define SAM_IRQ_SFC            SAM_PID_SFC       /* Fuse Controller */
#define SAM_IRQ_SECURAM        SAM_PID_SECURAM   /* Secured RAM */
#define SAM_IRQ_QSPI0          SAM_PID_QSPI0     /* QuadSPI 0 */
#define SAM_IRQ_QSPI1          SAM_PID_QSPI1     /* QuadSPI 1 */
#define SAM_IRQ_I2SC0          SAM_PID_I2SC0     /* Inter-IC Sound Controller 0 */
#define SAM_IRQ_I2SC1          SAM_PID_I2SC1     /* Inter-IC Sound Controller 1 */
#define SAM_IRQ_MCAN00         SAM_PID_MCAN00    /* MCAN controller 0, Interrupt 0 */
#define SAM_IRQ_MCAN10         SAM_PID_MCAN10    /* MCAN controller 1, Interrupt 0 */
#define SAM_IRQ_CLASSD         SAM_PID_CLASSD    /* Audio Class D Amplifier */

                                                 /* Special Function Register (no interrupt) */
                                                 /* Secured Advanced Interrupt Controller (no interrupt) */
                                                 /* Advanced Interrupt Controller (no interrupt) */
#define SAM_IRQ_L2CC           SAM_PID_L2CC      /* L2 Cache Controller */
#define SAM_IRQ_MCAN01         SAM_PID_MCAN01    /* MCAN controller 0, Interrupt 1 */
#define SAM_IRQ_MCAN11         SAM_PID_MCAN11    /* MCAN controller 1, Interrupt 1 */
#define SAM_IRQ_EMACQ1         SAM_PID_EMACQ1    /* EMAC Queue 1 Interrupt */
#define SAM_IRQ_EMACQ2         SAM_PID_EMACQ2    /* EMAC Queue 2 Interrupt */
#define SAM_IRQ_PIOB           SAM_PID_PIOB      /* Parallel I/O Controller B */
#define SAM_IRQ_PIOC           SAM_PID_PIOC      /* Parallel I/O Controller C */

#define SAM_IRQ_PIOD           SAM_PID_PIOD      /* Parallel I/O Controller D */
#define SAM_IRQ_SDMMC0T        SAM_PID_SDMMC0T   /* Secure Data Memory Card Controller 0 */
#define SAM_IRQ_SDMMC1T        SAM_PID_SDMMC1T   /* Secure Data Memory Card Controller 1 */
#define SAM_IRQ_SYS            SAM_PID_SYS       /* System Controller Interrupt PMC, RTC, RSTC */
#define SAM_IRQ_ACC            SAM_PID_ACC       /* Analog Comparator */
#define SAM_IRQ_RXLP           SAM_PID_RXLP      /* UART Low-Power */
                                                 /* Special Function Register BackUp (no interrupt) */
                                                 /* Chip ID (no interrupt) */

#define SAM_IRQ_NINT           (SAM_PID_RXLP + 1)

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

/* Total number of IRQ numbers */

#define NR_IRQS               (SAM_IRQ_NINT + \
                               SAM_NPIOAIRQS + SAM_NPIOBIRQS + SAM_NPIOCIRQS + \
                               SAM_NPIODIRQS)

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Inline functions
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Function Prototypes
 ****************************************************************************************/

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

#endif /* __ARCH_ARM_INCLUDE_SAMA5_SAMA5D2_IRQ_H */
