/****************************************************************************************
 * arch/arm/include/sam34/sam4l_irq.h
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
 ****************************************************************************************/

/* This file should never be included directly but, rather, only indirectly through
 * nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_SAM34_SAM4L_IRQ_H
#define __ARCH_ARM_INCLUDE_SAM34_SAM4L_IRQ_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* SAM4L Peripheral Identifiers.  NOTE:  Interrupts are not tied to the DMA peripheral
 * identifiers in the SAM4L as they were with the SAM3U.  However, for consistency, those
 * peripheral identifiers are defined in this file as well.
 */

#define SAM_PID_USART0_RHR     (0)   /* DIR=RX REGISTER: USART0 RHR */
#define SAM_PID_USART1_RHR     (1)   /* DIR=RX REGISTER: USART1 RHR */
#define SAM_PID_USART2_RHR     (2)   /* DIR=RX REGISTER: USART2 RHR */
#define SAM_PID_USART3_RHR     (3)   /* DIR=RX REGISTER: USART3 RHR */
#define SAM_PID_SPI0_RDR       (4)   /* DIR=RX REGISTER: SPI RDR */
#define SAM_PID_TWIM0_RHR      (5)   /* DIR=RX REGISTER: TWIM0 RHR */
#define SAM_PID_TWIM1_RHR      (6)   /* DIR=RX REGISTER: TWIM1 RHR */
#define SAM_PID_TWIM2_RHR      (7)   /* DIR=RX REGISTER: TWIM2 RHR */
#define SAM_PID_TWIM3_RHR      (8)   /* DIR=RX REGISTER: TWIM3 RHR */
#define SAM_PID_TWIS0_RHR      (9)   /* DIR=RX REGISTER: TWIS0 RHR */
#define SAM_PID_TWIS1_RHR      (10)  /* DIR=RX REGISTER: TWIS1 RHR */
#define SAM_PID_ADCIFE_LCV     (11)  /* DIR=RX REGISTER: ADCIFE LCV */
#define SAM_PID_CATB_RX        (12)  /* DIR=RX REGISTER: CATB Multiple */
                                     /* 13: Reserved */
#define SAM_PID_IISC0_RHR      (14)  /* DIR=RX REGISTER: IISC RHR (CH0) */
#define SAM_PID_IISC1_RHR      (15)  /* DIR=RX REGISTER: IISC RHR (CH1) */
#define SAM_PID_PARC_RHR       (16)  /* DIR=RX REGISTER: PARC RHR */
#define SAM_PID_AESA_ODATA     (17)  /* DIR=RX REGISTER: AESA ODATA */
#define SAM_PID_USART0_THR     (18)  /* DIR=TX REGISTER: USART0 THR */
#define SAM_PID_USART1_THR     (19)  /* DIR=TX REGISTER: USART1 THR */
#define SAM_PID_USART2_THR     (20)  /* DIR=TX REGISTER: USART2 THR */
#define SAM_PID_USART3_THR     (21)  /* DIR=TX REGISTER: USART3 THR */
#define SAM_PID_SPI0_TDR       (22)  /* DIR=TX REGISTER: SPI TDR */
#define SAM_PID_TWIM0_THR      (23)  /* DIR=TX REGISTER: TWIM0 THR */
#define SAM_PID_TWIM1_THR      (24)  /* DIR=TX REGISTER: TWIM1 THR */
#define SAM_PID_TWIM2_THR      (25)  /* DIR=TX REGISTER: TWIM2 THR */
#define SAM_PID_TWIM3_THR      (26)  /* DIR=TX REGISTER: TWIM3 THR */
#define SAM_PID_TWIS0_THR      (27)  /* DIR=TX REGISTER: TWIS0 THR */
#define SAM_PID_TWIS1_THR      (28)  /* DIR=TX REGISTER: TWIS1 THR */
#define SAM_PID_ADCIFE_CDMA    (29)  /* DIR=TX REGISTER: ADCIFE CDMA */
#define SAM_PID_CATB_TX        (30)  /* DIR=TX REGISTER: CATB Multiple */
#define SAM_PID_ABDACB_SDR0    (31)  /* DIR=TX REGISTER: ABDACB SDR0 */
#define SAM_PID_ABDACB_SDR1    (32)  /* DIR=TX REGISTER: ABDACB SDR1 */
#define SAM_PID_IISC0_THR      (33)  /* DIR=TX REGISTER: IISC THR (CH0) */
#define SAM_PID_IISC1_THR      (34)  /* DIR=TX REGISTER: IISC THR (CH1) */
#define SAM_PID_DACC_CDR       (35)  /* DIR=TX REGISTER: DACC CDR */
#define SAM_PID_AESA_IDATA     (36)  /* DIR=TX REGISTER: AESA IDATA */
#define SAM_PID_LCDCA_ACMDR    (37)  /* DIR=TX REGISTER: LCDCA ACMDR */
#define SAM_PID_LCDCA_ABMDR    (38)  /* DIR=TX REGISTER: LCDCA ABMDR */

/* External interrupts (vectors >= 16) */

#define SAM_IRQ_HFLASHC        (SAM_IRQ_EXTINT+0)    /* 0 Flash Controller */
#define SAM_IRQ_PDCA0          (SAM_IRQ_EXTINT+1)    /* 1 Peripheral DMA Controller 0 */
#define SAM_IRQ_PDCA1          (SAM_IRQ_EXTINT+2)    /* 2 Peripheral DMA Controller 1 */
#define SAM_IRQ_PDCA2          (SAM_IRQ_EXTINT+3)    /* 3 Peripheral DMA Controller 2 */
#define SAM_IRQ_PDCA3          (SAM_IRQ_EXTINT+4)    /* 4 Peripheral DMA Controller 3 */
#define SAM_IRQ_PDCA4          (SAM_IRQ_EXTINT+5)    /* 5 Peripheral DMA Controller 4 */
#define SAM_IRQ_PDCA5          (SAM_IRQ_EXTINT+6)    /* 6 Peripheral DMA Controller 5 */
#define SAM_IRQ_PDCA6          (SAM_IRQ_EXTINT+7)    /* 7 Peripheral DMA Controller 6 */
#define SAM_IRQ_PDCA7          (SAM_IRQ_EXTINT+8)    /* 8 Peripheral DMA Controller 7 */
#define SAM_IRQ_PDCA8          (SAM_IRQ_EXTINT+9)    /* 9 Peripheral DMA Controller 8 */
#define SAM_IRQ_PDCA9          (SAM_IRQ_EXTINT+10)   /* 10 Peripheral DMA Controller 9 */
#define SAM_IRQ_PDCA10         (SAM_IRQ_EXTINT+11)   /* 11 Peripheral DMA Controller 10 */
#define SAM_IRQ_PDCA11         (SAM_IRQ_EXTINT+12)   /* 12 Peripheral DMA Controller 11 */
#define SAM_IRQ_PDCA12         (SAM_IRQ_EXTINT+13)   /* 13 Peripheral DMA Controller 12 */
#define SAM_IRQ_PDCA13         (SAM_IRQ_EXTINT+14)   /* 14 Peripheral DMA Controller 13 */
#define SAM_IRQ_PDCA14         (SAM_IRQ_EXTINT+15)   /* 15 Peripheral DMA Controller 14 */
#define SAM_IRQ_PDCA15         (SAM_IRQ_EXTINT+16)   /* 16 Peripheral DMA Controller 15 */
#define SAM_IRQ_CRCCU          (SAM_IRQ_EXTINT+17)   /* 17 CRC Calculation Unit */
#define SAM_IRQ_USBC           (SAM_IRQ_EXTINT+18)   /* 18 USB 2.0 Interface */
#define SAM_IRQ_PEVC_TR        (SAM_IRQ_EXTINT+19)   /* 19 Peripheral Event Controller TR */
#define SAM_IRQ_PEVC_OV        (SAM_IRQ_EXTINT+20)   /* 20 Peripheral Event Controller OV */
#define SAM_IRQ_AESA           (SAM_IRQ_EXTINT+21)   /* 21 Advanced Encryption Standard AESA */
#define SAM_IRQ_PM             (SAM_IRQ_EXTINT+22)   /* 22 Power Manager */
#define SAM_IRQ_SCIF           (SAM_IRQ_EXTINT+23)   /* 23 System Control Interface */
#define SAM_IRQ_FREQM          (SAM_IRQ_EXTINT+24)   /* 24 Frequency Meter */
#define SAM_IRQ_GPIO0          (SAM_IRQ_EXTINT+25)   /* 25 General-Purpose Input/Output Controller 0 */
#define SAM_IRQ_GPIO1          (SAM_IRQ_EXTINT+26)   /* 26 General-Purpose Input/Output Controller 1 */
#define SAM_IRQ_GPIO2          (SAM_IRQ_EXTINT+27)   /* 27 General-Purpose Input/Output Controller 2 */
#define SAM_IRQ_GPIO3          (SAM_IRQ_EXTINT+28)   /* 28 General-Purpose Input/Output Controller 3 */
#define SAM_IRQ_GPIO4          (SAM_IRQ_EXTINT+29)   /* 29 General-Purpose Input/Output Controller 4 */
#define SAM_IRQ_GPIO5          (SAM_IRQ_EXTINT+30)   /* 30 General-Purpose Input/Output Controller 5 */
#define SAM_IRQ_GPIO6          (SAM_IRQ_EXTINT+31)   /* 31 General-Purpose Input/Output Controller 6 */
#define SAM_IRQ_GPIO7          (SAM_IRQ_EXTINT+32)   /* 32 General-Purpose Input/Output Controller 7 */
#define SAM_IRQ_GPIO8          (SAM_IRQ_EXTINT+33)   /* 33 General-Purpose Input/Output Controller 8 */
#define SAM_IRQ_GPIO9          (SAM_IRQ_EXTINT+34)   /* 34 General-Purpose Input/Output Controller 9 */
#define SAM_IRQ_GPIO10         (SAM_IRQ_EXTINT+35)   /* 35 General-Purpose Input/Output Controller 10 */
#define SAM_IRQ_GPIO11         (SAM_IRQ_EXTINT+36)   /* 36 General-Purpose Input/Output Controller 11 */
#define SAM_IRQ_BPM            (SAM_IRQ_EXTINT+37)   /* 37 Backup Power Manager */
#define SAM_IRQ_BSCIF          (SAM_IRQ_EXTINT+38)   /* 38 Backup System Control Interface */
#define SAM_IRQ_AST_ALARM      (SAM_IRQ_EXTINT+39)   /* 39 Asynchronous Timer ALARM */
#define SAM_IRQ_AST_PER        (SAM_IRQ_EXTINT+40)   /* 40 Asynchronous Timer PER */
#define SAM_IRQ_AST_OVF        (SAM_IRQ_EXTINT+41)   /* 41 Asynchronous Timer OVF */
#define SAM_IRQ_AST_READY      (SAM_IRQ_EXTINT+42)   /* 42 Asynchronous Timer READY */
#define SAM_IRQ_AST_CLKREADY   (SAM_IRQ_EXTINT+43)   /* 43 Asynchronous Timer CLKREADY */
#define SAM_IRQ_WDT            (SAM_IRQ_EXTINT+44)   /* 44 Watchdog Timer */
#define SAM_IRQ_EIC1           (SAM_IRQ_EXTINT+45)   /* 45 External Interrupt Controller 1 */
#define SAM_IRQ_EIC2           (SAM_IRQ_EXTINT+46)   /* 46 External Interrupt Controller 2 */
#define SAM_IRQ_EIC3           (SAM_IRQ_EXTINT+47)   /* 47 External Interrupt Controller 3 */
#define SAM_IRQ_EIC4           (SAM_IRQ_EXTINT+48)   /* 48 External Interrupt Controller 4 */
#define SAM_IRQ_EIC5           (SAM_IRQ_EXTINT+49)   /* 49 External Interrupt Controller 5 */
#define SAM_IRQ_EIC6           (SAM_IRQ_EXTINT+50)   /* 50 External Interrupt Controller 6 */
#define SAM_IRQ_EIC7           (SAM_IRQ_EXTINT+51)   /* 51 External Interrupt Controller 7 */
#define SAM_IRQ_EIC8           (SAM_IRQ_EXTINT+52)   /* 52 External Interrupt Controller 8 */
#define SAM_IRQ_IISC           (SAM_IRQ_EXTINT+53)   /* 53 Inter-IC Sound (I2S) Controller */
#define SAM_IRQ_SPI0           (SAM_IRQ_EXTINT+54)   /* 54 Serial Peripheral Interface */
#define SAM_IRQ_TC00           (SAM_IRQ_EXTINT+55)   /* 55 Timer/Counter 0 */
#define SAM_IRQ_TC01           (SAM_IRQ_EXTINT+56)   /* 56 Timer/Counter 1 */
#define SAM_IRQ_TC02           (SAM_IRQ_EXTINT+57)   /* 57 Timer/Counter 2 */
#define SAM_IRQ_TC10           (SAM_IRQ_EXTINT+58)   /* 58 Timer/Counter 10 */
#define SAM_IRQ_TC11           (SAM_IRQ_EXTINT+59)   /* 59 Timer/Counter 11 */
#define SAM_IRQ_TC12           (SAM_IRQ_EXTINT+60)   /* 60 Timer/Counter 12 */
#define SAM_IRQ_TWIM0          (SAM_IRQ_EXTINT+61)   /* 61 Two-wire Master Interface TWIM0 */
#define SAM_IRQ_TWIS0          (SAM_IRQ_EXTINT+62)   /* 62 Two-wire Slave Interface TWIS0 */
#define SAM_IRQ_TWIM1          (SAM_IRQ_EXTINT+63)   /* 63 Two-wire Master Interface TWIM1 */
#define SAM_IRQ_TWIS1          (SAM_IRQ_EXTINT+64)   /* 64 Two-wire Slave Interface TWIS1 */
#define SAM_IRQ_USART0         (SAM_IRQ_EXTINT+65)   /* 65 USART0 */
#define SAM_IRQ_USART1         (SAM_IRQ_EXTINT+66)   /* 66 USART1 */
#define SAM_IRQ_USART2         (SAM_IRQ_EXTINT+67)   /* 67 USART2 */
#define SAM_IRQ_USART3         (SAM_IRQ_EXTINT+68)   /* 68 USART3 */
#define SAM_IRQ_ADCIFE         (SAM_IRQ_EXTINT+69)   /* 69 ADC controller interface  */
#define SAM_IRQ_DACC           (SAM_IRQ_EXTINT+70)   /* 70 DAC Controller */
#define SAM_IRQ_ACIFC          (SAM_IRQ_EXTINT+71)   /* 71 Analog Comparator Interface */
#define SAM_IRQ_ABDACB         (SAM_IRQ_EXTINT+72)   /* 72 Audio Bitstream DAC */
#define SAM_IRQ_TRNG           (SAM_IRQ_EXTINT+73)   /* 73 True Random Number Generator */
#define SAM_IRQ_PARC           (SAM_IRQ_EXTINT+74)   /* 74 Parallel Capture */
#define SAM_IRQ_CATB           (SAM_IRQ_EXTINT+75)   /* 75 Capacitive Touch Module B */
#define SAM_IRQ_TWIM2          (SAM_IRQ_EXTINT+77)   /* 77 Two-wire Master Interface */
#define SAM_IRQ_TWIM3          (SAM_IRQ_EXTINT+78)   /* 78 Two-wire Master Interface */
#define SAM_IRQ_LCDCA          (SAM_IRQ_EXTINT+79)   /* 79 LCD Controller A */
#define SAM_IRQ_NEXTINT        80                    /* Total number of external interrupt numbers */

#define SAM_IRQ_NIRQS          (SAM_IRQ_EXTINT+SAM_IRQ_NEXTINT)        /* The number of real IRQs */

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

/* Total number of IRQ numbers */

#define NR_IRQS               (SAM_IRQ_EXTINT + SAM_IRQ_NEXTINT + \
                               SAM_NGPIOAIRQS + SAM_NGPIOBIRQS + SAM_NGPIOCIRQS)

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

#endif /* __ARCH_ARM_INCLUDE_SAM34_SAM4L_IRQ_H */
