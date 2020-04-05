/****************************************************************************************
 * arch/arm/include/am335x/am335x_irq.h
 *
 *   Copyright (C) 2019 Petro Karashchenko. All rights reserved.
 *   Author: Petro Karashchenko <petro.karashchenko@gmail.com>
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

#ifndef __ARCH_ARM_INCLUDE_AM335X_AM335X_IRQ_H
#define __ARCH_ARM_INCLUDE_AM335X_AM335X_IRQ_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* External interrupts numbers */
/* Interrupt number list */

#define AM335X_IRQ_EMU                  (0) /* Emulation interrupt */
#define AM335X_IRQ_COMMTX               (1)
#define AM335X_IRQ_COMMRX               (2)
#define AM335X_IRQ_BENCH                (3)
#define AM335X_IRQ_ELM                  (4)
#define AM335X_IRQ_NMI                  (7) /* External Non-Mask Interrupt */
#define AM335X_IRQ_L3DEBUG              (9)
#define AM335X_IRQ_L3APPINT             (10)
#define AM335X_IRQ_PRCM                 (11)
#define AM335X_IRQ_EDMACOMP             (12)
#define AM335X_IRQ_EDMAMPERR            (13)
#define AM335X_IRQ_EDMAERR              (14)
#define AM335X_IRQ_ADC_TSC_GEN          (16)
#define AM335X_IRQ_USBSS                (17)
#define AM335X_IRQ_USB0                 (18)
#define AM335X_IRQ_USB1                 (19)
#define AM335X_IRQ_PRUSS1_EVTOUT0       (20)
#define AM335X_IRQ_PRUSS1_EVTOUT1       (21)
#define AM335X_IRQ_PRUSS1_EVTOUT2       (22)
#define AM335X_IRQ_PRUSS1_EVTOUT3       (23)
#define AM335X_IRQ_PRUSS1_EVTOUT4       (24)
#define AM335X_IRQ_PRUSS1_EVTOUT5       (25)
#define AM335X_IRQ_PRUSS1_EVTOUT6       (26)
#define AM335X_IRQ_PRUSS1_EVTOUT7       (27)
#define AM335X_IRQ_MMCSD1               (28)
#define AM335X_IRQ_MMCSD2               (29)
#define AM335X_IRQ_I2C2                 (30) /* I2C 2 interrupt */
#define AM335X_IRQ_eCAP0                (31)
#define AM335X_IRQ_GPIO2A               (32)
#define AM335X_IRQ_GPIO2B               (33)
#define AM335X_IRQ_USBWAKEUP            (34)
#define AM335X_IRQ_LCDC                 (36)
#define AM335X_IRQ_GFX                  (37)
#define AM335X_IRQ_ePWM2                (39)
#define AM335X_IRQ_3PGSWRXTHR0          (40)
#define AM335X_IRQ_3PGSWRX0             (41)
#define AM335X_IRQ_3PGSWTX0             (42)
#define AM335X_IRQ_3PGSWMISC0           (43)
#define AM335X_IRQ_UART3                (44) /* UART 3 interrupt */
#define AM335X_IRQ_UART4                (45) /* UART 4 interrupt */
#define AM335X_IRQ_UART5                (46) /* UART 5 interrupt */
#define AM335X_IRQ_eCAP1                (47)
#define AM335X_IRQ_DCAN0_LINE0          (52)
#define AM335X_IRQ_DCAN0_LINE1          (53)
#define AM335X_IRQ_DCAN0_PARITY         (54)
#define AM335X_IRQ_DCAN1_LINE0          (55)
#define AM335X_IRQ_DCAN1_LINE1          (56)
#define AM335X_IRQ_DCAN1_PARITY         (57)
#define AM335X_IRQ_ePWM0_TZ             (58)
#define AM335X_IRQ_ePWM1_TZ             (59)
#define AM335X_IRQ_ePWM2_TZ             (60)
#define AM335X_IRQ_eCAP2                (61)
#define AM335X_IRQ_GPIO3A               (62)
#define AM335X_IRQ_GPIO3B               (63)
#define AM335X_IRQ_MMCSD0               (64)
#define AM335X_IRQ_SPI0                 (65) /* McSPI 0 interrupt */
#define AM335X_IRQ_TIMER0               (66) /* Timer 0 interrupt */
#define AM335X_IRQ_TIMER1_1MS           (67) /* Timer 1 interrupt */
#define AM335X_IRQ_TIMER2               (68) /* Timer 2 interrupt */
#define AM335X_IRQ_TIMER3               (69) /* Timer 3 interrupt */
#define AM335X_IRQ_I2C0                 (70) /* I2C 0 interrupt */
#define AM335X_IRQ_I2C1                 (71) /* I2C 1 interrupt */
#define AM335X_IRQ_UART0                (72) /* UART 0 interrupt */
#define AM335X_IRQ_UART1                (73) /* UART 1 interrupt */
#define AM335X_IRQ_UART2                (74) /* UART 2 interrupt */
#define AM335X_IRQ_RTC                  (75)
#define AM335X_IRQ_RTCALARM             (76)
#define AM335X_IRQ_MB0                  (77)
#define AM335X_IRQ_M3_TXEV              (78)
#define AM335X_IRQ_eQEP0                (79)
#define AM335X_IRQ_MCATX0               (80)
#define AM335X_IRQ_MCARX0               (81)
#define AM335X_IRQ_MCATX1               (82)
#define AM335X_IRQ_MCARX1               (83)
#define AM335X_IRQ_ePWM0                (86)
#define AM335X_IRQ_ePWM1                (87)
#define AM335X_IRQ_eQEP1                (88)
#define AM335X_IRQ_eQEP2                (89)
#define AM335X_IRQ_DMA_PIN2             (90)
#define AM335X_IRQ_WDT1                 (91) /* Watchdog interrupt */
#define AM335X_IRQ_TIMER4               (92) /* Timer 4 interrupt */
#define AM335X_IRQ_TIMER5               (93) /* Timer 5 interrupt */
#define AM335X_IRQ_TIMER6               (94) /* Timer 6 interrupt */
#define AM335X_IRQ_TIMER7               (95) /* Timer 7 interrupt */
#define AM335X_IRQ_GPIO0A               (96)
#define AM335X_IRQ_GPIO0B               (97)
#define AM335X_IRQ_GPIO1A               (98)
#define AM335X_IRQ_GPIO1B               (99)
#define AM335X_IRQ_GPMC                 (100)
#define AM335X_IRQ_DDRERR0              (101)
#define AM335X_IRQ_TCERR0               (112)
#define AM335X_IRQ_TCERR1               (113)
#define AM335X_IRQ_TCERR2               (114)
#define AM335X_IRQ_ADC_TSC_PEND         (115)
#define AM335X_IRQ_SMRFLX_MPU           (120)
#define AM335X_IRQ_SMRFLX_CORE          (121)
#define AM335X_IRQ_DMA_PIN0             (123)
#define AM335X_IRQ_DMA_PIN1             (124)
#define AM335X_IRQ_SPI1                 (125) /* McSPI 1 interrupt */

/* Total number of interrupts */

#define AM335X_IRQ_NINT                 (AM335X_IRQ_SPI1 + 1)

/* Up to 128 GPIO interrupts (derived from AM335X_IRQ_GPIO0/1/2/3) */

#ifdef CONFIG_AM335X_GPIO0_IRQ
#  define AM335X_IRQ_GPIO0_PINS         (AM335X_IRQ_NINT)
#  define AM335X_IRQ_GPIO0P0            (AM335X_IRQ_GPIO0_PINS + 0)  /* GPIO0, PIN 0 */
#  define AM335X_IRQ_GPIO0P1            (AM335X_IRQ_GPIO0_PINS + 1)  /* GPIO0, PIN 1 */
#  define AM335X_IRQ_GPIO0P2            (AM335X_IRQ_GPIO0_PINS + 2)  /* GPIO0, PIN 2 */
#  define AM335X_IRQ_GPIO0P3            (AM335X_IRQ_GPIO0_PINS + 3)  /* GPIO0, PIN 3 */
#  define AM335X_IRQ_GPIO0P4            (AM335X_IRQ_GPIO0_PINS + 4)  /* GPIO0, PIN 4 */
#  define AM335X_IRQ_GPIO0P5            (AM335X_IRQ_GPIO0_PINS + 5)  /* GPIO0, PIN 5 */
#  define AM335X_IRQ_GPIO0P6            (AM335X_IRQ_GPIO0_PINS + 6)  /* GPIO0, PIN 6 */
#  define AM335X_IRQ_GPIO0P7            (AM335X_IRQ_GPIO0_PINS + 7)  /* GPIO0, PIN 7 */
#  define AM335X_IRQ_GPIO0P8            (AM335X_IRQ_GPIO0_PINS + 8)  /* GPIO0, PIN 8 */
#  define AM335X_IRQ_GPIO0P9            (AM335X_IRQ_GPIO0_PINS + 9)  /* GPIO0, PIN 9 */
#  define AM335X_IRQ_GPIO0P10           (AM335X_IRQ_GPIO0_PINS + 10) /* GPIO0, PIN 10 */
#  define AM335X_IRQ_GPIO0P11           (AM335X_IRQ_GPIO0_PINS + 11) /* GPIO0, PIN 11 */
#  define AM335X_IRQ_GPIO0P12           (AM335X_IRQ_GPIO0_PINS + 12) /* GPIO0, PIN 12 */
#  define AM335X_IRQ_GPIO0P13           (AM335X_IRQ_GPIO0_PINS + 13) /* GPIO0, PIN 13 */
#  define AM335X_IRQ_GPIO0P14           (AM335X_IRQ_GPIO0_PINS + 14) /* GPIO0, PIN 14 */
#  define AM335X_IRQ_GPIO0P15           (AM335X_IRQ_GPIO0_PINS + 15) /* GPIO0, PIN 15 */
#  define AM335X_IRQ_GPIO0P16           (AM335X_IRQ_GPIO0_PINS + 16) /* GPIO0, PIN 16 */
#  define AM335X_IRQ_GPIO0P17           (AM335X_IRQ_GPIO0_PINS + 17) /* GPIO0, PIN 17 */
#  define AM335X_IRQ_GPIO0P18           (AM335X_IRQ_GPIO0_PINS + 18) /* GPIO0, PIN 18 */
#  define AM335X_IRQ_GPIO0P19           (AM335X_IRQ_GPIO0_PINS + 19) /* GPIO0, PIN 19 */
#  define AM335X_IRQ_GPIO0P20           (AM335X_IRQ_GPIO0_PINS + 20) /* GPIO0, PIN 20 */
#  define AM335X_IRQ_GPIO0P21           (AM335X_IRQ_GPIO0_PINS + 21) /* GPIO0, PIN 21 */
#  define AM335X_IRQ_GPIO0P22           (AM335X_IRQ_GPIO0_PINS + 22) /* GPIO0, PIN 22 */
#  define AM335X_IRQ_GPIO0P23           (AM335X_IRQ_GPIO0_PINS + 23) /* GPIO0, PIN 23 */
#  define AM335X_IRQ_GPIO0P24           (AM335X_IRQ_GPIO0_PINS + 24) /* GPIO0, PIN 24 */
#  define AM335X_IRQ_GPIO0P25           (AM335X_IRQ_GPIO0_PINS + 25) /* GPIO0, PIN 25 */
#  define AM335X_IRQ_GPIO0P26           (AM335X_IRQ_GPIO0_PINS + 26) /* GPIO0, PIN 26 */
#  define AM335X_IRQ_GPIO0P27           (AM335X_IRQ_GPIO0_PINS + 27) /* GPIO0, PIN 27 */
#  define AM335X_IRQ_GPIO0P28           (AM335X_IRQ_GPIO0_PINS + 28) /* GPIO0, PIN 28 */
#  define AM335X_IRQ_GPIO0P29           (AM335X_IRQ_GPIO0_PINS + 29) /* GPIO0, PIN 29 */
#  define AM335X_IRQ_GPIO0P30           (AM335X_IRQ_GPIO0_PINS + 30) /* GPIO0, PIN 30 */
#  define AM335X_IRQ_GPIO0P31           (AM335X_IRQ_GPIO0_PINS + 31) /* GPIO0, PIN 31 */
#  define AM335X_NGPIO0IRQS             (32)
#else
#  define AM335X_NGPIO0IRQS             (0)
#endif

#ifdef CONFIG_AM335X_GPIO1_IRQ
#  define AM335X_IRQ_GPIO1_PINS         (AM335X_IRQ_NINT + AM335X_NGPIO0IRQS)
#  define AM335X_IRQ_GPIO1P0            (AM335X_IRQ_GPIO1_PINS + 0)  /* GPIO1, PIN 0 */
#  define AM335X_IRQ_GPIO1P1            (AM335X_IRQ_GPIO1_PINS + 1)  /* GPIO1, PIN 1 */
#  define AM335X_IRQ_GPIO1P2            (AM335X_IRQ_GPIO1_PINS + 2)  /* GPIO1, PIN 2 */
#  define AM335X_IRQ_GPIO1P3            (AM335X_IRQ_GPIO1_PINS + 3)  /* GPIO1, PIN 3 */
#  define AM335X_IRQ_GPIO1P4            (AM335X_IRQ_GPIO1_PINS + 4)  /* GPIO1, PIN 4 */
#  define AM335X_IRQ_GPIO1P5            (AM335X_IRQ_GPIO1_PINS + 5)  /* GPIO1, PIN 5 */
#  define AM335X_IRQ_GPIO1P6            (AM335X_IRQ_GPIO1_PINS + 6)  /* GPIO1, PIN 6 */
#  define AM335X_IRQ_GPIO1P7            (AM335X_IRQ_GPIO1_PINS + 7)  /* GPIO1, PIN 7 */
#  define AM335X_IRQ_GPIO1P8            (AM335X_IRQ_GPIO1_PINS + 8)  /* GPIO1, PIN 8 */
#  define AM335X_IRQ_GPIO1P9            (AM335X_IRQ_GPIO1_PINS + 9)  /* GPIO1, PIN 9 */
#  define AM335X_IRQ_GPIO1P10           (AM335X_IRQ_GPIO1_PINS + 10) /* GPIO1, PIN 10 */
#  define AM335X_IRQ_GPIO1P11           (AM335X_IRQ_GPIO1_PINS + 11) /* GPIO1, PIN 11 */
#  define AM335X_IRQ_GPIO1P12           (AM335X_IRQ_GPIO1_PINS + 12) /* GPIO1, PIN 12 */
#  define AM335X_IRQ_GPIO1P13           (AM335X_IRQ_GPIO1_PINS + 13) /* GPIO1, PIN 13 */
#  define AM335X_IRQ_GPIO1P14           (AM335X_IRQ_GPIO1_PINS + 14) /* GPIO1, PIN 14 */
#  define AM335X_IRQ_GPIO1P15           (AM335X_IRQ_GPIO1_PINS + 15) /* GPIO1, PIN 15 */
#  define AM335X_IRQ_GPIO1P16           (AM335X_IRQ_GPIO1_PINS + 16) /* GPIO1, PIN 16 */
#  define AM335X_IRQ_GPIO1P17           (AM335X_IRQ_GPIO1_PINS + 17) /* GPIO1, PIN 17 */
#  define AM335X_IRQ_GPIO1P18           (AM335X_IRQ_GPIO1_PINS + 18) /* GPIO1, PIN 18 */
#  define AM335X_IRQ_GPIO1P19           (AM335X_IRQ_GPIO1_PINS + 19) /* GPIO1, PIN 19 */
#  define AM335X_IRQ_GPIO1P20           (AM335X_IRQ_GPIO1_PINS + 20) /* GPIO1, PIN 20 */
#  define AM335X_IRQ_GPIO1P21           (AM335X_IRQ_GPIO1_PINS + 21) /* GPIO1, PIN 21 */
#  define AM335X_IRQ_GPIO1P22           (AM335X_IRQ_GPIO1_PINS + 22) /* GPIO1, PIN 22 */
#  define AM335X_IRQ_GPIO1P23           (AM335X_IRQ_GPIO1_PINS + 23) /* GPIO1, PIN 23 */
#  define AM335X_IRQ_GPIO1P24           (AM335X_IRQ_GPIO1_PINS + 24) /* GPIO1, PIN 24 */
#  define AM335X_IRQ_GPIO1P25           (AM335X_IRQ_GPIO1_PINS + 25) /* GPIO1, PIN 25 */
#  define AM335X_IRQ_GPIO1P26           (AM335X_IRQ_GPIO1_PINS + 26) /* GPIO1, PIN 26 */
#  define AM335X_IRQ_GPIO1P27           (AM335X_IRQ_GPIO1_PINS + 27) /* GPIO1, PIN 27 */
#  define AM335X_IRQ_GPIO1P28           (AM335X_IRQ_GPIO1_PINS + 28) /* GPIO1, PIN 28 */
#  define AM335X_IRQ_GPIO1P29           (AM335X_IRQ_GPIO1_PINS + 29) /* GPIO1, PIN 29 */
#  define AM335X_IRQ_GPIO1P30           (AM335X_IRQ_GPIO1_PINS + 30) /* GPIO1, PIN 30 */
#  define AM335X_IRQ_GPIO1P31           (AM335X_IRQ_GPIO1_PINS + 31) /* GPIO1, PIN 31 */
#  define AM335X_NGPIO1IRQS             (32)
#else
#  define AM335X_NGPIO1IRQS             (0)
#endif

#ifdef CONFIG_AM335X_GPIO2_IRQ
#  define AM335X_IRQ_GPIO2_PINS         (AM335X_IRQ_NINT + AM335X_NGPIO0IRQS + AM335X_NGPIO1IRQS)
#  define AM335X_IRQ_GPIO2P0            (AM335X_IRQ_GPIO2_PINS + 0)  /* GPIO2, PIN 0 */
#  define AM335X_IRQ_GPIO2P1            (AM335X_IRQ_GPIO2_PINS + 1)  /* GPIO2, PIN 1 */
#  define AM335X_IRQ_GPIO2P2            (AM335X_IRQ_GPIO2_PINS + 2)  /* GPIO2, PIN 2 */
#  define AM335X_IRQ_GPIO2P3            (AM335X_IRQ_GPIO2_PINS + 3)  /* GPIO2, PIN 3 */
#  define AM335X_IRQ_GPIO2P4            (AM335X_IRQ_GPIO2_PINS + 4)  /* GPIO2, PIN 4 */
#  define AM335X_IRQ_GPIO2P5            (AM335X_IRQ_GPIO2_PINS + 5)  /* GPIO2, PIN 5 */
#  define AM335X_IRQ_GPIO2P6            (AM335X_IRQ_GPIO2_PINS + 6)  /* GPIO2, PIN 6 */
#  define AM335X_IRQ_GPIO2P7            (AM335X_IRQ_GPIO2_PINS + 7)  /* GPIO2, PIN 7 */
#  define AM335X_IRQ_GPIO2P8            (AM335X_IRQ_GPIO2_PINS + 8)  /* GPIO2, PIN 8 */
#  define AM335X_IRQ_GPIO2P9            (AM335X_IRQ_GPIO2_PINS + 9)  /* GPIO2, PIN 9 */
#  define AM335X_IRQ_GPIO2P10           (AM335X_IRQ_GPIO2_PINS + 10) /* GPIO2, PIN 10 */
#  define AM335X_IRQ_GPIO2P11           (AM335X_IRQ_GPIO2_PINS + 11) /* GPIO2, PIN 11 */
#  define AM335X_IRQ_GPIO2P12           (AM335X_IRQ_GPIO2_PINS + 12) /* GPIO2, PIN 12 */
#  define AM335X_IRQ_GPIO2P13           (AM335X_IRQ_GPIO2_PINS + 13) /* GPIO2, PIN 13 */
#  define AM335X_IRQ_GPIO2P14           (AM335X_IRQ_GPIO2_PINS + 14) /* GPIO2, PIN 14 */
#  define AM335X_IRQ_GPIO2P15           (AM335X_IRQ_GPIO2_PINS + 15) /* GPIO2, PIN 15 */
#  define AM335X_IRQ_GPIO2P16           (AM335X_IRQ_GPIO2_PINS + 16) /* GPIO2, PIN 16 */
#  define AM335X_IRQ_GPIO2P17           (AM335X_IRQ_GPIO2_PINS + 17) /* GPIO2, PIN 17 */
#  define AM335X_IRQ_GPIO2P18           (AM335X_IRQ_GPIO2_PINS + 18) /* GPIO2, PIN 18 */
#  define AM335X_IRQ_GPIO2P19           (AM335X_IRQ_GPIO2_PINS + 19) /* GPIO2, PIN 19 */
#  define AM335X_IRQ_GPIO2P20           (AM335X_IRQ_GPIO2_PINS + 20) /* GPIO2, PIN 20 */
#  define AM335X_IRQ_GPIO2P21           (AM335X_IRQ_GPIO2_PINS + 21) /* GPIO2, PIN 21 */
#  define AM335X_IRQ_GPIO2P22           (AM335X_IRQ_GPIO2_PINS + 22) /* GPIO2, PIN 22 */
#  define AM335X_IRQ_GPIO2P23           (AM335X_IRQ_GPIO2_PINS + 23) /* GPIO2, PIN 23 */
#  define AM335X_IRQ_GPIO2P24           (AM335X_IRQ_GPIO2_PINS + 24) /* GPIO2, PIN 24 */
#  define AM335X_IRQ_GPIO2P25           (AM335X_IRQ_GPIO2_PINS + 25) /* GPIO2, PIN 25 */
#  define AM335X_IRQ_GPIO2P26           (AM335X_IRQ_GPIO2_PINS + 26) /* GPIO2, PIN 26 */
#  define AM335X_IRQ_GPIO2P27           (AM335X_IRQ_GPIO2_PINS + 27) /* GPIO2, PIN 27 */
#  define AM335X_IRQ_GPIO2P28           (AM335X_IRQ_GPIO2_PINS + 28) /* GPIO2, PIN 28 */
#  define AM335X_IRQ_GPIO2P29           (AM335X_IRQ_GPIO2_PINS + 29) /* GPIO2, PIN 29 */
#  define AM335X_IRQ_GPIO2P30           (AM335X_IRQ_GPIO2_PINS + 30) /* GPIO2, PIN 30 */
#  define AM335X_IRQ_GPIO2P31           (AM335X_IRQ_GPIO2_PINS + 31) /* GPIO2, PIN 31 */
#  define AM335X_NGPIO2IRQS             (32)
#else
#  define AM335X_NGPIO2IRQS             (0)
#endif

#ifdef CONFIG_AM335X_GPIO3_IRQ
#  define AM335X_IRQ_GPIO3_PINS         (AM335X_IRQ_NINT + AM335X_NGPIO0IRQS + AM335X_NGPIO1IRQS + \
                                         AM335X_NGPIO2IRQS)
#  define AM335X_IRQ_GPIO3P0            (AM335X_IRQ_GPIO3_PINS + 0)  /* GPIO3, PIN 0 */
#  define AM335X_IRQ_GPIO3P1            (AM335X_IRQ_GPIO3_PINS + 1)  /* GPIO3, PIN 1 */
#  define AM335X_IRQ_GPIO3P2            (AM335X_IRQ_GPIO3_PINS + 2)  /* GPIO3, PIN 2 */
#  define AM335X_IRQ_GPIO3P3            (AM335X_IRQ_GPIO3_PINS + 3)  /* GPIO3, PIN 3 */
#  define AM335X_IRQ_GPIO3P4            (AM335X_IRQ_GPIO3_PINS + 4)  /* GPIO3, PIN 4 */
#  define AM335X_IRQ_GPIO3P5            (AM335X_IRQ_GPIO3_PINS + 5)  /* GPIO3, PIN 5 */
#  define AM335X_IRQ_GPIO3P6            (AM335X_IRQ_GPIO3_PINS + 6)  /* GPIO3, PIN 6 */
#  define AM335X_IRQ_GPIO3P7            (AM335X_IRQ_GPIO3_PINS + 7)  /* GPIO3, PIN 7 */
#  define AM335X_IRQ_GPIO3P8            (AM335X_IRQ_GPIO3_PINS + 8)  /* GPIO3, PIN 8 */
#  define AM335X_IRQ_GPIO3P9            (AM335X_IRQ_GPIO3_PINS + 9)  /* GPIO3, PIN 9 */
#  define AM335X_IRQ_GPIO3P10           (AM335X_IRQ_GPIO3_PINS + 10) /* GPIO3, PIN 10 */
#  define AM335X_IRQ_GPIO3P11           (AM335X_IRQ_GPIO3_PINS + 11) /* GPIO3, PIN 11 */
#  define AM335X_IRQ_GPIO3P12           (AM335X_IRQ_GPIO3_PINS + 12) /* GPIO3, PIN 12 */
#  define AM335X_IRQ_GPIO3P13           (AM335X_IRQ_GPIO3_PINS + 13) /* GPIO3, PIN 13 */
#  define AM335X_IRQ_GPIO3P14           (AM335X_IRQ_GPIO3_PINS + 14) /* GPIO3, PIN 14 */
#  define AM335X_IRQ_GPIO3P15           (AM335X_IRQ_GPIO3_PINS + 15) /* GPIO3, PIN 15 */
#  define AM335X_IRQ_GPIO3P16           (AM335X_IRQ_GPIO3_PINS + 16) /* GPIO3, PIN 16 */
#  define AM335X_IRQ_GPIO3P17           (AM335X_IRQ_GPIO3_PINS + 17) /* GPIO3, PIN 17 */
#  define AM335X_IRQ_GPIO3P18           (AM335X_IRQ_GPIO3_PINS + 18) /* GPIO3, PIN 18 */
#  define AM335X_IRQ_GPIO3P19           (AM335X_IRQ_GPIO3_PINS + 19) /* GPIO3, PIN 19 */
#  define AM335X_IRQ_GPIO3P20           (AM335X_IRQ_GPIO3_PINS + 20) /* GPIO3, PIN 20 */
#  define AM335X_IRQ_GPIO3P21           (AM335X_IRQ_GPIO3_PINS + 21) /* GPIO3, PIN 21 */
#  define AM335X_IRQ_GPIO3P22           (AM335X_IRQ_GPIO3_PINS + 22) /* GPIO3, PIN 22 */
#  define AM335X_IRQ_GPIO3P23           (AM335X_IRQ_GPIO3_PINS + 23) /* GPIO3, PIN 23 */
#  define AM335X_IRQ_GPIO3P24           (AM335X_IRQ_GPIO3_PINS + 24) /* GPIO3, PIN 24 */
#  define AM335X_IRQ_GPIO3P25           (AM335X_IRQ_GPIO3_PINS + 25) /* GPIO3, PIN 25 */
#  define AM335X_IRQ_GPIO3P26           (AM335X_IRQ_GPIO3_PINS + 26) /* GPIO3, PIN 26 */
#  define AM335X_IRQ_GPIO3P27           (AM335X_IRQ_GPIO3_PINS + 27) /* GPIO3, PIN 27 */
#  define AM335X_IRQ_GPIO3P28           (AM335X_IRQ_GPIO3_PINS + 28) /* GPIO3, PIN 28 */
#  define AM335X_IRQ_GPIO3P29           (AM335X_IRQ_GPIO3_PINS + 29) /* GPIO3, PIN 29 */
#  define AM335X_IRQ_GPIO3P30           (AM335X_IRQ_GPIO3_PINS + 30) /* GPIO3, PIN 30 */
#  define AM335X_IRQ_GPIO3P31           (AM335X_IRQ_GPIO3_PINS + 31) /* GPIO3, PIN 31 */
#  define AM335X_NGPIO3IRQS             (32)
#else
#  define AM335X_NGPIO3IRQS             (0)
#endif

/* Total number of IRQ numbers */

#define NR_IRQS                         (AM335X_IRQ_NINT + \
                                         AM335X_NGPIO0IRQS + AM335X_NGPIO1IRQS + \
                                         AM335X_NGPIO2IRQS + AM335X_NGPIO3IRQS )

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

#endif /* __ARCH_ARM_INCLUDE_AM335X_AM335X_IRQ_H */
