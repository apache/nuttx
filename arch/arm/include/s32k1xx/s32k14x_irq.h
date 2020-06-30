/************************************************************************************
 * arch/arm/include/s32k1xx/s32k14x_irq.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/* This file should never be included directly but, rather, only indirectly through
 * nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_S32K1XX_S32K14XX_IRQ_H
#define __ARCH_ARM_INCLUDE_S32K1XX_S32K14XX_IRQ_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* IRQ numbers.  The IRQ number corresponds vector number and hence map directly to
 * bits in the NVIC.  This does, however, waste several words of memory in the IRQ
 * to handle mapping tables.
 */

/* Processor Exceptions (vectors 0-15) */

#define S32K1XX_IRQ_RESERVED       (0)   /* Reserved vector (only used with CONFIG_DEBUG_FEATURES) */
                                         /* Vector  0: Reset stack pointer value */
                                         /* Vector  1: Reset (not handler as an IRQ) */
#define S32K1XX_IRQ_NMI            (2)   /* Vector  2: Non-Maskable Interrupt (NMI) */
#define S32K1XX_IRQ_HARDFAULT      (3)   /* Vector  3: Hard fault */
#define S32K1XX_IRQ_MEMFAULT       (4)   /* Vector  4: Memory management (MPU) */
#define S32K1XX_IRQ_BUSFAULT       (5)   /* Vector  5: Bus fault */
#define S32K1XX_IRQ_USAGEFAULT     (6)   /* Vector  6: Usage fault */
#define S32K1XX_IRQ_SVCALL        (11)   /* Vector 11: SVC call */
#define S32K1XX_IRQ_DBGMONITOR    (12)   /* Vector 12: Debug Monitor */
                                         /* Vector 13: Reserved */
#define S32K1XX_IRQ_PENDSV        (14)   /* Vector 14: Pendable system service request */
#define S32K1XX_IRQ_SYSTICK       (15)   /* Vector 15: System tick */

/* External interrupts (vectors >= 16).  These definitions are chip-specific */

#define S32K1XX_IRQ_INTERRUPT     (16)   /* Vector number of the first external interrupt */

#define S32K1XX_IRQ_DMACH0        (16)   /* DMA channel 0 transfer complete */
#define S32K1XX_IRQ_DMACH1        (17)   /* DMA channel 1 transfer complete */
#define S32K1XX_IRQ_DMACH2        (18)   /* DMA channel 2 transfer complete */
#define S32K1XX_IRQ_DMACH3        (19)   /* DMA channel 3 transfer complete */
#define S32K1XX_IRQ_DMACH4        (20)   /* DMA channel 4 transfer complete */
#define S32K1XX_IRQ_DMACH5        (21)   /* DMA channel 5 transfer complete */
#define S32K1XX_IRQ_DMACH6        (22)   /* DMA channel 6 transfer complete */
#define S32K1XX_IRQ_DMACH7        (23)   /* DMA channel 7 transfer complete */
#define S32K1XX_IRQ_DMACH8        (24)   /* DMA channel 8 transfer complete */
#define S32K1XX_IRQ_DMACH9        (25)   /* DMA channel 9 transfer complete */
#define S32K1XX_IRQ_DMACH10       (26)   /* DMA channel 10 transfer complete */
#define S32K1XX_IRQ_DMACH11       (27)   /* DMA channel 11 transfer complete */
#define S32K1XX_IRQ_DMACH12       (28)   /* DMA channel 12 transfer complete */
#define S32K1XX_IRQ_DMACH13       (29)   /* DMA channel 13 transfer complete */
#define S32K1XX_IRQ_DMACH14       (30)   /* DMA channel 14 transfer complete */
#define S32K1XX_IRQ_DMACH15       (31)   /* DMA channel 15 transfer complete */
#define S32K1XX_IRQ_DMACH_ERR     (32)   /* DMA error interrupt channels 0-15 */
#define S32K1XX_IRQ_MCM           (33)   /* FPU sources */
#define S32K1XX_IRQ_FTFCCMD       (34)   /* FTFC command complete */
#define S32K1XX_IRQ_FTFCCOL       (35)   /* FTFC read collition */
#define S32K1XX_IRQ_PMC           (36)   /* PMC Low voltage detect interrupt */
#define S32K1XX_IRQ_FTFCDBL       (37)   /* FTFC Double bit */
#define S32K1XX_IRQ_WDOG          (38)   /* WDOG interrupt request out before wdg reset out */
#define S32K1XX_IRQ_EWM           (38)   /* EWM output as interrupt */
#define S32K1XX_IRQ_RCM           (39)   /* RCM Asynchronous Interrupt */
#define S32K1XX_IRQ_LPI2C0M       (40)   /* LPI2C Master Interrupt */
#define S32K1XX_IRQ_LPI2C0S       (41)   /* LPI2C Slave Interrupt */
#define S32K1XX_IRQ_LPSPI0        (42)   /* LPSPI0 Interrupt */
#define S32K1XX_IRQ_LPSPI1        (43)   /* LPSPI1 Interrupt */
#define S32K1XX_IRQ_LPSPI2        (44)   /* LPSPI2 Interrupt */
#define S32K1XX_IRQ_LPI2C1M       (45)   /* LPI21 Master Interrupt */
#define S32K1XX_IRQ_LPI2C1S       (46)   /* LPI21 Slave Interrupt */
#define S32K1XX_IRQ_LPUART0       (47)   /* LPUART0 Interrupt */
                                         /* Reserved (48) */
#define S32K1XX_IRQ_LPUART1       (49)   /* LPUART1 Interrupt */
                                         /* Reserved (50) */
#define S32K1XX_IRQ_LPUART2       (51)   /* LPUART0 Interrupt */
                                         /* Reserved (52-54) */
#define S32K1XX_IRQ_ADC0          (55)   /* ADC0 Interrupt */
#define S32K1XX_IRQ_ADC1          (56)   /* ADC1 Interrupt */
#define S32K1XX_IRQ_CMP0          (57)   /* CMP0 Interrupt */
                                         /* Reserved (58-59) */
#define S32K1XX_IRQ_ERMS          (60)   /* ERM single bit error correction */
#define S32K1XX_IRQ_ERMD          (61)   /* ERM double bit error non-correctable */
#define S32K1XX_IRQ_RTC_ALARM     (62)   /* RTC alarm interrupt */
#define S32K1XX_IRQ_RTC_SEC       (63)   /* RTC seconds interrupt */
#define S32K1XX_IRQ_LPIT0         (64)   /* LPIT interrupt */
#define S32K1XX_IRQ_LPIT1         (65)   /* LPIT interrupt */
#define S32K1XX_IRQ_LPIT2         (66)   /* LPIT interrupt */
#define S32K1XX_IRQ_LPIT3         (67)   /* LPIT interrupt */
#define S32K1XX_IRQ_PDB0          (68)   /* PDB0 interrupt */
                                         /* Reserved (69-70) */
#define S32K1XX_IRQ_SAI1TX        (71)   /* SAI1 Transmit Synchronous Interrupt */
#define S32K1XX_IRQ_SAI1RX        (72)   /* SAI1 Receive Synchronous Interrupt */
#define S32K1XX_IRQ_SCGBUS        (73)   /* SCG bus interrupt request */
#define S32K1XX_IRQ_LPTIMER       (74)   /* LPTIMER interrupt request */
#define S32K1XX_IRQ_PORTA         (75)   /* PORTA Interrupt */
#define S32K1XX_IRQ_PORTB         (76)   /* PORTB Interrupt */
#define S32K1XX_IRQ_PORTC         (77)   /* PORTC Interrupt */
#define S32K1XX_IRQ_PORTD         (78)   /* PORTD Interrupt */
#define S32K1XX_IRQ_PORTE         (79)   /* PORTE Interrupt */
#define S32K1XX_IRQ_SOFTWARE      (80)   /* Software Interrupt */
#define S32K1XX_IRQ_QUADSPI       (81)   /* QuadSPI Interrupts */
                                         /* Reserved (82-83) */
#define S32K1XX_IRQ_PDB1          (84)   /* PDB1 Interrupt */
#define S32K1XX_IRQ_FLEXIO        (85)   /* FlexIO Interrupt */
#define S32K1XX_IRQ_SAI0_TX       (86)   /* SAI0 Transmit Synchronous Interrupt */
#define S32K1XX_IRQ_SAI0_RX       (87)   /* SAI0 Receive Synchronous Interrupt */
#define S32K1XX_IRQ_ENET_TIMER    (88)   /* ENET 1588 timer, time stamp, etc. */
#define S32K1XX_IRQ_ENET_TXDONE   (89)   /* ENET Data Tx Transfer Done Interrupt */
#define S32K1XX_IRQ_ENET_RXDONE   (90)   /* ENET Data Rx Transfer Done Interrupt */
#define S32K1XX_IRQ_ENET_ERROR    (91)   /* ENET Error Interrupt */
#define S32K1XX_IRQ_ENET_STOP     (92)   /* ENET Graceful Stop Interrupt */
#define S32K1XX_IRQ_ENET_WAKE     (93)   /* ENET Wake from Sleep Interrupt */
#define S32K1XX_IRQ_CAN0_BUS      (94)   /* CAN0 Bus On/Off Tx/Rx Warning */
#define S32K1XX_IRQ_CAN0_ERROR    (95)   /* CAN0 Bus Error Interrupt */
#define S32K1XX_IRQ_CAN0_LPRX     (96)   /* CAN0 Message Rx/Timeout in low power mode */
#define S32K1XX_IRQ_CAN0_0_15     (97)   /* CAN0 OR'ed Message buffer (0-15) */
#define S32K1XX_IRQ_CAN0_16_31    (98)   /* CAN0 OR'ed Message buffer (16-31) */
                                         /* Reserved (99-100) */
#define S32K1XX_IRQ_CAN1_BUS      (101)  /* CAN1 Bus On/Off Tx/Rx Warning */
#define S32K1XX_IRQ_CAN1_ERROR    (102)  /* CAN1 Bus Error Interrupt */
                                         /* Reserved (103) */
#define S32K1XX_IRQ_CAN1_0_15     (104)  /* CAN1 OR'ed Message buffer (0-15) */
#define S32K1XX_IRQ_CAN1_16_31    (105)  /* CAN1 OR'ed Message buffer (16-31) */
                                         /* Reserved (106-107) */
#define S32K1XX_IRQ_CAN2_BUS      (108)  /* CAN2 Bus On/Off Tx/Rx Warning */
#define S32K1XX_IRQ_CAN2_ERROR    (109)  /* CAN2 Bus Error Interrupt */
                                         /* Reserved (110) */
#define S32K1XX_IRQ_CAN2_0_15     (111)  /* CAN2 OR'ed Message buffer (0-15) */
#define S32K1XX_IRQ_CAN2_16_31    (112)  /* CAN2 OR'ed Message buffer (16-31) */
                                         /* Reserved (113-114) */
#define S32K1XX_IRQ_FTM0_CH0_1    (115)  /* FTM0 Channel 0/1 Interrupt */
#define S32K1XX_IRQ_FTM0_CH2_3    (116)  /* FTM0 Channel 2/3 Interrupt */
#define S32K1XX_IRQ_FTM0_CH4_5    (117)  /* FTM0 Channel 4/5 Interrupt */
#define S32K1XX_IRQ_FTM0_CH6_7    (118)  /* FTM0 Channel 6/7 Interrupt */
#define S32K1XX_IRQ_FTM0_FAULT    (119)  /* FTM0 Fault Interrupt */
#define S32K1XX_IRQ_FTM0_OVERFLOW (120)  /* FTM0 Counter Overflow/Reload Interrupt */
#define S32K1XX_IRQ_FTM1_CH0_1    (121)  /* FTM1 Channel 0/1 Interrupt */
#define S32K1XX_IRQ_FTM1_CH2_2    (122)  /* FTM1 Channel 2/3 Interrupt */
#define S32K1XX_IRQ_FTM1_CH4_5    (123)  /* FTM1 Channel 4/5 Interrupt */
#define S32K1XX_IRQ_FTM1_CH6_7    (124)  /* FTM1 Channel 6/7 Interrupt */
#define S32K1XX_IRQ_FTM1_FAULT    (125)  /* FTM1 Fault Interrupt */
#define S32K1XX_IRQ_FTM1_OVERFLOW (126)  /* FTM1 Counter Overflow/Reload Interrupt */
#define S32K1XX_IRQ_FTM2_CH0_1    (127)  /* FTM0 Channel 0/1 Interrupt */
#define S32K1XX_IRQ_FTM2_CH2_2    (128)  /* FTM0 Channel 2/3 Interrupt */
#define S32K1XX_IRQ_FTM2_CH4_5    (129)  /* FTM0 Channel 4/5 Interrupt */
#define S32K1XX_IRQ_FTM2_CH6_7    (130)  /* FTM0 Channel 6/7 Interrupt */
#define S32K1XX_IRQ_FTM2_FAULT    (131)  /* FTM0 Fault Interrupt */
#define S32K1XX_IRQ_FTM2_OVERFLOW (132)  /* FTM0 Counter Overflow/Reload Interrupt */
#define S32K1XX_IRQ_FTM3_CH0_1    (133)  /* FTM3 Channel 0/1 Interrupt */
#define S32K1XX_IRQ_FTM3_CH2_2    (134)  /* FTM3 Channel 2/3 Interrupt */
#define S32K1XX_IRQ_FTM3_CH4_5    (135)  /* FTM3 Channel 4/5 Interrupt */
#define S32K1XX_IRQ_FTM3_CH6_7    (136)  /* FTM3 Channel 6/7 Interrupt */
#define S32K1XX_IRQ_FTM3_FAULT    (137)  /* FTM3 Fault Interrupt */
#define S32K1XX_IRQ_FTM3_OVERFLOW (138)  /* FTM3 Counter Overflow/Reload Interrupt */
#define S32K1XX_IRQ_FTM4_CH0_1    (139)  /* FTM4 Channel 0/1 Interrupt */
#define S32K1XX_IRQ_FTM4_CH2_2    (140)  /* FTM4 Channel 2/3 Interrupt */
#define S32K1XX_IRQ_FTM4_CH4_5    (141)  /* FTM4 Channel 4/5 Interrupt */
#define S32K1XX_IRQ_FTM4_CH6_7    (142)  /* FTM4 Channel 6/7 Interrupt */
#define S32K1XX_IRQ_FTM4_FAULT    (143)  /* FTM4 Fault Interrupt */
#define S32K1XX_IRQ_FTM4_OVERFLOW (144)  /* FTM4 Counter Overflow/Reload Interrupt */
#define S32K1XX_IRQ_FTM5_CH0_1    (145)  /* FTM5 Channel 0/1 Interrupt */
#define S32K1XX_IRQ_FTM5_CH2_2    (146)  /* FTM5 Channel 2/3 Interrupt */
#define S32K1XX_IRQ_FTM5_CH4_5    (147)  /* FTM5 Channel 4/5 Interrupt */
#define S32K1XX_IRQ_FTM5_CH6_7    (148)  /* FTM5 Channel 6/7 Interrupt */
#define S32K1XX_IRQ_FTM5_FAULT    (149)  /* FTM5 Fault Interrupt */
#define S32K1XX_IRQ_FTM5_OVERFLOW (150)  /* FTM5 Counter Overflow/Reload Interrupt */
#define S32K1XX_IRQ_FTM6_CH0_1    (151)  /* FTM6 Channel 0/1 Interrupt */
#define S32K1XX_IRQ_FTM6_CH2_2    (152)  /* FTM6 Channel 2/3 Interrupt */
#define S32K1XX_IRQ_FTM6_CH4_5    (153)  /* FTM6 Channel 4/5 Interrupt */
#define S32K1XX_IRQ_FTM6_CH6_7    (154)  /* FTM6 Channel 6/7 Interrupt */
#define S32K1XX_IRQ_FTM6_FAULT    (155)  /* FTM6 Fault Interrupt */
#define S32K1XX_IRQ_FTM6_OVERFLOW (156)  /* FTM6 Counter Overflow/Reload Interrupt */
#define S32K1XX_IRQ_FTM7_CH0_1    (157)  /* FTM7 Channel 0/1 Interrupt */
#define S32K1XX_IRQ_FTM7_CH2_2    (158)  /* FTM7 Channel 2/3 Interrupt */
#define S32K1XX_IRQ_FTM7_CH4_5    (159)  /* FTM7 Channel 4/5 Interrupt */
#define S32K1XX_IRQ_FTM7_CH6_7    (160)  /* FTM7 Channel 6/7 Interrupt */
#define S32K1XX_IRQ_FTM7_FAULT    (161)  /* FTM7 Fault Interrupt */
#define S32K1XX_IRQ_FTM7_OVERFLOW (162)  /* FTM7 Counter Overflow/Reload Interrupt */

#define S32K1XX_IRQ_NIRQS         (163)
#define S32K1XX_IRQ_NEXTINT       (S32K1XX_IRQ_NIRQS - S32K1XX_IRQ_INTERRUPT)

/* Total number of IRQ numbers */

#define NR_IRQS                    S32K1XX_IRQ_NIRQS

#endif /* __ARCH_ARM_INCLUDE_S32K1XX_S32K14XX_IRQ_H */
