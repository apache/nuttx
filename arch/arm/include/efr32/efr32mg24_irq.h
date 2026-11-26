/****************************************************************************
 * arch/arm/include/efr32/efr32mg24_irq.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_INCLUDE_EFR32_EFR32MG24_IRQ_H
#define __ARCH_ARM_INCLUDE_EFR32_EFR32MG24_IRQ_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* EFR32xG24 external interrupts (EFR32xG24 Reference Manual, "Interrupt
 * Request Lines").
 */

#define EFR32_IRQ_SMU_SECURE        (EFR32_IRQ_FIRST + 0)
#define EFR32_IRQ_SMU_S_PRIVILEGED  (EFR32_IRQ_FIRST + 1)
#define EFR32_IRQ_SMU_NS_PRIVILEGED (EFR32_IRQ_FIRST + 2)
#define EFR32_IRQ_EMU               (EFR32_IRQ_FIRST + 3)
#define EFR32_IRQ_TIMER0            (EFR32_IRQ_FIRST + 4)
#define EFR32_IRQ_TIMER1            (EFR32_IRQ_FIRST + 5)
#define EFR32_IRQ_TIMER2            (EFR32_IRQ_FIRST + 6)
#define EFR32_IRQ_TIMER3            (EFR32_IRQ_FIRST + 7)
#define EFR32_IRQ_TIMER4            (EFR32_IRQ_FIRST + 8)
#define EFR32_IRQ_USART0_RX         (EFR32_IRQ_FIRST + 9)
#define EFR32_IRQ_USART0_TX         (EFR32_IRQ_FIRST + 10)
#define EFR32_IRQ_EUSART0_RX        (EFR32_IRQ_FIRST + 11)
#define EFR32_IRQ_EUSART0_TX        (EFR32_IRQ_FIRST + 12)
#define EFR32_IRQ_EUSART1_RX        (EFR32_IRQ_FIRST + 13)
#define EFR32_IRQ_EUSART1_TX        (EFR32_IRQ_FIRST + 14)
#define EFR32_IRQ_MVP               (EFR32_IRQ_FIRST + 15)
#define EFR32_IRQ_ICACHE0           (EFR32_IRQ_FIRST + 16)
#define EFR32_IRQ_BURTC             (EFR32_IRQ_FIRST + 17)
#define EFR32_IRQ_LETIMER0          (EFR32_IRQ_FIRST + 18)
#define EFR32_IRQ_SYSCFG            (EFR32_IRQ_FIRST + 19)
#define EFR32_IRQ_MPAHBRAM          (EFR32_IRQ_FIRST + 20)
#define EFR32_IRQ_LDMA              (EFR32_IRQ_FIRST + 21)
#define EFR32_IRQ_LFXO              (EFR32_IRQ_FIRST + 22)
#define EFR32_IRQ_LFRCO             (EFR32_IRQ_FIRST + 23)
#define EFR32_IRQ_ULFRCO            (EFR32_IRQ_FIRST + 24)
#define EFR32_IRQ_GPIO_ODD          (EFR32_IRQ_FIRST + 25)
#define EFR32_IRQ_GPIO_EVEN         (EFR32_IRQ_FIRST + 26)
#define EFR32_IRQ_I2C0              (EFR32_IRQ_FIRST + 27)
#define EFR32_IRQ_I2C1              (EFR32_IRQ_FIRST + 28)
#define EFR32_IRQ_EMUDG             (EFR32_IRQ_FIRST + 29)
#define EFR32_IRQ_AGC               (EFR32_IRQ_FIRST + 30)
#define EFR32_IRQ_BUFC              (EFR32_IRQ_FIRST + 31)
#define EFR32_IRQ_FRC_PRI           (EFR32_IRQ_FIRST + 32)
#define EFR32_IRQ_FRC               (EFR32_IRQ_FIRST + 33)
#define EFR32_IRQ_MODEM             (EFR32_IRQ_FIRST + 34)
#define EFR32_IRQ_PROTIMER          (EFR32_IRQ_FIRST + 35)
#define EFR32_IRQ_RAC_RSM           (EFR32_IRQ_FIRST + 36)
#define EFR32_IRQ_RAC_SEQ           (EFR32_IRQ_FIRST + 37)
#define EFR32_IRQ_HOSTMAILBOX       (EFR32_IRQ_FIRST + 38)
#define EFR32_IRQ_SYNTH             (EFR32_IRQ_FIRST + 39)
#define EFR32_IRQ_ACMP0             (EFR32_IRQ_FIRST + 40)
#define EFR32_IRQ_ACMP1             (EFR32_IRQ_FIRST + 41)
#define EFR32_IRQ_WDOG0             (EFR32_IRQ_FIRST + 42)
#define EFR32_IRQ_WDOG1             (EFR32_IRQ_FIRST + 43)
#define EFR32_IRQ_HFXO0             (EFR32_IRQ_FIRST + 44)
#define EFR32_IRQ_HFRCO0            (EFR32_IRQ_FIRST + 45)
#define EFR32_IRQ_HFRCOEM23         (EFR32_IRQ_FIRST + 46)
#define EFR32_IRQ_CMU               (EFR32_IRQ_FIRST + 47)
#define EFR32_IRQ_AES               (EFR32_IRQ_FIRST + 48)
#define EFR32_IRQ_IADC              (EFR32_IRQ_FIRST + 49)
#define EFR32_IRQ_MSC               (EFR32_IRQ_FIRST + 50)
#define EFR32_IRQ_DPLL0             (EFR32_IRQ_FIRST + 51)
#define EFR32_IRQ_EMUEFP            (EFR32_IRQ_FIRST + 52)
#define EFR32_IRQ_DCDC              (EFR32_IRQ_FIRST + 53)
#define EFR32_IRQ_PCNT0             (EFR32_IRQ_FIRST + 54)
#define EFR32_IRQ_SW0               (EFR32_IRQ_FIRST + 55)
#define EFR32_IRQ_SW1               (EFR32_IRQ_FIRST + 56)
#define EFR32_IRQ_SW2               (EFR32_IRQ_FIRST + 57)
#define EFR32_IRQ_SW3               (EFR32_IRQ_FIRST + 58)
#define EFR32_IRQ_KERNEL0           (EFR32_IRQ_FIRST + 59)
#define EFR32_IRQ_KERNEL1           (EFR32_IRQ_FIRST + 60)
#define EFR32_IRQ_M33CTI0           (EFR32_IRQ_FIRST + 61)
#define EFR32_IRQ_M33CTI1           (EFR32_IRQ_FIRST + 62)
#define EFR32_IRQ_FPUEXH            (EFR32_IRQ_FIRST + 63)
#define EFR32_IRQ_SETAMPERHOST      (EFR32_IRQ_FIRST + 64)
#define EFR32_IRQ_SEMBRX            (EFR32_IRQ_FIRST + 65)
#define EFR32_IRQ_SEMBTX            (EFR32_IRQ_FIRST + 66)
#define EFR32_IRQ_SYSRTC_APP        (EFR32_IRQ_FIRST + 67)
#define EFR32_IRQ_SYSRTC_SEQ        (EFR32_IRQ_FIRST + 68)
#define EFR32_IRQ_KEYSCAN           (EFR32_IRQ_FIRST + 69)
#define EFR32_IRQ_RFECA0            (EFR32_IRQ_FIRST + 70)
#define EFR32_IRQ_RFECA1            (EFR32_IRQ_FIRST + 71)
#define EFR32_IRQ_VDAC0             (EFR32_IRQ_FIRST + 72)
#define EFR32_IRQ_VDAC1             (EFR32_IRQ_FIRST + 73)
#define EFR32_IRQ_AHB2AHB0          (EFR32_IRQ_FIRST + 74)
#define EFR32_IRQ_AHB2AHB1          (EFR32_IRQ_FIRST + 75)

#define EFR32_IRQ_NEXTINTS          76

#endif /* __ARCH_ARM_INCLUDE_EFR32_EFR32MG24_IRQ_H */
