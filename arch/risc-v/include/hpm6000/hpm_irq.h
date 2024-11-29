/****************************************************************************
 * arch/risc-v/include/hpm6000/hpm_irq.h
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

#ifndef __ARCH_RISCV_INCLUDE_HPM6000_HPM_IRQ_H
#define __ARCH_RISCV_INCLUDE_HPM6000_HPM_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Map RISC-V exception code to NuttX IRQ */

#define HPM_IRQ_PERI_START    (RISCV_IRQ_ASYNC + 20)

/* Machine Global External Interrupt */

#define HPM_IRQ_GPIO0_A       (HPM_IRQ_PERI_START + 1)
#define HPM_IRQ_GPIO0_B       (HPM_IRQ_PERI_START + 2)
#define HPM_IRQ_GPIO0_C       (HPM_IRQ_PERI_START + 3)
#define HPM_IRQ_GPIO0_D       (HPM_IRQ_PERI_START + 4)
#define HPM_IRQ_GPIO0_X       (HPM_IRQ_PERI_START + 5)
#define HPM_IRQ_GPIO0_Y       (HPM_IRQ_PERI_START + 6)
#define HPM_IRQ_GPIO0_Z       (HPM_IRQ_PERI_START + 7)
#define HPM_IRQ_ADC0          (HPM_IRQ_PERI_START + 8)
#define HPM_IRQ_ADC1          (HPM_IRQ_PERI_START + 9)
#define HPM_IRQ_ADC2          (HPM_IRQ_PERI_START + 10)
#define HPM_IRQ_DAC           (HPM_IRQ_PERI_START + 11)
#define HPM_IRQ_ACMP0         (HPM_IRQ_PERI_START + 12)
#define HPM_IRQ_ACMP1         (HPM_IRQ_PERI_START + 13)
#define HPM_IRQ_SPI0          (HPM_IRQ_PERI_START + 14)
#define HPM_IRQ_SPI1          (HPM_IRQ_PERI_START + 15)
#define HPM_IRQ_SPI2          (HPM_IRQ_PERI_START + 16)
#define HPM_IRQ_SPI3          (HPM_IRQ_PERI_START + 17)
#define HPM_IRQ_UART0         (HPM_IRQ_PERI_START + 18)
#define HPM_IRQ_UART1         (HPM_IRQ_PERI_START + 19)
#define HPM_IRQ_UART2         (HPM_IRQ_PERI_START + 20)
#define HPM_IRQ_UART3         (HPM_IRQ_PERI_START + 21)
#define HPM_IRQ_UART4         (HPM_IRQ_PERI_START + 22)
#define HPM_IRQ_UART5         (HPM_IRQ_PERI_START + 23)
#define HPM_IRQ_UART6         (HPM_IRQ_PERI_START + 24)
#define HPM_IRQ_UART7         (HPM_IRQ_PERI_START + 25)
#define HPM_IRQ_CAN0          (HPM_IRQ_PERI_START + 26)
#define HPM_IRQ_CAN1          (HPM_IRQ_PERI_START + 27)
#define HPM_IRQ_PTPC          (HPM_IRQ_PERI_START + 28)
#define HPM_IRQ_WDG0          (HPM_IRQ_PERI_START + 29)
#define HPM_IRQ_WDG1          (HPM_IRQ_PERI_START + 30)
#define HPM_IRQ_TSNS          (HPM_IRQ_PERI_START + 31)
#define HPM_IRQ_MBX0A         (HPM_IRQ_PERI_START + 32)
#define HPM_IRQ_MBX0B         (HPM_IRQ_PERI_START + 33)
#define HPM_IRQ_GPTMR0        (HPM_IRQ_PERI_START + 34)
#define HPM_IRQ_GPTMR1        (HPM_IRQ_PERI_START + 35)
#define HPM_IRQ_GPTMR2        (HPM_IRQ_PERI_START + 36)
#define HPM_IRQ_GPTMR3        (HPM_IRQ_PERI_START + 37)
#define HPM_IRQ_I2C0          (HPM_IRQ_PERI_START + 38)
#define HPM_IRQ_I2C1          (HPM_IRQ_PERI_START + 39)
#define HPM_IRQ_I2C2          (HPM_IRQ_PERI_START + 40)
#define HPM_IRQ_I2C3          (HPM_IRQ_PERI_START + 41)
#define HPM_IRQ_PWM0          (HPM_IRQ_PERI_START + 42)
#define HPM_IRQ_HALL0         (HPM_IRQ_PERI_START + 43)
#define HPM_IRQ_QEI0          (HPM_IRQ_PERI_START + 44)
#define HPM_IRQ_PWM1          (HPM_IRQ_PERI_START + 45)
#define HPM_IRQ_HALL1         (HPM_IRQ_PERI_START + 46)
#define HPM_IRQ_QEI1          (HPM_IRQ_PERI_START + 47)
#define HPM_IRQ_SDP           (HPM_IRQ_PERI_START + 48)
#define HPM_IRQ_XPI0          (HPM_IRQ_PERI_START + 49)
#define HPM_IRQ_XPI1          (HPM_IRQ_PERI_START + 50)
#define HPM_IRQ_XDMA          (HPM_IRQ_PERI_START + 51)
#define HPM_IRQ_HDMA          (HPM_IRQ_PERI_START + 52)
#define HPM_IRQ_FEMC          (HPM_IRQ_PERI_START + 53)
#define HPM_IRQ_RNG           (HPM_IRQ_PERI_START + 54)
#define HPM_IRQ_I2S0          (HPM_IRQ_PERI_START + 55)
#define HPM_IRQ_I2S1          (HPM_IRQ_PERI_START + 56)
#define HPM_IRQ_DAO           (HPM_IRQ_PERI_START + 57)
#define HPM_IRQ_PDM           (HPM_IRQ_PERI_START + 58)
#define HPM_IRQ_EFA           (HPM_IRQ_PERI_START + 59)
#define HPM_IRQ_NTMR0         (HPM_IRQ_PERI_START + 60)
#define HPM_IRQ_USB0          (HPM_IRQ_PERI_START + 61)
#define HPM_IRQ_ENET0         (HPM_IRQ_PERI_START + 62)
#define HPM_IRQ_SDXC0         (HPM_IRQ_PERI_START + 63)
#define HPM_IRQ_PSEC          (HPM_IRQ_PERI_START + 64)
#define HPM_IRQ_PGPIO         (HPM_IRQ_PERI_START + 65)
#define HPM_IRQ_PWDG          (HPM_IRQ_PERI_START + 66)
#define HPM_IRQ_PTMR          (HPM_IRQ_PERI_START + 67)
#define HPM_IRQ_PUART         (HPM_IRQ_PERI_START + 68)
#define HPM_IRQ_FUSE          (HPM_IRQ_PERI_START + 69)
#define HPM_IRQ_SECMON        (HPM_IRQ_PERI_START + 70)
#define HPM_IRQ_RTC           (HPM_IRQ_PERI_START + 71)
#define HPM_IRQ_BUTN          (HPM_IRQ_PERI_START + 72)
#define HPM_IRQ_BGPIO         (HPM_IRQ_PERI_START + 73)
#define HPM_IRQ_BVIO          (HPM_IRQ_PERI_START + 74)
#define HPM_IRQ_BROWNOUT      (HPM_IRQ_PERI_START + 75)
#define HPM_IRQ_SYSCTL        (HPM_IRQ_PERI_START + 76)

/* Total number of IRQs */

#define NR_IRQS               (HPM_IRQ_PERI_START + 76)

#endif /* __ARCH_RISCV_INCLUDE_HPM6000_HPM_IRQ_H */
