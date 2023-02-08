/****************************************************************************
 * arch/risc-v/include/hpm6750/irq.h
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

#ifndef __ARCH_RISCV_INCLUDE_HPM6750_IRQ_H
#define __ARCH_RISCV_INCLUDE_HPM6750_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Map RISC-V exception code to NuttX IRQ */

#define HPM6750_IRQ_PERI_START     (RISCV_IRQ_ASYNC + 20)

/* Machine Global External Interrupt */

#define HPM6750_IRQ_GPIO0_A       (HPM6750_IRQ_PERI_START + 1)
#define HPM6750_IRQ_GPIO0_B       (HPM6750_IRQ_PERI_START + 2)
#define HPM6750_IRQ_GPIO0_C       (HPM6750_IRQ_PERI_START + 3)
#define HPM6750_IRQ_GPIO0_D       (HPM6750_IRQ_PERI_START + 4)
#define HPM6750_IRQ_GPIO0_E       (HPM6750_IRQ_PERI_START + 5)
#define HPM6750_IRQ_GPIO0_F       (HPM6750_IRQ_PERI_START + 6)
#define HPM6750_IRQ_GPIO0_X       (HPM6750_IRQ_PERI_START + 7)
#define HPM6750_IRQ_GPIO0_Y       (HPM6750_IRQ_PERI_START + 8)
#define HPM6750_IRQ_GPIO0_Z       (HPM6750_IRQ_PERI_START + 9)
#define HPM6750_IRQ_GPIO1_A       (HPM6750_IRQ_PERI_START + 10)
#define HPM6750_IRQ_GPIO1_B       (HPM6750_IRQ_PERI_START + 11)
#define HPM6750_IRQ_GPIO1_C       (HPM6750_IRQ_PERI_START + 12)
#define HPM6750_IRQ_GPIO1_D       (HPM6750_IRQ_PERI_START + 13)
#define HPM6750_IRQ_GPIO1_E       (HPM6750_IRQ_PERI_START + 14)
#define HPM6750_IRQ_GPIO1_F       (HPM6750_IRQ_PERI_START + 15)
#define HPM6750_IRQ_GPIO1_X       (HPM6750_IRQ_PERI_START + 16)
#define HPM6750_IRQ_GPIO1_Y       (HPM6750_IRQ_PERI_START + 17)
#define HPM6750_IRQ_GPIO1_Z       (HPM6750_IRQ_PERI_START + 18)
#define HPM6750_IRQ_ADC0          (HPM6750_IRQ_PERI_START + 19)
#define HPM6750_IRQ_ADC1          (HPM6750_IRQ_PERI_START + 20)
#define HPM6750_IRQ_ADC2          (HPM6750_IRQ_PERI_START + 21)
#define HPM6750_IRQ_ADC3          (HPM6750_IRQ_PERI_START + 22)
#define HPM6750_IRQ_ACMP_0        (HPM6750_IRQ_PERI_START + 23)
#define HPM6750_IRQ_ACMP_1        (HPM6750_IRQ_PERI_START + 24)
#define HPM6750_IRQ_ACMP_2        (HPM6750_IRQ_PERI_START + 25)
#define HPM6750_IRQ_ACMP_3        (HPM6750_IRQ_PERI_START + 26)
#define HPM6750_IRQ_SPI0          (HPM6750_IRQ_PERI_START + 27)
#define HPM6750_IRQ_SPI1          (HPM6750_IRQ_PERI_START + 28)
#define HPM6750_IRQ_SPI2          (HPM6750_IRQ_PERI_START + 29)
#define HPM6750_IRQ_SPI3          (HPM6750_IRQ_PERI_START + 30)
#define HPM6750_IRQ_UART0         (HPM6750_IRQ_PERI_START + 31)
#define HPM6750_IRQ_UART1         (HPM6750_IRQ_PERI_START + 32)
#define HPM6750_IRQ_UART2         (HPM6750_IRQ_PERI_START + 33)
#define HPM6750_IRQ_UART3         (HPM6750_IRQ_PERI_START + 34)
#define HPM6750_IRQ_UART4         (HPM6750_IRQ_PERI_START + 35)
#define HPM6750_IRQ_UART5         (HPM6750_IRQ_PERI_START + 36)
#define HPM6750_IRQ_UART6         (HPM6750_IRQ_PERI_START + 37)
#define HPM6750_IRQ_UART7         (HPM6750_IRQ_PERI_START + 38)
#define HPM6750_IRQ_UART8         (HPM6750_IRQ_PERI_START + 39)
#define HPM6750_IRQ_UART9         (HPM6750_IRQ_PERI_START + 40)
#define HPM6750_IRQ_UART10        (HPM6750_IRQ_PERI_START + 41)
#define HPM6750_IRQ_UART11        (HPM6750_IRQ_PERI_START + 42)
#define HPM6750_IRQ_UART12        (HPM6750_IRQ_PERI_START + 43)
#define HPM6750_IRQ_UART13        (HPM6750_IRQ_PERI_START + 44)
#define HPM6750_IRQ_UART14        (HPM6750_IRQ_PERI_START + 45)
#define HPM6750_IRQ_UART15        (HPM6750_IRQ_PERI_START + 46)
#define HPM6750_IRQ_CAN0          (HPM6750_IRQ_PERI_START + 47)
#define HPM6750_IRQ_CAN1          (HPM6750_IRQ_PERI_START + 48)
#define HPM6750_IRQ_CAN2          (HPM6750_IRQ_PERI_START + 49)
#define HPM6750_IRQ_CAN3          (HPM6750_IRQ_PERI_START + 50)
#define HPM6750_IRQ_PTPC          (HPM6750_IRQ_PERI_START + 51)
#define HPM6750_IRQ_WDG0          (HPM6750_IRQ_PERI_START + 52)
#define HPM6750_IRQ_WDG1          (HPM6750_IRQ_PERI_START + 53)
#define HPM6750_IRQ_WDG2          (HPM6750_IRQ_PERI_START + 54)
#define HPM6750_IRQ_WDG3          (HPM6750_IRQ_PERI_START + 55)
#define HPM6750_IRQ_MBX0A         (HPM6750_IRQ_PERI_START + 56)
#define HPM6750_IRQ_MBX0B         (HPM6750_IRQ_PERI_START + 57)
#define HPM6750_IRQ_MBX1A         (HPM6750_IRQ_PERI_START + 58)
#define HPM6750_IRQ_MBX1B         (HPM6750_IRQ_PERI_START + 59)
#define HPM6750_IRQ_GPTMR0        (HPM6750_IRQ_PERI_START + 60)
#define HPM6750_IRQ_GPTMR1        (HPM6750_IRQ_PERI_START + 61)
#define HPM6750_IRQ_GPTMR2        (HPM6750_IRQ_PERI_START + 62)
#define HPM6750_IRQ_GPTMR3        (HPM6750_IRQ_PERI_START + 63)
#define HPM6750_IRQ_GPTMR4        (HPM6750_IRQ_PERI_START + 64)
#define HPM6750_IRQ_GPTMR5        (HPM6750_IRQ_PERI_START + 65)
#define HPM6750_IRQ_GPTMR6        (HPM6750_IRQ_PERI_START + 66)
#define HPM6750_IRQ_GPTMR7        (HPM6750_IRQ_PERI_START + 67)
#define HPM6750_IRQ_I2C0          (HPM6750_IRQ_PERI_START + 68)
#define HPM6750_IRQ_I2C1          (HPM6750_IRQ_PERI_START + 69)
#define HPM6750_IRQ_I2C2          (HPM6750_IRQ_PERI_START + 70)
#define HPM6750_IRQ_I2C3          (HPM6750_IRQ_PERI_START + 71)
#define HPM6750_IRQ_PWM0          (HPM6750_IRQ_PERI_START + 72)
#define HPM6750_IRQ_HALL0         (HPM6750_IRQ_PERI_START + 73)
#define HPM6750_IRQ_QEI0          (HPM6750_IRQ_PERI_START + 74)
#define HPM6750_IRQ_PWM1          (HPM6750_IRQ_PERI_START + 75)
#define HPM6750_IRQ_HALL1         (HPM6750_IRQ_PERI_START + 76)
#define HPM6750_IRQ_QEI1          (HPM6750_IRQ_PERI_START + 77)
#define HPM6750_IRQ_PWM2          (HPM6750_IRQ_PERI_START + 78)
#define HPM6750_IRQ_HALL2         (HPM6750_IRQ_PERI_START + 79)
#define HPM6750_IRQ_QEI2          (HPM6750_IRQ_PERI_START + 80)
#define HPM6750_IRQ_PWM3          (HPM6750_IRQ_PERI_START + 81)
#define HPM6750_IRQ_HALL3         (HPM6750_IRQ_PERI_START + 82)
#define HPM6750_IRQ_QEI3          (HPM6750_IRQ_PERI_START + 83)
#define HPM6750_IRQ_SDP           (HPM6750_IRQ_PERI_START + 84)
#define HPM6750_IRQ_XPI0          (HPM6750_IRQ_PERI_START + 85)
#define HPM6750_IRQ_XPI1          (HPM6750_IRQ_PERI_START + 86)
#define HPM6750_IRQ_XDMA          (HPM6750_IRQ_PERI_START + 87)
#define HPM6750_IRQ_HDMA          (HPM6750_IRQ_PERI_START + 88)
#define HPM6750_IRQ_FEMC          (HPM6750_IRQ_PERI_START + 89)
#define HPM6750_IRQ_RNG           (HPM6750_IRQ_PERI_START + 90)
#define HPM6750_IRQ_I2S0          (HPM6750_IRQ_PERI_START + 91)
#define HPM6750_IRQ_I2S1          (HPM6750_IRQ_PERI_START + 92)
#define HPM6750_IRQ_I2S2          (HPM6750_IRQ_PERI_START + 93)
#define HPM6750_IRQ_I2S3          (HPM6750_IRQ_PERI_START + 94)
#define HPM6750_IRQ_DAO           (HPM6750_IRQ_PERI_START + 95)
#define HPM6750_IRQ_PDM           (HPM6750_IRQ_PERI_START + 96)
#define HPM6750_IRQ_CAM0          (HPM6750_IRQ_PERI_START + 97)
#define HPM6750_IRQ_CAM1          (HPM6750_IRQ_PERI_START + 98)
#define HPM6750_IRQ_LCDC_D0       (HPM6750_IRQ_PERI_START + 99)
#define HPM6750_IRQ_LCDC_D1       (HPM6750_IRQ_PERI_START + 100)
#define HPM6750_IRQ_PDMA_D0       (HPM6750_IRQ_PERI_START + 101)
#define HPM6750_IRQ_PDMA_D1       (HPM6750_IRQ_PERI_START + 102)
#define HPM6750_IRQ_JPEG          (HPM6750_IRQ_PERI_START + 103)
#define HPM6750_IRQ_NTMR0         (HPM6750_IRQ_PERI_START + 104)
#define HPM6750_IRQ_NTMR1         (HPM6750_IRQ_PERI_START + 105)
#define HPM6750_IRQ_USB0          (HPM6750_IRQ_PERI_START + 106)
#define HPM6750_IRQ_USB1          (HPM6750_IRQ_PERI_START + 107)
#define HPM6750_IRQ_ENET0         (HPM6750_IRQ_PERI_START + 108)
#define HPM6750_IRQ_ENET1         (HPM6750_IRQ_PERI_START + 109)
#define HPM6750_IRQ_SDXC0         (HPM6750_IRQ_PERI_START + 110)
#define HPM6750_IRQ_SDXC1         (HPM6750_IRQ_PERI_START + 111)
#define HPM6750_IRQ_PSEC          (HPM6750_IRQ_PERI_START + 112)
#define HPM6750_IRQ_PGPIO         (HPM6750_IRQ_PERI_START + 113)
#define HPM6750_IRQ_PWDG          (HPM6750_IRQ_PERI_START + 114)
#define HPM6750_IRQ_PTMR          (HPM6750_IRQ_PERI_START + 115)
#define HPM6750_IRQ_PUART         (HPM6750_IRQ_PERI_START + 116)
#define HPM6750_IRQ_VAD           (HPM6750_IRQ_PERI_START + 117)
#define HPM6750_IRQ_FUSE          (HPM6750_IRQ_PERI_START + 118)
#define HPM6750_IRQ_SECMON        (HPM6750_IRQ_PERI_START + 119)
#define HPM6750_IRQ_RTC           (HPM6750_IRQ_PERI_START + 120)
#define HPM6750_IRQ_BUTN          (HPM6750_IRQ_PERI_START + 121)
#define HPM6750_IRQ_BGPIO         (HPM6750_IRQ_PERI_START + 122)
#define HPM6750_IRQ_BVIO          (HPM6750_IRQ_PERI_START + 123)
#define HPM6750_IRQ_BROWNOUT      (HPM6750_IRQ_PERI_START + 124)
#define HPM6750_IRQ_SYSCTL        (HPM6750_IRQ_PERI_START + 125)
#define HPM6750_IRQ_DEBUG_0       (HPM6750_IRQ_PERI_START + 126)
#define HPM6750_IRQ_DEBUG_1       (HPM6750_IRQ_PERI_START + 127)

/* Total number of IRQs */

#define NR_IRQS              (HPM6750_IRQ_PERI_START + 127)

#endif /* __ARCH_RISCV_INCLUDE_HPM6750_IRQ_H */
