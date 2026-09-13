/****************************************************************************
 * arch/arm/include/n32h7/n32h76x_irq.h
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
 * only indirectly through arch/irq.h
 */

 #ifndef __ARCH_ARM_INCLUDE_N32H7_N32H76X_IRQ_H
 #define __ARCH_ARM_INCLUDE_N32H7_N32H76X_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* IRQ numbers.
 * The IRQ number corresponds vector number and hence map directly to bits
 * in the NVIC.
 * This does, however, waste several words of memory in the IRQ to handle
 * mapping tables.
 *
 * Processor Exceptions (vectors 0-15).  These common definitions can be
 * found in the file nuttx/arch/arm/include/n32h7/irq.h which includes
 * this file
 *
 * External interrupts (vectors >= 16)
 */

/* IRQ numbers for N32H76x CM7 (NVIC1)
 * The IRQ number corresponds to vector number (offset from 0)
 * External interrupts (vectors 16-249)
 */

#define N32_IRQ_WWDG1             (N32_IRQ_FIRST + 0)
#define N32_IRQ_PVD               (N32_IRQ_FIRST + 1)
#define N32_IRQ_RTC_TAMPER        (N32_IRQ_FIRST + 2)
#define N32_IRQ_RTC_WKUP          (N32_IRQ_FIRST + 3)
#define N32_IRQ_RCC               (N32_IRQ_FIRST + 4)
#define N32_IRQ_EXTI0             (N32_IRQ_FIRST + 5)
#define N32_IRQ_EXTI1             (N32_IRQ_FIRST + 6)
#define N32_IRQ_EXTI2             (N32_IRQ_FIRST + 7)
#define N32_IRQ_EXTI3             (N32_IRQ_FIRST + 8)
#define N32_IRQ_EXTI4             (N32_IRQ_FIRST + 9)
#define N32_IRQ_EXTI9_5           (N32_IRQ_FIRST + 10)
#define N32_IRQ_EXTI15_10         (N32_IRQ_FIRST + 11)
#define N32_IRQ_DMA1_CHANNEL0     (N32_IRQ_FIRST + 12)
#define N32_IRQ_DMA1_CHANNEL1     (N32_IRQ_FIRST + 13)
#define N32_IRQ_DMA1_CHANNEL2     (N32_IRQ_FIRST + 14)
#define N32_IRQ_DMA1_CHANNEL3     (N32_IRQ_FIRST + 15)
#define N32_IRQ_DMA1_CHANNEL4     (N32_IRQ_FIRST + 16)
#define N32_IRQ_DMA1_CHANNEL5     (N32_IRQ_FIRST + 17)
#define N32_IRQ_DMA1_CHANNEL6     (N32_IRQ_FIRST + 18)
#define N32_IRQ_DMA1_CHANNEL7     (N32_IRQ_FIRST + 19)
#define N32_IRQ_DMA2_CHANNEL0     (N32_IRQ_FIRST + 20)
#define N32_IRQ_DMA2_CHANNEL1     (N32_IRQ_FIRST + 21)
#define N32_IRQ_DMA2_CHANNEL2     (N32_IRQ_FIRST + 22)
#define N32_IRQ_DMA2_CHANNEL3     (N32_IRQ_FIRST + 23)
#define N32_IRQ_DMA2_CHANNEL4     (N32_IRQ_FIRST + 24)
#define N32_IRQ_DMA2_CHANNEL5     (N32_IRQ_FIRST + 25)
#define N32_IRQ_DMA2_CHANNEL6     (N32_IRQ_FIRST + 26)
#define N32_IRQ_DMA2_CHANNEL7     (N32_IRQ_FIRST + 27)
#define N32_IRQ_DMA3_CHANNEL0     (N32_IRQ_FIRST + 28)
#define N32_IRQ_DMA3_CHANNEL1     (N32_IRQ_FIRST + 29)
#define N32_IRQ_DMA3_CHANNEL2     (N32_IRQ_FIRST + 30)
#define N32_IRQ_DMA3_CHANNEL3     (N32_IRQ_FIRST + 31)
#define N32_IRQ_DMA3_CHANNEL4     (N32_IRQ_FIRST + 32)
#define N32_IRQ_DMA3_CHANNEL5     (N32_IRQ_FIRST + 33)
#define N32_IRQ_DMA3_CHANNEL6     (N32_IRQ_FIRST + 34)
#define N32_IRQ_DMA3_CHANNEL7     (N32_IRQ_FIRST + 35)
#define N32_IRQ_MDMA_CHANNEL0     (N32_IRQ_FIRST + 36)
#define N32_IRQ_MDMA_CHANNEL1     (N32_IRQ_FIRST + 37)
#define N32_IRQ_MDMA_CHANNEL2     (N32_IRQ_FIRST + 38)
#define N32_IRQ_MDMA_CHANNEL3     (N32_IRQ_FIRST + 39)
#define N32_IRQ_MDMA_CHANNEL4     (N32_IRQ_FIRST + 40)
#define N32_IRQ_MDMA_CHANNEL5     (N32_IRQ_FIRST + 41)
#define N32_IRQ_MDMA_CHANNEL6     (N32_IRQ_FIRST + 42)
#define N32_IRQ_MDMA_CHANNEL7     (N32_IRQ_FIRST + 43)
#define N32_IRQ_MDMA_CHANNEL8     (N32_IRQ_FIRST + 44)
#define N32_IRQ_MDMA_CHANNEL9     (N32_IRQ_FIRST + 45)
#define N32_IRQ_MDMA_CHANNEL10    (N32_IRQ_FIRST + 46)
#define N32_IRQ_MDMA_CHANNEL11    (N32_IRQ_FIRST + 47)
#define N32_IRQ_MDMA_CHANNEL12    (N32_IRQ_FIRST + 48)
#define N32_IRQ_MDMA_CHANNEL13    (N32_IRQ_FIRST + 49)
#define N32_IRQ_MDMA_CHANNEL14    (N32_IRQ_FIRST + 50)
#define N32_IRQ_MDMA_CHANNEL15    (N32_IRQ_FIRST + 51)
#define N32_IRQ_SDPU              (N32_IRQ_FIRST + 52)

/* Reserved (53) */

/* Reserved (54) */
#define N32_IRQ_FPU_CPU1          (N32_IRQ_FIRST + 55)
#define N32_IRQ_ECCMON            (N32_IRQ_FIRST + 56)
#define N32_IRQ_RTC_ALARM         (N32_IRQ_FIRST + 57)
#define N32_IRQ_I2C1_EVENT        (N32_IRQ_FIRST + 58)
#define N32_IRQ_I2C1_ERR          (N32_IRQ_FIRST + 59)
#define N32_IRQ_I2C2_EVENT        (N32_IRQ_FIRST + 60)
#define N32_IRQ_I2C2_ERR          (N32_IRQ_FIRST + 61)
#define N32_IRQ_I2C3_EVENT        (N32_IRQ_FIRST + 62)
#define N32_IRQ_I2C3_ERR          (N32_IRQ_FIRST + 63)
#define N32_IRQ_I2C4_EVENT        (N32_IRQ_FIRST + 64)
#define N32_IRQ_I2C4_ERR          (N32_IRQ_FIRST + 65)
#define N32_IRQ_I2C5_EVENT        (N32_IRQ_FIRST + 66)
#define N32_IRQ_I2C5_ERR          (N32_IRQ_FIRST + 67)
#define N32_IRQ_I2C6_EVENT        (N32_IRQ_FIRST + 68)
#define N32_IRQ_I2C6_ERR          (N32_IRQ_FIRST + 69)
#define N32_IRQ_I2C7_EVENT        (N32_IRQ_FIRST + 70)
#define N32_IRQ_I2C7_ERR          (N32_IRQ_FIRST + 71)
#define N32_IRQ_I2C8_EVENT        (N32_IRQ_FIRST + 72)
#define N32_IRQ_I2C8_ERR          (N32_IRQ_FIRST + 73)
#define N32_IRQ_I2C9_EVENT        (N32_IRQ_FIRST + 74)
#define N32_IRQ_I2C9_ERR          (N32_IRQ_FIRST + 75)
#define N32_IRQ_I2C10_EVENT       (N32_IRQ_FIRST + 76)
#define N32_IRQ_I2C10_ERR         (N32_IRQ_FIRST + 77)
#define N32_IRQ_I2S1              (N32_IRQ_FIRST + 78)
#define N32_IRQ_I2S2              (N32_IRQ_FIRST + 79)
#define N32_IRQ_I2S3              (N32_IRQ_FIRST + 80)
#define N32_IRQ_I2S4              (N32_IRQ_FIRST + 81)
#define N32_IRQ_xSPI1_IRQ         (N32_IRQ_FIRST + 82)
#define N32_IRQ_xSPI2_IRQ         (N32_IRQ_FIRST + 83)
#define N32_IRQ_SPI1              (N32_IRQ_FIRST + 84)
#define N32_IRQ_SPI2              (N32_IRQ_FIRST + 85)
#define N32_IRQ_SPI3              (N32_IRQ_FIRST + 86)
#define N32_IRQ_SPI4              (N32_IRQ_FIRST + 87)
#define N32_IRQ_SPI5              (N32_IRQ_FIRST + 88)
#define N32_IRQ_SPI6              (N32_IRQ_FIRST + 89)
#define N32_IRQ_SPI7              (N32_IRQ_FIRST + 90)
#define N32_IRQ_LCD_EVENT         (N32_IRQ_FIRST + 91)
#define N32_IRQ_LCD_ERR           (N32_IRQ_FIRST + 92)
#define N32_IRQ_DVP1              (N32_IRQ_FIRST + 93)
#define N32_IRQ_DVP2              (N32_IRQ_FIRST + 94)
#define N32_IRQ_DMAMUX2           (N32_IRQ_FIRST + 95)
#define N32_IRQ_USBHS1_HS_EPx_OUT (N32_IRQ_FIRST + 96)
#define N32_IRQ_USBHS1_HS_EPx_IN  (N32_IRQ_FIRST + 97)
#define N32_IRQ_USBHS1_HS_WKUP    (N32_IRQ_FIRST + 98)
#define N32_IRQ_USBHS1_HS         (N32_IRQ_FIRST + 99)
#define N32_IRQ_USBHS2_HS_EPx_OUT (N32_IRQ_FIRST + 100)
#define N32_IRQ_USBHS2_HS_EPx_IN  (N32_IRQ_FIRST + 101)
#define N32_IRQ_USBHS2_HS_WKUP    (N32_IRQ_FIRST + 102)
#define N32_IRQ_USBHS2_HS         (N32_IRQ_FIRST + 103)
#define N32_IRQ_ETH1              (N32_IRQ_FIRST + 104)
#define N32_IRQ_ETH1_PMT_LPI      (N32_IRQ_FIRST + 105)
#define N32_IRQ_ETH2              (N32_IRQ_FIRST + 106)
#define N32_IRQ_ETH2_PMT_LPI      (N32_IRQ_FIRST + 107)
#define N32_IRQ_FDCAN1_INT0       (N32_IRQ_FIRST + 108)
#define N32_IRQ_FDCAN2_INT0       (N32_IRQ_FIRST + 109)
#define N32_IRQ_FDCAN3_INT0       (N32_IRQ_FIRST + 110)
#define N32_IRQ_FDCAN4_INT0       (N32_IRQ_FIRST + 111)
#define N32_IRQ_FDCAN1_INT1       (N32_IRQ_FIRST + 112)
#define N32_IRQ_FDCAN2_INT1       (N32_IRQ_FIRST + 113)
#define N32_IRQ_FDCAN3_INT1       (N32_IRQ_FIRST + 114)
#define N32_IRQ_FDCAN4_INT1       (N32_IRQ_FIRST + 115)
#define N32_IRQ_USART1            (N32_IRQ_FIRST + 116)
#define N32_IRQ_USART2            (N32_IRQ_FIRST + 117)
#define N32_IRQ_USART3            (N32_IRQ_FIRST + 118)
#define N32_IRQ_USART4            (N32_IRQ_FIRST + 119)
#define N32_IRQ_USART5            (N32_IRQ_FIRST + 120)
#define N32_IRQ_USART6            (N32_IRQ_FIRST + 121)
#define N32_IRQ_USART7            (N32_IRQ_FIRST + 122)
#define N32_IRQ_USART8            (N32_IRQ_FIRST + 123)
#define N32_IRQ_UART9             (N32_IRQ_FIRST + 124)
#define N32_IRQ_UART10            (N32_IRQ_FIRST + 125)
#define N32_IRQ_UART11            (N32_IRQ_FIRST + 126)
#define N32_IRQ_UART12            (N32_IRQ_FIRST + 127)
#define N32_IRQ_UART13            (N32_IRQ_FIRST + 128)
#define N32_IRQ_UART14            (N32_IRQ_FIRST + 129)
#define N32_IRQ_UART15            (N32_IRQ_FIRST + 130)
#define N32_IRQ_LPUART1           (N32_IRQ_FIRST + 131)
#define N32_IRQ_LPUART2           (N32_IRQ_FIRST + 132)
#define N32_IRQ_GPU               (N32_IRQ_FIRST + 133)

/* Reserved (134) */
#define N32_IRQ_SDMMC1            (N32_IRQ_FIRST + 135)
#define N32_IRQ_SDMMC2            (N32_IRQ_FIRST + 136)
#define N32_IRQ_ADC1              (N32_IRQ_FIRST + 137)
#define N32_IRQ_ADC2              (N32_IRQ_FIRST + 138)
#define N32_IRQ_ADC3              (N32_IRQ_FIRST + 139)
#define N32_IRQ_COMP12            (N32_IRQ_FIRST + 140)
#define N32_IRQ_COMP34            (N32_IRQ_FIRST + 141)
#define N32_IRQ_SHRTIM1_INT1      (N32_IRQ_FIRST + 142)
#define N32_IRQ_SHRTIM1_INT2      (N32_IRQ_FIRST + 143)
#define N32_IRQ_SHRTIM1_INT3      (N32_IRQ_FIRST + 144)
#define N32_IRQ_SHRTIM1_INT4      (N32_IRQ_FIRST + 145)
#define N32_IRQ_SHRTIM1_INT5      (N32_IRQ_FIRST + 146)
#define N32_IRQ_SHRTIM1_INT6      (N32_IRQ_FIRST + 147)
#define N32_IRQ_SHRTIM1_INT7      (N32_IRQ_FIRST + 148)
#define N32_IRQ_SHRTIM1_INT8      (N32_IRQ_FIRST + 149)
#define N32_IRQ_SHRTIM2_INT1      (N32_IRQ_FIRST + 150)
#define N32_IRQ_SHRTIM2_INT2      (N32_IRQ_FIRST + 151)
#define N32_IRQ_SHRTIM2_INT3      (N32_IRQ_FIRST + 152)
#define N32_IRQ_SHRTIM2_INT4      (N32_IRQ_FIRST + 153)
#define N32_IRQ_SHRTIM2_INT5      (N32_IRQ_FIRST + 154)
#define N32_IRQ_SHRTIM2_INT6      (N32_IRQ_FIRST + 155)
#define N32_IRQ_SHRTIM2_INT7      (N32_IRQ_FIRST + 156)
#define N32_IRQ_SHRTIM2_INT8      (N32_IRQ_FIRST + 157)
#define N32_IRQ_FDCAN5_INT0       (N32_IRQ_FIRST + 158)
#define N32_IRQ_FDCAN6_INT0       (N32_IRQ_FIRST + 159)
#define N32_IRQ_FDCAN7_INT0       (N32_IRQ_FIRST + 160)
#define N32_IRQ_FDCAN8_INT0       (N32_IRQ_FIRST + 161)
#define N32_IRQ_FDCAN5_INT1       (N32_IRQ_FIRST + 162)
#define N32_IRQ_FDCAN6_INT1       (N32_IRQ_FIRST + 163)
#define N32_IRQ_FDCAN7_INT1       (N32_IRQ_FIRST + 164)
#define N32_IRQ_FDCAN8_INT1       (N32_IRQ_FIRST + 165)
#define N32_IRQ_DSI               (N32_IRQ_FIRST + 166)

/* Reserved (167) */
#define N32_IRQ_AHB_CACHE_PARMON  (N32_IRQ_FIRST + 168)
#define N32_IRQ_LPTIM5_WKUP       (N32_IRQ_FIRST + 169)
#define N32_IRQ_JPEG_SGDMA_H2P    (N32_IRQ_FIRST + 170)
#define N32_IRQ_JPEG_SGDMA_P2H    (N32_IRQ_FIRST + 171)
#define N32_IRQ_WAKEUP_IO         (N32_IRQ_FIRST + 172)
#define N32_IRQ_SEMA4_INT1        (N32_IRQ_FIRST + 173)
#define N32_IRQ_WWDG1_RST         (N32_IRQ_FIRST + 174)
#define N32_IRQ_OTPC              (N32_IRQ_FIRST + 175)
#define N32_IRQ_FEMC              (N32_IRQ_FIRST + 176)
#define N32_IRQ_DCMUA             (N32_IRQ_FIRST + 177)
#define N32_IRQ_DAC1              (N32_IRQ_FIRST + 178)
#define N32_IRQ_DAC2              (N32_IRQ_FIRST + 179)
#define N32_IRQ_MDMA_AHBS_ER      (N32_IRQ_FIRST + 180)
#define N32_IRQ_CM7_CACHE_READ_ER (N32_IRQ_FIRST + 181)
#define N32_IRQ_DAC3              (N32_IRQ_FIRST + 182)
#define N32_IRQ_DAC4              (N32_IRQ_FIRST + 183)
#define N32_IRQ_EMC               (N32_IRQ_FIRST + 184)
#define N32_IRQ_DAC5              (N32_IRQ_FIRST + 185)
#define N32_IRQ_DAC6              (N32_IRQ_FIRST + 186)
#define N32_IRQ_ESC_OPB           (N32_IRQ_FIRST + 187)
#define N32_IRQ_ESC_SYNC0         (N32_IRQ_FIRST + 188)
#define N32_IRQ_ESC_SYNC1         (N32_IRQ_FIRST + 189)
#define N32_IRQ_ESC_WRP           (N32_IRQ_FIRST + 190)

/* Reserved (191) */
#define N32_IRQ_ATIM1_BRK         (N32_IRQ_FIRST + 192)
#define N32_IRQ_ATIM1_TRG_COM     (N32_IRQ_FIRST + 193)
#define N32_IRQ_ATIM1_CC          (N32_IRQ_FIRST + 194)
#define N32_IRQ_ATIM1_UP          (N32_IRQ_FIRST + 195)
#define N32_IRQ_ATIM2_BRK         (N32_IRQ_FIRST + 196)
#define N32_IRQ_ATIM2_TRG_COM     (N32_IRQ_FIRST + 197)
#define N32_IRQ_ATIM2_CC          (N32_IRQ_FIRST + 198)
#define N32_IRQ_ATIM2_UP          (N32_IRQ_FIRST + 199)
#define N32_IRQ_ATIM3_BRK         (N32_IRQ_FIRST + 200)
#define N32_IRQ_ATIM3_TRG_COM     (N32_IRQ_FIRST + 201)
#define N32_IRQ_ATIM3_CC          (N32_IRQ_FIRST + 202)
#define N32_IRQ_ATIM3_UP          (N32_IRQ_FIRST + 203)
#define N32_IRQ_ATIM4_BRK         (N32_IRQ_FIRST + 204)
#define N32_IRQ_ATIM4_TRG_COM     (N32_IRQ_FIRST + 205)
#define N32_IRQ_ATIM4_CC          (N32_IRQ_FIRST + 206)
#define N32_IRQ_ATIM4_UP          (N32_IRQ_FIRST + 207)
#define N32_IRQ_GTIMA1            (N32_IRQ_FIRST + 208)
#define N32_IRQ_GTIMA2            (N32_IRQ_FIRST + 209)
#define N32_IRQ_GIIMA3            (N32_IRQ_FIRST + 210)
#define N32_IRQ_GTIMA4            (N32_IRQ_FIRST + 211)
#define N32_IRQ_GTIMA5            (N32_IRQ_FIRST + 212)
#define N32_IRQ_GTIMA6            (N32_IRQ_FIRST + 213)
#define N32_IRQ_GTIMA7            (N32_IRQ_FIRST + 214)
#define N32_IRQ_GTIMB1            (N32_IRQ_FIRST + 215)
#define N32_IRQ_GTIMB2            (N32_IRQ_FIRST + 216)
#define N32_IRQ_GTIMB3            (N32_IRQ_FIRST + 217)
#define N32_IRQ_BTIM1             (N32_IRQ_FIRST + 218)
#define N32_IRQ_BTIM2             (N32_IRQ_FIRST + 219)
#define N32_IRQ_BTIM3             (N32_IRQ_FIRST + 220)
#define N32_IRQ_BTIM4             (N32_IRQ_FIRST + 221)
#define N32_IRQ_LPTIM1_WKUP       (N32_IRQ_FIRST + 222)
#define N32_IRQ_LPTIM2_WKUP       (N32_IRQ_FIRST + 223)
#define N32_IRQ_LPTIM3_WKUP       (N32_IRQ_FIRST + 224)
#define N32_IRQ_LPTIM4_WKUP       (N32_IRQ_FIRST + 225)
#define N32_IRQ_DSMU_FLT0         (N32_IRQ_FIRST + 226)
#define N32_IRQ_DSMU_FLT1         (N32_IRQ_FIRST + 227)
#define N32_IRQ_DSMU_FLT2         (N32_IRQ_FIRST + 228)
#define N32_IRQ_DSMU_FLT3         (N32_IRQ_FIRST + 229)
#define N32_IRQ_FMAC              (N32_IRQ_FIRST + 230)
#define N32_IRQ_CORDIC            (N32_IRQ_FIRST + 231)
#define N32_IRQ_DMAMUX1           (N32_IRQ_FIRST + 232)
#define N32_IRQ_MMU               (N32_IRQ_FIRST + 233)

#define N32H7_IRQ_NEXTINTS        234
#define NR_IRQS                   (N32_IRQ_FIRST + N32H7_IRQ_NEXTINTS)

#endif /* __ARCH_ARM_INCLUDE_N32H7_N32H76X_IRQ_H */
