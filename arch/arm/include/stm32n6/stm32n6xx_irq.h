/****************************************************************************
 * arch/arm/include/stm32n6/stm32n6xx_irq.h
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

#ifndef __ARCH_ARM_INCLUDE_STM32N6_STM32N6XX_IRQ_H
#define __ARCH_ARM_INCLUDE_STM32N6_STM32N6XX_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* IRQ numbers.  The IRQ number corresponds vector number and hence map
 * directly to bits in the NVIC.  This does, however, waste several words of
 * memory in the IRQ to handle mapping tables.
 *
 * Processor Exceptions (vectors 0-15). These common definitions can be found
 * in arch/arm/include/stm32n6/irq.h.
 *
 * External interrupts (vectors >= 16)
 *
 * These interrupt vectors are based on the STM32N657xx CMSIS device header
 * (stm32n657xx.h IRQn_Type enumeration).  195 external interrupts total.
 */

#define STM32_IRQ_PVD_PVM         (STM32_IRQ_FIRST + 0)   /* 0:   PVD/PVM through EXTI Line detection */
#define STM32_IRQ_DTS             (STM32_IRQ_FIRST + 2)   /* 2:   Thermal sensor interrupt */
#define STM32_IRQ_RCC             (STM32_IRQ_FIRST + 3)   /* 3:   RCC non-secure global interrupt */
#define STM32_IRQ_LOCKUP          (STM32_IRQ_FIRST + 4)   /* 4:   LOCKUP interrupt */
#define STM32_IRQ_CACHE_ECC       (STM32_IRQ_FIRST + 5)   /* 5:   Cache ECC error interrupt */
#define STM32_IRQ_TCM_ECC         (STM32_IRQ_FIRST + 6)   /* 6:   TCM ECC interrupt */
#define STM32_IRQ_BKP_ECC         (STM32_IRQ_FIRST + 7)   /* 7:   Backup RAM ECC interrupt */
#define STM32_IRQ_FPU             (STM32_IRQ_FIRST + 8)   /* 8:   FPU interrupt */
#define STM32_IRQ_RTC_S           (STM32_IRQ_FIRST + 10)  /* 10:  RTC secure interrupt */
#define STM32_IRQ_TAMP            (STM32_IRQ_FIRST + 11)  /* 11:  Tamper interrupt */
#define STM32_IRQ_RIFSC_TAMPER    (STM32_IRQ_FIRST + 12)  /* 12:  RIFSC tamper interrupt */
#define STM32_IRQ_IAC             (STM32_IRQ_FIRST + 13)  /* 13:  IAC interrupt */
#define STM32_IRQ_RCC_S           (STM32_IRQ_FIRST + 14)  /* 14:  RCC secure global interrupt */
#define STM32_IRQ_RTC             (STM32_IRQ_FIRST + 16)  /* 16:  RTC non-secure interrupt */
#define STM32_IRQ_IWDG            (STM32_IRQ_FIRST + 18)  /* 18:  Independent Watchdog interrupt */
#define STM32_IRQ_WWDG            (STM32_IRQ_FIRST + 19)  /* 19:  Window Watchdog interrupt */
#define STM32_IRQ_EXTI0           (STM32_IRQ_FIRST + 20)  /* 20:  EXTI Line 0 interrupt */
#define STM32_IRQ_EXTI1           (STM32_IRQ_FIRST + 21)  /* 21:  EXTI Line 1 interrupt */
#define STM32_IRQ_EXTI2           (STM32_IRQ_FIRST + 22)  /* 22:  EXTI Line 2 interrupt */
#define STM32_IRQ_EXTI3           (STM32_IRQ_FIRST + 23)  /* 23:  EXTI Line 3 interrupt */
#define STM32_IRQ_EXTI4           (STM32_IRQ_FIRST + 24)  /* 24:  EXTI Line 4 interrupt */
#define STM32_IRQ_EXTI5           (STM32_IRQ_FIRST + 25)  /* 25:  EXTI Line 5 interrupt */
#define STM32_IRQ_EXTI6           (STM32_IRQ_FIRST + 26)  /* 26:  EXTI Line 6 interrupt */
#define STM32_IRQ_EXTI7           (STM32_IRQ_FIRST + 27)  /* 27:  EXTI Line 7 interrupt */
#define STM32_IRQ_EXTI8           (STM32_IRQ_FIRST + 28)  /* 28:  EXTI Line 8 interrupt */
#define STM32_IRQ_EXTI9           (STM32_IRQ_FIRST + 29)  /* 29:  EXTI Line 9 interrupt */
#define STM32_IRQ_EXTI10          (STM32_IRQ_FIRST + 30)  /* 30:  EXTI Line 10 interrupt */
#define STM32_IRQ_EXTI11          (STM32_IRQ_FIRST + 31)  /* 31:  EXTI Line 11 interrupt */
#define STM32_IRQ_EXTI12          (STM32_IRQ_FIRST + 32)  /* 32:  EXTI Line 12 interrupt */
#define STM32_IRQ_EXTI13          (STM32_IRQ_FIRST + 33)  /* 33:  EXTI Line 13 interrupt */
#define STM32_IRQ_EXTI14          (STM32_IRQ_FIRST + 34)  /* 34:  EXTI Line 14 interrupt */
#define STM32_IRQ_EXTI15          (STM32_IRQ_FIRST + 35)  /* 35:  EXTI Line 15 interrupt */
#define STM32_IRQ_SAES            (STM32_IRQ_FIRST + 36)  /* 36:  SAES interrupt */
#define STM32_IRQ_CRYP            (STM32_IRQ_FIRST + 37)  /* 37:  CRYP interrupt */
#define STM32_IRQ_PKA             (STM32_IRQ_FIRST + 38)  /* 38:  PKA interrupt */
#define STM32_IRQ_HASH            (STM32_IRQ_FIRST + 39)  /* 39:  HASH interrupt */
#define STM32_IRQ_RNG             (STM32_IRQ_FIRST + 40)  /* 40:  RNG global interrupt */
#define STM32_IRQ_MCE1            (STM32_IRQ_FIRST + 42)  /* 42:  MCE1 global interrupt */
#define STM32_IRQ_MCE2            (STM32_IRQ_FIRST + 43)  /* 43:  MCE2 global interrupt */
#define STM32_IRQ_MCE3            (STM32_IRQ_FIRST + 44)  /* 44:  MCE3 global interrupt */
#define STM32_IRQ_MCE4            (STM32_IRQ_FIRST + 45)  /* 45:  MCE4 global interrupt */
#define STM32_IRQ_ADC1_2          (STM32_IRQ_FIRST + 46)  /* 46:  ADC1 & ADC2 interrupt */
#define STM32_IRQ_CSI             (STM32_IRQ_FIRST + 47)  /* 47:  CSI global interrupt */
#define STM32_IRQ_DCMIPP          (STM32_IRQ_FIRST + 48)  /* 48:  DCMIPP global interrupt */
#define STM32_IRQ_PAHB_ERR        (STM32_IRQ_FIRST + 52)  /* 52:  PAHB error interrupt */
#define STM32_IRQ_LTDC_LO         (STM32_IRQ_FIRST + 58)  /* 58:  LTDC low-layer global interrupt */
#define STM32_IRQ_LTDC_LO_ERR     (STM32_IRQ_FIRST + 59)  /* 59:  LTDC low-layer error interrupt */
#define STM32_IRQ_DMA2D           (STM32_IRQ_FIRST + 60)  /* 60:  DMA2D global interrupt */
#define STM32_IRQ_JPEG            (STM32_IRQ_FIRST + 61)  /* 61:  JPEG global interrupt */
#define STM32_IRQ_VENC            (STM32_IRQ_FIRST + 62)  /* 62:  VENC global interrupt */
#define STM32_IRQ_GFXMMU          (STM32_IRQ_FIRST + 63)  /* 63:  GFXMMU global interrupt */
#define STM32_IRQ_GFXTIM          (STM32_IRQ_FIRST + 64)  /* 64:  GFXTIM global interrupt */
#define STM32_IRQ_GPU2D           (STM32_IRQ_FIRST + 65)  /* 65:  GPU2D interrupt */
#define STM32_IRQ_GPU2D_ER        (STM32_IRQ_FIRST + 66)  /* 66:  GPU2D error interrupt */
#define STM32_IRQ_ICACHE          (STM32_IRQ_FIRST + 67)  /* 67:  ICACHE interrupt */
#define STM32_IRQ_HPDMA1_CH0      (STM32_IRQ_FIRST + 68)  /* 68:  HPDMA1 Channel 0 interrupt */
#define STM32_IRQ_HPDMA1_CH1      (STM32_IRQ_FIRST + 69)  /* 69:  HPDMA1 Channel 1 interrupt */
#define STM32_IRQ_HPDMA1_CH2      (STM32_IRQ_FIRST + 70)  /* 70:  HPDMA1 Channel 2 interrupt */
#define STM32_IRQ_HPDMA1_CH3      (STM32_IRQ_FIRST + 71)  /* 71:  HPDMA1 Channel 3 interrupt */
#define STM32_IRQ_HPDMA1_CH4      (STM32_IRQ_FIRST + 72)  /* 72:  HPDMA1 Channel 4 interrupt */
#define STM32_IRQ_HPDMA1_CH5      (STM32_IRQ_FIRST + 73)  /* 73:  HPDMA1 Channel 5 interrupt */
#define STM32_IRQ_HPDMA1_CH6      (STM32_IRQ_FIRST + 74)  /* 74:  HPDMA1 Channel 6 interrupt */
#define STM32_IRQ_HPDMA1_CH7      (STM32_IRQ_FIRST + 75)  /* 75:  HPDMA1 Channel 7 interrupt */
#define STM32_IRQ_HPDMA1_CH8      (STM32_IRQ_FIRST + 76)  /* 76:  HPDMA1 Channel 8 interrupt */
#define STM32_IRQ_HPDMA1_CH9      (STM32_IRQ_FIRST + 77)  /* 77:  HPDMA1 Channel 9 interrupt */
#define STM32_IRQ_HPDMA1_CH10     (STM32_IRQ_FIRST + 78)  /* 78:  HPDMA1 Channel 10 interrupt */
#define STM32_IRQ_HPDMA1_CH11     (STM32_IRQ_FIRST + 79)  /* 79:  HPDMA1 Channel 11 interrupt */
#define STM32_IRQ_HPDMA1_CH12     (STM32_IRQ_FIRST + 80)  /* 80:  HPDMA1 Channel 12 interrupt */
#define STM32_IRQ_HPDMA1_CH13     (STM32_IRQ_FIRST + 81)  /* 81:  HPDMA1 Channel 13 interrupt */
#define STM32_IRQ_HPDMA1_CH14     (STM32_IRQ_FIRST + 82)  /* 82:  HPDMA1 Channel 14 interrupt */
#define STM32_IRQ_HPDMA1_CH15     (STM32_IRQ_FIRST + 83)  /* 83:  HPDMA1 Channel 15 interrupt */
#define STM32_IRQ_GPDMA1_CH0      (STM32_IRQ_FIRST + 84)  /* 84:  GPDMA1 Channel 0 interrupt */
#define STM32_IRQ_GPDMA1_CH1      (STM32_IRQ_FIRST + 85)  /* 85:  GPDMA1 Channel 1 interrupt */
#define STM32_IRQ_GPDMA1_CH2      (STM32_IRQ_FIRST + 86)  /* 86:  GPDMA1 Channel 2 interrupt */
#define STM32_IRQ_GPDMA1_CH3      (STM32_IRQ_FIRST + 87)  /* 87:  GPDMA1 Channel 3 interrupt */
#define STM32_IRQ_GPDMA1_CH4      (STM32_IRQ_FIRST + 88)  /* 88:  GPDMA1 Channel 4 interrupt */
#define STM32_IRQ_GPDMA1_CH5      (STM32_IRQ_FIRST + 89)  /* 89:  GPDMA1 Channel 5 interrupt */
#define STM32_IRQ_GPDMA1_CH6      (STM32_IRQ_FIRST + 90)  /* 90:  GPDMA1 Channel 6 interrupt */
#define STM32_IRQ_GPDMA1_CH7      (STM32_IRQ_FIRST + 91)  /* 91:  GPDMA1 Channel 7 interrupt */
#define STM32_IRQ_GPDMA1_CH8      (STM32_IRQ_FIRST + 92)  /* 92:  GPDMA1 Channel 8 interrupt */
#define STM32_IRQ_GPDMA1_CH9      (STM32_IRQ_FIRST + 93)  /* 93:  GPDMA1 Channel 9 interrupt */
#define STM32_IRQ_GPDMA1_CH10     (STM32_IRQ_FIRST + 94)  /* 94:  GPDMA1 Channel 10 interrupt */
#define STM32_IRQ_GPDMA1_CH11     (STM32_IRQ_FIRST + 95)  /* 95:  GPDMA1 Channel 11 interrupt */
#define STM32_IRQ_GPDMA1_CH12     (STM32_IRQ_FIRST + 96)  /* 96:  GPDMA1 Channel 12 interrupt */
#define STM32_IRQ_GPDMA1_CH13     (STM32_IRQ_FIRST + 97)  /* 97:  GPDMA1 Channel 13 interrupt */
#define STM32_IRQ_GPDMA1_CH14     (STM32_IRQ_FIRST + 98)  /* 98:  GPDMA1 Channel 14 interrupt */
#define STM32_IRQ_GPDMA1_CH15     (STM32_IRQ_FIRST + 99)  /* 99:  GPDMA1 Channel 15 interrupt */
#define STM32_IRQ_I2C1_EV         (STM32_IRQ_FIRST + 100) /* 100: I2C1 event interrupt */
#define STM32_IRQ_I2C1_ER         (STM32_IRQ_FIRST + 101) /* 101: I2C1 error interrupt */
#define STM32_IRQ_I2C2_EV         (STM32_IRQ_FIRST + 102) /* 102: I2C2 event interrupt */
#define STM32_IRQ_I2C2_ER         (STM32_IRQ_FIRST + 103) /* 103: I2C2 error interrupt */
#define STM32_IRQ_I2C3_EV         (STM32_IRQ_FIRST + 104) /* 104: I2C3 event interrupt */
#define STM32_IRQ_I2C3_ER         (STM32_IRQ_FIRST + 105) /* 105: I2C3 error interrupt */
#define STM32_IRQ_I2C4_EV         (STM32_IRQ_FIRST + 106) /* 106: I2C4 event interrupt */
#define STM32_IRQ_I2C4_ER         (STM32_IRQ_FIRST + 107) /* 107: I2C4 error interrupt */
#define STM32_IRQ_I3C1_EV         (STM32_IRQ_FIRST + 108) /* 108: I3C1 event interrupt */
#define STM32_IRQ_I3C1_ER         (STM32_IRQ_FIRST + 109) /* 109: I3C1 error interrupt */
#define STM32_IRQ_I3C2_EV         (STM32_IRQ_FIRST + 110) /* 110: I3C2 event interrupt */
#define STM32_IRQ_I3C2_ER         (STM32_IRQ_FIRST + 111) /* 111: I3C2 error interrupt */
#define STM32_IRQ_TIM1_BRK        (STM32_IRQ_FIRST + 112) /* 112: TIM1 Break interrupt */
#define STM32_IRQ_TIM1_UP         (STM32_IRQ_FIRST + 113) /* 113: TIM1 Update interrupt */
#define STM32_IRQ_TIM1_TRG_COM    (STM32_IRQ_FIRST + 114) /* 114: TIM1 Trigger and Commutation interrupt */
#define STM32_IRQ_TIM1_CC         (STM32_IRQ_FIRST + 115) /* 115: TIM1 Capture Compare interrupt */
#define STM32_IRQ_TIM2            (STM32_IRQ_FIRST + 116) /* 116: TIM2 global interrupt */
#define STM32_IRQ_TIM3            (STM32_IRQ_FIRST + 117) /* 117: TIM3 global interrupt */
#define STM32_IRQ_TIM4            (STM32_IRQ_FIRST + 118) /* 118: TIM4 global interrupt */
#define STM32_IRQ_TIM5            (STM32_IRQ_FIRST + 119) /* 119: TIM5 global interrupt */
#define STM32_IRQ_TIM6            (STM32_IRQ_FIRST + 120) /* 120: TIM6 global interrupt */
#define STM32_IRQ_TIM7            (STM32_IRQ_FIRST + 121) /* 121: TIM7 global interrupt */
#define STM32_IRQ_TIM8_BRK        (STM32_IRQ_FIRST + 122) /* 122: TIM8 Break interrupt */
#define STM32_IRQ_TIM8_UP         (STM32_IRQ_FIRST + 123) /* 123: TIM8 Update interrupt */
#define STM32_IRQ_TIM8_TRG_COM    (STM32_IRQ_FIRST + 124) /* 124: TIM8 Trigger and Commutation interrupt */
#define STM32_IRQ_TIM8_CC         (STM32_IRQ_FIRST + 125) /* 125: TIM8 Capture Compare interrupt */
#define STM32_IRQ_TIM9            (STM32_IRQ_FIRST + 126) /* 126: TIM9 global interrupt */
#define STM32_IRQ_TIM10           (STM32_IRQ_FIRST + 127) /* 127: TIM10 global interrupt */
#define STM32_IRQ_TIM11           (STM32_IRQ_FIRST + 128) /* 128: TIM11 global interrupt */
#define STM32_IRQ_TIM12           (STM32_IRQ_FIRST + 129) /* 129: TIM12 global interrupt */
#define STM32_IRQ_TIM13           (STM32_IRQ_FIRST + 130) /* 130: TIM13 global interrupt */
#define STM32_IRQ_TIM14           (STM32_IRQ_FIRST + 131) /* 131: TIM14 global interrupt */
#define STM32_IRQ_TIM15           (STM32_IRQ_FIRST + 132) /* 132: TIM15 global interrupt */
#define STM32_IRQ_TIM16           (STM32_IRQ_FIRST + 133) /* 133: TIM16 global interrupt */
#define STM32_IRQ_TIM17           (STM32_IRQ_FIRST + 134) /* 134: TIM17 global interrupt */
#define STM32_IRQ_TIM18           (STM32_IRQ_FIRST + 135) /* 135: TIM18 global interrupt */
#define STM32_IRQ_LPTIM1          (STM32_IRQ_FIRST + 136) /* 136: LPTIM1 global interrupt */
#define STM32_IRQ_LPTIM2          (STM32_IRQ_FIRST + 137) /* 137: LPTIM2 global interrupt */
#define STM32_IRQ_LPTIM3          (STM32_IRQ_FIRST + 138) /* 138: LPTIM3 global interrupt */
#define STM32_IRQ_LPTIM4          (STM32_IRQ_FIRST + 139) /* 139: LPTIM4 global interrupt */
#define STM32_IRQ_LPTIM5          (STM32_IRQ_FIRST + 140) /* 140: LPTIM5 global interrupt */
#define STM32_IRQ_ADF1_FLT0       (STM32_IRQ_FIRST + 141) /* 141: ADF1 Filter 0 interrupt */
#define STM32_IRQ_MDF1_FLT0       (STM32_IRQ_FIRST + 142) /* 142: MDF1 Filter 0 interrupt */
#define STM32_IRQ_MDF1_FLT1       (STM32_IRQ_FIRST + 143) /* 143: MDF1 Filter 1 interrupt */
#define STM32_IRQ_MDF1_FLT2       (STM32_IRQ_FIRST + 144) /* 144: MDF1 Filter 2 interrupt */
#define STM32_IRQ_MDF1_FLT3       (STM32_IRQ_FIRST + 145) /* 145: MDF1 Filter 3 interrupt */
#define STM32_IRQ_MDF1_FLT4       (STM32_IRQ_FIRST + 146) /* 146: MDF1 Filter 4 interrupt */
#define STM32_IRQ_MDF1_FLT5       (STM32_IRQ_FIRST + 147) /* 147: MDF1 Filter 5 interrupt */
#define STM32_IRQ_SAI1_A          (STM32_IRQ_FIRST + 148) /* 148: SAI1 block A interrupt */
#define STM32_IRQ_SAI1_B          (STM32_IRQ_FIRST + 149) /* 149: SAI1 block B interrupt */
#define STM32_IRQ_SAI2_A          (STM32_IRQ_FIRST + 150) /* 150: SAI2 block A interrupt */
#define STM32_IRQ_SAI2_B          (STM32_IRQ_FIRST + 151) /* 151: SAI2 block B interrupt */
#define STM32_IRQ_SPDIFRX1        (STM32_IRQ_FIRST + 152) /* 152: SPDIFRX1 interrupt */
#define STM32_IRQ_SPI1            (STM32_IRQ_FIRST + 153) /* 153: SPI1 global interrupt */
#define STM32_IRQ_SPI2            (STM32_IRQ_FIRST + 154) /* 154: SPI2 global interrupt */
#define STM32_IRQ_SPI3            (STM32_IRQ_FIRST + 155) /* 155: SPI3 global interrupt */
#define STM32_IRQ_SPI4            (STM32_IRQ_FIRST + 156) /* 156: SPI4 global interrupt */
#define STM32_IRQ_SPI5            (STM32_IRQ_FIRST + 157) /* 157: SPI5 global interrupt */
#define STM32_IRQ_SPI6            (STM32_IRQ_FIRST + 158) /* 158: SPI6 global interrupt */
#define STM32_IRQ_USART1          (STM32_IRQ_FIRST + 159) /* 159: USART1 global interrupt */
#define STM32_IRQ_USART2          (STM32_IRQ_FIRST + 160) /* 160: USART2 global interrupt */
#define STM32_IRQ_USART3          (STM32_IRQ_FIRST + 161) /* 161: USART3 global interrupt */
#define STM32_IRQ_UART4           (STM32_IRQ_FIRST + 162) /* 162: UART4 global interrupt */
#define STM32_IRQ_UART5           (STM32_IRQ_FIRST + 163) /* 163: UART5 global interrupt */
#define STM32_IRQ_USART6          (STM32_IRQ_FIRST + 164) /* 164: USART6 global interrupt */
#define STM32_IRQ_UART7           (STM32_IRQ_FIRST + 165) /* 165: UART7 global interrupt */
#define STM32_IRQ_UART8           (STM32_IRQ_FIRST + 166) /* 166: UART8 global interrupt */
#define STM32_IRQ_UART9           (STM32_IRQ_FIRST + 167) /* 167: UART9 global interrupt */
#define STM32_IRQ_USART10         (STM32_IRQ_FIRST + 168) /* 168: USART10 global interrupt */
#define STM32_IRQ_LPUART1         (STM32_IRQ_FIRST + 169) /* 169: LPUART1 global interrupt */
#define STM32_IRQ_XSPI1           (STM32_IRQ_FIRST + 170) /* 170: XSPI1 global interrupt */
#define STM32_IRQ_XSPI2           (STM32_IRQ_FIRST + 171) /* 171: XSPI2 global interrupt */
#define STM32_IRQ_XSPI3           (STM32_IRQ_FIRST + 172) /* 172: XSPI3 global interrupt */
#define STM32_IRQ_FMC             (STM32_IRQ_FIRST + 173) /* 173: FMC global interrupt */
#define STM32_IRQ_SDMMC1          (STM32_IRQ_FIRST + 174) /* 174: SDMMC1 global interrupt */
#define STM32_IRQ_SDMMC2          (STM32_IRQ_FIRST + 175) /* 175: SDMMC2 global interrupt */
#define STM32_IRQ_UCPD1           (STM32_IRQ_FIRST + 176) /* 176: UCPD1 global interrupt */
#define STM32_IRQ_USB1_OTG_HS     (STM32_IRQ_FIRST + 177) /* 177: USB1 OTG HS interrupt */
#define STM32_IRQ_USB2_OTG_HS     (STM32_IRQ_FIRST + 178) /* 178: USB2 OTG HS interrupt */
#define STM32_IRQ_ETH1            (STM32_IRQ_FIRST + 179) /* 179: ETH1 global interrupt */
#define STM32_IRQ_FDCAN1_IT0      (STM32_IRQ_FIRST + 180) /* 180: FDCAN1 interrupt 0 */
#define STM32_IRQ_FDCAN1_IT1      (STM32_IRQ_FIRST + 181) /* 181: FDCAN1 interrupt 1 */
#define STM32_IRQ_FDCAN2_IT0      (STM32_IRQ_FIRST + 182) /* 182: FDCAN2 interrupt 0 */
#define STM32_IRQ_FDCAN2_IT1      (STM32_IRQ_FIRST + 183) /* 183: FDCAN2 interrupt 1 */
#define STM32_IRQ_FDCAN3_IT0      (STM32_IRQ_FIRST + 184) /* 184: FDCAN3 interrupt 0 */
#define STM32_IRQ_FDCAN3_IT1      (STM32_IRQ_FIRST + 185) /* 185: FDCAN3 interrupt 1 */
#define STM32_IRQ_FDCAN_CU        (STM32_IRQ_FIRST + 186) /* 186: FDCAN Clock Unit interrupt */
#define STM32_IRQ_MDIOS           (STM32_IRQ_FIRST + 187) /* 187: MDIOS global interrupt */
#define STM32_IRQ_DCMI_PSSI       (STM32_IRQ_FIRST + 188) /* 188: DCMI/PSSI global interrupt */
#define STM32_IRQ_WAKEUP_PIN      (STM32_IRQ_FIRST + 189) /* 189: Wake-up pins interrupt */
#define STM32_IRQ_CTI_INT0        (STM32_IRQ_FIRST + 190) /* 190: CTI INT0 interrupt */
#define STM32_IRQ_CTI_INT1        (STM32_IRQ_FIRST + 191) /* 191: CTI INT1 interrupt */
#define STM32_IRQ_LTDC_UP         (STM32_IRQ_FIRST + 193) /* 193: LTDC up-layer global interrupt */
#define STM32_IRQ_LTDC_UP_ERR     (STM32_IRQ_FIRST + 194) /* 194: LTDC up-layer error interrupt */

/* Total number of external interrupts (0-194) */

#define STM32_IRQ_NEXTINTS  195

/* Total number of IRQ numbers */

#define NR_IRQS             (STM32_IRQ_FIRST + STM32_IRQ_NEXTINTS)

#endif /* __ARCH_ARM_INCLUDE_STM32N6_STM32N6XX_IRQ_H */
