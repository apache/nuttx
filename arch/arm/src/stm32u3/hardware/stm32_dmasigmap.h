/****************************************************************************
 * arch/arm/src/stm32u3/hardware/stm32_dmasigmap.h
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

#ifndef __ARCH_ARM_SRC_STM32U3_HARDWARE_STM32_DMASIGMAP_H
#define __ARCH_ARM_SRC_STM32U3_HARDWARE_STM32_DMASIGMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPDMA1 request signals */

#define GPDMA_REQ_ADC1         (0)
#define GPDMA_REQ_ADC2         (1)
#define GPDMA_REQ_DAC1_CH1     (2)
#define GPDMA_REQ_DAC1_CH2     (3)
#define GPDMA_REQ_TIM6_UP      (4)
#define GPDMA_REQ_TIM7_UP      (5)
#define GPDMA_REQ_SPI1_RX      (6)
#define GPDMA_REQ_SPI1_TX      (7)
#define GPDMA_REQ_SPI2_RX      (8)
#define GPDMA_REQ_SPI2_TX      (9)
#define GPDMA_REQ_SPI3_RX      (10)
#define GPDMA_REQ_SPI3_TX      (11)
#define GPDMA_REQ_I2C1_RX      (12)
#define GPDMA_REQ_I2C1_TX      (13)
#define GPDMA_REQ_I2C1_EVC     (14)
#define GPDMA_REQ_I2C2_RX      (15)
#define GPDMA_REQ_I2C2_TX      (16)
#define GPDMA_REQ_I2C2_EVC     (17)
#define GPDMA_REQ_I2C3_RX      (18)
#define GPDMA_REQ_I2C3_TX      (19)
#define GPDMA_REQ_I2C3_EVC     (20)
#define GPDMA_REQ_I2C4_RX      (21)
#define GPDMA_REQ_I2C4_TX      (22)
#define GPDMA_REQ_I2C4_EVC     (23)
#define GPDMA_REQ_USART1_RX    (24)
#define GPDMA_REQ_USART1_TX    (25)
#define GPDMA_REQ_USART2_RX    (26)
#define GPDMA_REQ_USART2_TX    (27)
#define GPDMA_REQ_USART3_RX    (28)
#define GPDMA_REQ_USART3_TX    (29)
#define GPDMA_REQ_UART4_RX     (30)
#define GPDMA_REQ_UART4_TX     (31)
#define GPDMA_REQ_UART5_RX     (32)
#define GPDMA_REQ_UART5_TX     (33)
#define GPDMA_REQ_LPUART1_RX   (34)
#define GPDMA_REQ_LPUART1_TX   (35)
#define GPDMA_REQ_SAI1_A       (36)
#define GPDMA_REQ_SAI1_B       (37)
#define GPDMA_REQ_OCTOSPI1     (40)
#define GPDMA_REQ_TIM1_CH1     (42)
#define GPDMA_REQ_TIM1_CH2     (43)
#define GPDMA_REQ_TIM1_CH3     (44)
#define GPDMA_REQ_TIM1_CH4     (45)
#define GPDMA_REQ_TIM1_UP      (46)
#define GPDMA_REQ_TIM1_TRIG    (47)
#define GPDMA_REQ_TIM1_COM     (48)
#define GPDMA_REQ_I3C1_RX      (49)
#define GPDMA_REQ_I3C1_TX      (50)
#define GPDMA_REQ_I3C1_TC      (51)
#define GPDMA_REQ_I3C1_RS      (52)
#define GPDMA_REQ_GFXPAND_IN   (53)
#define GPDMA_REQ_GFXPAND_OUT  (54)
#define GPDMA_REQ_TIM2_CH1     (56)
#define GPDMA_REQ_TIM2_CH2     (57)
#define GPDMA_REQ_TIM2_CH3     (58)
#define GPDMA_REQ_TIM2_CH4     (59)
#define GPDMA_REQ_TIM2_UP      (60)
#define GPDMA_REQ_TIM3_CH1     (61)
#define GPDMA_REQ_TIM3_CH2     (62)
#define GPDMA_REQ_TIM3_CH3     (63)
#define GPDMA_REQ_TIM3_CH4     (64)
#define GPDMA_REQ_TIM3_UP      (65)
#define GPDMA_REQ_TIM3_TRIG    (66)
#define GPDMA_REQ_TIM4_CH1     (67)
#define GPDMA_REQ_TIM4_CH2     (68)
#define GPDMA_REQ_TIM4_CH3     (69)
#define GPDMA_REQ_TIM4_CH4     (70)
#define GPDMA_REQ_TIM4_UP      (71)
#define GPDMA_REQ_I3C2_RX      (72)
#define GPDMA_REQ_I3C2_TX      (73)
#define GPDMA_REQ_I3C2_TC      (74)
#define GPDMA_REQ_I3C2_RS      (75)
#define GPDMA_REQ_SPI4_RX      (76)
#define GPDMA_REQ_SPI4_TX      (77)
#define GPDMA_REQ_TIM15_CH1    (78)
#define GPDMA_REQ_TIM15_UP     (79)
#define GPDMA_REQ_TIM15_TRIG   (80)
#define GPDMA_REQ_TIM15_COM    (81)
#define GPDMA_REQ_TIM16_CH1    (82)
#define GPDMA_REQ_TIM16_UP     (83)
#define GPDMA_REQ_TIM17_CH1    (84)
#define GPDMA_REQ_TIM17_UP     (85)
#define GPDMA_REQ_AES_IN       (87)
#define GPDMA_REQ_AES_OUT      (88)
#define GPDMA_REQ_HASH_IN      (89)
#define GPDMA_REQ_TIM8_CH1     (91)
#define GPDMA_REQ_TIM8_CH2     (92)
#define GPDMA_REQ_TIM8_CH3     (93)
#define GPDMA_REQ_TIM8_CH4     (94)
#define GPDMA_REQ_TIM8_UP      (95)
#define GPDMA_REQ_TIM8_TRIG    (96)
#define GPDMA_REQ_TIM8_COM     (97)
#define GPDMA_REQ_ADF1_FLT0    (98)
#define GPDMA_REQ_SAES_IN      (103)
#define GPDMA_REQ_SAES_OUT     (104)
#define GPDMA_REQ_LPTIM1_IC1   (105)
#define GPDMA_REQ_LPTIM1_IC2   (106)
#define GPDMA_REQ_LPTIM1_UE    (107)
#define GPDMA_REQ_LPTIM2_IC1   (108)
#define GPDMA_REQ_LPTIM2_IC2   (109)
#define GPDMA_REQ_LPTIM2_UE    (110)
#define GPDMA_REQ_LPTIM3_IC1   (111)
#define GPDMA_REQ_LPTIM3_IC2   (112)
#define GPDMA_REQ_LPTIM3_UE    (113)

/* USART RX DMA request numbers */

#define STM32_LPUART1_RXDMA_REQ  GPDMA_REQ_LPUART1_RX
#define STM32_USART1_RXDMA_REQ   GPDMA_REQ_USART1_RX
#define STM32_USART2_RXDMA_REQ   GPDMA_REQ_USART2_RX
#define STM32_USART3_RXDMA_REQ   GPDMA_REQ_USART3_RX
#define STM32_UART4_RXDMA_REQ    GPDMA_REQ_UART4_RX
#define STM32_UART5_RXDMA_REQ    GPDMA_REQ_UART5_RX

#endif /* __ARCH_ARM_SRC_STM32U3_HARDWARE_STM32_DMASIGMAP_H */
