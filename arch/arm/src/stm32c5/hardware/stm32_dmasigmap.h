/****************************************************************************
 * arch/arm/src/stm32c5/hardware/stm32_dmasigmap.h
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

#ifndef __ARCH_ARM_SRC_STM32C5_HARDWARE_STM32_DMASIGMAP_H
#define __ARCH_ARM_SRC_STM32C5_HARDWARE_STM32_DMASIGMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LPDMA request signals, common to the LPDMA1 and LPDMA2 instances */

#define LPDMA_REQ_ADC1        (0)
#define LPDMA_REQ_ADC2        (1)
#define LPDMA_REQ_TIM6_UPD    (2)
#define LPDMA_REQ_TIM7_UPD    (3)
#define LPDMA_REQ_SPI1_RX     (4)
#define LPDMA_REQ_SPI1_TX     (5)
#define LPDMA_REQ_SPI2_RX     (6)
#define LPDMA_REQ_SPI2_TX     (7)
#define LPDMA_REQ_SPI3_RX     (8)
#define LPDMA_REQ_SPI3_TX     (9)
#define LPDMA_REQ_I2C1_RX     (10)
#define LPDMA_REQ_I2C1_TX     (11)
#define LPDMA_REQ_USART1_RX   (12)
#define LPDMA_REQ_USART1_TX   (13)
#define LPDMA_REQ_USART2_RX   (14)
#define LPDMA_REQ_USART2_TX   (15)
#define LPDMA_REQ_USART3_RX   (16)
#define LPDMA_REQ_USART3_TX   (17)
#define LPDMA_REQ_UART4_RX    (18)
#define LPDMA_REQ_UART4_TX    (19)
#define LPDMA_REQ_UART5_RX    (20)
#define LPDMA_REQ_UART5_TX    (21)
#define LPDMA_REQ_LPUART1_RX  (22)
#define LPDMA_REQ_LPUART1_TX  (23)
#define LPDMA_REQ_TIM1_CC1    (24)
#define LPDMA_REQ_TIM1_CC2    (25)
#define LPDMA_REQ_TIM1_CC3    (26)
#define LPDMA_REQ_TIM1_CC4    (27)
#define LPDMA_REQ_TIM1_UPD    (28)
#define LPDMA_REQ_TIM1_TRGI   (29)
#define LPDMA_REQ_TIM1_COM    (30)
#define LPDMA_REQ_TIM2_CC1    (31)
#define LPDMA_REQ_TIM2_CC2    (32)
#define LPDMA_REQ_TIM2_CC3    (33)
#define LPDMA_REQ_TIM2_CC4    (34)
#define LPDMA_REQ_TIM2_UPD    (35)
#define LPDMA_REQ_TIM2_TRGI   (36)
#define LPDMA_REQ_TIM5_CC1    (37)
#define LPDMA_REQ_TIM5_CC2    (38)
#define LPDMA_REQ_TIM5_CC3    (39)
#define LPDMA_REQ_TIM5_CC4    (40)
#define LPDMA_REQ_TIM5_UPD    (41)
#define LPDMA_REQ_TIM5_TRGI   (42)
#define LPDMA_REQ_TIM15_CC1   (43)
#define LPDMA_REQ_TIM15_CC2   (44)
#define LPDMA_REQ_TIM15_UPD   (45)
#define LPDMA_REQ_TIM15_TRGI  (46)
#define LPDMA_REQ_TIM15_COM   (47)
#define LPDMA_REQ_TIM16_CC1   (48)
#define LPDMA_REQ_TIM16_UPD   (49)
#define LPDMA_REQ_TIM17_CC1   (50)
#define LPDMA_REQ_TIM17_UPD   (51)
#define LPDMA_REQ_LPTIM1_IC1  (52)
#define LPDMA_REQ_LPTIM1_IC2  (53)
#define LPDMA_REQ_LPTIM1_UE   (54)
#define LPDMA_REQ_CORDIC_RD   (55)
#define LPDMA_REQ_CORDIC_WR   (56)
#define LPDMA_REQ_I3C1_RX     (57)
#define LPDMA_REQ_I3C1_TX     (58)
#define LPDMA_REQ_I3C1_TC     (59)
#define LPDMA_REQ_I3C1_RS     (60)
#define LPDMA_REQ_AES_OUT     (61)
#define LPDMA_REQ_AES_IN      (62)
#define LPDMA_REQ_HASH_IN     (63)
#define LPDMA_REQ_I2C2_RX     (64)
#define LPDMA_REQ_I2C2_TX     (65)
#define LPDMA_REQ_TIM8_CC1    (66)
#define LPDMA_REQ_TIM8_CC2    (67)
#define LPDMA_REQ_TIM8_CC3    (68)
#define LPDMA_REQ_TIM8_CC4    (69)
#define LPDMA_REQ_TIM8_UPD    (70)
#define LPDMA_REQ_TIM8_TRGI   (71)
#define LPDMA_REQ_TIM8_COM    (72)
#define LPDMA_REQ_DAC1_CH1    (73)
#define LPDMA_REQ_DAC1_CH2    (74)
#define LPDMA_REQ_USART6_RX   (75)
#define LPDMA_REQ_USART6_TX   (76)
#define LPDMA_REQ_UART7_TX    (77)
#define LPDMA_REQ_UART7_RX    (78)
#define LPDMA_REQ_ADC3        (79)
#define LPDMA_REQ_TIM3_CC1    (80)
#define LPDMA_REQ_TIM3_CC2    (81)
#define LPDMA_REQ_TIM3_CC3    (82)
#define LPDMA_REQ_TIM3_CC4    (83)
#define LPDMA_REQ_TIM3_UPD    (84)
#define LPDMA_REQ_TIM3_TRGI   (85)
#define LPDMA_REQ_TIM4_CC1    (86)
#define LPDMA_REQ_TIM4_CC2    (87)
#define LPDMA_REQ_TIM4_CC3    (88)
#define LPDMA_REQ_TIM4_CC4    (89)
#define LPDMA_REQ_TIM4_UPD    (90)
#define LPDMA_REQ_TIM4_TRGI   (91)
#define LPDMA_REQ_SAES_OUT    (92)
#define LPDMA_REQ_SAES_IN     (93)
#define LPDMA_REQ_XSPI1       (94)

/* USART RX DMA request numbers */

#define STM32_LPUART1_RXDMA_REQ  LPDMA_REQ_LPUART1_RX
#define STM32_USART1_RXDMA_REQ   LPDMA_REQ_USART1_RX
#define STM32_USART2_RXDMA_REQ   LPDMA_REQ_USART2_RX
#define STM32_USART3_RXDMA_REQ   LPDMA_REQ_USART3_RX
#define STM32_UART4_RXDMA_REQ    LPDMA_REQ_UART4_RX
#define STM32_UART5_RXDMA_REQ    LPDMA_REQ_UART5_RX

#endif /* __ARCH_ARM_SRC_STM32C5_HARDWARE_STM32_DMASIGMAP_H */
