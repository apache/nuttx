/****************************************************************************
 * arch/arm/src/stm32n6/hardware/stm32n6xxx_dmasigmap.h
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

#ifndef __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_DMASIGMAP_H
#define __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_DMASIGMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DMA hardware request lines (RM0486, GPDMA1 and HPDMA1 request mapping).
 * GPDMA1 and HPDMA1 share the same request numbering.
 */

#define GPDMA_REQ_JPEG_RX           (0)
#define GPDMA_REQ_JPEG_TX           (1)
#define GPDMA_REQ_XSPI1             (2)
#define GPDMA_REQ_XSPI2             (3)
#define GPDMA_REQ_XSPI3             (4)
#define GPDMA_REQ_FMC2_TXRX         (5)
#define GPDMA_REQ_FMC2_BCH          (6)
#define GPDMA_REQ_ADC1              (7)
#define GPDMA_REQ_ADC2              (8)
#define GPDMA_REQ_CRYP_IN           (9)
#define GPDMA_REQ_CRYP_OUT          (10)
#define GPDMA_REQ_SAES_OUT          (11)
#define GPDMA_REQ_SAES_IN           (12)
#define GPDMA_REQ_HASH_IN           (13)
#define GPDMA_REQ_TIM1_CH1          (14)
#define GPDMA_REQ_TIM1_CH2          (15)
#define GPDMA_REQ_TIM1_CH3          (16)
#define GPDMA_REQ_TIM1_CH4          (17)
#define GPDMA_REQ_TIM1_UP           (18)
#define GPDMA_REQ_TIM1_TRIG         (19)
#define GPDMA_REQ_TIM1_COM          (20)
#define GPDMA_REQ_TIM2_CH1          (21)
#define GPDMA_REQ_TIM2_CH2          (22)
#define GPDMA_REQ_TIM2_CH3          (23)
#define GPDMA_REQ_TIM2_CH4          (24)
#define GPDMA_REQ_TIM2_UP           (25)
#define GPDMA_REQ_TIM2_TRIG         (26)
#define GPDMA_REQ_TIM3_CH1          (27)
#define GPDMA_REQ_TIM3_CH2          (28)
#define GPDMA_REQ_TIM3_CH3          (29)
#define GPDMA_REQ_TIM3_CH4          (30)
#define GPDMA_REQ_TIM3_UP           (31)
#define GPDMA_REQ_TIM3_TRIG         (32)
#define GPDMA_REQ_TIM4_CH1          (33)
#define GPDMA_REQ_TIM4_CH2          (34)
#define GPDMA_REQ_TIM4_CH3          (35)
#define GPDMA_REQ_TIM4_CH4          (36)
#define GPDMA_REQ_TIM4_UP           (37)
#define GPDMA_REQ_TIM4_TRIG         (38)
#define GPDMA_REQ_TIM5_CH1          (39)
#define GPDMA_REQ_TIM5_CH2          (40)
#define GPDMA_REQ_TIM5_CH3          (41)
#define GPDMA_REQ_TIM5_CH4          (42)
#define GPDMA_REQ_TIM5_UP           (43)
#define GPDMA_REQ_TIM5_TRIG         (44)
#define GPDMA_REQ_TIM6_UP           (45)
#define GPDMA_REQ_TIM7_UP           (46)
#define GPDMA_REQ_TIM8_CH1          (47)
#define GPDMA_REQ_TIM8_CH2          (48)
#define GPDMA_REQ_TIM8_CH3          (49)
#define GPDMA_REQ_TIM8_CH4          (50)
#define GPDMA_REQ_TIM8_UP           (51)
#define GPDMA_REQ_TIM8_TRIG         (52)
#define GPDMA_REQ_TIM8_COM          (53)
                                 /* (54)-(55) RESERVED */
#define GPDMA_REQ_TIM15_CH1         (56)
#define GPDMA_REQ_TIM15_CH2         (57)
#define GPDMA_REQ_TIM15_UP          (58)
#define GPDMA_REQ_TIM15_TRIG        (59)
#define GPDMA_REQ_TIM15_COM         (60)
#define GPDMA_REQ_TIM16_CH1         (61)
#define GPDMA_REQ_TIM16_UP          (62)
#define GPDMA_REQ_TIM16_COM         (63)
#define GPDMA_REQ_TIM17_CH1         (64)
#define GPDMA_REQ_TIM17_UP          (65)
#define GPDMA_REQ_TIM17_COM         (66)
#define GPDMA_REQ_TIM18_CH1         (67)
#define GPDMA_REQ_TIM18_UP          (68)
#define GPDMA_REQ_TIM18_COM         (69)
#define GPDMA_REQ_LPTIM1_IC1        (70)
#define GPDMA_REQ_LPTIM1_IC2        (71)
#define GPDMA_REQ_LPTIM1_UE         (72)
#define GPDMA_REQ_LPTIM2_IC1        (73)
#define GPDMA_REQ_LPTIM2_IC2        (74)
#define GPDMA_REQ_LPTIM2_UE         (75)
#define GPDMA_REQ_LPTIM3_IC1        (76)
#define GPDMA_REQ_LPTIM3_IC2        (77)
#define GPDMA_REQ_LPTIM3_UE         (78)
#define GPDMA_REQ_SPI1_RX           (79)
#define GPDMA_REQ_SPI1_TX           (80)
#define GPDMA_REQ_SPI2_RX           (81)
#define GPDMA_REQ_SPI2_TX           (82)
#define GPDMA_REQ_SPI3_RX           (83)
#define GPDMA_REQ_SPI3_TX           (84)
#define GPDMA_REQ_SPI4_RX           (85)
#define GPDMA_REQ_SPI4_TX           (86)
#define GPDMA_REQ_SPI5_RX           (87)
#define GPDMA_REQ_SPI5_TX           (88)
#define GPDMA_REQ_SPI6_RX           (89)
#define GPDMA_REQ_SPI6_TX           (90)
#define GPDMA_REQ_SAI1_A            (91)
#define GPDMA_REQ_SAI1_B            (92)
#define GPDMA_REQ_SAI2_A            (93)
#define GPDMA_REQ_SAI2_B            (94)
#define GPDMA_REQ_I2C1_RX           (95)
#define GPDMA_REQ_I2C1_TX           (96)
#define GPDMA_REQ_I2C2_RX           (97)
#define GPDMA_REQ_I2C2_TX           (98)
#define GPDMA_REQ_I2C3_RX           (99)
#define GPDMA_REQ_I2C3_TX           (100)
#define GPDMA_REQ_I2C4_RX           (101)
#define GPDMA_REQ_I2C4_TX           (102)
#define GPDMA_REQ_I3C1_RX           (103)
#define GPDMA_REQ_I3C1_TX           (104)
#define GPDMA_REQ_I3C2_RX           (105)
#define GPDMA_REQ_I3C2_TX           (106)
#define GPDMA_REQ_USART1_RX         (107)
#define GPDMA_REQ_USART1_TX         (108)
#define GPDMA_REQ_USART2_RX         (109)
#define GPDMA_REQ_USART2_TX         (110)
#define GPDMA_REQ_USART3_RX         (111)
#define GPDMA_REQ_USART3_TX         (112)
#define GPDMA_REQ_UART4_RX          (113)
#define GPDMA_REQ_UART4_TX          (114)
#define GPDMA_REQ_UART5_RX          (115)
#define GPDMA_REQ_UART5_TX          (116)
#define GPDMA_REQ_USART6_RX         (117)
#define GPDMA_REQ_USART6_TX         (118)
#define GPDMA_REQ_UART7_RX          (119)
#define GPDMA_REQ_UART7_TX          (120)
#define GPDMA_REQ_UART8_RX          (121)
#define GPDMA_REQ_UART8_TX          (122)
#define GPDMA_REQ_UART9_RX          (123)
#define GPDMA_REQ_UART9_TX          (124)
#define GPDMA_REQ_USART10_RX        (125)
#define GPDMA_REQ_USART10_TX        (126)
#define GPDMA_REQ_LPUART1_RX        (127)
#define GPDMA_REQ_LPUART1_TX        (128)
#define GPDMA_REQ_SPDIFRX_CS        (129)
#define GPDMA_REQ_SPDIFRX_DT        (130)
#define GPDMA_REQ_ADF1_FLT0         (131)
#define GPDMA_REQ_MDF1_FLT0         (132)
#define GPDMA_REQ_MDF1_FLT1         (133)
#define GPDMA_REQ_MDF1_FLT2         (134)
#define GPDMA_REQ_MDF1_FLT3         (135)
#define GPDMA_REQ_MDF1_FLT4         (136)
#define GPDMA_REQ_MDF1_FLT5         (137)
#define GPDMA_REQ_UCPD1_TX          (138)
#define GPDMA_REQ_UCPD1_RX          (139)
#define GPDMA_REQ_DCMI_PSSI         (140)
#define GPDMA_REQ_I3C1_TC           (141)
#define GPDMA_REQ_I3C1_RS           (142)
#define GPDMA_REQ_I3C2_TC           (143)
#define GPDMA_REQ_I3C2_RS           (144)

/* U[S]ART RX DMA request numbers */

#define STM32_LPUART1_RXDMA_REQ  GPDMA_REQ_LPUART1_RX
#define STM32_USART1_RXDMA_REQ   GPDMA_REQ_USART1_RX
#define STM32_USART2_RXDMA_REQ   GPDMA_REQ_USART2_RX
#define STM32_USART3_RXDMA_REQ   GPDMA_REQ_USART3_RX
#define STM32_UART4_RXDMA_REQ    GPDMA_REQ_UART4_RX
#define STM32_UART5_RXDMA_REQ    GPDMA_REQ_UART5_RX
#define STM32_USART6_RXDMA_REQ   GPDMA_REQ_USART6_RX
#define STM32_UART7_RXDMA_REQ    GPDMA_REQ_UART7_RX
#define STM32_UART8_RXDMA_REQ    GPDMA_REQ_UART8_RX
#define STM32_UART9_RXDMA_REQ    GPDMA_REQ_UART9_RX
#define STM32_USART10_RXDMA_REQ  GPDMA_REQ_USART10_RX

#endif /* __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_DMASIGMAP_H */
