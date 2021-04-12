/****************************************************************************
 * arch/arm/src/stm32/hardware/stm32g4xxxx_dmamux.h
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

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXXX_DMAMUX_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXXX_DMAMUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DMAMUX1 mapping **********************************************************/

/* For category 3 and category 4 devices:
 *       DMAMUX1 channels 0 to 7 are connected to DMA1 channels 1 to 8.
 *       DMAMUX1 channels 8 to 15 are connected to DMA2 channels 1 to 8.
 *
 * For category 2:
 *       DMAMUX1 channels 0 to 5 are connected to DMA1 channels 1 to 6.
 *       DMAMUX1 channels 6 to 11 are connected to DMA2 channels 1 to 6.
 */

#define DMAMUX1_REQ_MEM2MEM            (0)                   /* Memory to memory transfer */
#define DMAMUX1_REQ_GEN0               (1)                   /* DMAMUX Request Generator 0 */
#define DMAMUX1_REQ_GEN1               (2)                   /* DMAMUX Request Generator 1 */
#define DMAMUX1_REQ_GEN2               (3)                   /* DMAMUX Request Generator 2 */
#define DMAMUX1_REQ_GEN3               (4)                   /* DMAMUX Request Generator 3 */
#define DMAMUX1_ADC1                   (5)                   /* DMAMUX ADC1 request */
#define DMAMUX1_DAC1_CH1               (6)                   /* DMAMUX DAC1 Channel 1 request */
#define DMAMUX1_DAC1_CH2               (7)                   /* DMAMUX DAC1 Channel 2 request */
#define DMAMUX1_TIM6_UP                (8)                   /* DMAMUX TIM6 Update request */
#define DMAMUX1_TIM7_UP                (9)                   /* DMAMUX TIM7 Update request */
#define DMAMUX1_SPI1_RX                (10)                  /* DMAMUX SPI1 Rx request */
#define DMAMUX1_SPI1_TX                (11)                  /* DMAMUX SPI1 Tx request */
#define DMAMUX1_SPI2_RX                (12)                  /* DMAMUX SPI2 Rx request */
#define DMAMUX1_SPI2_TX                (13)                  /* DMAMUX SPI2 Tx request */
#define DMAMUX1_SPI3_RX                (14)                  /* DMAMUX SPI3 Rx request */
#define DMAMUX1_SPI3_TX                (15)                  /* DMAMUX SPI3 Tx request */
#define DMAMUX1_I2C1_RX                (16)                  /* DMAMUX I2C1 Rx request */
#define DMAMUX1_I2C1_TX                (17)                  /* DMAMUX I2C1 Tx request */
#define DMAMUX1_I2C2_RX                (18)                  /* DMAMUX I2C2 Rx request */
#define DMAMUX1_I2C2_TX                (19)                  /* DMAMUX I2C2 Tx request */
#define DMAMUX1_I2C3_RX                (20)                  /* DMAMUX I2C3 Rx request */
#define DMAMUX1_I2C3_TX                (21)                  /* DMAMUX I2C3 Tx request */
#define DMAMUX1_I2C4_RX                (22)                  /* DMAMUX I2C4 Rx request */
#define DMAMUX1_I2C4_TX                (23)                  /* DMAMUX I2C4 Tx request */
#define DMAMUX1_USART1_RX              (24)                  /* DMAMUX USART1 Rx request */
#define DMAMUX1_USART1_TX              (25)                  /* DMAMUX USART1 Tx request */
#define DMAMUX1_USART2_RX              (26)                  /* DMAMUX USART2 Rx request */
#define DMAMUX1_USART2_TX              (27)                  /* DMAMUX USART2 Tx request */
#define DMAMUX1_USART3_RX              (28)                  /* DMAMUX USART3 Rx request */
#define DMAMUX1_USART3_TX              (29)                  /* DMAMUX USART3 Tx request */
#define DMAMUX1_UART4_RX               (30)                  /* DMAMUX UART4 Rx request */
#define DMAMUX1_UART4_TX               (31)                  /* DMAMUX UART4 Tx request */
#define DMAMUX1_UART5_RX               (32)                  /* DMAMUX UART5 Rx request */
#define DMAMUX1_UART5_TX               (33)                  /* DMAMUX UART5 Tx request */
#define DMAMUX1_LPUART1_RX             (34)                  /* DMAMUX LPUART1 Rx request */
#define DMAMUX1_LPUART1_TX             (35)                  /* DMAMUX LPUART1 Tx request */
#define DMAMUX1_ADC2                   (36)                  /* DMAMUX ADC2 request */
#define DMAMUX1_ADC3                   (37)                  /* DMAMUX ADC3 request */
#define DMAMUX1_ADC4                   (38)                  /* DMAMUX ADC4 request */
#define DMAMUX1_ADC5                   (39)                  /* DMAMUX ADC5 request */
#define DMAMUX1_QSPI                   (40)                  /* DMAMUX QSPI request */
#define DMAMUX1_DAC2_CH1               (41)                  /* DMAMUX DAC2 Channel 1 request */
#define DMAMUX1_TIM1_CH1               (42)                  /* DMAMUX TIM1 Channel 1 request */
#define DMAMUX1_TIM1_CH2               (43)                  /* DMAMUX TIM1 Channel 2 request */
#define DMAMUX1_TIM1_CH3               (44)                  /* DMAMUX TIM1 Channel 3 request */
#define DMAMUX1_TIM1_CH4               (45)                  /* DMAMUX TIM1 Channel 4 request */
#define DMAMUX1_TIM1_UP                (46)                  /* DMAMUX TIM1 Update request */
#define DMAMUX1_TIM1_TRIG              (47)                  /* DMAMUX TIM1 Trigger request */
#define DMAMUX1_TIM1_COM               (48)                  /* DMAMUX TIM1 Commutation request */
#define DMAMUX1_TIM8_CH1               (49)                  /* DMAMUX TIM8 Channel 1 request */
#define DMAMUX1_TIM8_CH2               (50)                  /* DMAMUX TIM8 Channel 2 request */
#define DMAMUX1_TIM8_CH3               (51)                  /* DMAMUX TIM8 Channel 3 request */
#define DMAMUX1_TIM8_CH4               (52)                  /* DMAMUX TIM8 Channel 4 request */
#define DMAMUX1_TIM8_UP                (53)                  /* DMAMUX TIM8 Update request */
#define DMAMUX1_TIM8_TRIG              (54)                  /* DMAMUX TIM8 Trigger request */
#define DMAMUX1_TIM8_COM               (55)                  /* DMAMUX TIM8 Commutation request */
#define DMAMUX1_TIM2_CH1               (56)                  /* DMAMUX TIM2 Channel 1 request */
#define DMAMUX1_TIM2_CH2               (57)                  /* DMAMUX TIM2 Channel 2 request */
#define DMAMUX1_TIM2_CH3               (58)                  /* DMAMUX TIM2 Channel 3 request */
#define DMAMUX1_TIM2_CH4               (59)                  /* DMAMUX TIM2 Channel 4 request */
#define DMAMUX1_TIM2_UP                (60)                  /* DMAMUX TIM2 Update request */
#define DMAMUX1_TIM3_CH1               (61)                  /* DMAMUX TIM3 Channel 1 request */
#define DMAMUX1_TIM3_CH2               (62)                  /* DMAMUX TIM3 Channel 2 request */
#define DMAMUX1_TIM3_CH3               (63)                  /* DMAMUX TIM3 Channel 3 request */
#define DMAMUX1_TIM3_CH4               (64)                  /* DMAMUX TIM3 Channel 4 request */
#define DMAMUX1_TIM3_UP                (65)                  /* DMAMUX TIM3 Update request */
#define DMAMUX1_TIM3_TRIG              (66)                  /* DMAMUX TIM3 Trigger request */
#define DMAMUX1_TIM4_CH1               (67)                  /* DMAMUX TIM4 Channel 1 request */
#define DMAMUX1_TIM4_CH2               (68)                  /* DMAMUX TIM4 Channel 2 request */
#define DMAMUX1_TIM4_CH3               (69)                  /* DMAMUX TIM4 Channel 3 request */
#define DMAMUX1_TIM4_CH4               (70)                  /* DMAMUX TIM4 Channel 4 request */
#define DMAMUX1_TIM4_UP                (71)                  /* DMAMUX TIM4 Update request */
#define DMAMUX1_TIM5_CH1               (72)                  /* DMAMUX TIM5 Channel 1 request */
#define DMAMUX1_TIM5_CH2               (73)                  /* DMAMUX TIM5 Channel 2 request */
#define DMAMUX1_TIM5_CH3               (74)                  /* DMAMUX TIM5 Channel 3 request */
#define DMAMUX1_TIM5_CH4               (75)                  /* DMAMUX TIM5 Channel 4 request */
#define DMAMUX1_TIM5_UP                (76)                  /* DMAMUX TIM5 Update request */
#define DMAMUX1_TIM5_TRIG              (77)                  /* DMAMUX TIM5 Trigger request */
#define DMAMUX1_TIM15_CH1              (78)                  /* DMAMUX TIM15 Channel 1 request */
#define DMAMUX1_TIM15_UP               (79)                  /* DMAMUX TIM15 Update request */
#define DMAMUX1_TIM15_TRIG             (80)                  /* DMAMUX TIM15 Trigger request */
#define DMAMUX1_TIM15_COM              (81)                  /* DMAMUX TIM15 Commutation request */
#define DMAMUX1_TIM16_CH1              (82)                  /* DMAMUX TIM16 Channel 1 request */
#define DMAMUX1_TIM16_UP               (83)                  /* DMAMUX TIM16 Update request */
#define DMAMUX1_TIM17_CH1              (84)                  /* DMAMUX TIM17 Channel 1 request */
#define DMAMUX1_TIM17_UP               (85)                  /* DMAMUX TIM17 Update request */
#define DMAMUX1_TIM20_CH1              (86)                  /* DMAMUX TIM20 Channel 1 request */
#define DMAMUX1_TIM20_CH2              (87)                  /* DMAMUX TIM20 Channel 2 request */
#define DMAMUX1_TIM20_CH3              (88)                  /* DMAMUX TIM20 Channel 3 request */
#define DMAMUX1_TIM20_CH4              (89)                  /* DMAMUX TIM20 Channel 4 request */
#define DMAMUX1_TIM20_UP               (90)                  /* DMAMUX TIM20 Update request */
#define DMAMUX1_AES_IN                 (91)                  /* DMAMUX AES In request */
#define DMAMUX1_AES_OUT                (92)                  /* DMAMUX AES Out request */
#define DMAMUX1_TIM20_TRIG             (93)                  /* DMAMUX TIM20 Trigger request */
#define DMAMUX1_TIM20_COM              (94)                  /* DMAMUX TIM20 Commutation request */
#define DMAMUX1_HRTIM1_M               (95)                  /* DMAMUX HRTIM M request */
#define DMAMUX1_HRTIM1_A               (96)                  /* DMAMUX HRTIM A request */
#define DMAMUX1_HRTIM1_B               (97)                  /* DMAMUX HRTIM B request */
#define DMAMUX1_HRTIM1_C               (98)                  /* DMAMUX HRTIM C request */
#define DMAMUX1_HRTIM1_D               (99)                  /* DMAMUX HRTIM D request */
#define DMAMUX1_HRTIM1_E               (100)                 /* DMAMUX HRTIM E request */
#define DMAMUX1_HRTIM1_F               (101)                 /* DMAMUX HRTIM F request */
#define DMAMUX1_DAC3_CH1               (102)                 /* DMAMUX DAC3 Channel 1 request */
#define DMAMUX1_DAC3_CH2               (103)                 /* DMAMUX DAC3 Channel 2 request */
#define DMAMUX1_DAC4_CH1               (104)                 /* DMAMUX DAC4 Channel 1 request */
#define DMAMUX1_DAC4_CH2               (105)                 /* DMAMUX DAC4 Channel 2 request */
#define DMAMUX1_SPI4_RX                (106)                 /* DMAMUX SPI4 Rx request */
#define DMAMUX1_SPI4_TX                (107)                 /* DMAMUX SPI4 Tx request */
#define DMAMUX1_SAI1_A                 (108)                 /* DMAMUX SAI1 A request */
#define DMAMUX1_SAI1_B                 (109)                 /* DMAMUX SAI1 B request */
#define DMAMUX1_FMAC_READ              (110)                 /* DMAMUX FMAC Read request */
#define DMAMUX1_FMAC_WRITE             (111)                 /* DMAMUX FMAC Write request */
#define DMAMUX1_CORDIC_READ            (112)                 /* DMAMUX CORDIC Read request */
#define DMAMUX1_CORDIC_WRITE           (113)                 /* DMAMUX CORDIC Write request */
#define DMAMUX1_UCPD1_RX               (114)                 /* DMAMUX USBPD1 Rx request */
#define DMAMUX1_UCPD1_TX               (115)                 /* DMAMUX USBPD1 Tx request */

/* DMAMAP for DMA1 and DMA2 (DMAMUX1) */

#define DMAMAP_DMA12_MEM2MEM_0     DMAMAP_MAP(DMA1, DMAMUX1_MEM2MEM)
#define DMAMAP_DMA12_MEM2MEM_1     DMAMAP_MAP(DMA2, DMAMUX1_MEM2MEM)
#define DMAMAP_DMA12_REQGEN0_0     DMAMAP_MAP(DMA1, DMAMUX1_REQ_GEN0)
#define DMAMAP_DMA12_REQGEN0_1     DMAMAP_MAP(DMA2, DMAMUX1_REQ_GEN0)
#define DMAMAP_DMA12_REQGEN1_0     DMAMAP_MAP(DMA1, DMAMUX1_REQ_GEN1)
#define DMAMAP_DMA12_REQGEN1_1     DMAMAP_MAP(DMA2, DMAMUX1_REQ_GEN1)
#define DMAMAP_DMA12_REQGEN2_0     DMAMAP_MAP(DMA1, DMAMUX1_REQ_GEN2)
#define DMAMAP_DMA12_REQGEN2_1     DMAMAP_MAP(DMA2, DMAMUX1_REQ_GEN2)
#define DMAMAP_DMA12_REQGEN3_0     DMAMAP_MAP(DMA1, DMAMUX1_REQ_GEN3)
#define DMAMAP_DMA12_REQGEN3_1     DMAMAP_MAP(DMA2, DMAMUX1_REQ_GEN3)
#define DMAMAP_DMA12_ADC1_0        DMAMAP_MAP(DMA1, DMAMUX1_ADC1)
#define DMAMAP_DMA12_ADC1_1        DMAMAP_MAP(DMA2, DMAMUX1_ADC1)
#define DMAMAP_DMA12_DAC1CH1_0     DMAMAP_MAP(DMA1, DMAMUX1_DAC1_CH1)
#define DMAMAP_DMA12_DAC1CH1_1     DMAMAP_MAP(DMA2, DMAMUX1_DAC1_CH1)
#define DMAMAP_DMA12_DAC1CH2_0     DMAMAP_MAP(DMA1, DMAMUX1_DAC1_CH2)
#define DMAMAP_DMA12_DAC1CH2_1     DMAMAP_MAP(DMA2, DMAMUX1_DAC1_CH2)
#define DMAMAP_DMA12_TIM6UP_0      DMAMAP_MAP(DMA1, DMAMUX1_TIM6_UP)
#define DMAMAP_DMA12_TIM6UP_1      DMAMAP_MAP(DMA2, DMAMUX1_TIM6_UP)
#define DMAMAP_DMA12_TIM7UP_0      DMAMAP_MAP(DMA1, DMAMUX1_TIM7_UP)
#define DMAMAP_DMA12_TIM7UP_1      DMAMAP_MAP(DMA2, DMAMUX1_TIM7_UP)
#define DMAMAP_DMA12_SPI1RX_0      DMAMAP_MAP(DMA1, DMAMUX1_SPI1_RX)
#define DMAMAP_DMA12_SPI1RX_1      DMAMAP_MAP(DMA2, DMAMUX1_SPI1_RX)
#define DMAMAP_DMA12_SPI1TX_0      DMAMAP_MAP(DMA1, DMAMUX1_SPI1_TX)
#define DMAMAP_DMA12_SPI1TX_1      DMAMAP_MAP(DMA2, DMAMUX1_SPI1_TX)
#define DMAMAP_DMA12_SPI2RX_0      DMAMAP_MAP(DMA1, DMAMUX1_SPI2_RX)
#define DMAMAP_DMA12_SPI2RX_1      DMAMAP_MAP(DMA2, DMAMUX1_SPI2_RX)
#define DMAMAP_DMA12_SPI2TX_0      DMAMAP_MAP(DMA1, DMAMUX1_SPI2_TX)
#define DMAMAP_DMA12_SPI2TX_1      DMAMAP_MAP(DMA2, DMAMUX1_SPI2_TX)
#define DMAMAP_DMA12_SPI3RX_0      DMAMAP_MAP(DMA1, DMAMUX1_SPI3_RX)
#define DMAMAP_DMA12_SPI3RX_1      DMAMAP_MAP(DMA2, DMAMUX1_SPI3_RX)
#define DMAMAP_DMA12_SPI3TX_0      DMAMAP_MAP(DMA1, DMAMUX1_SPI3_TX)
#define DMAMAP_DMA12_SPI3TX_1      DMAMAP_MAP(DMA2, DMAMUX1_SPI3_TX)
#define DMAMAP_DMA12_I2C1RX_0      DMAMAP_MAP(DMA1, DMAMUX1_I2C1_RX)
#define DMAMAP_DMA12_I2C1RX_1      DMAMAP_MAP(DMA2, DMAMUX1_I2C1_RX)
#define DMAMAP_DMA12_I2C1TX_0      DMAMAP_MAP(DMA1, DMAMUX1_I2C1_TX)
#define DMAMAP_DMA12_I2C1TX_1      DMAMAP_MAP(DMA2, DMAMUX1_I2C1_TX)
#define DMAMAP_DMA12_I2C2RX_0      DMAMAP_MAP(DMA1, DMAMUX1_I2C2_RX)
#define DMAMAP_DMA12_I2C2RX_1      DMAMAP_MAP(DMA2, DMAMUX1_I2C2_RX)
#define DMAMAP_DMA12_I2C2TX_0      DMAMAP_MAP(DMA1, DMAMUX1_I2C2_TX)
#define DMAMAP_DMA12_I2C2TX_1      DMAMAP_MAP(DMA2, DMAMUX1_I2C2_TX)
#define DMAMAP_DMA12_I2C3RX_0      DMAMAP_MAP(DMA1, DMAMUX1_I2C3_RX)
#define DMAMAP_DMA12_I2C3RX_1      DMAMAP_MAP(DMA2, DMAMUX1_I2C3_RX)
#define DMAMAP_DMA12_I2C3TX_0      DMAMAP_MAP(DMA1, DMAMUX1_I2C3_TX)
#define DMAMAP_DMA12_I2C3TX_1      DMAMAP_MAP(DMA2, DMAMUX1_I2C3_TX)
#define DMAMAP_DMA12_I2C4RX_0      DMAMAP_MAP(DMA1, DMAMUX1_I2C4_RX)
#define DMAMAP_DMA12_I2C4RX_1      DMAMAP_MAP(DMA2, DMAMUX1_I2C4_RX)
#define DMAMAP_DMA12_I2C4TX_0      DMAMAP_MAP(DMA1, DMAMUX1_I2C4_TX)
#define DMAMAP_DMA12_I2C4TX_1      DMAMAP_MAP(DMA2, DMAMUX1_I2C4_TX)
#define DMAMAP_DMA12_USART1RX_0    DMAMAP_MAP(DMA1, DMAMUX1_USART1_RX)
#define DMAMAP_DMA12_USART1RX_1    DMAMAP_MAP(DMA2, DMAMUX1_USART1_RX)
#define DMAMAP_DMA12_USART1TX_0    DMAMAP_MAP(DMA1, DMAMUX1_USART1_TX)
#define DMAMAP_DMA12_USART1TX_1    DMAMAP_MAP(DMA2, DMAMUX1_USART1_TX)
#define DMAMAP_DMA12_USART2RX_0    DMAMAP_MAP(DMA1, DMAMUX1_USART2_RX)
#define DMAMAP_DMA12_USART2RX_1    DMAMAP_MAP(DMA2, DMAMUX1_USART2_RX)
#define DMAMAP_DMA12_USART2TX_0    DMAMAP_MAP(DMA1, DMAMUX1_USART2_TX)
#define DMAMAP_DMA12_USART2TX_1    DMAMAP_MAP(DMA2, DMAMUX1_USART2_TX)
#define DMAMAP_DMA12_USART3RX_0    DMAMAP_MAP(DMA1, DMAMUX1_USART3_RX)
#define DMAMAP_DMA12_USART3RX_1    DMAMAP_MAP(DMA2, DMAMUX1_USART3_RX)
#define DMAMAP_DMA12_USART3TX_0    DMAMAP_MAP(DMA1, DMAMUX1_USART3_TX)
#define DMAMAP_DMA12_USART3TX_1    DMAMAP_MAP(DMA2, DMAMUX1_USART3_TX)
#define DMAMAP_DMA12_UART4RX_0     DMAMAP_MAP(DMA1, DMAMUX1_UART4_RX)
#define DMAMAP_DMA12_UART4RX_1     DMAMAP_MAP(DMA2, DMAMUX1_UART4_RX)
#define DMAMAP_DMA12_UART4TX_0     DMAMAP_MAP(DMA1, DMAMUX1_UART4_TX)
#define DMAMAP_DMA12_UART4TX_1     DMAMAP_MAP(DMA2, DMAMUX1_UART4_TX)
#define DMAMAP_DMA12_UART5RX_0     DMAMAP_MAP(DMA1, DMAMUX1_UART5_RX)
#define DMAMAP_DMA12_UART5RX_1     DMAMAP_MAP(DMA2, DMAMUX1_UART5_RX)
#define DMAMAP_DMA12_UART5TX_0     DMAMAP_MAP(DMA1, DMAMUX1_UART5_TX)
#define DMAMAP_DMA12_UART5TX_1     DMAMAP_MAP(DMA2, DMAMUX1_UART5_TX)
#define DMAMAP_DMA12_LPUART1RX_0   DMAMAP_MAP(DMA1, DMAMUX1_LPUART1_RX)
#define DMAMAP_DMA12_LPUART1RX_1   DMAMAP_MAP(DMA2, DMAMUX1_LPUART1_RX)
#define DMAMAP_DMA12_LPUART1TX_0   DMAMAP_MAP(DMA1, DMAMUX1_LPUART1_TX)
#define DMAMAP_DMA12_LPUART1TX_1   DMAMAP_MAP(DMA2, DMAMUX1_LPUART1_TX)
#define DMAMAP_DMA12_ADC2_0        DMAMAP_MAP(DMA1, DMAMUX1_ADC2)
#define DMAMAP_DMA12_ADC2_1        DMAMAP_MAP(DMA2, DMAMUX1_ADC2)
#define DMAMAP_DMA12_ADC3_0        DMAMAP_MAP(DMA1, DMAMUX1_ADC3)
#define DMAMAP_DMA12_ADC3_1        DMAMAP_MAP(DMA2, DMAMUX1_ADC3)
#define DMAMAP_DMA12_ADC4_0        DMAMAP_MAP(DMA1, DMAMUX1_ADC4)
#define DMAMAP_DMA12_ADC4_1        DMAMAP_MAP(DMA2, DMAMUX1_ADC4)
#define DMAMAP_DMA12_ADC5_0        DMAMAP_MAP(DMA1, DMAMUX1_ADC5)
#define DMAMAP_DMA12_ADC5_1        DMAMAP_MAP(DMA2, DMAMUX1_ADC5)
#define DMAMAP_DMA12_QSPI_0        DMAMAP_MAP(DMA1, DMAMUX1_QSPI)
#define DMAMAP_DMA12_QSPI_1        DMAMAP_MAP(DMA2, DMAMUX1_QSPI)
#define DMAMAP_DMA12_DAC2CH1_0     DMAMAP_MAP(DMA1, DMAMUX1_DAC2_CH1)
#define DMAMAP_DMA12_DAC2CH1_1     DMAMAP_MAP(DMA2, DMAMUX1_DAC2_CH1)
#define DMAMAP_DMA12_TIM1CH1_0     DMAMAP_MAP(DMA1, DMAMUX1_TIM1_CH1)
#define DMAMAP_DMA12_TIM1CH1_1     DMAMAP_MAP(DMA2, DMAMUX1_TIM1_CH1)
#define DMAMAP_DMA12_TIM1CH2_0     DMAMAP_MAP(DMA1, DMAMUX1_TIM1_CH2)
#define DMAMAP_DMA12_TIM1CH2_1     DMAMAP_MAP(DMA2, DMAMUX1_TIM1_CH2)
#define DMAMAP_DMA12_TIM1CH3_0     DMAMAP_MAP(DMA1, DMAMUX1_TIM1_CH3)
#define DMAMAP_DMA12_TIM1CH3_1     DMAMAP_MAP(DMA2, DMAMUX1_TIM1_CH3)
#define DMAMAP_DMA12_TIM1CH4_0     DMAMAP_MAP(DMA1, DMAMUX1_TIM1_CH4)
#define DMAMAP_DMA12_TIM1CH4_1     DMAMAP_MAP(DMA2, DMAMUX1_TIM1_CH4)
#define DMAMAP_DMA12_TIM1UP_0      DMAMAP_MAP(DMA1, DMAMUX1_TIM1_UP)
#define DMAMAP_DMA12_TIM1UP_1      DMAMAP_MAP(DMA2, DMAMUX1_TIM1_UP)
#define DMAMAP_DMA12_TIM1TRIG_0    DMAMAP_MAP(DMA1, DMAMUX1_TIM1_TRIG)
#define DMAMAP_DMA12_TIM1TRIG_1    DMAMAP_MAP(DMA2, DMAMUX1_TIM1_TRIG)
#define DMAMAP_DMA12_TIM1COM_0     DMAMAP_MAP(DMA1, DMAMUX1_TIM1_COM)
#define DMAMAP_DMA12_TIM1COM_1     DMAMAP_MAP(DMA2, DMAMUX1_TIM1_COM)
#define DMAMAP_DMA12_TIM8CH1_0     DMAMAP_MAP(DMA1, DMAMUX1_TIM8_CH1)
#define DMAMAP_DMA12_TIM8CH1_1     DMAMAP_MAP(DMA2, DMAMUX1_TIM8_CH1)
#define DMAMAP_DMA12_TIM8CH2_0     DMAMAP_MAP(DMA1, DMAMUX1_TIM8_CH2)
#define DMAMAP_DMA12_TIM8CH2_1     DMAMAP_MAP(DMA2, DMAMUX1_TIM8_CH2)
#define DMAMAP_DMA12_TIM8CH3_0     DMAMAP_MAP(DMA1, DMAMUX1_TIM8_CH3)
#define DMAMAP_DMA12_TIM8CH3_1     DMAMAP_MAP(DMA2, DMAMUX1_TIM8_CH3)
#define DMAMAP_DMA12_TIM8CH4_0     DMAMAP_MAP(DMA1, DMAMUX1_TIM8_CH4)
#define DMAMAP_DMA12_TIM8CH4_1     DMAMAP_MAP(DMA2, DMAMUX1_TIM8_CH4)
#define DMAMAP_DMA12_TIM8UP_0      DMAMAP_MAP(DMA1, DMAMUX1_TIM8_UP)
#define DMAMAP_DMA12_TIM8UP_1      DMAMAP_MAP(DMA2, DMAMUX1_TIM8_UP)
#define DMAMAP_DMA12_TIM8TRIG_0    DMAMAP_MAP(DMA1, DMAMUX1_TIM8_TRIG)
#define DMAMAP_DMA12_TIM8TRIG_1    DMAMAP_MAP(DMA2, DMAMUX1_TIM8_TRIG)
#define DMAMAP_DMA12_TIM8COM_0     DMAMAP_MAP(DMA1, DMAMUX1_TIM8_COM)
#define DMAMAP_DMA12_TIM8COM_1     DMAMAP_MAP(DMA2, DMAMUX1_TIM8_COM)
#define DMAMAP_DMA12_TIM2CH1_0     DMAMAP_MAP(DMA1, DMAMUX1_TIM2_CH1)
#define DMAMAP_DMA12_TIM2CH1_1     DMAMAP_MAP(DMA2, DMAMUX1_TIM2_CH1)
#define DMAMAP_DMA12_TIM2CH2_0     DMAMAP_MAP(DMA1, DMAMUX1_TIM2_CH2)
#define DMAMAP_DMA12_TIM2CH2_1     DMAMAP_MAP(DMA2, DMAMUX1_TIM2_CH2)
#define DMAMAP_DMA12_TIM2CH3_0     DMAMAP_MAP(DMA1, DMAMUX1_TIM2_CH3)
#define DMAMAP_DMA12_TIM2CH3_1     DMAMAP_MAP(DMA2, DMAMUX1_TIM2_CH3)
#define DMAMAP_DMA12_TIM2CH4_0     DMAMAP_MAP(DMA1, DMAMUX1_TIM2_CH4)
#define DMAMAP_DMA12_TIM2CH4_1     DMAMAP_MAP(DMA2, DMAMUX1_TIM2_CH4)
#define DMAMAP_DMA12_TIM2UP_0      DMAMAP_MAP(DMA1, DMAMUX1_TIM2_UP)
#define DMAMAP_DMA12_TIM2UP_1      DMAMAP_MAP(DMA2, DMAMUX1_TIM2_UP)
#define DMAMAP_DMA12_TIM3CH1_0     DMAMAP_MAP(DMA1, DMAMUX1_TIM3_CH1)
#define DMAMAP_DMA12_TIM3CH1_1     DMAMAP_MAP(DMA2, DMAMUX1_TIM3_CH1)
#define DMAMAP_DMA12_TIM3CH2_0     DMAMAP_MAP(DMA1, DMAMUX1_TIM3_CH2)
#define DMAMAP_DMA12_TIM3CH2_1     DMAMAP_MAP(DMA2, DMAMUX1_TIM3_CH2)
#define DMAMAP_DMA12_TIM3CH3_0     DMAMAP_MAP(DMA1, DMAMUX1_TIM3_CH3)
#define DMAMAP_DMA12_TIM3CH3_1     DMAMAP_MAP(DMA2, DMAMUX1_TIM3_CH3)
#define DMAMAP_DMA12_TIM3CH4_0     DMAMAP_MAP(DMA1, DMAMUX1_TIM3_CH4)
#define DMAMAP_DMA12_TIM3CH4_1     DMAMAP_MAP(DMA2, DMAMUX1_TIM3_CH4)
#define DMAMAP_DMA12_TIM3UP_0      DMAMAP_MAP(DMA1, DMAMUX1_TIM3_UP)
#define DMAMAP_DMA12_TIM3UP_1      DMAMAP_MAP(DMA2, DMAMUX1_TIM3_UP)
#define DMAMAP_DMA12_TIM3TRIG_0    DMAMAP_MAP(DMA1, DMAMUX1_TIM3_TRIG)
#define DMAMAP_DMA12_TIM3TRIG_1    DMAMAP_MAP(DMA2, DMAMUX1_TIM3_TRIG)
#define DMAMAP_DMA12_TIM4CH1_0     DMAMAP_MAP(DMA1, DMAMUX1_TIM4_CH1)
#define DMAMAP_DMA12_TIM4CH1_1     DMAMAP_MAP(DMA2, DMAMUX1_TIM4_CH1)
#define DMAMAP_DMA12_TIM4CH2_0     DMAMAP_MAP(DMA1, DMAMUX1_TIM4_CH2)
#define DMAMAP_DMA12_TIM4CH2_1     DMAMAP_MAP(DMA2, DMAMUX1_TIM4_CH2)
#define DMAMAP_DMA12_TIM4CH3_0     DMAMAP_MAP(DMA1, DMAMUX1_TIM4_CH3)
#define DMAMAP_DMA12_TIM4CH3_1     DMAMAP_MAP(DMA2, DMAMUX1_TIM4_CH3)
#define DMAMAP_DMA12_TIM4CH4_0     DMAMAP_MAP(DMA1, DMAMUX1_TIM4_CH4)
#define DMAMAP_DMA12_TIM4CH4_1     DMAMAP_MAP(DMA2, DMAMUX1_TIM4_CH4)
#define DMAMAP_DMA12_TIM4UP_0      DMAMAP_MAP(DMA1, DMAMUX1_TIM4_UP)
#define DMAMAP_DMA12_TIM4UP_1      DMAMAP_MAP(DMA2, DMAMUX1_TIM4_UP)
#define DMAMAP_DMA12_TIM5CH1_0     DMAMAP_MAP(DMA1, DMAMUX1_TIM5_CH1)
#define DMAMAP_DMA12_TIM5CH1_1     DMAMAP_MAP(DMA2, DMAMUX1_TIM5_CH1)
#define DMAMAP_DMA12_TIM5CH2_0     DMAMAP_MAP(DMA1, DMAMUX1_TIM5_CH2)
#define DMAMAP_DMA12_TIM5CH2_1     DMAMAP_MAP(DMA2, DMAMUX1_TIM5_CH2)
#define DMAMAP_DMA12_TIM5CH3_0     DMAMAP_MAP(DMA1, DMAMUX1_TIM5_CH3)
#define DMAMAP_DMA12_TIM5CH3_1     DMAMAP_MAP(DMA2, DMAMUX1_TIM5_CH3)
#define DMAMAP_DMA12_TIM5CH4_0     DMAMAP_MAP(DMA1, DMAMUX1_TIM5_CH4)
#define DMAMAP_DMA12_TIM5CH4_1     DMAMAP_MAP(DMA2, DMAMUX1_TIM5_CH4)
#define DMAMAP_DMA12_TIM5UP_0      DMAMAP_MAP(DMA1, DMAMUX1_TIM5_UP)
#define DMAMAP_DMA12_TIM5UP_1      DMAMAP_MAP(DMA2, DMAMUX1_TIM5_UP)
#define DMAMAP_DMA12_TIM5TRIG_0    DMAMAP_MAP(DMA1, DMAMUX1_TIM5_TRIG)
#define DMAMAP_DMA12_TIM5TRIG_1    DMAMAP_MAP(DMA2, DMAMUX1_TIM5_TRIG)
#define DMAMAP_DMA12_TIM15CH1_0    DMAMAP_MAP(DMA1, DMAMUX1_TIM15_CH1)
#define DMAMAP_DMA12_TIM15CH1_1    DMAMAP_MAP(DMA2, DMAMUX1_TIM15_CH1)
#define DMAMAP_DMA12_TIM15UP_0     DMAMAP_MAP(DMA1, DMAMUX1_TIM15_UP)
#define DMAMAP_DMA12_TIM15UP_1     DMAMAP_MAP(DMA2, DMAMUX1_TIM15_UP)
#define DMAMAP_DMA12_TIM15TRIG_0   DMAMAP_MAP(DMA1, DMAMUX1_TIM15_TRIG)
#define DMAMAP_DMA12_TIM15TRIG_1   DMAMAP_MAP(DMA2, DMAMUX1_TIM15_TRIG)
#define DMAMAP_DMA12_TIM16CH1_0    DMAMAP_MAP(DMA1, DMAMUX1_TIM16_CH1)
#define DMAMAP_DMA12_TIM16CH1_1    DMAMAP_MAP(DMA2, DMAMUX1_TIM16_CH1)
#define DMAMAP_DMA12_TIM16UP_0     DMAMAP_MAP(DMA1, DMAMUX1_TIM16_UP)
#define DMAMAP_DMA12_TIM16UP_1     DMAMAP_MAP(DMA2, DMAMUX1_TIM16_UP)
#define DMAMAP_DMA12_TIM17CH1_0    DMAMAP_MAP(DMA1, DMAMUX1_TIM17_CH1)
#define DMAMAP_DMA12_TIM17CH1_1    DMAMAP_MAP(DMA2, DMAMUX1_TIM17_CH1)
#define DMAMAP_DMA12_TIM17UP_0     DMAMAP_MAP(DMA1, DMAMUX1_TIM17_UP)
#define DMAMAP_DMA12_TIM17UP_1     DMAMAP_MAP(DMA2, DMAMUX1_TIM17_UP)
#define DMAMAP_DMA12_TIM20CH1_0    DMAMAP_MAP(DMA1, DMAMUX1_TIM20_CH1)
#define DMAMAP_DMA12_TIM20CH1_1    DMAMAP_MAP(DMA2, DMAMUX1_TIM20_CH1)
#define DMAMAP_DMA12_TIM20CH2_0    DMAMAP_MAP(DMA1, DMAMUX1_TIM20_CH2)
#define DMAMAP_DMA12_TIM20CH2_1    DMAMAP_MAP(DMA2, DMAMUX1_TIM20_CH2)
#define DMAMAP_DMA12_TIM20CH3_0    DMAMAP_MAP(DMA1, DMAMUX1_TIM20_CH3)
#define DMAMAP_DMA12_TIM20CH3_1    DMAMAP_MAP(DMA2, DMAMUX1_TIM20_CH3)
#define DMAMAP_DMA12_TIM20CH4_0    DMAMAP_MAP(DMA1, DMAMUX1_TIM20_CH4)
#define DMAMAP_DMA12_TIM20CH4_1    DMAMAP_MAP(DMA2, DMAMUX1_TIM20_CH4)
#define DMAMAP_DMA12_TIM20UP_0     DMAMAP_MAP(DMA1, DMAMUX1_TIM20_UP)
#define DMAMAP_DMA12_TIM20UP_1     DMAMAP_MAP(DMA2, DMAMUX1_TIM20_UP)
#define DMAMAP_DMA12_AESIN_0       DMAMAP_MAP(DMA1, DMAMUX1_AES_IN)
#define DMAMAP_DMA12_AESIN_1       DMAMAP_MAP(DMA2, DMAMUX1_AES_IN)
#define DMAMAP_DMA12_AESOUT_0      DMAMAP_MAP(DMA1, DMAMUX1_AES_OUT)
#define DMAMAP_DMA12_AESOUT_1      DMAMAP_MAP(DMA2, DMAMUX1_AES_OUT)
#define DMAMAP_DMA12_TIM20TRIG_0   DMAMAP_MAP(DMA1, DMAMUX1_TIM20_TRIG)
#define DMAMAP_DMA12_TIM20TRIG_1   DMAMAP_MAP(DMA2, DMAMUX1_TIM20_TRIG)
#define DMAMAP_DMA12_TIM20COM_0    DMAMAP_MAP(DMA1, DMAMUX1_TIM20_COM)
#define DMAMAP_DMA12_TIM20COM_1    DMAMAP_MAP(DMA2, DMAMUX1_TIM20_COM)
#define DMAMAP_DMA12_HRTIM1M_0     DMAMAP_MAP(DMA1, DMAMUX1_HRTIM1_M)
#define DMAMAP_DMA12_HRTIM1M_1     DMAMAP_MAP(DMA2, DMAMUX1_HRTIM1_M)
#define DMAMAP_DMA12_HRTIM1A_0     DMAMAP_MAP(DMA1, DMAMUX1_HRTIM1_A)
#define DMAMAP_DMA12_HRTIM1A_1     DMAMAP_MAP(DMA2, DMAMUX1_HRTIM1_A)
#define DMAMAP_DMA12_HRTIM1B_0     DMAMAP_MAP(DMA1, DMAMUX1_HRTIM1_B)
#define DMAMAP_DMA12_HRTIM1B_1     DMAMAP_MAP(DMA2, DMAMUX1_HRTIM1_B)
#define DMAMAP_DMA12_HRTIM1C_0     DMAMAP_MAP(DMA1, DMAMUX1_HRTIM1_C)
#define DMAMAP_DMA12_HRTIM1C_1     DMAMAP_MAP(DMA2, DMAMUX1_HRTIM1_C)
#define DMAMAP_DMA12_HRTIM1D_0     DMAMAP_MAP(DMA1, DMAMUX1_HRTIM1_D)
#define DMAMAP_DMA12_HRTIM1D_1     DMAMAP_MAP(DMA2, DMAMUX1_HRTIM1_D)
#define DMAMAP_DMA12_HRTIM1E_0     DMAMAP_MAP(DMA1, DMAMUX1_HRTIM1_E)
#define DMAMAP_DMA12_HRTIM1E_1     DMAMAP_MAP(DMA2, DMAMUX1_HRTIM1_E)
#define DMAMAP_DMA12_HRTIM1F_0     DMAMAP_MAP(DMA1, DMAMUX1_HRTIM1_F)
#define DMAMAP_DMA12_HRTIM1F_1     DMAMAP_MAP(DMA2, DMAMUX1_HRTIM1_F)
#define DMAMAP_DMA12_DAC3CH1_0     DMAMAP_MAP(DMA1, DMAMUX1_DAC3_CH1)
#define DMAMAP_DMA12_DAC3CH1_1     DMAMAP_MAP(DMA2, DMAMUX1_DAC3_CH1)
#define DMAMAP_DMA12_DAC3CH2_0     DMAMAP_MAP(DMA1, DMAMUX1_DAC3_CH2)
#define DMAMAP_DMA12_DAC3CH2_1     DMAMAP_MAP(DMA2, DMAMUX1_DAC3_CH2)
#define DMAMAP_DMA12_DAC4CH1_0     DMAMAP_MAP(DMA1, DMAMUX1_DAC4_CH1)
#define DMAMAP_DMA12_DAC4CH1_1     DMAMAP_MAP(DMA2, DMAMUX1_DAC4_CH1)
#define DMAMAP_DMA12_DAC4CH2_0     DMAMAP_MAP(DMA1, DMAMUX1_DAC4_CH2)
#define DMAMAP_DMA12_DAC4CH2_1     DMAMAP_MAP(DMA2, DMAMUX1_DAC4_CH2)
#define DMAMAP_DMA12_SPI4RX_0      DMAMAP_MAP(DMA1, DMAMUX1_SPI4_RX)
#define DMAMAP_DMA12_SPI4RX_1      DMAMAP_MAP(DMA2, DMAMUX1_SPI4_RX)
#define DMAMAP_DMA12_SPI4TX_0      DMAMAP_MAP(DMA1, DMAMUX1_SPI4_TX)
#define DMAMAP_DMA12_SPI4TX_1      DMAMAP_MAP(DMA2, DMAMUX1_SPI4_TX)
#define DMAMAP_DMA12_SAI1A_0       DMAMAP_MAP(DMA1, DMAMUX1_SAI1_A)
#define DMAMAP_DMA12_SAI1A_1       DMAMAP_MAP(DMA2, DMAMUX1_SAI1_A)
#define DMAMAP_DMA12_SAI1B_0       DMAMAP_MAP(DMA1, DMAMUX1_SAI1_B)
#define DMAMAP_DMA12_SAI1B_1       DMAMAP_MAP(DMA2, DMAMUX1_SAI1_B)
#define DMAMAP_DMA12_FMACREAD_0    DMAMAP_MAP(DMA1, DMAMUX1_FMAC_READ)
#define DMAMAP_DMA12_FMACREAD_1    DMAMAP_MAP(DMA2, DMAMUX1_FMAC_READ)
#define DMAMAP_DMA12_FMACWRITE_0   DMAMAP_MAP(DMA1, DMAMUX1_FMAC_WRITE)
#define DMAMAP_DMA12_FMACWRITE_1   DMAMAP_MAP(DMA2, DMAMUX1_FMAC_WRITE)
#define DMAMAP_DMA12_CORDICREAD_0  DMAMAP_MAP(DMA1, DMAMUX1_CORDIC_READ)
#define DMAMAP_DMA12_CORDICREAD_1  DMAMAP_MAP(DMA2, DMAMUX1_CORDIC_READ)
#define DMAMAP_DMA12_CORDICWRITE_0 DMAMAP_MAP(DMA1, DMAMUX1_CORDIC_WRITE)
#define DMAMAP_DMA12_CORDICWRITE_1 DMAMAP_MAP(DMA2, DMAMUX1_CORDIC_WRITE)
#define DMAMAP_DMA12_UCPD1RX_0     DMAMAP_MAP(DMA1, DMAMUX1_UCPD1_RX)
#define DMAMAP_DMA12_UCPD1RX_1     DMAMAP_MAP(DMA2, DMAMUX1_UCPD1_RX)
#define DMAMAP_DMA12_UCPD1TX_0     DMAMAP_MAP(DMA1, DMAMUX1_UCPD1_TX)
#define DMAMAP_DMA12_UCPD1TX_1     DMAMAP_MAP(DMA2, DMAMUX1_UCPD1_TX)

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXXX_DMAMUX_H */
