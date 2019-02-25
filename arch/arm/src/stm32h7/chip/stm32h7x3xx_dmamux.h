/************************************************************************************
 * arch/arm/src/stm32h7/chip/stm32h7x3xx_dmamux.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Mateusz Szafoni <raiden00@railab.me>
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

#ifndef __ARCH_ARM_SRC_STM32H7_CHIP_STM32H7X3XX_DMAMUX_H
#define __ARCH_ARM_SRC_STM32H7_CHIP_STM32H7X3XX_DMAMUX_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* DMAMUX1 mapping ****************************************************/

/* NOTE: DMAMUX1 channels 0 to 7 are connected to DMA1 channels 0 to 7.
 *       DMAMUX1 channels 8 to 15 are connected to DMA2 channels 0 to 7.
 */

#define DMAMUX1_REQ_GEN0       (1)
#define DMAMUX1_REQ_GEN1       (2)
#define DMAMUX1_REQ_GEN2       (3)
#define DMAMUX1_REQ_GEN3       (4)
#define DMAMUX1_REQ_GEN4       (5)
#define DMAMUX1_REQ_GEN5       (6)
#define DMAMUX1_REQ_GEN6       (7)
#define DMAMUX1_REQ_GEN7       (8)
#define DMAMUX1_ADC1           (9)
#define DMAMUX1_ADC2           (10)
#define DMAMUX1_TIM1_CH1       (11)
#define DMAMUX1_TIM1_CH2       (12)
#define DMAMUX1_TIM1_CH3       (13)
#define DMAMUX1_TIM1_CH4       (14)
#define DMAMUX1_TIM1_UP        (15)
#define DMAMUX1_TIM1_TRIG      (16)
#define DMAMUX1_TIM1_COM       (17)
#define DMAMUX1_TIM2_CH1       (18)
#define DMAMUX1_TIM2_CH2       (19)
#define DMAMUX1_TIM2_CH3       (20)
#define DMAMUX1_TIM2_CH4       (21)
#define DMAMUX1_TIM2_UP        (22)
#define DMAMUX1_TIM3_CH1       (23)
#define DMAMUX1_TIM3_CH2       (24)
#define DMAMUX1_TIM3_CH3       (25)
#define DMAMUX1_TIM3_CH4       (26)
#define DMAMUX1_TIM3_UP        (27)
#define DMAMUX1_TIM3_TRIG      (28)
#define DMAMUX1_TIM4_CH1       (29)
#define DMAMUX1_TIM4_CH2       (30)
#define DMAMUX1_TIM4_CH3       (31)
#define DMAMUX1_TIM4_UP        (32)
#define DMAMUX1_I2C1_RX        (33)
#define DMAMUX1_I2C1_TX        (34)
#define DMAMUX1_I2C2_RX        (35)
#define DMAMUX1_I2C2_TX        (36)
#define DMAMUX1_SPI1_RX        (37)
#define DMAMUX1_SPI1_TX        (38)
#define DMAMUX1_SPI2_RX        (39)
#define DMAMUX1_SPI2_TX        (40)
#define DMAMUX1_USART1_TX      (41)
#define DMAMUX1_USART1_RX      (42)
#define DMAMUX1_USART2_RX      (43)
#define DMAMUX1_USART2_TX      (44)
#define DMAMUX1_USART3_RX      (45)
#define DMAMUX1_USART3_TX      (46)
#define DMAMUX1_TIM8_CH1       (47)
#define DMAMUX1_TIM8_CH2       (48)
#define DMAMUX1_TIM8_CH3       (49)
#define DMAMUX1_TIM8_CH4       (50)
#define DMAMUX1_TIM8_UP        (51)
#define DMAMUX1_TIM8_TRIG      (52)
#define DMAMUX1_TIM8_COM       (53)
/* DMAMUX1 54: Reserved */
#define DMAMUX1_TIM5_CH1       (55)
#define DMAMUX1_TIM5_CH2       (56)
#define DMAMUX1_TIM5_CH3       (57)
#define DMAMUX1_TIM5_CH4       (58)
#define DMAMUX1_TIM5_UP        (59)
#define DMAMUX1_TIM5_TRIG      (60)
#define DMAMUX1_SPI3_RX        (61)
#define DMAMUX1_SPI3_TX        (62)
#define DMAMUX1_UART4_RX       (63)
#define DMAMUX1_UART4_TX       (64)
#define DMAMUX1_UART5_RX       (65)
#define DMAMUX1_UART5_TX       (66)
#define DMAMUX1_DAC_CH1        (67)
#define DMAMUX1_DAC_CH2        (68)
#define DMAMUX1_TIM6_UP        (69)
#define DMAMUX1_TIM7_UP        (70)
#define DMAMUX1_USART6_RX      (71)
#define DMAMUX1_USART6_TX      (72)
#define DMAMUX1_I2C3_RX        (73)
#define DMAMUX1_I2C3_TX        (74)
#define DMAMUX1_DCMI           (75)
#define DMAMUX1_CRYPT_IN       (76)
#define DMAMUX1_CRYPT_OUT      (77)
#define DMAMUX1_HASH_IN        (78)
#define DMAMUX1_UART7_RX       (70)
#define DMAMUX1_UART7_TX       (80)
#define DMAMUX1_UART8_RX       (81)
#define DMAMUX1_UART8_TX       (82)
#define DMAMUX1_SPI4_RX        (83)
#define DMAMUX1_SPI4_TX        (84)
#define DMAMUX1_SPI5_RX        (85)
#define DMAMUX1_SPI5_TX        (86)
#define DMAMUX1_SAI1A          (87)
#define DMAMUX1_SAI1B          (88)
#define DMAMUX1_SAI2A          (89)
#define DMAMUX1_SAI2B          (90)
#define DMAMUX1_SWPMI_RX       (91)
#define DMAMUX1_SWPMI_TX       (92)
#define DMAMUX1_SPDIFRX_DAT    (93)
#define DMAMUX1_SPDIFRX_CTRL   (94)
#define DMAMUX1_HR_REQ1        (95)
#define DMAMUX1_HR_REQ2        (96)
#define DMAMUX1_HR_REQ3        (97)
#define DMAMUX1_HR_REQ4        (98)
#define DMAMUX1_HR_REQ5        (99)
#define DMAMUX1_HR_REQ6        (100)
#define DMAMUX1_DFSDM1_0       (101)
#define DMAMUX1_DFSDM1_1       (102)
#define DMAMUX1_DFSDM1_2       (103)
#define DMAMUX1_DFSDM1_3       (104)
#define DMAMUX1_TIM15_CH1      (105)
#define DMAMUX1_TIM15_UP       (106)
#define DMAMUX1_TIM15_TRIG     (107)
#define DMAMUX1_TIM15_COM      (108)
#define DMAMUX1_TIM16_CH1      (109)
#define DMAMUX1_TIM16_UP       (110)
#define DMAMUX1_TIM17_CH1      (111)
#define DMAMUX1_TIM17_UP       (112)
#define DMAMUX1_SAI3A          (113)
#define DMAMUX1_SAI3B          (114)
#define DMAMUX1_ADC3           (115)
/* DMAMUX1 116-127: Reserved */

/* DMAMUX2 mapping ****************************************************/

/* NOTE: DMAMUX2 channels 0 to 7 are connected to BDMA channels 0 to 7 */

#define DMAMUX2_REQ_GEN0       (1)
#define DMAMUX2_REQ_GEN1       (2)
#define DMAMUX2_REQ_GEN2       (3)
#define DMAMUX2_REQ_GEN3       (4)
#define DMAMUX2_REQ_GEN4       (5)
#define DMAMUX2_REQ_GEN5       (6)
#define DMAMUX2_REQ_GEN6       (7)
#define DMAMUX2_REQ_GEN7       (8)
#define DMAMUX2_LPUART1_RX     (9)
#define DMAMUX2_LPUART1_TX     (10)
#define DMAMUX2_SPI6_RX        (11)
#define DMAMUX2_SPI6_TX        (12)
#define DMAMUX2_I2C4_RX        (13)
#define DMAMUX2_I2C4_TX        (14)
#define DMAMUX2_SAI4A          (15)
#define DMAMUX2_SAI4B          (16)
#define DMAMUX2_ADC3           (17)
/* DMAMUX2 18-32: Reserved */

#endif /* __ARCH_ARM_SRC_STM32H7_CHIP_STM32H7X3XX_DMAMUX_H */
