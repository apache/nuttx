/****************************************************************************
 * arch/arm/src/stm32wb/hardware/stm32wb_dmamux.h
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

#ifndef __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_DMAMUX_H
#define __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_DMAMUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"
#include "stm32wb_dma.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DMAMUX1 0

/* Register Offsets *********************************************************/

#define STM32WB_DMAMUX_CXCR_OFFSET(x)  (0x0000 + 0x0004 * (x)) /* DMAMUX1 request line multiplexer channel x configuration register */
#define STM32WB_DMAMUX_C0CR_OFFSET     STM32WB_DMAMUX_CXCR_OFFSET(0)
#define STM32WB_DMAMUX_C1CR_OFFSET     STM32WB_DMAMUX_CXCR_OFFSET(1)
#define STM32WB_DMAMUX_C2CR_OFFSET     STM32WB_DMAMUX_CXCR_OFFSET(2)
#define STM32WB_DMAMUX_C3CR_OFFSET     STM32WB_DMAMUX_CXCR_OFFSET(3)
#define STM32WB_DMAMUX_C4CR_OFFSET     STM32WB_DMAMUX_CXCR_OFFSET(4)
#define STM32WB_DMAMUX_C5CR_OFFSET     STM32WB_DMAMUX_CXCR_OFFSET(5)
#define STM32WB_DMAMUX_C6CR_OFFSET     STM32WB_DMAMUX_CXCR_OFFSET(6)
#define STM32WB_DMAMUX_C7CR_OFFSET     STM32WB_DMAMUX_CXCR_OFFSET(7)
#define STM32WB_DMAMUX_C8CR_OFFSET     STM32WB_DMAMUX_CXCR_OFFSET(8)
#define STM32WB_DMAMUX_C9CR_OFFSET     STM32WB_DMAMUX_CXCR_OFFSET(9)
#define STM32WB_DMAMUX_C10CR_OFFSET    STM32WB_DMAMUX_CXCR_OFFSET(10)
#define STM32WB_DMAMUX_C11CR_OFFSET    STM32WB_DMAMUX_CXCR_OFFSET(11)
#define STM32WB_DMAMUX_C12CR_OFFSET    STM32WB_DMAMUX_CXCR_OFFSET(12)
#define STM32WB_DMAMUX_C13CR_OFFSET    STM32WB_DMAMUX_CXCR_OFFSET(13)
                                              /* 0x034-0x07C: Reserved */
#define STM32WB_DMAMUX_CSR_OFFSET      0x0080 /* DMAMUX1 request line multiplexer interrupt channel status register */
#define STM32WB_DMAMUX_CFR_OFFSET      0x0084 /* DMAMUX1 request line multiplexer interrupt clear flag register */
                                              /* 0x088-0x0FC: Reserved */

#define STM32WB_DMAMUX_RGXCR_OFFSET(x) (0x0100 + 0x004 * (x)) /* DMAMUX1 request generator channel x configuration register */
#define STM32WB_DMAMUX_RG0CR_OFFSET    STM32WB_DMAMUX_RGXCR_OFFSET(0)
#define STM32WB_DMAMUX_RG1CR_OFFSET    STM32WB_DMAMUX_RGXCR_OFFSET(1)
#define STM32WB_DMAMUX_RG2CR_OFFSET    STM32WB_DMAMUX_RGXCR_OFFSET(2)
#define STM32WB_DMAMUX_RG3CR_OFFSET    STM32WB_DMAMUX_RGXCR_OFFSET(3)
#define STM32WB_DMAMUX_RGSR_OFFSET     0x0140 /* DMAMUX1 request generator interrupt status register */
#define STM32WB_DMAMUX_RGCFR_OFFSET    0x0144 /* DMAMUX1 request generator interrupt clear flag register */
                                              /* 0x148-0x3fc: Reserved */

/* Register Addresses *******************************************************/

#define STM32WB_DMAMUX1_CXCR(x)  (STM32WB_DMAMUX1_BASE + STM32WB_DMAMUX_CXCR_OFFSET(x))
#define STM32WB_DMAMUX1_C0CR     (STM32WB_DMAMUX1_BASE + STM32WB_DMAMUX_C0CR_OFFSET)
#define STM32WB_DMAMUX1_C1CR     (STM32WB_DMAMUX1_BASE + STM32WB_DMAMUX_C1CR_OFFSET)
#define STM32WB_DMAMUX1_C2CR     (STM32WB_DMAMUX1_BASE + STM32WB_DMAMUX_C2CR_OFFSET)
#define STM32WB_DMAMUX1_C3CR     (STM32WB_DMAMUX1_BASE + STM32WB_DMAMUX_C3CR_OFFSET)
#define STM32WB_DMAMUX1_C4CR     (STM32WB_DMAMUX1_BASE + STM32WB_DMAMUX_C4CR_OFFSET)
#define STM32WB_DMAMUX1_C5CR     (STM32WB_DMAMUX1_BASE + STM32WB_DMAMUX_C5CR_OFFSET)
#define STM32WB_DMAMUX1_C6CR     (STM32WB_DMAMUX1_BASE + STM32WB_DMAMUX_C6CR_OFFSET)
#define STM32WB_DMAMUX1_C7CR     (STM32WB_DMAMUX1_BASE + STM32WB_DMAMUX_C7CR_OFFSET)
#define STM32WB_DMAMUX1_C8CR     (STM32WB_DMAMUX1_BASE + STM32WB_DMAMUX_C8CR_OFFSET)
#define STM32WB_DMAMUX1_C9CR     (STM32WB_DMAMUX1_BASE + STM32WB_DMAMUX_C9CR_OFFSET)
#define STM32WB_DMAMUX1_C10CR    (STM32WB_DMAMUX1_BASE + STM32WB_DMAMUX_C10CR_OFFSET)
#define STM32WB_DMAMUX1_C11CR    (STM32WB_DMAMUX1_BASE + STM32WB_DMAMUX_C11CR_OFFSET)
#define STM32WB_DMAMUX1_C12CR    (STM32WB_DMAMUX1_BASE + STM32WB_DMAMUX_C12CR_OFFSET)
#define STM32WB_DMAMUX1_C13CR    (STM32WB_DMAMUX1_BASE + STM32WB_DMAMUX_C13CR_OFFSET)

#define STM32WB_DMAMUX1_CSR      (STM32WB_DMAMUX1_BASE + STM32WB_DMAMUX_CSR_OFFSET)
#define STM32WB_DMAMUX1_CFR      (STM32WB_DMAMUX1_BASE + STM32WB_DMAMUX_CFR_OFFSET)

#define STM32WB_DMAMUX1_RGXCR(x) (STM32WB_DMAMUX1_BASE + STM32WB_DMAMUX_RGXCR_OFFSET(x))
#define STM32WB_DMAMUX1_RG0CR    (STM32WB_DMAMUX1_BASE + STM32WB_DMAMUX_RG0CR_OFFSET)
#define STM32WB_DMAMUX1_RG1CR    (STM32WB_DMAMUX1_BASE + STM32WB_DMAMUX_RG1CR_OFFSET)
#define STM32WB_DMAMUX1_RG2CR    (STM32WB_DMAMUX1_BASE + STM32WB_DMAMUX_RG2CR_OFFSET)
#define STM32WB_DMAMUX1_RG3CR    (STM32WB_DMAMUX1_BASE + STM32WB_DMAMUX_RG3CR_OFFSET)

#define STM32WB_DMAMUX1_RGSR     (STM32WB_DMAMUX1_BASE + STM32WB_DMAMUX_RGSR_OFFSET)
#define STM32WB_DMAMUX1_RGCFR    (STM32WB_DMAMUX1_BASE + STM32WB_DMAMUX_RGCFR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* DMAMUX1 request line multiplexer channel x configuration register */

#define DMAMUX_CCR_DMAREQID_SHIFT (0)  /* Bits 0-5: DMA request identification */
#define DMAMUX_CCR_DMAREQID_MASK  (0x3f << DMAMUX_CCR_DMAREQID_SHIFT)
#define DMAMUX_CCR_SOIE           (8)  /* Bit 8: Synchronization overrun interrupt enable */
#define DMAMUX_CCR_EGE            (9)  /* Bit 9: Event generation enable */
#define DMAMUX_CCR_SE             (16) /* Bit 16: Synchronization enable */
#define DMAMUX_CCR_SPOL_SHIFT     (17) /* Bits 17-18: Synchronization polarity */
#define DMAMUX_CCR_SPOL_MASK      (0x3 << DMAMUX_CCR_SPOL_SHIFT)
#define DMAMUX_CCR_NBREQ_SHIFT    (19) /* Bits 19-23: Number of DMA request minus 1 to forward */
#define DMAMUX_CCR_NBREQ_MASK     (0x1f << DMAMUX_CCR_NBREQ_SHIFT)
#define DMAMUX_CCR_SYNCID_SHIFT   (24) /* Bits 24-28: Synchronization identification */
#define DMAMUX_CCR_SYNCID_MASK    (0x1f << DMAMUX_CCR_SYNCID_SHIFT)

/* DMAMUX1 request line multiplexer interrupt channel status register */

#define DMAMUX_CSR_SOF(x)         (1 << (x)) /* Synchronization overrun event flag */

/* DMAMUX1 request line multiplexer interrupt clear flag register */

#define DMAMUX_CFR_CSOF(x)        (1 << (x)) /* Clear synchronization overrun event flag */

/* DMAMUX1 request generator channel x configuration register */

#define DMAMUX_RGCR_SIGID_SHIFT   (0)  /* Bits 0-4: Signal identifiaction */
#define DMAMUX_RGCR_SIGID_MASK    (0x1f << DMAMUX_RGCR_SIGID_SHIFT)
#define DMAMUX_RGCR_OIE           (8)  /* Bit 8: Trigger overrun interrupt enable */
#define DMAMUX_RGCR_GE            (16) /* Bit 16: DMA request generator channel X enable*/
#define DMAMUX_RGCR_GPOL_SHIFT    (17) /* Bits 17-18: DMA request generator trigger polarity */
#define DMAMUX_RGCR_GPOL_MASK     (0x3 << DMAMUX_RGCR_GPOL_SHIFT)
#define DMAMUX_RGCR_GNBREQ_SHIFT  (19) /* Bits 19-23: Number of DMA requests to be generated minus 1 */
#define DMAMUX_RGCR_GNBREQ_MASK   (0x1f << DMAMUX_RGCR_GNBREQ_SHIFT)

/* DMAMUX1 request generator interrupt status register */

#define DMAMUX_RGSR_OF(x)         (1 << (x)) /* Trigger overrun event flag */

/* DMAMUX1 request generator interrupt clear flag register */

#define DMAMUX_RGCFR_COF(x)       (1 << (x)) /* Clear trigger overrun event flag */

/* DMA channel mapping
 *
 * D.CCCCCCC
 * C - DMAMUX1 request
 * D - DMA controller (DMA1, DMA2)
 */

#define DMAMAP_MAP(d,c)           ((((d) & 0x01) << 7) | ((c) & 0x7f))
#define DMAMAP_CONTROLLER(m)      (((m) >> 7) & 0x01)
#define DMAMAP_REQUEST(m)         (((m) >> 0) & 0x7f)

/* DMAMUX1 mapping **********************************************************/

/* NOTE: DMAMUX1 channels 0 to 6 are connected to DMA1 channels 1 to 7.
 *       DMAMUX1 channels 7 to 13 are connected to DMA2 channels 1 to 7.
 */

#define DMAMUX1_REQ_GEN0       (1)
#define DMAMUX1_REQ_GEN1       (2)
#define DMAMUX1_REQ_GEN2       (3)
#define DMAMUX1_REQ_GEN3       (4)
#define DMAMUX1_ADC1           (5)
#define DMAMUX1_SPI1_RX        (6)
#define DMAMUX1_SPI1_TX        (7)
#define DMAMUX1_SPI2_RX        (8)
#define DMAMUX1_SPI2_TX        (9)
#define DMAMUX1_I2C1_RX        (10)
#define DMAMUX1_I2C1_TX        (11)
#define DMAMUX1_I2C3_RX        (12)
#define DMAMUX1_I2C3_TX        (13)
#define DMAMUX1_USART1_RX      (14)
#define DMAMUX1_USART1_TX      (15)
#define DMAMUX1_LPUART1_RX     (16)
#define DMAMUX1_LPUART1_TX     (17)
#define DMAMUX1_SAI1_A         (18)
#define DMAMUX1_SAI1_B         (19)
#define DMAMUX1_QSPI           (20)
#define DMAMUX1_TIM1_CH1       (21)
#define DMAMUX1_TIM1_CH2       (22)
#define DMAMUX1_TIM1_CH3       (23)
#define DMAMUX1_TIM1_CH4       (24)
#define DMAMUX1_TIM1_UP        (25)
#define DMAMUX1_TIM1_TRIG      (26)
#define DMAMUX1_TIM1_COM       (27)
#define DMAMUX1_TIM2_CH1       (28)
#define DMAMUX1_TIM2_CH2       (29)
#define DMAMUX1_TIM2_CH3       (30)
#define DMAMUX1_TIM2_CH4       (31)
#define DMAMUX1_TIM2_UP        (32)
#define DMAMUX1_TIM16_CH1      (33)
#define DMAMUX1_TIM16_UP       (34)
#define DMAMUX1_TIM17_CH1      (35)
#define DMAMUX1_TIM17_UP       (36)
#define DMAMUX1_AES1_IN        (37)
#define DMAMUX1_AES1_OUT       (38)
#define DMAMUX1_AES2_IN        (39)
#define DMAMUX1_AES2_OUT       (40)
/* DMAMUX1 41-63: Reserved */

/* DMAMAP for DMA1 and DMA2 (DMAMUX1) ***************************************/

#define DMAMAP_REQ_GEN0_0      DMAMAP_MAP(DMA1, DMAMUX1_REQ_GEN0)
#define DMAMAP_REQ_GEN0_1      DMAMAP_MAP(DMA2, DMAMUX1_REQ_GEN0)
#define DMAMAP_REQ_GEN1_0      DMAMAP_MAP(DMA1, DMAMUX1_REQ_GEN1)
#define DMAMAP_REQ_GEN1_1      DMAMAP_MAP(DMA2, DMAMUX1_REQ_GEN1)
#define DMAMAP_REQ_GEN2_0      DMAMAP_MAP(DMA1, DMAMUX1_REQ_GEN2)
#define DMAMAP_REQ_GEN2_1      DMAMAP_MAP(DMA2, DMAMUX1_REQ_GEN2)
#define DMAMAP_REQ_GEN3_0      DMAMAP_MAP(DMA1, DMAMUX1_REQ_GEN3)
#define DMAMAP_REQ_GEN3_1      DMAMAP_MAP(DMA2, DMAMUX1_REQ_GEN3)
#define DMAMAP_ADC1_0          DMAMAP_MAP(DMA1, DMAMUX1_ADC1)
#define DMAMAP_ADC1_1          DMAMAP_MAP(DMA2, DMAMUX1_ADC1)
#define DMAMAP_SPI1_RX_0       DMAMAP_MAP(DMA1, DMAMUX1_SPI1_RX)
#define DMAMAP_SPI1_RX_1       DMAMAP_MAP(DMA2, DMAMUX1_SPI1_RX)
#define DMAMAP_SPI1_TX_0       DMAMAP_MAP(DMA1, DMAMUX1_SPI1_TX)
#define DMAMAP_SPI1_TX_1       DMAMAP_MAP(DMA2, DMAMUX1_SPI1_TX)
#define DMAMAP_SPI2_RX_0       DMAMAP_MAP(DMA1, DMAMUX1_SPI2_RX)
#define DMAMAP_SPI2_RX_1       DMAMAP_MAP(DMA2, DMAMUX1_SPI2_RX)
#define DMAMAP_SPI2_TX_0       DMAMAP_MAP(DMA1, DMAMUX1_SPI2_TX)
#define DMAMAP_SPI2_TX_1       DMAMAP_MAP(DMA2, DMAMUX1_SPI2_TX)
#define DMAMAP_I2C1_RX_0       DMAMAP_MAP(DMA1, DMAMUX1_I2C1_RX)
#define DMAMAP_I2C1_RX_1       DMAMAP_MAP(DMA2, DMAMUX1_I2C1_RX)
#define DMAMAP_I2C1_TX_0       DMAMAP_MAP(DMA1, DMAMUX1_I2C1_TX)
#define DMAMAP_I2C1_TX_1       DMAMAP_MAP(DMA2, DMAMUX1_I2C1_TX)
#define DMAMAP_I2C3_RX_0       DMAMAP_MAP(DMA1, DMAMUX1_I2C3_RX)
#define DMAMAP_I2C3_RX_1       DMAMAP_MAP(DMA2, DMAMUX1_I2C3_RX)
#define DMAMAP_I2C3_TX_0       DMAMAP_MAP(DMA1, DMAMUX1_I2C3_TX)
#define DMAMAP_I2C3_TX_1       DMAMAP_MAP(DMA2, DMAMUX1_I2C3_TX)
#define DMAMAP_USART1_RX_0     DMAMAP_MAP(DMA1, DMAMUX1_USART1_RX)
#define DMAMAP_USART1_RX_1     DMAMAP_MAP(DMA2, DMAMUX1_USART1_RX)
#define DMAMAP_USART1_TX_0     DMAMAP_MAP(DMA1, DMAMUX1_USART1_TX)
#define DMAMAP_USART1_TX_1     DMAMAP_MAP(DMA2, DMAMUX1_USART1_TX)
#define DMAMAP_LPUART1_RX_0    DMAMAP_MAP(DMA1, DMAMUX1_LPUART1_RX)
#define DMAMAP_LPUART1_RX_1    DMAMAP_MAP(DMA2, DMAMUX1_LPUART1_RX)
#define DMAMAP_LPUART1_TX_0    DMAMAP_MAP(DMA1, DMAMUX1_LPUART1_TX)
#define DMAMAP_LPUART1_TX_1    DMAMAP_MAP(DMA2, DMAMUX1_LPUART1_TX)
#define DMAMAP_SAI1_A_0        DMAMAP_MAP(DMA1, DMAMUX1_SAI1_A)
#define DMAMAP_SAI1_A_1        DMAMAP_MAP(DMA2, DMAMUX1_SAI1_A)
#define DMAMAP_SAI1_B_0        DMAMAP_MAP(DMA1, DMAMUX1_SAI1_B)
#define DMAMAP_SAI1_B_1        DMAMAP_MAP(DMA2, DMAMUX1_SAI1_B)
#define DMAMAP_QSPI_0          DMAMAP_MAP(DMA1, DMAMUX1_QSPI)
#define DMAMAP_QSPI_1          DMAMAP_MAP(DMA2, DMAMUX1_QSPI)
#define DMAMAP_TIM1_CH1_0      DMAMAP_MAP(DMA1, DMAMUX1_TIM1_CH1)
#define DMAMAP_TIM1_CH1_1      DMAMAP_MAP(DMA2, DMAMUX1_TIM1_CH1)
#define DMAMAP_TIM1_CH2_0      DMAMAP_MAP(DMA1, DMAMUX1_TIM1_CH2)
#define DMAMAP_TIM1_CH2_1      DMAMAP_MAP(DMA2, DMAMUX1_TIM1_CH2)
#define DMAMAP_TIM1_CH3_0      DMAMAP_MAP(DMA1, DMAMUX1_TIM1_CH3)
#define DMAMAP_TIM1_CH3_1      DMAMAP_MAP(DMA2, DMAMUX1_TIM1_CH3)
#define DMAMAP_TIM1_CH4_0      DMAMAP_MAP(DMA1, DMAMUX1_TIM1_CH4)
#define DMAMAP_TIM1_CH4_1      DMAMAP_MAP(DMA2, DMAMUX1_TIM1_CH4)
#define DMAMAP_TIM1_UP_0       DMAMAP_MAP(DMA1, DMAMUX1_TIM1_UP)
#define DMAMAP_TIM1_UP_1       DMAMAP_MAP(DMA2, DMAMUX1_TIM1_UP)
#define DMAMAP_TIM1_TRIG_0     DMAMAP_MAP(DMA1, DMAMUX1_TIM1_TRIG)
#define DMAMAP_TIM1_TRIG_1     DMAMAP_MAP(DMA2, DMAMUX1_TIM1_TRIG)
#define DMAMAP_TIM1_COM_0      DMAMAP_MAP(DMA1, DMAMUX1_TIM1_COM)
#define DMAMAP_TIM1_COM_1      DMAMAP_MAP(DMA2, DMAMUX1_TIM1_COM)
#define DMAMAP_TIM2_CH1_0      DMAMAP_MAP(DMA1, DMAMUX1_TIM2_CH1)
#define DMAMAP_TIM2_CH1_1      DMAMAP_MAP(DMA2, DMAMUX1_TIM2_CH1)
#define DMAMAP_TIM2_CH2_0      DMAMAP_MAP(DMA1, DMAMUX1_TIM2_CH2)
#define DMAMAP_TIM2_CH2_1      DMAMAP_MAP(DMA2, DMAMUX1_TIM2_CH2)
#define DMAMAP_TIM2_CH3_0      DMAMAP_MAP(DMA1, DMAMUX1_TIM2_CH3)
#define DMAMAP_TIM2_CH3_1      DMAMAP_MAP(DMA2, DMAMUX1_TIM2_CH3)
#define DMAMAP_TIM2_CH4_0      DMAMAP_MAP(DMA1, DMAMUX1_TIM2_CH4)
#define DMAMAP_TIM2_CH4_1      DMAMAP_MAP(DMA2, DMAMUX1_TIM2_CH4)
#define DMAMAP_TIM2_UP_0       DMAMAP_MAP(DMA1, DMAMUX1_TIM2_UP)
#define DMAMAP_TIM2_UP_1       DMAMAP_MAP(DMA2, DMAMUX1_TIM2_UP)
#define DMAMAP_TIM16_CH1_0     DMAMAP_MAP(DMA1, DMAMUX1_TIM16_CH1)
#define DMAMAP_TIM16_CH1_1     DMAMAP_MAP(DMA2, DMAMUX1_TIM16_CH1)
#define DMAMAP_TIM16_UP_0      DMAMAP_MAP(DMA1, DMAMUX1_TIM16_UP)
#define DMAMAP_TIM16_UP_1      DMAMAP_MAP(DMA2, DMAMUX1_TIM16_UP)
#define DMAMAP_TIM17_CH1_0     DMAMAP_MAP(DMA1, DMAMUX1_TIM17_CH1)
#define DMAMAP_TIM17_CH1_1     DMAMAP_MAP(DMA2, DMAMUX1_TIM17_CH1)
#define DMAMAP_TIM17_UP_0      DMAMAP_MAP(DMA1, DMAMUX1_TIM17_UP)
#define DMAMAP_TIM17_UP_1      DMAMAP_MAP(DMA2, DMAMUX1_TIM17_UP)
#define DMAMAP_AES1_IN_0       DMAMAP_MAP(DMA1, DMAMUX1_AES1_IN)
#define DMAMAP_AES1_IN_1       DMAMAP_MAP(DMA2, DMAMUX1_AES1_IN)
#define DMAMAP_AES1_OUT_0      DMAMAP_MAP(DMA1, DMAMUX1_AES1_OUT)
#define DMAMAP_AES1_OUT_1      DMAMAP_MAP(DMA2, DMAMUX1_AES1_OUT)
#define DMAMAP_AES2_IN_0       DMAMAP_MAP(DMA1, DMAMUX1_AES2_IN)
#define DMAMAP_AES2_IN_1       DMAMAP_MAP(DMA2, DMAMUX1_AES2_IN)
#define DMAMAP_AES2_OUT_0      DMAMAP_MAP(DMA1, DMAMUX1_AES2_OUT)
#define DMAMAP_AES2_OUT_1      DMAMAP_MAP(DMA2, DMAMUX1_AES2_OUT)

#endif /* __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_DMAMUX_H */
