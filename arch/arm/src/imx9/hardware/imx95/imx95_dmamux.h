/****************************************************************************
 * arch/arm/src/imx9/hardware/imx95/imx95_dmamux.h
 *
 * SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2024 NXP
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

#ifndef __ARCH_ARM_SRC_IMX9_HARDWARE_IMX95_IMX95_DMAMUX_H
#define __ARCH_ARM_SRC_IMX9_HARDWARE_IMX95_IMX95_DMAMUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "imx95_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Identify channel MUX from 9th bit */

#define EDMA3_MUX_ID                0x0000
#define EDMA4_MUX_ID                0x0100
#define EDMA_MUX_ID_MASK            0xff00
#define EDMA_MUX_MASK               0x00ff

/* eDMA3 MUXs */

#define DMA_REQUEST_DISABLED                (0 | EDMA3_MUX_ID)  /**< DSisabled*/
#define DMA_REQUEST_MUXCAN1                 (1 | EDMA3_MUX_ID)  /**< CAN1 */
#define DMA_REQUEST_MUXLPTMR1REQUEST        (2 | EDMA3_MUX_ID)  /**< LPTMR1 Request */
#define DMA_REQUEST_MUXELEREQUEST           (3 | EDMA3_MUX_ID)  /**< ELE Request */
#define DMA_REQUEST_MUXTPM1OVERFLOWREQUEST  (4 | EDMA3_MUX_ID)  /**< TPM1 Overflow Request */
#define DMA_REQUEST_MUXTPM2OVERFLOWREQUEST  (5 | EDMA3_MUX_ID)  /**< TPM2 Overflow Request */
#define DMA_REQUEST_MUXPDMREQUEST           (6 | EDMA3_MUX_ID)  /**< PDM */
#define DMA_REQUEST_MUXADC1REQUEST          (7 | EDMA3_MUX_ID)  /**< ADC1 */
#define DMA_REQUEST_MUXGPIO1REQUEST0        (8 | EDMA3_MUX_ID)  /**< GPIO1 channel 0 */
#define DMA_REQUEST_MUXGPIO1REQUEST1        (9 | EDMA3_MUX_ID)  /**< GPIO1 channel 1 */
#define DMA_REQUEST_MUXI3C1TOBUSREQUEST     (10 | EDMA3_MUX_ID) /**< I3C1 To-bus Request */
#define DMA_REQUEST_MUXI3C1FROMBUSREQUEST   (11 | EDMA3_MUX_ID) /**< I3C1 From-bus Request */
#define DMA_REQUEST_MUXLPI2C1TX             (12 | EDMA3_MUX_ID) /**< LPI2C1 */
#define DMA_REQUEST_MUXLPI2C1RX             (13 | EDMA3_MUX_ID) /**< LPI2C1 */
#define DMA_REQUEST_MUXLPI2C2TX             (14 | EDMA3_MUX_ID) /**< LPI2C2 */
#define DMA_REQUEST_MUXLPI2C2RX             (15 | EDMA3_MUX_ID) /**< LPI2C2 */
#define DMA_REQUEST_MUXLPSPI1TX             (16 | EDMA3_MUX_ID) /**< LPSPI1 Transmit */
#define DMA_REQUEST_MUXLPSPI1RX             (17 | EDMA3_MUX_ID) /**< LPSPI1 Receive */
#define DMA_REQUEST_MUXLPSPI2TX             (18 | EDMA3_MUX_ID) /**< LPSPI2 Transmit */
#define DMA_REQUEST_MUXLPSPI2RX             (19 | EDMA3_MUX_ID) /**< LPSPI2 Receive */
#define DMA_REQUEST_MUXLPUART1TX            (20 | EDMA3_MUX_ID) /**< LPUART1 Transmit */
#define DMA_REQUEST_MUXLPUART1RX            (21 | EDMA3_MUX_ID) /**< LPUART1 Receive */
#define DMA_REQUEST_MUXLPUART2TX            (22 | EDMA3_MUX_ID) /**< LPUART2 Transmit */
#define DMA_REQUEST_MUXLPUART2RX            (23 | EDMA3_MUX_ID) /**< LPUART2 Receive */
#define DMA_REQUEST_MUXSAI1TX               (24 | EDMA3_MUX_ID) /**< SAI1 Transmit */
#define DMA_REQUEST_MUXSAI1RX               (25 | EDMA3_MUX_ID) /**< SAI1 Receive */
#define DMA_REQUEST_MUXTPM1REQUEST0REQUEST2 (26 | EDMA3_MUX_ID) /**< TPM1 request 0 and request 2 */
#define DMA_REQUEST_MUXTPM1REQUEST1REQUEST3 (27 | EDMA3_MUX_ID) /**< TPM1 request 1 and request 3 */
#define DMA_REQUEST_MUXTPM2REQUEST0REQUEST2 (28 | EDMA3_MUX_ID) /**< TPM2 request 0 and request 2 */
#define DMA_REQUEST_MUXTPM2REQUEST1REQUEST3 (29 | EDMA3_MUX_ID) /**< TPM2 request 1 and request 3 */
#define DMA3_REQUEST_MUX_COUNT      (30)

/* eDMA4 MUXs */

#define DMA_REQUEST_MUXCAN2                 (1U | EDMA4_MUX_ID)   /**< CAN2 */
#define DMA_REQUEST_MUXGPIO2REQUEST0        (2U | EDMA4_MUX_ID)   /**< GPIO2 channel 0 */
#define DMA_REQUEST_MUXGPIO2REQUEST1        (3U | EDMA4_MUX_ID)   /**< GPIO2 channel 1 */
#define DMA_REQUEST_MUXGPIO3REQUEST0        (4U | EDMA4_MUX_ID)   /**< GPIO3 channel 0 */
#define DMA_REQUEST_MUXGPIO3REQUEST1        (5U | EDMA4_MUX_ID)   /**< GPIO3 channel 1 */
#define DMA_REQUEST_MUXI3C2TOBUSREQUEST     (6U | EDMA4_MUX_ID)   /**< I3C2 To-bus Request */
#define DMA_REQUEST_MUXI3C2FROMBUSREQUEST   (7U | EDMA4_MUX_ID)   /**< I3C2 From-bus Request */
#define DMA_REQUEST_MUXLPI2C3TX             (8U | EDMA4_MUX_ID)   /**< LPI2C3 */
#define DMA_REQUEST_MUXLPI2C3RX             (9U | EDMA4_MUX_ID)   /**< LPI2C3 */
#define DMA_REQUEST_MUXLPI2C4TX             (10U | EDMA4_MUX_ID)  /**< LPI2C4 */
#define DMA_REQUEST_MUXLPI2C4RX             (11U | EDMA4_MUX_ID)  /**< LPI2C2 */
#define DMA_REQUEST_MUXLPSPI3TX             (12U | EDMA4_MUX_ID)  /**< LPSPI3 Transmit */
#define DMA_REQUEST_MUXLPSPI3RX             (13U | EDMA4_MUX_ID)  /**< LPSPI3 Receive */
#define DMA_REQUEST_MUXLPSPI4TX             (14U | EDMA4_MUX_ID)  /**< LPSPI4 Transmit */
#define DMA_REQUEST_MUXLPSPI4RX             (15U | EDMA4_MUX_ID)  /**< LPSPI4 Receive */
#define DMA_REQUEST_MUXLPTMR2REQUEST        (16U | EDMA4_MUX_ID)  /**< LPTMR2 Request */
#define DMA_REQUEST_MUXLPUART3TX            (17U | EDMA4_MUX_ID)  /**< LPUART3 Transmit */
#define DMA_REQUEST_MUXLPUART3RX            (18U | EDMA4_MUX_ID)  /**< LPUART3 Receive */
#define DMA_REQUEST_MUXLPUART4TX            (19U | EDMA4_MUX_ID)  /**< LPUART4 Transmit */
#define DMA_REQUEST_MUXLPUART4RX            (20U | EDMA4_MUX_ID)  /**< LPUART4 Receive */
#define DMA_REQUEST_MUXLPUART5TX            (21U | EDMA4_MUX_ID)  /**< LPUART5 Transmit */
#define DMA_REQUEST_MUXLPUART5RX            (22U | EDMA4_MUX_ID)  /**< LPUART5 Receive */
#define DMA_REQUEST_MUXLPUART6TX            (23U | EDMA4_MUX_ID)  /**< LPUART6 Transmit */
#define DMA_REQUEST_MUXLPUART6RX            (24U | EDMA4_MUX_ID)  /**< LPUART6 Receive */
#define DMA_REQUEST_MUXTPM3REQUEST0REQUEST2 (25U | EDMA4_MUX_ID)  /**< TPM3 request 0 and request 2 */
#define DMA_REQUEST_MUXTPM3REQUEST1REQUEST3 (26U | EDMA4_MUX_ID)  /**< TPM3 request 1 and request 3 */
#define DMA_REQUEST_MUXTPM3OVERFLOWREQUEST  (27U | EDMA4_MUX_ID)  /**< TPM3 Overflow request */
#define DMA_REQUEST_MUXTPM4REQUEST0REQUEST2 (28U | EDMA4_MUX_ID)  /**< TPM4 request 0 and request 2 */
#define DMA_REQUEST_MUXTPM4REQUEST1REQUEST3 (29U | EDMA4_MUX_ID)  /**< TPM4 request 1 and request 3 */
#define DMA_REQUEST_MUXTPM4OVERFLOWREQUEST  (30U | EDMA4_MUX_ID)  /**< TPM4 Overflow request */
#define DMA_REQUEST_MUXTPM5REQUEST0REQUEST2 (31U | EDMA4_MUX_ID)  /**< TPM5 request 0 and request 2 */
#define DMA_REQUEST_MUXTPM5REQUEST1REQUEST3 (32U | EDMA4_MUX_ID)  /**< TPM5 request 1 and request 3 */
#define DMA_REQUEST_MUXTPM5OVERFLOWREQUEST  (33U | EDMA4_MUX_ID)  /**< TPM5 Overflow request */
#define DMA_REQUEST_MUXTPM6REQUEST0REQUEST2 (34U | EDMA4_MUX_ID)  /**< TPM6 request 0 and request 2 */
#define DMA_REQUEST_MUXTPM6REQUEST1REQUEST3 (35U | EDMA4_MUX_ID)  /**< TPM6 request 1 and request 3 */
#define DMA_REQUEST_MUXTPM6OVERFLOWREQUEST  (36U | EDMA4_MUX_ID)  /**< TPM6 Overflow request */
#define DMA_REQUEST_MUXFLEXIO1REQUEST0      (37U | EDMA4_MUX_ID)  /**< FlexIO1 Request0 */
#define DMA_REQUEST_MUXFLEXIO1REQUEST1      (38U | EDMA4_MUX_ID)  /**< FlexIO1 Request1 */
#define DMA_REQUEST_MUXFLEXIO1REQUEST2      (39U | EDMA4_MUX_ID)  /**< FlexIO1 Request2 */
#define DMA_REQUEST_MUXFLEXIO1REQUEST3      (40U | EDMA4_MUX_ID)  /**< FlexIO1 Request3 */
#define DMA_REQUEST_MUXFLEXIO1REQUEST4      (41U | EDMA4_MUX_ID)  /**< FlexIO1 Request4 */
#define DMA_REQUEST_MUXFLEXIO1REQUEST5      (42U | EDMA4_MUX_ID)  /**< FlexIO1 Request5 */
#define DMA_REQUEST_MUXFLEXIO1REQUEST6      (43U | EDMA4_MUX_ID)  /**< FlexIO1 Request6 */
#define DMA_REQUEST_MUXFLEXIO1REQUEST7      (44U | EDMA4_MUX_ID)  /**< FlexIO1 Request7 */
#define DMA_REQUEST_MUXFLEXIO2REQUEST0      (45U | EDMA4_MUX_ID)  /**< FlexIO2 Request0 */
#define DMA_REQUEST_MUXFLEXIO2REQUEST1      (46U | EDMA4_MUX_ID)  /**< FlexIO2 Request1 */
#define DMA_REQUEST_MUXFLEXIO2REQUEST2      (47U | EDMA4_MUX_ID)  /**< FlexIO2 Request2 */
#define DMA_REQUEST_MUXFLEXIO2REQUEST3      (48U | EDMA4_MUX_ID)  /**< FlexIO2 Request3 */
#define DMA_REQUEST_MUXFLEXIO2REQUEST4      (49U | EDMA4_MUX_ID)  /**< FlexIO2 Request4 */
#define DMA_REQUEST_MUXFLEXIO2REQUEST5      (50U | EDMA4_MUX_ID)  /**< FlexIO2 Request5 */
#define DMA_REQUEST_MUXFLEXIO2REQUEST6      (51U | EDMA4_MUX_ID)  /**< FlexIO2 Request6 */
#define DMA_REQUEST_MUXFLEXIO2REQUEST7      (52U | EDMA4_MUX_ID)  /**< FlexIO2 Request7 */
#define DMA_REQUEST_MUXFLEXSPI1TX           (53U | EDMA4_MUX_ID)  /**< FlexSPI1 Transmit */
#define DMA_REQUEST_MUXFLEXSPI1RX           (54U | EDMA4_MUX_ID)  /**< FlexSPI1 Receive */
#define DMA_REQUEST_MUXGPIO5REQUEST0        (53U | EDMA4_MUX_ID)  /**< GPIO5 Request0 */
#define DMA_REQUEST_MUXGPIO5REQUEST1        (54U | EDMA4_MUX_ID)  /**< GPIO5 Request1 */
#define DMA_REQUEST_MUXCAN3                 (57U | EDMA4_MUX_ID)  /**< CAN3 */
#define DMA_REQUEST_MUXSAI2TX               (58U | EDMA4_MUX_ID)  /**< SAI2 Transmit */
#define DMA_REQUEST_MUXSAI2RX               (59U | EDMA4_MUX_ID)  /**< SAI2 Receive */
#define DMA_REQUEST_MUXSAI3TX               (60U | EDMA4_MUX_ID)  /**< SAI3 Transmit */
#define DMA_REQUEST_MUXSAI3RX               (61U | EDMA4_MUX_ID)  /**< SAI3 Receive */
#define DMA_REQUEST_MUXGPIO4REQUEST0        (62U | EDMA4_MUX_ID)  /**< GPIO4 Request0 */
#define DMA_REQUEST_MUXGPIO4REQUEST1        (63U | EDMA4_MUX_ID)  /**< GPIO4 Request1 */
#define DMA_REQUEST_MUXEARCREQUEST0         (65U | EDMA4_MUX_ID)  /**< eARC enhanced Audio Return Channel */
#define DMA_REQUEST_MUXEARCREQUEST1         (66U | EDMA4_MUX_ID)  /**< eARC enhanced Audio Return Channel */
#define DMA_REQUEST_MUXSAI4TX               (67U | EDMA4_MUX_ID)  /**< SAI4 Transmit */
#define DMA_REQUEST_MUXSAI4RX               (68U | EDMA4_MUX_ID)  /**< SAI4 Receive */
#define DMA_REQUEST_MUXSAI5TX               (69U | EDMA4_MUX_ID)  /**< SAI5 Transmit */
#define DMA_REQUEST_MUXSAI5RX               (70U | EDMA4_MUX_ID)  /**< SAI5 Receive */
#define DMA_REQUEST_MUXLPI2C5TX             (71U | EDMA4_MUX_ID)  /**< LPI2C5 */
#define DMA_REQUEST_MUXLPI2C5RX             (72U | EDMA4_MUX_ID)  /**< LPI2C5 */
#define DMA_REQUEST_MUXLPI2C6TX             (73U | EDMA4_MUX_ID)  /**< LPI2C6 */
#define DMA_REQUEST_MUXLPI2C6RX             (74U | EDMA4_MUX_ID)  /**< LPI2C6 */
#define DMA_REQUEST_MUXLPI2C7TX             (75U | EDMA4_MUX_ID)  /**< LPI2C7 */
#define DMA_REQUEST_MUXLPI2C7RX             (76U | EDMA4_MUX_ID)  /**< LPI2C7 */
#define DMA_REQUEST_MUXLPI2C8TX             (77U | EDMA4_MUX_ID)  /**< LPI2C8 */
#define DMA_REQUEST_MUXLPI2C8RX             (78U | EDMA4_MUX_ID)  /**< LPI2C8 */
#define DMA_REQUEST_MUXLPSPI5TX             (79U | EDMA4_MUX_ID)  /**< LPSPI5 Transmit */
#define DMA_REQUEST_MUXLPSPI5RX             (80U | EDMA4_MUX_ID)  /**< LPSPI5 Receive */
#define DMA_REQUEST_MUXLPSPI6TX             (81U | EDMA4_MUX_ID)  /**< LPSPI6 Transmit */
#define DMA_REQUEST_MUXLPSPI6RX             (82U | EDMA4_MUX_ID)  /**< LPSPI6 Receive */
#define DMA_REQUEST_MUXLPSPI7TX             (83U | EDMA4_MUX_ID)  /**< LPSPI7 Transmit */
#define DMA_REQUEST_MUXLPSPI7RX             (84U | EDMA4_MUX_ID)  /**< LPSPI7 Receive */
#define DMA_REQUEST_MUXLPSPI8TX             (85U | EDMA4_MUX_ID)  /**< LPSPI8 Transmit */
#define DMA_REQUEST_MUXLPSPI8RX             (86U | EDMA4_MUX_ID)  /**< LPSPI8 Receive */
#define DMA_REQUEST_MUXLPUART7TX            (87U | EDMA4_MUX_ID)  /**< LPUART7 Transmit */
#define DMA_REQUEST_MUXLPUART7RX            (88U | EDMA4_MUX_ID)  /**< LPUART7 Receive */
#define DMA_REQUEST_MUXLPUART8TX            (89U | EDMA4_MUX_ID)  /**< LPUART8 Transmit */
#define DMA_REQUEST_MUXLPUART8RX            (90U | EDMA4_MUX_ID)  /**< LPUART8 Receive */
#define DMA_REQUEST_MUXCAN4                 (91U | EDMA4_MUX_ID)  /**< CAN4 */
#define DMA_REQUEST_MUXCAN5                 (92U | EDMA4_MUX_ID)  /**< CAN5 */

#define DMA4_REQUEST_MUX_COUNT      (95)

/* Combined MUX count (eDMA3 and eDMA4) */

#define DMA_REQUEST_MUX_COUNT       (DMA3_REQUEST_MUX_COUNT + DMA4_REQUEST_MUX_COUNT)

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_dmamux_get_dmabase
 *
 * Description:
 *   Get DMA engine base address from MUX identifier.
 *
 * Input Parameters:
 *   dmamux - The DMA MUX identifier.
 *
 * Returned Value:
 *   Base address of the associated DMA engine.
 *
 ****************************************************************************/

static inline uintptr_t imx9_dmamux_get_dmabase(uint16_t dmamux)
{
  if ((dmamux & EDMA_MUX_ID_MASK) == EDMA3_MUX_ID)
    {
      return IMX9_DMA3_BASE;
    }
  else
    {
      return IMX9_EDMA5_2_BASE;
    }
}

#endif /* __ARCH_ARM_SRC_IMX9_HARDWARE_IMX95_IMX95_DMAMUX_H */
