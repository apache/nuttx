/****************************************************************************
 * arch/arm64/src/imx9/hardware/imx93/imx93_dmamux.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX93_IMX93_DMAMUX_H
#define __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX93_IMX93_DMAMUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "imx93_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Identify channel MUX from 9th bit */

#define EDMA3_MUX_ID                0x0000
#define EDMA4_MUX_ID                0x0100
#define EDMA_MUX_ID_MASK            0xff00
#define EDMA_MUX_MASK               0x00ff

/* eDMA3 MUXs */

#define DMA_REQUEST_DISABLED        (0)                 /**< Channel disabled */
#define DMA_REQUEST_MUXCAN1         (1 | EDMA3_MUX_ID)  /**< CAN1 */
#define DMA_REQUEST_MUXGPIO1_0      (3 | EDMA3_MUX_ID)  /**< GPIO1 channel 0 */
#define DMA_REQUEST_MUXGPIO1_1      (4 | EDMA3_MUX_ID)  /**< GPIO1 channel 1 */
#define DMA_REQUEST_MUXI3C1TOBUS    (5 | EDMA3_MUX_ID)  /**< I3C1 To-bus Request */
#define DMA_REQUEST_MUXI3C1FROMBUS  (6 | EDMA3_MUX_ID)  /**< I3C1 From-bus Request */
#define DMA_REQUEST_MUXLPI2C1TX     (7 | EDMA3_MUX_ID)  /**< LPI2C1 */
#define DMA_REQUEST_MUXLPI2C1RX     (8 | EDMA3_MUX_ID)  /**< LPI2C1 */
#define DMA_REQUEST_MUXLPI2C2TX     (9 | EDMA3_MUX_ID)  /**< LPI2C2 */
#define DMA_REQUEST_MUXLPI2C2RX     (10 | EDMA3_MUX_ID) /**< LPI2C2 */
#define DMA_REQUEST_MUXLPSPI1TX     (11 | EDMA3_MUX_ID) /**< LPSPI1 Transmit */
#define DMA_REQUEST_MUXLPSPI1RX     (12 | EDMA3_MUX_ID) /**< LPSPI1 Receive */
#define DMA_REQUEST_MUXLPSPI2TX     (13 | EDMA3_MUX_ID) /**< LPSPI2 Transmit */
#define DMA_REQUEST_MUXLPSPI2RX     (14 | EDMA3_MUX_ID) /**< LPSPI2 Receive */
#define DMA_REQUEST_MUXLPTMR1       (15 | EDMA3_MUX_ID) /**< LPTMR1 Request */
#define DMA_REQUEST_MUXLPUART1TX    (16 | EDMA3_MUX_ID) /**< LPUART1 Transmit */
#define DMA_REQUEST_MUXLPUART1RX    (17 | EDMA3_MUX_ID) /**< LPUART1 Receive */
#define DMA_REQUEST_MUXLPUART2TX    (18 | EDMA3_MUX_ID) /**< LPUART2 Transmit */
#define DMA_REQUEST_MUXLPUART2RX    (19 | EDMA3_MUX_ID) /**< LPUART2 Receive */
#define DMA_REQUEST_MUXEDGELOCK     (20 | EDMA3_MUX_ID) /**< Edgelock enclave DMA Request */
#define DMA_REQUEST_MUXSAI1TX       (21 | EDMA3_MUX_ID) /**< SAI1 Transmit */
#define DMA_REQUEST_MUXSAI1RX       (22 | EDMA3_MUX_ID) /**< SAI1 Receive */
#define DMA_REQUEST_MUXTPM1_0_2     (23 | EDMA3_MUX_ID) /**< TPM1 request 0 and request 2 */
#define DMA_REQUEST_MUXTPM1_1_3     (24 | EDMA3_MUX_ID) /**< TPM1 request 1 and request 3 */
#define DMA_REQUEST_MUXTPM1OVERFLOW (25 | EDMA3_MUX_ID) /**< TPM1 Overflow request */
#define DMA_REQUEST_MUXTPM2_0_2     (26 | EDMA3_MUX_ID) /**< TPM2 request 0 and request 2 */
#define DMA_REQUEST_MUXTPM2_1_3     (27 | EDMA3_MUX_ID) /**< TPM2 request 1 and request 3 */
#define DMA_REQUEST_MUXTPM2OVERFLOW (28 | EDMA3_MUX_ID) /**< TPM2 Overflow request */
#define DMA_REQUEST_MUXPDM          (29 | EDMA3_MUX_ID) /**< PDM */
#define DMA_REQUEST_MUXADC1         (30 | EDMA3_MUX_ID) /**< ADC1 */

#define DMA3_REQUEST_MUX_COUNT      (31)

/* eDMA4 MUXs */

#define DMA_REQUEST_MUXCAN2         (1 | EDMA4_MUX_ID)  /**< CAN2 */
#define DMA_REQUEST_MUXGPIO2_0      (2 | EDMA4_MUX_ID)  /**< GPIO2 channel 0 */
#define DMA_REQUEST_MUXGPIO2_1      (3 | EDMA4_MUX_ID)  /**< GPIO2 channel 1 */
#define DMA_REQUEST_MUXGPIO3_0      (4 | EDMA4_MUX_ID)  /**< GPIO3 channel 0 */
#define DMA_REQUEST_MUXGPIO3_1      (5 | EDMA4_MUX_ID)  /**< GPIO3 channel 1 */
#define DMA_REQUEST_MUXI3C2TOBUS    (6 | EDMA4_MUX_ID)  /**< I3C2 To-bus Request */
#define DMA_REQUEST_MUXI3C2FROMBUS  (7 | EDMA4_MUX_ID)  /**< I3C2 From-bus Request */
#define DMA_REQUEST_MUXLPI2C3TX     (8 | EDMA4_MUX_ID)  /**< LPI2C3 */
#define DMA_REQUEST_MUXLPI2C3RX     (9 | EDMA4_MUX_ID)  /**< LPI2C3 */
#define DMA_REQUEST_MUXLPI2C4TX     (10 | EDMA4_MUX_ID) /**< LPI2C4 */
#define DMA_REQUEST_MUXLPI2C4RX     (11 | EDMA4_MUX_ID) /**< LPI2C4 */
#define DMA_REQUEST_MUXLPSPI3TX     (12 | EDMA4_MUX_ID) /**< LPSPI3 Transmit */
#define DMA_REQUEST_MUXLPSPI3RX     (13 | EDMA4_MUX_ID) /**< LPSPI3 Receive */
#define DMA_REQUEST_MUXLPSPI4TX     (14 | EDMA4_MUX_ID) /**< LPSPI4 Transmit */
#define DMA_REQUEST_MUXLPSPI4RX     (15 | EDMA4_MUX_ID) /**< LPSPI4 Receive */
#define DMA_REQUEST_MUXLPTMR2       (16 | EDMA4_MUX_ID) /**< LPTMR2 Request */
#define DMA_REQUEST_MUXLPUART3TX    (17 | EDMA4_MUX_ID) /**< LPUART3 Transmit */
#define DMA_REQUEST_MUXLPUART3RX    (18 | EDMA4_MUX_ID) /**< LPUART3 Receive */
#define DMA_REQUEST_MUXLPUART4TX    (19 | EDMA4_MUX_ID) /**< LPUART4 Transmit */
#define DMA_REQUEST_MUXLPUART4RX    (20 | EDMA4_MUX_ID) /**< LPUART4 Receive */
#define DMA_REQUEST_MUXLPUART5TX    (21 | EDMA4_MUX_ID) /**< LPUART5 Transmit */
#define DMA_REQUEST_MUXLPUART5RX    (22 | EDMA4_MUX_ID) /**< LPUART5 Receive */
#define DMA_REQUEST_MUXLPUART6TX    (23 | EDMA4_MUX_ID) /**< LPUART6 Transmit */
#define DMA_REQUEST_MUXLPUART6RX    (24 | EDMA4_MUX_ID) /**< LPUART6 Receive */
#define DMA_REQUEST_MUXTPM3_0_2     (25 | EDMA4_MUX_ID) /**< TPM3 request 0 and request 2 */
#define DMA_REQUEST_MUXTPM3_1_3     (26 | EDMA4_MUX_ID) /**< TPM3 request 1 and request 3 */
#define DMA_REQUEST_MUXTPM3OVERFLOW (27 | EDMA4_MUX_ID) /**< TPM3 Overflow request */
#define DMA_REQUEST_MUXTPM4_0_2     (28 | EDMA4_MUX_ID) /**< TPM4 request 0 and request 2 */
#define DMA_REQUEST_MUXTPM4_1_3     (29 | EDMA4_MUX_ID) /**< TPM4 request 1 and request 3 */
#define DMA_REQUEST_MUXTPM4OVERFLOW (30 | EDMA4_MUX_ID) /**< TPM4 Overflow request */
#define DMA_REQUEST_MUXTPM5_0_2     (31 | EDMA4_MUX_ID) /**< TPM5 request 0 and request 2 */
#define DMA_REQUEST_MUXTPM5_1_3     (32 | EDMA4_MUX_ID) /**< TPM5 request 1 and request 3 */
#define DMA_REQUEST_MUXTPM5OVERFLOW (33 | EDMA4_MUX_ID) /**< TPM5 Overflow request */
#define DMA_REQUEST_MUXTPM6_0_2     (34 | EDMA4_MUX_ID) /**< TPM6 request 0 and request 2 */
#define DMA_REQUEST_MUXTPM6_1_3     (35 | EDMA4_MUX_ID) /**< TPM6 request 1 and request 3 */
#define DMA_REQUEST_MUXTPM6OVERFLOW (36 | EDMA4_MUX_ID) /**< TPM6 Overflow request */
#define DMA_REQUEST_MUXFLEXIO1_0    (37 | EDMA4_MUX_ID) /**< FlexIO1 Request0 */
#define DMA_REQUEST_MUXFLEXIO1_1    (38 | EDMA4_MUX_ID) /**< FlexIO1 Request1 */
#define DMA_REQUEST_MUXFLEXIO1_2    (39 | EDMA4_MUX_ID) /**< FlexIO1 Request2 */
#define DMA_REQUEST_MUXFLEXIO1_3    (40 | EDMA4_MUX_ID) /**< FlexIO1 Request3 */
#define DMA_REQUEST_MUXFLEXIO1_4    (41 | EDMA4_MUX_ID) /**< FlexIO1 Request4 */
#define DMA_REQUEST_MUXFLEXIO1_5    (42 | EDMA4_MUX_ID) /**< FlexIO1 Request5 */
#define DMA_REQUEST_MUXFLEXIO1_6    (43 | EDMA4_MUX_ID) /**< FlexIO1 Request6 */
#define DMA_REQUEST_MUXFLEXIO1_7    (44 | EDMA4_MUX_ID) /**< FlexIO1 Request7 */
#define DMA_REQUEST_MUXFLEXIO2_0    (45 | EDMA4_MUX_ID) /**< FlexIO2 Request0 */
#define DMA_REQUEST_MUXFLEXIO2_1    (46 | EDMA4_MUX_ID) /**< FlexIO2 Request1 */
#define DMA_REQUEST_MUXFLEXIO2_2    (47 | EDMA4_MUX_ID) /**< FlexIO2 Request2 */
#define DMA_REQUEST_MUXFLEXIO2_3    (48 | EDMA4_MUX_ID) /**< FlexIO2 Request3 */
#define DMA_REQUEST_MUXFLEXIO2_4    (49 | EDMA4_MUX_ID) /**< FlexIO2 Request4 */
#define DMA_REQUEST_MUXFLEXIO2_5    (50 | EDMA4_MUX_ID) /**< FlexIO2 Request5 */
#define DMA_REQUEST_MUXFLEXIO2_6    (51 | EDMA4_MUX_ID) /**< FlexIO2 Request6 */
#define DMA_REQUEST_MUXFLEXIO2_7    (52 | EDMA4_MUX_ID) /**< FlexIO2 Request7 */
#define DMA_REQUEST_MUXFLEXSPI1TX   (53 | EDMA4_MUX_ID) /**< FlexSPI1 Transmit */
#define DMA_REQUEST_MUXFLEXSPI1RX   (54 | EDMA4_MUX_ID) /**< FlexSPI1 Receive */
#define DMA_REQUEST_MUXSAI2TX       (58 | EDMA4_MUX_ID) /**< SAI2 Transmit */
#define DMA_REQUEST_MUXSAI2RX       (59 | EDMA4_MUX_ID) /**< SAI2 Receive */
#define DMA_REQUEST_MUXSAI3TX       (60 | EDMA4_MUX_ID) /**< SAI3 Transmit */
#define DMA_REQUEST_MUXSAI3RX       (61 | EDMA4_MUX_ID) /**< SAI3 Receive */
#define DMA_REQUEST_MUXGPIO4_0      (62 | EDMA4_MUX_ID) /**< GPIO4 channel 0 */
#define DMA_REQUEST_MUXGPIO4_1      (63 | EDMA4_MUX_ID) /**< GPIO4 channel 1 */
#define DMA_REQUEST_MUXSPDIF        (65 | EDMA4_MUX_ID) /**< SPDIF */
#define DMA_REQUEST_MUXSPDIF_1      (66 | EDMA4_MUX_ID) /**< SPDIF */
#define DMA_REQUEST_MUXENET         (67 | EDMA4_MUX_ID) /**< ENET */
#define DMA_REQUEST_MUXENET_1       (68 | EDMA4_MUX_ID) /**< ENET */
#define DMA_REQUEST_MUXENET_2       (69 | EDMA4_MUX_ID) /**< ENET */
#define DMA_REQUEST_MUXENET_3       (70 | EDMA4_MUX_ID) /**< ENET */
#define DMA_REQUEST_MUXLPI2C5TX     (71 | EDMA4_MUX_ID) /**< LPI2C5 */
#define DMA_REQUEST_MUXLPI2C5RX     (72 | EDMA4_MUX_ID) /**< LPI2C5 */
#define DMA_REQUEST_MUXLPI2C6TX     (73 | EDMA4_MUX_ID) /**< LPI2C6 */
#define DMA_REQUEST_MUXLPI2C6RX     (74 | EDMA4_MUX_ID) /**< LPI2C6 */
#define DMA_REQUEST_MUXLPI2C7TX     (75 | EDMA4_MUX_ID) /**< LPI2C7 */
#define DMA_REQUEST_MUXLPI2C7RX     (76 | EDMA4_MUX_ID) /**< LPI2C7 */
#define DMA_REQUEST_MUXLPI2C8TX     (77 | EDMA4_MUX_ID) /**< LPI2C8 */
#define DMA_REQUEST_MUXLPI2C8RX     (78 | EDMA4_MUX_ID) /**< LPI2C8 */
#define DMA_REQUEST_MUXLPSPI5TX     (79 | EDMA4_MUX_ID) /**< LPSPI5 Transmit */
#define DMA_REQUEST_MUXLPSPI5RX     (80 | EDMA4_MUX_ID) /**< LPSPI5 Receive */
#define DMA_REQUEST_MUXLPSPI6TX     (81 | EDMA4_MUX_ID) /**< LPSPI6 Transmit */
#define DMA_REQUEST_MUXLPSPI6RX     (82 | EDMA4_MUX_ID) /**< LPSPI6 Receive */
#define DMA_REQUEST_MUXLPSPI7TX     (83 | EDMA4_MUX_ID) /**< LPSPI7 Transmit */
#define DMA_REQUEST_MUXLPSPI7RX     (84 | EDMA4_MUX_ID) /**< LPSPI7 Receive */
#define DMA_REQUEST_MUXLPSPI8TX     (85 | EDMA4_MUX_ID) /**< LPSPI8 Transmit */
#define DMA_REQUEST_MUXLPSPI8RX     (86 | EDMA4_MUX_ID) /**< LPSPI8 Receive */
#define DMA_REQUEST_MUXLPUART7TX    (87 | EDMA4_MUX_ID) /**< LPUART7 Transmit */
#define DMA_REQUEST_MUXLPUART7RX    (88 | EDMA4_MUX_ID) /**< LPUART7 Receive */
#define DMA_REQUEST_MUXLPUART8TX    (89 | EDMA4_MUX_ID) /**< LPUART8 Transmit */
#define DMA_REQUEST_MUXLPUART8RX    (90 | EDMA4_MUX_ID) /**< LPUART8 Receive */
#define DMA_REQUEST_MUXENET_QOS     (91 | EDMA4_MUX_ID) /**< ENET_QOS */
#define DMA_REQUEST_MUXENET_QOS_1   (92 | EDMA4_MUX_ID) /**< ENET_QOS */
#define DMA_REQUEST_MUXENET_QOS_2   (93 | EDMA4_MUX_ID) /**< ENET_QOS */
#define DMA_REQUEST_MUXENET_QOS_3   (94 | EDMA4_MUX_ID) /**< ENET_QOS */

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
      return IMX9_DMA4_BASE;
    }
}

#endif /* __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX93_IMX93_DMAMUX_H */
