/****************************************************************************
 * arch/risc-v/src/bl602/hardware/bl602_gpip.h
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

#ifndef __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_GPIP_H
#define __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_GPIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "bl602_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define BL602_GPADC_CONFIG_OFFSET          0x000000  /* gpadc_config */
#define BL602_GPADC_DMA_RDATA_OFFSET       0x000004  /* gpadc_dma_rdata */
#define BL602_GPDAC_CONFIG_OFFSET          0x000040  /* gpdac_config */
#define BL602_GPDAC_DMA_CONFIG_OFFSET      0x000044  /* gpdac_dma_config */
#define BL602_GPDAC_DMA_WDATA_OFFSET       0x000048  /* gpdac_dma_wdata */
#define BL602_GPDAC_TX_FIFO_STATUS_OFFSET  0x00004c  /* gpdac_tx_fifo_status */

/* Register definitions *****************************************************/

#define BL602_GPADC_CONFIG          (BL602_GPIP_BASE + BL602_GPADC_CONFIG_OFFSET)
#define BL602_GPADC_DMA_RDATA       (BL602_GPIP_BASE + BL602_GPADC_DMA_RDATA_OFFSET)
#define BL602_GPDAC_CONFIG          (BL602_GPIP_BASE + BL602_GPDAC_CONFIG_OFFSET)
#define BL602_GPDAC_DMA_CONFIG      (BL602_GPIP_BASE + BL602_GPDAC_DMA_CONFIG_OFFSET)
#define BL602_GPDAC_DMA_WDATA       (BL602_GPIP_BASE + BL602_GPDAC_DMA_WDATA_OFFSET)
#define BL602_GPDAC_TX_FIFO_STATUS  (BL602_GPIP_BASE + BL602_GPDAC_TX_FIFO_STATUS_OFFSET)

/* Register bit definitions *************************************************/

#define GPADC_CONFIG_RSVD_31_24_SHIFT             (24)
#define GPADC_CONFIG_RSVD_31_24_MASK              (0xff << GPADC_CONFIG_RSVD_31_24_SHIFT)
#define GPADC_CONFIG_GPADC_FIFO_THL_SHIFT         (22)
#define GPADC_CONFIG_GPADC_FIFO_THL_MASK          (0x03 << GPADC_CONFIG_GPADC_FIFO_THL_SHIFT)
#define GPADC_CONFIG_GPADC_FIFO_DATA_COUNT_SHIFT  (16)
#define GPADC_CONFIG_GPADC_FIFO_DATA_COUNT_MASK   (0x3f << GPADC_CONFIG_GPADC_FIFO_DATA_COUNT_SHIFT)
#define GPADC_CONFIG_GPADC_FIFO_UNDERRUN_MASK     (1 << 14)
#define GPADC_CONFIG_GPADC_FIFO_OVERRUN_MASK      (1 << 13)
#define GPADC_CONFIG_GPADC_RDY_MASK               (1 << 12)
#define GPADC_CONFIG_GPADC_FIFO_UNDERRUN_CLR      (1 << 10)
#define GPADC_CONFIG_GPADC_FIFO_OVERRUN_CLR       (1 << 9)
#define GPADC_CONFIG_GPADC_RDY_CLR                (1 << 8)
#define GPADC_CONFIG_GPADC_FIFO_UNDERRUN          (1 << 6)
#define GPADC_CONFIG_GPADC_FIFO_OVERRUN           (1 << 5)
#define GPADC_CONFIG_GPADC_RDY                    (1 << 4)
#define GPADC_CONFIG_GPADC_FIFO_FULL              (1 << 3)
#define GPADC_CONFIG_GPADC_FIFO_NE                (1 << 2)
#define GPADC_CONFIG_GPADC_FIFO_CLR               (1 << 1)
#define GPADC_CONFIG_GPADC_DMA_EN                 (1 << 0)

#define GPADC_DMA_RDATA_RSVD_31_26_SHIFT          (26)
#define GPADC_DMA_RDATA_RSVD_31_26_MASK           (0x3f << GPADC_DMA_RDATA_RSVD_31_26_SHIFT)
#define GPADC_DMA_RDATA_MASK                      (0x3ffffff)

#define GPDAC_CONFIG_RSVD_31_24_SHIFT             (24)
#define GPDAC_CONFIG_RSVD_31_24_MASK              (0xff << GPDAC_CONFIG_RSVD_31_24_SHIFT)
#define GPDAC_CONFIG_GPDAC_CH_B_SEL_SHIFT         (20)
#define GPDAC_CONFIG_GPDAC_CH_B_SEL_MASK          (0x0f << GPDAC_CONFIG_GPDAC_CH_B_SEL_SHIFT)
#define GPDAC_CONFIG_GPDAC_CH_A_SEL_SHIFT         (16)
#define GPDAC_CONFIG_GPDAC_CH_A_SEL_MASK          (0x0f << GPDAC_CONFIG_GPDAC_CH_A_SEL_SHIFT)
#define GPDAC_CONFIG_GPDAC_MODE_SHIFT             (8)
#define GPDAC_CONFIG_GPDAC_MODE_MASK              (0x07 << GPDAC_CONFIG_GPDAC_MODE_SHIFT)
#define GPDAC_CONFIG_DSM_MODE_SHIFT               (4)
#define GPDAC_CONFIG_DSM_MODE_MASK                (0x03 << GPDAC_CONFIG_DSM_MODE_SHIFT)
#define GPDAC_CONFIG_GPDAC_EN2                    (1 << 1)
#define GPDAC_CONFIG_GPDAC_EN                     (1 << 0)

#define GPDAC_DMA_CONFIG_GPDAC_DMA_FORMAT_SHIFT   (4)
#define GPDAC_DMA_CONFIG_GPDAC_DMA_FORMAT_MASK    (0x03 << GPDAC_DMA_CONFIG_GPDAC_DMA_FORMAT_SHIFT)
#define GPDAC_DMA_CONFIG_GPDAC_DMA_TX_EN          (1 << 0)

#define GPDAC_TX_FIFO_STATUS_TXFIFOWRPTR_SHIFT    (8)
#define GPDAC_TX_FIFO_STATUS_TXFIFOWRPTR_MASK     (0x03 << GPDAC_TX_FIFO_STATUS_TXFIFOWRPTR_SHIFT)
#define GPDAC_TX_FIFO_STATUS_TXFIFORDPTR_SHIFT    (4)
#define GPDAC_TX_FIFO_STATUS_TXFIFORDPTR_MASK     (0x07 << GPDAC_TX_FIFO_STATUS_TXFIFORDPTR_SHIFT)
#define GPDAC_TX_FIFO_STATUS_TX_CS_SHIFT          (2)
#define GPDAC_TX_FIFO_STATUS_TX_CS_MASK           (0x03 << GPDAC_TX_FIFO_STATUS_TX_CS_SHIFT)
#define GPDAC_TX_FIFO_STATUS_TX_FIFO_FULL         (1 << 1)
#define GPDAC_TX_FIFO_STATUS_TX_FIFO_EMPTY        (1 << 0)

#endif /* __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_GPIP_H */
