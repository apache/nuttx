/****************************************************************************
 * arch/risc-v/src/bl602/hardware/bl602_spi.h
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

#ifndef __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_SPI_H
#define __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "bl602_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define BL602_SPI_CFG_OFFSET            0x000000  /* spi_config */
#define BL602_SPI_INT_STS_OFFSET        0x000004  /* spi_int_sts */
#define BL602_SPI_BUS_BUSY_OFFSET       0x000008  /* spi_bus_busy */
#define BL602_SPI_PRD_0_OFFSET          0x000010  /* spi_prd_0 */
#define BL602_SPI_PRD_1_OFFSET          0x000014  /* spi_prd_1 */
#define BL602_SPI_RXD_IGNR_OFFSET       0x000018  /* spi_rxd_ignr */
#define BL602_SPI_STO_VALUE_OFFSET      0x00001c  /* spi_sto_value */
#define BL602_SPI_FIFO_CFG_0_OFFSET     0x000080  /* spi_fifo_config_0 */
#define BL602_SPI_FIFO_CFG_1_OFFSET     0x000084  /* spi_fifo_config_1 */
#define BL602_SPI_FIFO_WDATA_OFFSET     0x000088  /* spi_fifo_wdata */
#define BL602_SPI_FIFO_RDATA_OFFSET     0x00008c  /* spi_fifo_rdata */

/* Register definitions *****************************************************/

#define BL602_SPI_CFG            (BL602_SPI_BASE + BL602_SPI_CFG_OFFSET)
#define BL602_SPI_INT_STS        (BL602_SPI_BASE + BL602_SPI_INT_STS_OFFSET)
#define BL602_SPI_BUS_BUSY       (BL602_SPI_BASE + BL602_SPI_BUS_BUSY_OFFSET)
#define BL602_SPI_PRD_0          (BL602_SPI_BASE + BL602_SPI_PRD_0_OFFSET)
#define BL602_SPI_PRD_1          (BL602_SPI_BASE + BL602_SPI_PRD_1_OFFSET)
#define BL602_SPI_RXD_IGNR       (BL602_SPI_BASE + BL602_SPI_RXD_IGNR_OFFSET)
#define BL602_SPI_STO_VALUE      (BL602_SPI_BASE + BL602_SPI_STO_VALUE_OFFSET)
#define BL602_SPI_FIFO_CFG_0     (BL602_SPI_BASE + BL602_SPI_FIFO_CFG_0_OFFSET)
#define BL602_SPI_FIFO_CFG_1     (BL602_SPI_BASE + BL602_SPI_FIFO_CFG_1_OFFSET)
#define BL602_SPI_FIFO_WDATA     (BL602_SPI_BASE + BL602_SPI_FIFO_WDATA_OFFSET)
#define BL602_SPI_FIFO_RDATA     (BL602_SPI_BASE + BL602_SPI_FIFO_RDATA_OFFSET)

/* Register bit definitions *************************************************/

#define SPI_CFG_CR_DEG_CNT_SHIFT      (12)
#define SPI_CFG_CR_DEG_CNT_MASK       (0x0f << SPI_CFG_CR_DEG_CNT_SHIFT)
#define SPI_CFG_CR_DEG_EN             (1 << 11)
#define SPI_CFG_CR_M_CONT_EN          (1 << 9)
#define SPI_CFG_CR_RXD_IGNR_EN        (1 << 8)
#define SPI_CFG_CR_BYTE_INV           (1 << 7)
#define SPI_CFG_CR_BIT_INV            (1 << 6)
#define SPI_CFG_CR_SCLK_PH            (1 << 5)
#define SPI_CFG_CR_SCLK_POL           (1 << 4)
#define SPI_CFG_CR_FRAME_SIZE_SHIFT   (2)
#define SPI_CFG_CR_FRAME_SIZE_MASK    (0x03 << SPI_CFG_CR_FRAME_SIZE_SHIFT)
#define SPI_CFG_CR_S_EN               (1 << 1)
#define SPI_CFG_CR_M_EN               (1 << 0)

#define SPI_INT_STS_CR_FER_EN         (1 << 29)
#define SPI_INT_STS_CR_TXU_EN         (1 << 28)
#define SPI_INT_STS_CR_STO_EN         (1 << 27)
#define SPI_INT_STS_CR_RXF_EN         (1 << 26)
#define SPI_INT_STS_CR_TXF_EN         (1 << 25)
#define SPI_INT_STS_CR_END_EN         (1 << 24)
#define SPI_INT_STS_RSVD_21           (1 << 21)
#define SPI_INT_STS_CR_TXU_CLR        (1 << 20)
#define SPI_INT_STS_CR_STO_CLR        (1 << 19)
#define SPI_INT_STS_RSVD_18           (1 << 18)
#define SPI_INT_STS_RSVD_17           (1 << 17)
#define SPI_INT_STS_CR_END_CLR        (1 << 16)
#define SPI_INT_STS_CR_FER_MASK       (1 << 13)
#define SPI_INT_STS_CR_TXU_MASK       (1 << 12)
#define SPI_INT_STS_CR_STO_MASK       (1 << 11)
#define SPI_INT_STS_CR_RXF_MASK       (1 << 10)
#define SPI_INT_STS_CR_TXF_MASK       (1 << 9)
#define SPI_INT_STS_CR_END_MASK       (1 << 8)
#define SPI_INT_STS_FER_INT           (1 << 5)
#define SPI_INT_STS_TXU_INT           (1 << 4)
#define SPI_INT_STS_STO_INT           (1 << 3)
#define SPI_INT_STS_RXF_INT           (1 << 2)
#define SPI_INT_STS_TXF_INT           (1 << 1)
#define SPI_INT_STS_END_INT           (1 << 0)

#define SPI_BUS_BUSY_STS_BUS_BUSY     (1 << 0)

#define SPI_PRD_0_CR_D_PH_1_SHIFT     (24)
#define SPI_PRD_0_CR_D_PH_1_MASK      (0xff << SPI_PRD_0_CR_D_PH_1_SHIFT)
#define SPI_PRD_0_CR_D_PH_0_SHIFT     (16)
#define SPI_PRD_0_CR_D_PH_0_MASK      (0xff << SPI_PRD_0_CR_D_PH_0_SHIFT)
#define SPI_PRD_0_CR_P_SHIFT          (8)
#define SPI_PRD_0_CR_P_MASK           (0xff << SPI_PRD_0_CR_P_SHIFT)
#define SPI_PRD_0_CR_S_MASK           (0xff)

#define SPI_PRD_1_CR_I_MASK           (0xff)

#define SPI_RXD_IGNR_CR_IGNR_S_SHIFT  (16)
#define SPI_RXD_IGNR_CR_IGNR_S_MASK   (0x1f << SPI_RXD_IGNR_CR_RXD_IGNR_S_SHIFT)
#define SPI_RXD_IGNR_CR_IGNR_P_MASK   (0x1f)

#define SPI_STO_VALUE_CR_VALUE_MASK   (0xfff)

#define SPI_FIFO_CFG_0_RX_UNDERFLOW   (1 << 7)
#define SPI_FIFO_CFG_0_RX_OVERFLOW    (1 << 6)
#define SPI_FIFO_CFG_0_TX_UNDERFLOW   (1 << 5)
#define SPI_FIFO_CFG_0_TX_OVERFLOW    (1 << 4)
#define SPI_FIFO_CFG_0_RX_CLR         (1 << 3)
#define SPI_FIFO_CFG_0_TX_CLR         (1 << 2)
#define SPI_FIFO_CFG_0_DMA_RX_EN       (1 << 1)
#define SPI_FIFO_CFG_0_DMA_TX_EN       (1 << 0)

#define SPI_FIFO_CFG_1_RX_TH_SHIFT    (24)
#define SPI_FIFO_CFG_1_RX_TH_MASK     (0x03 << SPI_FIFO_CFG_1_RX_TH_SHIFT)
#define SPI_FIFO_CFG_1_TX_TH_SHIFT    (16)
#define SPI_FIFO_CFG_1_TX_TH_MASK     (0x03 << SPI_FIFO_CFG_1_TX_TH_SHIFT)
#define SPI_FIFO_CFG_1_RX_CNT_SHIFT   (8)
#define SPI_FIFO_CFG_1_RX_CNT_MASK    (0x07 << SPI_FIFO_CFG_1_RX_CNT_SHIFT)
#define SPI_FIFO_CFG_1_TX_CNT_MASK    (0x07)

#endif /* __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_SPI_H */
