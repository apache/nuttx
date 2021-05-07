/****************************************************************************
 * arch/risc-v/src/bl602/hardware/bl602_i2c.h
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

#ifndef __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_I2C_H
#define __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "bl602_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define BL602_I2C_CONFIG_OFFSET         0x000000  /* i2c_config */
#define BL602_I2C_INT_STS_OFFSET        0x000004  /* i2c_int_sts */
#define BL602_I2C_SUB_ADDR_OFFSET       0x000008  /* i2c_sub_addr */
#define BL602_I2C_BUS_BUSY_OFFSET       0x00000c  /* i2c_bus_busy */
#define BL602_I2C_PRD_START_OFFSET      0x000010  /* i2c_prd_start */
#define BL602_I2C_PRD_STOP_OFFSET       0x000014  /* i2c_prd_stop */
#define BL602_I2C_PRD_DATA_OFFSET       0x000018  /* i2c_prd_data */
#define BL602_I2C_FIFO_CONFIG_0_OFFSET  0x000080  /* i2c_fifo_config_0 */
#define BL602_I2C_FIFO_CONFIG_1_OFFSET  0x000084  /* i2c_fifo_config_1 */
#define BL602_I2C_FIFO_WDATA_OFFSET     0x000088  /* i2c_fifo_wdata */
#define BL602_I2C_FIFO_RDATA_OFFSET     0x00008c  /* i2c_fifo_rdata */

/* Register definitions *****************************************************/

#define BL602_I2C_CONFIG     (BL602_I2C_BASE + BL602_I2C_CONFIG_OFFSET)
#define BL602_I2C_INT_STS    (BL602_I2C_BASE + BL602_I2C_INT_STS_OFFSET)
#define BL602_I2C_SUB_ADDR   (BL602_I2C_BASE + BL602_I2C_SUB_ADDR_OFFSET)
#define BL602_I2C_BUS_BUSY   (BL602_I2C_BASE + BL602_I2C_BUS_BUSY_OFFSET)
#define BL602_I2C_PRD_START  (BL602_I2C_BASE + BL602_I2C_PRD_START_OFFSET)
#define BL602_I2C_PRD_STOP   (BL602_I2C_BASE + BL602_I2C_PRD_STOP_OFFSET)
#define BL602_I2C_PRD_DATA   (BL602_I2C_BASE + BL602_I2C_PRD_DATA_OFFSET)
#define BL602_I2C_FIFO_CONFIG_0  \
          (BL602_I2C_BASE + BL602_I2C_FIFO_CONFIG_0_OFFSET)
#define BL602_I2C_FIFO_CONFIG_1  \
          (BL602_I2C_BASE + BL602_I2C_FIFO_CONFIG_1_OFFSET)
#define BL602_I2C_FIFO_WDATA     \
          (BL602_I2C_BASE + BL602_I2C_FIFO_WDATA_OFFSET)
#define BL602_I2C_FIFO_RDATA     \
          (BL602_I2C_BASE + BL602_I2C_FIFO_RDATA_OFFSET)

/* Register bit definitions *************************************************/

#define I2C_CONFIG_CR_I2C_DEG_CNT_SHIFT        (28)
#define I2C_CONFIG_CR_I2C_DEG_CNT_MASK         \
          (0x0f << I2C_CONFIG_CR_I2C_DEG_CNT_SHIFT)
#define I2C_CONFIG_CR_I2C_PKT_LEN_SHIFT        (16)
#define I2C_CONFIG_CR_I2C_PKT_LEN_MASK         \
          (0xff << I2C_CONFIG_CR_I2C_PKT_LEN_SHIFT)
#define I2C_CONFIG_CR_I2C_SLV_ADDR_SHIFT       (8)
#define I2C_CONFIG_CR_I2C_SLV_ADDR_MASK        \
          (0x7f << I2C_CONFIG_CR_I2C_SLV_ADDR_SHIFT)
#define I2C_CONFIG_CR_I2C_SUB_ADDR_BC_SHIFT    (5)
#define I2C_CONFIG_CR_I2C_SUB_ADDR_BC_MASK     \
          (0x03 << I2C_CONFIG_CR_I2C_SUB_ADDR_BC_SHIFT)
#define I2C_CONFIG_CR_I2C_SUB_ADDR_EN          (1 << 4)
#define I2C_CONFIG_CR_I2C_SCL_SYNC_EN          (1 << 3)
#define I2C_CONFIG_CR_I2C_DEG_EN               (1 << 2)
#define I2C_CONFIG_CR_I2C_PKT_DIR              (1 << 1)
#define I2C_CONFIG_CR_I2C_M_EN                 (1 << 0)

#define I2C_INT_STS_CR_FER_EN                  (1 << 29)
#define I2C_INT_STS_CR_ARB_EN                  (1 << 28)
#define I2C_INT_STS_CR_NAK_EN                  (1 << 27)
#define I2C_INT_STS_CR_RXF_EN                  (1 << 26)
#define I2C_INT_STS_CR_TXF_EN                  (1 << 25)
#define I2C_INT_STS_CR_END_EN                  (1 << 24)
#define I2C_INT_STS_RSVD_21                    (1 << 21)
#define I2C_INT_STS_CR_ARB_CLR                 (1 << 20)
#define I2C_INT_STS_CR_NAK_CLR                 (1 << 19)
#define I2C_INT_STS_RSVD_18                    (1 << 18)
#define I2C_INT_STS_RSVD_17                    (1 << 17)
#define I2C_INT_STS_CR_END_CLR                 (1 << 16)
#define I2C_INT_STS_CR_FER_MASK                (1 << 13)
#define I2C_INT_STS_CR_ARB_MASK                (1 << 12)
#define I2C_INT_STS_CR_NAK_MASK                (1 << 11)
#define I2C_INT_STS_CR_RXF_MASK                (1 << 10)
#define I2C_INT_STS_CR_TXF_MASK                (1 << 9)
#define I2C_INT_STS_CR_END_MASK                (1 << 8)
#define I2C_INT_STS_FER_INT                    (1 << 5)
#define I2C_INT_STS_ARB_INT                    (1 << 4)
#define I2C_INT_STS_NAK_INT                    (1 << 3)
#define I2C_INT_STS_RXF_INT                    (1 << 2)
#define I2C_INT_STS_TXF_INT                    (1 << 1)
#define I2C_INT_STS_END_INT                    (1 << 0)

#define I2C_SUB_ADDR_CR_SUB_ADDR_B3_SHIFT  (24)
#define I2C_SUB_ADDR_CR_SUB_ADDR_B3_MASK   \
          (0xff << I2C_SUB_ADDR_CR_SUB_ADDR_B3_SHIFT)
#define I2C_SUB_ADDR_CR_SUB_ADDR_B2_SHIFT  (16)
#define I2C_SUB_ADDR_CR_SUB_ADDR_B2_MASK   \
          (0xff << I2C_SUB_ADDR_CR_SUB_ADDR_B2_SHIFT)
#define I2C_SUB_ADDR_CR_SUB_ADDR_B1_SHIFT  (8)
#define I2C_SUB_ADDR_CR_SUB_ADDR_B1_MASK   \
          (0xff << I2C_SUB_ADDR_CR_SUB_ADDR_B1_SHIFT)
#define I2C_SUB_ADDR_CR_SUB_ADDR_B0_MASK   (0xff)

#define I2C_BUS_BUSY_CR_BUS_BUSY_CLR       (1 << 1)
#define I2C_BUS_BUSY_STS_BUS_BUSY          (1 << 0)

#define I2C_PRD_START_CR_PRD_S_PH_3_SHIFT  (24)
#define I2C_PRD_START_CR_PRD_S_PH_3_MASK   \
          (0xff << I2C_PRD_START_CR_PRD_S_PH_3_SHIFT)
#define I2C_PRD_START_CR_PRD_S_PH_2_SHIFT  (16)
#define I2C_PRD_START_CR_PRD_S_PH_2_MASK   \
          (0xff << I2C_PRD_START_CR_PRD_S_PH_2_SHIFT)
#define I2C_PRD_START_CR_PRD_S_PH_1_SHIFT  (8)
#define I2C_PRD_START_CR_PRD_S_PH_1_MASK   \
          (0xff << I2C_PRD_START_CR_PRD_S_PH_1_SHIFT)
#define I2C_PRD_START_CR_PRD_S_PH_0_MASK   (0xff)

#define I2C_PRD_STOP_CR_PRD_P_PH_3_SHIFT   (24)
#define I2C_PRD_STOP_CR_PRD_P_PH_3_MASK    \
          (0xff << I2C_PRD_STOP_CR_PRD_P_PH_3_SHIFT)
#define I2C_PRD_STOP_CR_PRD_P_PH_2_SHIFT   (16)
#define I2C_PRD_STOP_CR_PRD_P_PH_2_MASK    \
          (0xff << I2C_PRD_STOP_CR_PRD_P_PH_2_SHIFT)
#define I2C_PRD_STOP_CR_PRD_P_PH_1_SHIFT   (8)
#define I2C_PRD_STOP_CR_PRD_P_PH_1_MASK    \
          (0xff << I2C_PRD_STOP_CR_PRD_P_PH_1_SHIFT)
#define I2C_PRD_STOP_CR_PRD_P_PH_0_MASK    (0xff)

#define I2C_PRD_DATA_CR_PRD_D_PH_3_SHIFT   (24)
#define I2C_PRD_DATA_CR_PRD_D_PH_3_MASK    \
          (0xff << I2C_PRD_DATA_CR_PRD_D_PH_3_SHIFT)
#define I2C_PRD_DATA_CR_PRD_D_PH_2_SHIFT   (16)
#define I2C_PRD_DATA_CR_PRD_D_PH_2_MASK    \
          (0xff << I2C_PRD_DATA_CR_PRD_D_PH_2_SHIFT)
#define I2C_PRD_DATA_CR_PRD_D_PH_1_SHIFT   (8)
#define I2C_PRD_DATA_CR_PRD_D_PH_1_MASK    \
          (0xff << I2C_PRD_DATA_CR_PRD_D_PH_1_SHIFT)
#define I2C_PRD_DATA_CR_PRD_D_PH_0_MASK    (0xff)

#define I2C_FIFO_CONFIG_0_RX_FIFO_UNDERFLOW    (1 << 7)
#define I2C_FIFO_CONFIG_0_RX_FIFO_OVERFLOW     (1 << 6)
#define I2C_FIFO_CONFIG_0_TX_FIFO_UNDERFLOW    (1 << 5)
#define I2C_FIFO_CONFIG_0_TX_FIFO_OVERFLOW     (1 << 4)
#define I2C_FIFO_CONFIG_0_RX_FIFO_CLR          (1 << 3)
#define I2C_FIFO_CONFIG_0_TX_FIFO_CLR          (1 << 2)
#define I2C_FIFO_CONFIG_0_DMA_RX_EN            (1 << 1)
#define I2C_FIFO_CONFIG_0_DMA_TX_EN            (1 << 0)

#define I2C_FIFO_CONFIG_1_RX_FIFO_TH           (1 << 24)
#define I2C_FIFO_CONFIG_1_TX_FIFO_TH           (1 << 16)
#define I2C_FIFO_CONFIG_1_RX_FIFO_CNT_SHIFT    (8)
#define I2C_FIFO_CONFIG_1_RX_FIFO_CNT_MASK    \
          (0x03 << I2C_FIFO_CONFIG_1_RX_FIFO_CNT_SHIFT)
#define I2C_FIFO_CONFIG_1_TX_FIFO_CNT_MASK     (0x03)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* I2C interrupt type definition */

#define I2C_TRANS_END_INT     0 /* I2C transfer end interrupt */
#define I2C_TX_FIFO_READY_INT 1 /* I2C TX fifo ready interrupt */
#define I2C_RX_FIFO_READY_INT 2 /* I2C RX fifo ready interrupt */
#define I2C_NACK_RECV_INT     3 /* I2C nack received interrupt */
#define I2C_ARB_LOST_INT      4 /* I2C arbitration lost interrupt */
#define I2C_FIFO_ERR_INT      5 /* I2C TX/RX FIFO error interrupt */
#define I2C_INT_ALL           6 /* I2C interrupt all type */

#endif /* __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_I2C_H */
