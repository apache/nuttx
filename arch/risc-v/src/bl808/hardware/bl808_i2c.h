/****************************************************************************
 * arch/risc-v/src/bl808/hardware/bl808_i2c.h
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

#ifndef __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_I2C_H
#define __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "bl808_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BL808_I2C_BASE(n) (((n) == 0) ? BL808_I2C0_BASE \
                           : ((n) == 1) ? BL808_I2C1_BASE \
                           : ((n) == 2) ? BL808_I2C2_BASE \
                           : BL808_I2C3_BASE)

/* Register offsets *********************************************************/

#define BL808_I2C_CONFIG_OFFSET        0x00
#define BL808_I2C_INT_STS_OFFSET       0x04
#define BL808_I2C_SUB_ADDR_OFFSET      0x08
#define BL808_I2C_BUSY_OFFSET          0x0c
#define BL808_I2C_PRD_START_OFFSET     0x10
#define BL808_I2C_PRD_STOP_OFFSET      0x14
#define BL808_I2C_PRD_DATA_OFFSET      0x18
#define BL808_I2C_FIFO_CONFIG_0_OFFSET 0x80
#define BL808_I2C_FIFO_CONFIG_1_OFFSET 0x84
#define BL808_I2C_WDATA_OFFSET         0x88
#define BL808_I2C_RDATA_OFFSET         0X8c

/* Register definitions *****************************************************/

#define BL808_I2C_CONFIG(n) (BL808_I2C_BASE(n) + BL808_I2C_CONFIG_OFFSET)
#define BL808_I2C_INT_STS(n) (BL808_I2C_BASE(n) + BL808_I2C_INT_STS_OFFSET)
#define BL808_I2C_SUB_ADDR(n) (BL808_I2C_BASE(n) + BL808_I2C_SUB_ADDR_OFFSET)
#define BL808_I2C_BUSY(n) (BL808_I2C_BASE(n) + BL808_I2C_BUSY_OFFSET)
#define BL808_I2C_PRD_START(n) (BL808_I2C_BASE(n) + BL808_I2C_PRD_START_OFFSET)
#define BL808_I2C_PRD_STOP(n) (BL808_I2C_BASE(n) + BL808_I2C_PRD_STOP_OFFSET)
#define BL808_I2C_PRD_DATA(n) (BL808_I2C_BASE(n) + BL808_I2C_PRD_DATA_OFFSET)
#define BL808_I2C_FIFO_CONFIG_0(n) (BL808_I2C_BASE(n) + BL808_I2C_FIFO_CONFIG_0_OFFSET)
#define BL808_I2C_FIFO_CONFIG_1(n) (BL808_I2C_BASE(n) + BL808_I2C_FIFO_CONFIG_1_OFFSET)
#define BL808_I2C_WDATA(n) (BL808_I2C_BASE(n) + BL808_I2C_WDATA_OFFSET)
#define BL808_I2C_RDATA(n) (BL808_I2C_BASE(n) + BL808_I2C_RDATA_OFFSET)

/* Register bit definitions *************************************************/

/* I2C_CONFIG */

#define I2C_M_EN (1 << 0)
#define I2C_DIR_R (1 << 1)
#define I2C_SUB_ADDR_EN (1 << 4)
#define I2C_SUB_ADDR_LEN_SHIFT (5)
#define I2C_SUB_ADDR_LEN_MASK (0x3 << I2C_SUB_ADDR_LEN_SHIFT)
#define I2C_10B_ADDR_EN (1 << 7)
#define I2C_SLV_ADDR_SHIFT (8)
#define I2C_SLV_ADDR_MASK (0x3ff << I2C_SLV_ADDR_SHIFT)
#define I2C_PKT_LEN_SHIFT (20)
#define I2C_PKT_LEN_MASK (0xff << I2C_PKT_LEN_SHIFT)

/* I2C_INT_STS */

#define I2C_END_INT (1 << 0)
#define I2C_TX_FIFO_RDY_INT (1 << 1)
#define I2C_RX_FIFO_RDY_INT (1 << 2)
#define I2C_NAK_INT (1 << 3)
#define I2C_ARB_INT (1 << 4)
#define I2C_FIFO_ERROR_INT (1 << 5)
#define I2C_END_MASK (1 << 8)
#define I2C_TX_FIFO_RDY_MASK (1 << 9)
#define I2C_RX_FIFO_RDY_MASK (1 << 10)
#define I2C_NAK_MASK (1 << 11)
#define I2C_ARB_MASK (1 << 12)
#define I2C_FIFO_ERROR_MASK (1 << 13)
#define I2C_END_CLR (1 << 16)
#define I2C_NAK_CLR (1 << 19)
#define I2C_ARB_CLR (1 << 20)

/* I2C_PRD_x */

#define I2C_PRD_PH0_SHIFT (0)
#define I2C_PRD_PH0_MASK (0xff << I2C_PRD_PH0_SHIFT)
#define I2C_PRD_PH1_SHIFT (8)
#define I2C_PRD_PH1_MASK (0xff << I2C_PRD_PH1_SHIFT)
#define I2C_PRD_PH2_SHIFT (16)
#define I2C_PRD_PH2_MASK (0xff << I2C_PRD_PH2_SHIFT)
#define I2C_PRD_PH3_SHIFT (24)
#define I2C_PRD_PH3_MASK (0xff << I2C_PRD_PH3_SHIFT)

/* I2C_FIFO_CONFIG_0 */

#define TX_FIFO_CLR (1 << 2)
#define RX_FIFO_CLR (1 << 3)

#endif /* __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_I2C_H */
