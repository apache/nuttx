/****************************************************************************
 * arch/risc-v/src/bl602/hardware/bl602_l1c.h
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

#ifndef __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_L1C_H
#define __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_L1C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "bl602_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define BL602_L1C_CONFIG_OFFSET                0x000000  /* l1c_config */
#define BL602_L1C_HIT_CNT_LSB_OFFSET           0x000004  /* hit_cnt_lsb */
#define BL602_L1C_HIT_CNT_MSB_OFFSET           0x000008  /* hit_cnt_msb */
#define BL602_L1C_MISS_CNT_OFFSET              0x00000c  /* miss_cnt */
#define BL602_L1C_RANGE_OFFSET                 0x000010  /* l1c_range */
#define BL602_L1C_BMX_ERR_ADDR_EN_OFFSET       0x000200  /* l1c_bmx_err_addr_en */
#define BL602_L1C_BMX_ERR_ADDR_OFFSET          0x000204  /* l1c_bmx_err_addr */
#define BL602_L1C_IROM1_MISR_DATAOUT_0_OFFSET  0x000208  /* irom1_misr_dataout_0 */
#define BL602_L1C_IROM1_MISR_DATAOUT_1_OFFSET  0x00020c  /* irom1_misr_dataout_1 */
#define BL602_L1C_CPU_CLK_GATE_OFFSET          0x000210  /* cpu_clk_gate */

/* Register definitions *****************************************************/

#define BL602_L1C_CONFIG                (BL602_L1C_BASE + BL602_L1C_CONFIG_OFFSET)
#define BL602_L1C_HIT_CNT_LSB           (BL602_L1C_BASE + BL602_L1C_HIT_CNT_LSB_OFFSET)
#define BL602_L1C_HIT_CNT_MSB           (BL602_L1C_BASE + BL602_L1C_HIT_CNT_MSB_OFFSET)
#define BL602_L1C_MISS_CNT              (BL602_L1C_BASE + BL602_L1C_MISS_CNT_OFFSET)
#define BL602_L1C_RANGE                 (BL602_L1C_BASE + BL602_L1C_RANGE_OFFSET)
#define BL602_L1C_BMX_ERR_ADDR_EN       (BL602_L1C_BASE + BL602_L1C_BMX_ERR_ADDR_EN_OFFSET)
#define BL602_L1C_BMX_ERR_ADDR          (BL602_L1C_BASE + BL602_L1C_BMX_ERR_ADDR_OFFSET)
#define BL602_L1C_IROM1_MISR_DATAOUT_0  (BL602_L1C_BASE + BL602_L1C_IROM1_MISR_DATAOUT_0_OFFSET)
#define BL602_L1C_IROM1_MISR_DATAOUT_1  (BL602_L1C_BASE + BL602_L1C_IROM1_MISR_DATAOUT_1_OFFSET)
#define BL602_L1C_CPU_CLK_GATE          (BL602_L1C_BASE + BL602_L1C_CPU_CLK_GATE_OFFSET)

/* Register bit definitions *************************************************/

#define L1C_CONFIG_WRAP_DIS                    (1 << 26)
#define L1C_CONFIG_EARLY_RESP_DIS              (1 << 25)
#define L1C_CONFIG_BMX_BUSY_OPTION_DIS         (1 << 24)
#define L1C_CONFIG_BMX_TIMEOUT_EN_SHIFT        (20)
#define L1C_CONFIG_BMX_TIMEOUT_EN_MASK         (0x0f << L1C_CONFIG_BMX_TIMEOUT_EN_SHIFT)
#define L1C_CONFIG_BMX_ARB_MODE_SHIFT          (16)
#define L1C_CONFIG_BMX_ARB_MODE_MASK           (0x03 << L1C_CONFIG_BMX_ARB_MODE_SHIFT)
#define L1C_CONFIG_BMX_ERR_EN                  (1 << 15)
#define L1C_CONFIG_BYPASS                      (1 << 14)
#define L1C_CONFIG_IROM_2T_ACCESS              (1 << 12)
#define L1C_CONFIG_WAY_DIS_SHIFT               (8)
#define L1C_CONFIG_WAY_DIS_MASK                (0x0f << L1C_CONFIG_WAY_DIS_SHIFT)
#define L1C_CONFIG_INVALID_DONE                (1 << 3)
#define L1C_CONFIG_INVALID_EN                  (1 << 2)
#define L1C_CONFIG_CNT_EN                      (1 << 1)
#define L1C_CONFIG_CACHEABLE                   (1 << 0)

#define L1C_BMX_ERR_ADDR_EN_HSEL_OPTION_SHIFT  (16)
#define L1C_BMX_ERR_ADDR_EN_HSEL_OPTION_MASK   (0x0f << L1C_BMX_ERR_ADDR_EN_HSEL_OPTION_SHIFT)
#define L1C_BMX_ERR_ADDR_EN_BMX_ERR_TZ         (1 << 5)
#define L1C_BMX_ERR_ADDR_EN_BMX_ERR_DEC        (1 << 4)
#define L1C_BMX_ERR_ADDR_EN_BMX_ERR_ADDR_DIS   (1 << 0)

#define L1C_CPU_CLK_GATE_FORCE_E21_CLOCK_ON_2          (1 << 2)
#define L1C_CPU_CLK_GATE_FORCE_E21_CLOCK_ON_1          (1 << 1)
#define L1C_CPU_CLK_GATE_FORCE_E21_CLOCK_ON_0          (1 << 0)

#endif /* __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_L1C_H */
