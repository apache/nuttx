/****************************************************************************
 * arch/risc-v/src/bl602/hardware/bl602_ir.h
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

#ifndef __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_IR_H
#define __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_IR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "bl602_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define BL602_IRTX_CONFIG_OFFSET             0x000000  /* irtx_config */
#define BL602_IRTX_INT_STS_OFFSET            0x000004  /* irtx_int_sts */
#define BL602_IRTX_DATA_WORD0_OFFSET         0x000008  /* irtx_data_word0 */
#define BL602_IRTX_DATA_WORD1_OFFSET         0x00000c  /* irtx_data_word1 */
#define BL602_IRTX_PULSE_WIDTH_OFFSET        0x000010  /* irtx_pulse_width */
#define BL602_IRTX_PW_OFFSET                 0x000014  /* irtx_pw */
#define BL602_IRTX_SWM_PW_0_OFFSET           0x000040  /* irtx_swm_pw_0 */
#define BL602_IRTX_SWM_PW_1_OFFSET           0x000044  /* irtx_swm_pw_1 */
#define BL602_IRTX_SWM_PW_2_OFFSET           0x000048  /* irtx_swm_pw_2 */
#define BL602_IRTX_SWM_PW_3_OFFSET           0x00004c  /* irtx_swm_pw_3 */
#define BL602_IRTX_SWM_PW_4_OFFSET           0x000050  /* irtx_swm_pw_4 */
#define BL602_IRTX_SWM_PW_5_OFFSET           0x000054  /* irtx_swm_pw_5 */
#define BL602_IRTX_SWM_PW_6_OFFSET           0x000058  /* irtx_swm_pw_6 */
#define BL602_IRTX_SWM_PW_7_OFFSET           0x00005c  /* irtx_swm_pw_7 */
#define BL602_IRRX_CONFIG_OFFSET             0x000080  /* irrx_config */
#define BL602_IRRX_INT_STS_OFFSET            0x000084  /* irrx_int_sts */
#define BL602_IRRX_PW_CONFIG_OFFSET          0x000088  /* irrx_pw_config */
#define BL602_IRRX_DATA_COUNT_OFFSET         0x000090  /* irrx_data_count */
#define BL602_IRRX_DATA_WORD0_OFFSET         0x000094  /* irrx_data_word0 */
#define BL602_IRRX_DATA_WORD1_OFFSET         0x000098  /* irrx_data_word1 */
#define BL602_IRRX_SWM_FIFO_CONFIG_0_OFFSET  0x0000c0  /* irrx_swm_fifo_config_0 */
#define BL602_IRRX_SWM_FIFO_RDATA_OFFSET     0x0000c4  /* irrx_swm_fifo_rdata */

/* Register definitions *****************************************************/

#define BL602_IRTX_CONFIG             (BL602_IR_BASE + BL602_IRTX_CONFIG_OFFSET)
#define BL602_IRTX_INT_STS            (BL602_IR_BASE + BL602_IRTX_INT_STS_OFFSET)
#define BL602_IRTX_DATA_WORD0         (BL602_IR_BASE + BL602_IRTX_DATA_WORD0_OFFSET)
#define BL602_IRTX_DATA_WORD1         (BL602_IR_BASE + BL602_IRTX_DATA_WORD1_OFFSET)
#define BL602_IRTX_PULSE_WIDTH        (BL602_IR_BASE + BL602_IRTX_PULSE_WIDTH_OFFSET)
#define BL602_IRTX_PW                 (BL602_IR_BASE + BL602_IRTX_PW_OFFSET)
#define BL602_IRTX_SWM_PW_0           (BL602_IR_BASE + BL602_IRTX_SWM_PW_0_OFFSET)
#define BL602_IRTX_SWM_PW_1           (BL602_IR_BASE + BL602_IRTX_SWM_PW_1_OFFSET)
#define BL602_IRTX_SWM_PW_2           (BL602_IR_BASE + BL602_IRTX_SWM_PW_2_OFFSET)
#define BL602_IRTX_SWM_PW_3           (BL602_IR_BASE + BL602_IRTX_SWM_PW_3_OFFSET)
#define BL602_IRTX_SWM_PW_4           (BL602_IR_BASE + BL602_IRTX_SWM_PW_4_OFFSET)
#define BL602_IRTX_SWM_PW_5           (BL602_IR_BASE + BL602_IRTX_SWM_PW_5_OFFSET)
#define BL602_IRTX_SWM_PW_6           (BL602_IR_BASE + BL602_IRTX_SWM_PW_6_OFFSET)
#define BL602_IRTX_SWM_PW_7           (BL602_IR_BASE + BL602_IRTX_SWM_PW_7_OFFSET)
#define BL602_IRRX_CONFIG             (BL602_IR_BASE + BL602_IRRX_CONFIG_OFFSET)
#define BL602_IRRX_INT_STS            (BL602_IR_BASE + BL602_IRRX_INT_STS_OFFSET)
#define BL602_IRRX_PW_CONFIG          (BL602_IR_BASE + BL602_IRRX_PW_CONFIG_OFFSET)
#define BL602_IRRX_DATA_COUNT         (BL602_IR_BASE + BL602_IRRX_DATA_COUNT_OFFSET)
#define BL602_IRRX_DATA_WORD0         (BL602_IR_BASE + BL602_IRRX_DATA_WORD0_OFFSET)
#define BL602_IRRX_DATA_WORD1         (BL602_IR_BASE + BL602_IRRX_DATA_WORD1_OFFSET)
#define BL602_IRRX_SWM_FIFO_CONFIG_0  (BL602_IR_BASE + BL602_IRRX_SWM_FIFO_CONFIG_0_OFFSET)
#define BL602_IRRX_SWM_FIFO_RDATA     (BL602_IR_BASE + BL602_IRRX_SWM_FIFO_RDATA_OFFSET)

/* Register bit definitions *************************************************/

#define IRTX_CONFIG_CR_DATA_NUM_SHIFT        (12)
#define IRTX_CONFIG_CR_DATA_NUM_MASK         (0x3f << IRTX_CONFIG_CR_DATA_NUM_SHIFT)
#define IRTX_CONFIG_CR_TAIL_HL_INV           (1 << 11)
#define IRTX_CONFIG_CR_TAIL_EN               (1 << 10)
#define IRTX_CONFIG_CR_HEAD_HL_INV           (1 << 9)
#define IRTX_CONFIG_CR_HEAD_EN               (1 << 8)
#define IRTX_CONFIG_CR_LOGIC1_HL_INV         (1 << 6)
#define IRTX_CONFIG_CR_LOGIC0_HL_INV         (1 << 5)
#define IRTX_CONFIG_CR_DATA_EN               (1 << 4)
#define IRTX_CONFIG_CR_SWM_EN                (1 << 3)
#define IRTX_CONFIG_CR_MOD_EN                (1 << 2)
#define IRTX_CONFIG_CR_OUT_INV               (1 << 1)
#define IRTX_CONFIG_CR_EN                    (1 << 0)

#define IRTX_INT_STS_CR_END_EN               (1 << 24)
#define IRTX_INT_STS_CR_END_CLR              (1 << 16)
#define IRTX_INT_STS_CR_END_MASK             (1 << 8)
#define IRTX_INT_STS_END_INT                 (1 << 0)

#define IRTX_PULSE_WIDTH_CR_MOD_PH1_W_SHIFT  (24)
#define IRTX_PULSE_WIDTH_CR_MOD_PH1_W_MASK   (0xff << IRTX_PULSE_WIDTH_CR_MOD_PH1_W_SHIFT)
#define IRTX_PULSE_WIDTH_CR_MOD_PH0_W_SHIFT  (16)
#define IRTX_PULSE_WIDTH_CR_MOD_PH0_W_MASK   (0xff << IRTX_PULSE_WIDTH_CR_MOD_PH0_W_SHIFT)
#define IRTX_PULSE_WIDTH_CR_PW_UNIT_MASK     (0xfff)

#define IRTX_PW_CR_TAIL_PH1_W_SHIFT          (28)
#define IRTX_PW_CR_TAIL_PH1_W_MASK           (0x0f << IRTX_PW_CR_TAIL_PH1_W_SHIFT)
#define IRTX_PW_CR_TAIL_PH0_W_SHIFT          (24)
#define IRTX_PW_CR_TAIL_PH0_W_MASK           (0x0f << IRTX_PW_CR_TAIL_PH0_W_SHIFT)
#define IRTX_PW_CR_HEAD_PH1_W_SHIFT          (20)
#define IRTX_PW_CR_HEAD_PH1_W_MASK           (0x0f << IRTX_PW_CR_HEAD_PH1_W_SHIFT)
#define IRTX_PW_CR_HEAD_PH0_W_SHIFT          (16)
#define IRTX_PW_CR_HEAD_PH0_W_MASK           (0x0f << IRTX_PW_CR_HEAD_PH0_W_SHIFT)
#define IRTX_PW_CR_LOGIC1_PH1_W_SHIFT        (12)
#define IRTX_PW_CR_LOGIC1_PH1_W_MASK         (0x0f << IRTX_PW_CR_LOGIC1_PH1_W_SHIFT)
#define IRTX_PW_CR_LOGIC1_PH0_W_SHIFT        (8)
#define IRTX_PW_CR_LOGIC1_PH0_W_MASK         (0x0f << IRTX_PW_CR_LOGIC1_PH0_W_SHIFT)
#define IRTX_PW_CR_LOGIC0_PH1_W_SHIFT        (4)
#define IRTX_PW_CR_LOGIC0_PH1_W_MASK         (0x0f << IRTX_PW_CR_LOGIC0_PH1_W_SHIFT)
#define IRTX_PW_CR_LOGIC0_PH0_W_MASK         (0x0f)

#define IRRX_CONFIG_CR_DEG_CNT_SHIFT         (8)
#define IRRX_CONFIG_CR_DEG_CNT_MASK          (0x0f << IRRX_CONFIG_CR_DEG_CNT_SHIFT)
#define IRRX_CONFIG_CR_DEG_EN                (1 << 4)
#define IRRX_CONFIG_CR_MODE_SHIFT            (2)
#define IRRX_CONFIG_CR_MODE_MASK             (0x03 << IRRX_CONFIG_CR_MODE_SHIFT)
#define IRRX_CONFIG_CR_IN_INV                (1 << 1)
#define IRRX_CONFIG_CR_EN                    (1 << 0)

#define IRRX_INT_STS_CR_END_EN               (1 << 24)
#define IRRX_INT_STS_CR_END_CLR              (1 << 16)
#define IRRX_INT_STS_CR_END_MASK             (1 << 8)
#define IRRX_INT_STS_END_INT                 (1 << 0)

#define IRRX_PW_CONFIG_CR_END_TH_SHIFT       (16)
#define IRRX_PW_CONFIG_CR_END_TH_MASK        (0xffff << IRRX_PW_CONFIG_CR_END_TH_SHIFT)
#define IRRX_PW_CONFIG_CR_DATA_TH_MASK       (0xffff)

#define IRRX_DATA_COUNT_STS_DATA_CNT_MASK    (0x7f)

#define IRRX_SWM_FIFO_CONFIG_0_RX_FIFO_CNT_SHIFT  (4)
#define IRRX_SWM_FIFO_CONFIG_0_RX_FIFO_CNT_MASK   (0x7f << IRRX_SWM_FIFO_CONFIG_0_RX_FIFO_CNT_SHIFT)
#define IRRX_SWM_FIFO_CONFIG_0_RX_FIFO_UNDERFLOW  (1 << 3)
#define IRRX_SWM_FIFO_CONFIG_0_RX_FIFO_OVERFLOW   (1 << 2)
#define IRRX_SWM_FIFO_CONFIG_0_RX_FIFO_CLR        (1 << 0)

#define IRRX_SWM_FIFO_RDATA_RX_FIFO_RDATA_MASK    (0xffff)

#endif /* __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_IR_H */
