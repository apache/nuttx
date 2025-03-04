/****************************************************************************
 * arch/risc-v/src/bl808/hardware/bl808_gpadc.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_GPADC_H
#define __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_GPADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "bl808_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

/* gpadc base */
#define BL808_GPADC_CONFIG_OFFSET    (0x0)
#define BL808_GPADC_DMA_RDATA_OFFSET (0x4)
#define BL808_GPADC_PIR_TRAIN_OFFSET (0x20

/* aon base */
#define BL808_GPADC_CMD_OFFSET        (0x90C)
#define BL808_GPADC_CONFIG1_OFFSET    (0x910)
#define BL808_GPADC_CONFIG2_OFFSET    (0x914)
#define BL808_GPADC_SCAN_POS1_OFFSET   (0x918)
#define BL808_GPADC_SCAN_POS2_OFFSET   (0x91C)
#define BL808_GPADC_SCAN_NEG1_OFFSET   (0x920)
#define BL808_GPADC_SCAN_NEG2_OFFSET   (0x924)
#define BL808_GPADC_STATUS_OFFSET     (0x928)
#define BL808_GPADC_ISR_OFFSET        (0x92C)
#define BL808_GPADC_RESULT_OFFSET     (0x930)
#define BL808_GPADC_RAW_RESULT_OFFSET (0x934)
#define BL808_GPADC_DEFINE_OFFSET     (0x938)

/* Register definitions *****************************************************/

#define BL808_GPADC_CONFIG    (BL808_GPADC_BASE + BL808_GPADC_CONFIG_OFFSET)
#define BL808_GPADC_DMA_RDATA (BL808_GPADC_BASE + BL808_GPADC_DMA_RDATA_OFFSET)
#define BL808_GPADC_PIR_TRAIN (BL808_GPADC_BASE + BL808_GPADC_PIR_TRAIN_OFFSET)

#define BL808_GPADC_CMD (BL808_AON_BASE + BL808_GPADC_CMD_OFFSET)
#define BL808_GPADC_CONFIG1 (BL808_AON_BASE + BL808_GPADC_CONFIG1_OFFSET)
#define BL808_GPADC_CONFIG2 (BL808_AON_BASE + BL808_GPADC_CONFIG2_OFFSET)
#define BL808_GPADC_SCAN_POS1 (BL808_AON_BASE + BL808_GPADC_SCAN_POS1_OFFSET)
#define BL808_GPADC_SCAN_POS2 (BL808_AON_BASE + BL808_GPADC_SCAN_POS2_OFFSET)
#define BL808_GPADC_SCAN_NEG1 (BL808_AON_BASE + BL808_GPADC_SCAN_NEG1_OFFSET)
#define BL808_GPADC_SCAN_NEG2 (BL808_AON_BASE + BL808_GPADC_SCAN_NEG2_OFFSET)
#define BL808_GPADC_STATUS (BL808_AON_BASE + BL808_GPADC_STATUS_OFFSET)
#define BL808_GPADC_ISR (BL808_AON_BASE + BL808_GPADC_ISR_OFFSET)
#define BL808_GPADC_RESULT (BL808_AON_BASE + BL808_GPADC_RESULT_OFFSET)
#define BL808_GPADC_RAW_RESULT (BL808_AON_BASE + BL808_GPADC_RAW_RESULT_OFFSET)
#define BL808_GPADC_DEFINE (BL808_AON_BASE + BL808_GPADC_DEFINE_OFFSET)

/* Register bit definitions *************************************************/

/* GPADC_DMA_RDATA */

#define GPADC_DMA_RDATA_SHIFT (0U)
#define GPADC_DMA_RDATA_MASK  (0x3ffffff << GPADC_DMA_RDATA_SHIFT)
#define GPADC_RESULT_RAW_VAL_SHIFT (0U)
#define GPADC_RESULT_RAW_VAL_MASK (0xffff << GPADC_RESULT_RAW_VAL_SHIFT)
#define GPADC_RESULT_POS_CHN_SHIFT (21U)
#define GPADC_RESULT_POS_CHN_MASK (0x1f << GPADC_RESULT_POS_CHN_SHIFT)

/* GPADC_CONFIG */

#define GPADC_DMA_EN        (1 << 0U)
#define GPADC_FIFO_CLR      (1 << 1U)
#define GPADC_FIFO_NE       (1 << 2U)
#define GPADC_FIFO_FULL     (1 << 3U)
#define GPADC_RDY           (1 << 4U)
#define GPADC_FIFO_OVERRUN  (1 << 5U)
#define GPADC_FIFO_UNDERRUN (1 << 6U)
#define GPADC_RDY_CLR            (1 << 8U)
#define GPADC_FIFO_OVERRUN_CLR   (1 << 9U)
#define GPADC_FIFO_UNDERRUN_CLR  (1 << 10U)
#define GPADC_RDY_MASK           (1 << 12U)
#define GPADC_FIFO_OVERRUN_MASK  (1 << 13U)
#define GPADC_FIFO_UNDERRUN_MASK (1 << 14U)
#define GPADC_FIFO_DATA_COUNT_SHIFT (16U)
#define GPADC_FIFO_DATA_COUNT_MASK  (0x3f << GPADC_FIFO_DATA_COUNT_SHIFT)
#define GPADC_FIFO_THL_SHIFT        (22U)
#define GPADC_FIFO_THL_MASK         (0x3 << GPADC_FIFO_THL_SHIFT)

/* GPADC_CMD */

#define GPADC_GLOBAL_EN     (1 << 0U)
#define GPADC_CONV_START    (1 << 1U)
#define GPADC_SOFT_RST      (1 << 2U)
#define GPADC_NEG_SEL_SHIFT (3U)
#define GPADC_NEG_SEL_MASK  (0x1f << GPADC_NEG_SEL_SHIFT)
#define GPADC_POS_SEL_SHIFT (8U)
#define GPADC_POS_SEL_MASK  (0x1f << GPADC_POS_SEL_SHIFT)
#define GPADC_NEG_GND       (1 << 13U)
#define GPADC_MICBIAS_EN    (1 << 14U)
#define GPADC_MICPGA_EN     (1 << 15U)
#define GPADC_BYP_MICBOOST  (1 << 16U)
#define GPADC_RCAL_EN (1 << 17U)
#define GPADC_DWA_EN              (1 << 18U)
#define GPADC_MIC2_DIFF           (1 << 19U)
#define GPADC_MIC1_DIFF           (1 << 20U)
#define GPADC_MIC_PGA2_GAIN_SHIFT (21U)
#define GPADC_MIC_PGA2_GAIN_MASK  (0x3 << GPADC_MIC_PGA2_GAIN_SHIFT)
#define GPADC_MICBOOST_32DB_EN    (1 << 23U)
#define GPADC_CHIP_SEN_PU         (1 << 27U)
#define GPADC_SEN_SEL_SHIFT       (28U)
#define GPADC_SEN_SEL_MASK (0x7 << GPADC_SEN_SEL_SHIFT)
#define GPADC_SEN_TEST_EN  (1 << 31U)

/* GPADC_CONFIG1 */

#define GPADC_CAL_OS_EN     (1 << 0U)
#define GPADC_CONT_CONV_EN  (1 << 1U)
#define GPADC_RES_SEL_SHIFT (2U)
#define GPADC_RES_SEL_MASK  (0x7 << GPADC_RES_SEL_SHIFT)
#define GPADC_VCM_SEL_EN    (1 << 8U)
#define GPADC_VCM_HYST_SEL  (1 << 9U)
#define GPADC_LOWV_DET_EN   (1 << 10U)
#define GPADC_PWM_TRG_EN        (1 << 11U)
#define GPADC_CLK_ANA_DLY_SHIFT (12U)
#define GPADC_CLK_ANA_DLY_MASK  (0xf << GPADC_CLK_ANA_DLY_SHIFT)
#define GPADC_CLK_ANA_DLY_EN    (1 << 16U)
#define GPADC_CLK_ANA_INV         (1 << 17U)
#define GPADC_CLK_DIV_RATIO_SHIFT (18U)
#define GPADC_CLK_DIV_RATIO_MASK  (0x7 << GPADC_CLK_DIV_RATIO_SHIFT)
#define GPADC_SCAN_LENGTH_SHIFT   (21U)
#define GPADC_SCAN_LENGTH_MASK    (0xf << GPADC_SCAN_LENGTH_SHIFT)
#define GPADC_SCAN_EN             (1 << 25U)
#define GPADC_DITHER_EN           (1 << 26U)
#define GPADC_V11_SEL_SHIFT       (27U)
#define GPADC_V11_SEL_MASK        (0x3 << GPADC_V11_SEL_SHIFT)
#define GPADC_V18_SEL_SHIFT       (29U)
#define GPADC_V18_SEL_MASK        (0x3 << GPADC_V18_SEL_SHIFT)

/* GPADC_CONFIG2 */

#define GPADC_DIFF_MODE        (1 << 2U)
#define GPADC_VREF_SEL         (1 << 3U)
#define GPADC_VBAT_EN          (1 << 4U)
#define GPADC_TSEXT_SEL        (1 << 5U)
#define GPADC_TS_EN            (1 << 6U)
#define GPADC_PGA_VCM_SHIFT    (7U)
#define GPADC_PGA_VCM_MASK     (0x3 << GPADC_PGA_VCM_SHIFT)
#define GPADC_PGA_OS_CAL_SHIFT (9U)
#define GPADC_PGA_OS_CAL_MASK  (0xf << GPADC_PGA_OS_CAL_SHIFT)
#define GPADC_PGA_EN           (1 << 13U)
#define GPADC_PGA_VCMI_EN      (1 << 14U)
#define GPADC_CHOP_MODE_SHIFT  (15U)
#define GPADC_CHOP_MODE_MASK   (0x3 << GPADC_CHOP_MODE_SHIFT)
#define GPADC_BIAS_SEL         (1 << 17U)
#define GPADC_TEST_EN          (1 << 18U)
#define GPADC_TEST_SEL_SHIFT   (19U)
#define GPADC_TEST_SEL_MASK    (0x7 << GPADC_TEST_SEL_SHIFT)
#define GPADC_PGA2_GAIN_SHIFT  (22U)
#define GPADC_PGA2_GAIN_MASK   (0x7 << GPADC_PGA2_GAIN_SHIFT)
#define GPADC_PGA1_GAIN_SHIFT  (25U)
#define GPADC_PGA1_GAIN_MASK   (0x7 << GPADC_PGA1_GAIN_SHIFT)
#define GPADC_DLY_SEL_SHIFT    (28U)
#define GPADC_DLY_SEL_MASK     (0x7 << GPADC_DLY_SEL_SHIFT)
#define GPADC_TSVBE_LOW        (1 << 31U)

/* GPADC_SCAN_n */

#define GPADC_SCAN_SHIFT(n) (5 * (n % 6))
#define GPADC_SCAN_MASK(n) (0x1f << GPADC_SCAN_POS_SHIFT(n))

/* GPADC_ISR */

#define GPADC_NEG_SATUR      (1 << 0U)
#define GPADC_POS_SATUR      (1 << 1U)
#define GPADC_NEG_SATUR_CLR  (1 << 4U)
#define GPADC_POS_SATUR_CLR  (1 << 5U)
#define GPADC_NEG_SATUR_MASK (1 << 8U)
#define GPADC_POS_SATUR_MASK (1 << 9U)

#endif /* __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_GPADC_H */
