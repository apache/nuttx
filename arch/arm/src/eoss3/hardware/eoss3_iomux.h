/****************************************************************************
 * arch/arm/src/eoss3/hardware/eoss3_iomux.h
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_EOSS3_HARDWARE_EOSS3_IOMUX_H
#define __ARCH_ARM_SRC_EOSS3_HARDWARE_EOSS3_IOMUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Compute PAD Control Register Address */

#define EOSS3_PAD_X_CTRL(pad)  (EOSS3_IO_MUX_BASE + (pad << 2))
#define EOSS3_PAD_SEL(idx)     (EOSS3_IO_MUX_BASE + 0x100 + ((idx - 1) << 2))

#define EOSS3_IOMUX_PAD_0_CTRL    (EOSS3_PAD_X_CTRL(0))
#define EOSS3_IOMUX_PAD_1_CTRL    (EOSS3_PAD_X_CTRL(1))
#define EOSS3_IOMUX_PAD_2_CTRL    (EOSS3_PAD_X_CTRL(2))
#define EOSS3_IOMUX_PAD_3_CTRL    (EOSS3_PAD_X_CTRL(3))
#define EOSS3_IOMUX_PAD_4_CTRL    (EOSS3_PAD_X_CTRL(4))
#define EOSS3_IOMUX_PAD_5_CTRL    (EOSS3_PAD_X_CTRL(5))
#define EOSS3_IOMUX_PAD_6_CTRL    (EOSS3_PAD_X_CTRL(6))
#define EOSS3_IOMUX_PAD_7_CTRL    (EOSS3_PAD_X_CTRL(7))
#define EOSS3_IOMUX_PAD_8_CTRL    (EOSS3_PAD_X_CTRL(8))
#define EOSS3_IOMUX_PAD_9_CTRL    (EOSS3_PAD_X_CTRL(9))
#define EOSS3_IOMUX_PAD_10_CTRL   (EOSS3_PAD_X_CTRL(10))
#define EOSS3_IOMUX_PAD_11_CTRL   (EOSS3_PAD_X_CTRL(11))
#define EOSS3_IOMUX_PAD_12_CTRL   (EOSS3_PAD_X_CTRL(12))
#define EOSS3_IOMUX_PAD_13_CTRL   (EOSS3_PAD_X_CTRL(13))
#define EOSS3_IOMUX_PAD_14_CTRL   (EOSS3_PAD_X_CTRL(14))
#define EOSS3_IOMUX_PAD_15_CTRL   (EOSS3_PAD_X_CTRL(15))
#define EOSS3_IOMUX_PAD_16_CTRL   (EOSS3_PAD_X_CTRL(16))
#define EOSS3_IOMUX_PAD_17_CTRL   (EOSS3_PAD_X_CTRL(17))
#define EOSS3_IOMUX_PAD_18_CTRL   (EOSS3_PAD_X_CTRL(18))
#define EOSS3_IOMUX_PAD_19_CTRL   (EOSS3_PAD_X_CTRL(19))
#define EOSS3_IOMUX_PAD_20_CTRL   (EOSS3_PAD_X_CTRL(20))
#define EOSS3_IOMUX_PAD_21_CTRL   (EOSS3_PAD_X_CTRL(21))
#define EOSS3_IOMUX_PAD_22_CTRL   (EOSS3_PAD_X_CTRL(22))
#define EOSS3_IOMUX_PAD_23_CTRL   (EOSS3_PAD_X_CTRL(23))
#define EOSS3_IOMUX_PAD_24_CTRL   (EOSS3_PAD_X_CTRL(24))
#define EOSS3_IOMUX_PAD_25_CTRL   (EOSS3_PAD_X_CTRL(25))
#define EOSS3_IOMUX_PAD_26_CTRL   (EOSS3_PAD_X_CTRL(26))
#define EOSS3_IOMUX_PAD_27_CTRL   (EOSS3_PAD_X_CTRL(27))
#define EOSS3_IOMUX_PAD_28_CTRL   (EOSS3_PAD_X_CTRL(28))
#define EOSS3_IOMUX_PAD_29_CTRL   (EOSS3_PAD_X_CTRL(29))
#define EOSS3_IOMUX_PAD_30_CTRL   (EOSS3_PAD_X_CTRL(30))
#define EOSS3_IOMUX_PAD_31_CTRL   (EOSS3_PAD_X_CTRL(31))
#define EOSS3_IOMUX_PAD_32_CTRL   (EOSS3_PAD_X_CTRL(32))
#define EOSS3_IOMUX_PAD_33_CTRL   (EOSS3_PAD_X_CTRL(33))
#define EOSS3_IOMUX_PAD_34_CTRL   (EOSS3_PAD_X_CTRL(34))
#define EOSS3_IOMUX_PAD_35_CTRL   (EOSS3_PAD_X_CTRL(35))
#define EOSS3_IOMUX_PAD_36_CTRL   (EOSS3_PAD_X_CTRL(36))
#define EOSS3_IOMUX_PAD_37_CTRL   (EOSS3_PAD_X_CTRL(37))
#define EOSS3_IOMUX_PAD_38_CTRL   (EOSS3_PAD_X_CTRL(38))
#define EOSS3_IOMUX_PAD_39_CTRL   (EOSS3_PAD_X_CTRL(39))
#define EOSS3_IOMUX_PAD_40_CTRL   (EOSS3_PAD_X_CTRL(40))
#define EOSS3_IOMUX_PAD_41_CTRL   (EOSS3_PAD_X_CTRL(41))
#define EOSS3_IOMUX_PAD_42_CTRL   (EOSS3_PAD_X_CTRL(42))
#define EOSS3_IOMUX_PAD_43_CTRL   (EOSS3_PAD_X_CTRL(43))
#define EOSS3_IOMUX_PAD_44_CTRL   (EOSS3_PAD_X_CTRL(44))
#define EOSS3_IOMUX_PAD_45_CTRL   (EOSS3_PAD_X_CTRL(45))

/* IO_REG Select Register Address */

#define EOSS3_IO_REG_SEL_OFFSET  (0x160)
#define EOSS3_IO_REG_SEL         (EOSS3_IO_REG_SEL_OFFSET + EOSS3_IO_MUX_BASE)

/* MISC IO Register Addresses */

#define EOSS3_MISC_IO_INPUT_OFFSET   (0x100)
#define EOSS3_MISC_IO_OUTPUT_OFFSET  (0x104)

#define EOSS3_MISC_IO_INPUT   (EOSS3_MISC_IO_INPUT_OFFSET + EOSS3_MISC_BASE)
#define EOSS3_MISC_IO_OUTPUT  (EOSS3_MISC_IO_OUTPUT_OFFSET + EOSS3_MISC_BASE)

/* Pad Control Register [13:0] */

#define PAD_CTRL_MASK             (0x1fff)
#define PAD_FUNC_SEL_SHIFT        (0)
#define PAD_FUNC_SEL_MASK         (0x3 << PAD_FUNC_SEL_SHIFT)
#define PAD_CTRL_SEL_SHIFT        (3)
#define PAD_CTRL_SEL_MASK         (0x3 << PAD_CTRL_SEL_SHIFT)
#define PAD_OEN                   (1 << 5)
#define PAD_P_SHIFT               (1 << 6)
#define PAD_P_MASK                (0x3 << PAD_P_SHIFT)
#define PAD_E_SHIFT               (1 << 8)
#define PAD_E_MASK                (0x3 << PAD_E_SHIFT)
#define PAD_SR                    (1 << 10)
#define PAD_REN                   (1 << 11)
#define PAD_SMT                   (1 << 12)

#define PAD_FUNC_0                (0x0 << PAD_FUNC_SEL_SHIFT)
#define PAD_FUNC_1                (0x1 << PAD_FUNC_SEL_SHIFT)
#define PAD_FUNC_2                (0x2 << PAD_FUNC_SEL_SHIFT)
#define PAD_FUNC_3                (0x3 << PAD_FUNC_SEL_SHIFT)

#define PAD_CTRL_A0               (0x0 << PAD_CTRL_SEL_SHIFT)
#define PAD_CTRL_OTHER            (0x1 << PAD_CTRL_SEL_SHIFT)
#define PAD_CTRL_FABRIC           (0x2 << PAD_CTRL_SEL_SHIFT)

#define PAD_HIGHZ                 (0)
#define PAD_PULLUP                (0x1 << PAD_P_SHIFT)
#define PAD_PULLDOWN              (0x2 << PAD_P_SHIFT)
#define PAD_KEEP                  (0x3 << PAD_P_SHIFT)

#define PAD_SR_SLOW               (0)
#define PAD_SR_FAST               (PAD_SR)

/* Input PAD Selection
 * These vales are packed sel_val[2:0] idx[8:3]
 * idx can be converted to the SEL address with EOSS3_PAD_SEL(idx)
 * idx of 0 is used to indicate no PAD_SEL input feature should be configured
 */

#define EOSS3_PAD_SEL_IDX_SHIFT   (3)
#define EOSS3_PAD_SEL_IDX_MASK    (0x3f << EOSS3_PAD_SEL_IDX_SHIFT)
#define EOSS3_PAD_SEL_VAL_MASK    (0x7)

#define SDA0_SEL_1                (0x0 | (1 << EOSS3_PAD_SEL_IDX_SHIFT))
#define SDA0_SEL_P1               (0x1 | (1 << EOSS3_PAD_SEL_IDX_SHIFT))

#define SDA1_SEL_1                (0x0 | (2 << EOSS3_PAD_SEL_IDX_SHIFT))
#define SDA1_SEL_P15              (0x1 | (2 << EOSS3_PAD_SEL_IDX_SHIFT))
#define SDA1_SEL_P32              (0x2 | (2 << EOSS3_PAD_SEL_IDX_SHIFT))
#define SDA1_SEL_P44              (0x3 | (2 << EOSS3_PAD_SEL_IDX_SHIFT))

#define SDA2_SEL_1                (0x0 | (3 << EOSS3_PAD_SEL_IDX_SHIFT))
#define SDA2_SEL_P41              (0x1 | (3 << EOSS3_PAD_SEL_IDX_SHIFT))

#define SCL0_SEL_1                (0x0 | (4 << EOSS3_PAD_SEL_IDX_SHIFT))
#define SCL0_SEL_P0               (0x1 | (4 << EOSS3_PAD_SEL_IDX_SHIFT))

#define SCL1_SEL_1                (0x0 | (5 << EOSS3_PAD_SEL_IDX_SHIFT))
#define SCL1_SEL_P14              (0x1 | (5 << EOSS3_PAD_SEL_IDX_SHIFT))
#define SCL1_SEL_P33              (0x2 | (5 << EOSS3_PAD_SEL_IDX_SHIFT))
#define SCL1_SEL_P45              (0x3 | (5 << EOSS3_PAD_SEL_IDX_SHIFT))

#define SCL2_SEL_1                (0x0 | (6 << EOSS3_PAD_SEL_IDX_SHIFT))
#define SCL2_SEL_P40              (0x1 | (6 << EOSS3_PAD_SEL_IDX_SHIFT))

#define SPIS_CLK_SEL_P16          (0x0 | (7 << EOSS3_PAD_SEL_IDX_SHIFT))
#define SPIS_CLK_SEL_0            (0x1 | (7 << EOSS3_PAD_SEL_IDX_SHIFT))

#define SPIS_SSN_SEL_P20          (0x0 | (8 << EOSS3_PAD_SEL_IDX_SHIFT))
#define SPIS_SSN_SEL_0            (0x1 | (8 << EOSS3_PAD_SEL_IDX_SHIFT))

#define SPIS_MOSI_SEL_P19         (0x0 | (9 << EOSS3_PAD_SEL_IDX_SHIFT))
#define SPIS_MOSI_SEL_0           (0x1 | (9 << EOSS3_PAD_SEL_IDX_SHIFT))

#define SPIM_MISO_SEL_P36         (0x0 | (10 << EOSS3_PAD_SEL_IDX_SHIFT))
#define SPIM_MISO_SEL_0           (0x1 | (10 << EOSS3_PAD_SEL_IDX_SHIFT))

#define PDM_DATA_SEL_0            (0x0 | (11 << EOSS3_PAD_SEL_IDX_SHIFT))
#define PDM_DATA_SEL_P10          (0x1 | (11 << EOSS3_PAD_SEL_IDX_SHIFT))
#define PDM_DATA_SEL_P28          (0x2 | (11 << EOSS3_PAD_SEL_IDX_SHIFT))

#define I2S_DATA_SEL_0            (0x0 | (12 << EOSS3_PAD_SEL_IDX_SHIFT))
#define I2S_DATA_SEL_P10          (0x1 | (12 << EOSS3_PAD_SEL_IDX_SHIFT))
#define I2S_DATA_SEL_P28          (0x2 | (12 << EOSS3_PAD_SEL_IDX_SHIFT))

#define FCLK_SEL_P2               (0x0 | (13 << EOSS3_PAD_SEL_IDX_SHIFT))
#define FCLK_SEL_P7               (0x1 | (13 << EOSS3_PAD_SEL_IDX_SHIFT))
#define FCLK_SEL_P10              (0x2 | (13 << EOSS3_PAD_SEL_IDX_SHIFT))
#define FCLK_SEL_P26              (0x3 | (13 << EOSS3_PAD_SEL_IDX_SHIFT))
#define FCLK_SEL_P27              (0x4 | (13 << EOSS3_PAD_SEL_IDX_SHIFT))
#define FCLK_SEL_L                (0x6 | (13 << EOSS3_PAD_SEL_IDX_SHIFT))
#define FCLK_SEL_H                (0x7 | (13 << EOSS3_PAD_SEL_IDX_SHIFT))

#define UART_RXD_SEL_0            (0x0 | (14 << EOSS3_PAD_SEL_IDX_SHIFT))
#define UART_RXD_SEL_P14          (0x1 | (14 << EOSS3_PAD_SEL_IDX_SHIFT))
#define UART_RXD_SEL_P16          (0x2 | (14 << EOSS3_PAD_SEL_IDX_SHIFT))
#define UART_RXD_SEL_P25          (0x3 | (14 << EOSS3_PAD_SEL_IDX_SHIFT))
#define UART_RXD_SEL_P45          (0x4 | (14 << EOSS3_PAD_SEL_IDX_SHIFT))

#define IRDA_DIRIN_SEL_0          (0x0 | (15 << EOSS3_PAD_SEL_IDX_SHIFT))
#define IRDA_DIRIN_SEL_P6         (0x1 | (15 << EOSS3_PAD_SEL_IDX_SHIFT))
#define IRDA_DIRIN_SEL_P15        (0x2 | (15 << EOSS3_PAD_SEL_IDX_SHIFT))
#define IRDA_DIRIN_SEL_P21        (0x3 | (15 << EOSS3_PAD_SEL_IDX_SHIFT))
#define IRDA_DIRIN_SEL_P24        (0x4 | (15 << EOSS3_PAD_SEL_IDX_SHIFT))
#define IRDA_DIRIN_SEL_P28        (0x5 | (15 << EOSS3_PAD_SEL_IDX_SHIFT))
#define IRDA_DIRIN_SEL_P40        (0x6 | (15 << EOSS3_PAD_SEL_IDX_SHIFT))
#define IRDA_DIRIN_SEL_P44        (0x7 | (15 << EOSS3_PAD_SEL_IDX_SHIFT))

#define S_INTR_0_SEL_0            (0x0 | (16 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_0_SEL_P3           (0x1 | (16 << EOSS3_PAD_SEL_IDX_SHIFT))

#define S_INTR_1_SEL_0            (0x0 | (17 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_1_SEL_P2           (0x1 | (17 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_1_SEL_P6           (0x2 | (17 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_1_SEL_P18          (0x3 | (17 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_1_SEL_P24          (0x4 | (17 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_1_SEL_P35          (0x5 | (17 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_1_SEL_P36          (0x6 | (17 << EOSS3_PAD_SEL_IDX_SHIFT))

#define S_INTR_2_SEL_0            (0x0 | (18 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_2_SEL_P4           (0x1 | (18 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_2_SEL_P8           (0x2 | (18 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_2_SEL_P21          (0x3 | (18 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_2_SEL_P25          (0x4 | (18 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_2_SEL_P37          (0x5 | (18 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_2_SEL_P38          (0x6 | (18 << EOSS3_PAD_SEL_IDX_SHIFT))

#define S_INTR_3_SEL_0            (0x0 | (19 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_3_SEL_P5           (0x1 | (19 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_3_SEL_P8           (0x2 | (19 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_3_SEL_P22          (0x3 | (19 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_3_SEL_P28          (0x4 | (19 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_3_SEL_P39          (0x5 | (19 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_3_SEL_P40          (0x6 | (19 << EOSS3_PAD_SEL_IDX_SHIFT))

#define S_INTR_4_SEL_0            (0x0 | (20 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_4_SEL_P7           (0x1 | (20 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_4_SEL_P10          (0x2 | (20 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_4_SEL_P26          (0x3 | (20 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_4_SEL_P29          (0x4 | (20 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_4_SEL_P44          (0x5 | (20 << EOSS3_PAD_SEL_IDX_SHIFT))

#define S_INTR_5_SEL_0            (0x0 | (21 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_5_SEL_P11          (0x1 | (21 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_5_SEL_P14          (0x2 | (21 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_5_SEL_P27          (0x3 | (21 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_5_SEL_P30          (0x4 | (21 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_5_SEL_P45          (0x5 | (21 << EOSS3_PAD_SEL_IDX_SHIFT))

#define S_INTR_6_SEL_0            (0x0 | (22 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_6_SEL_P12          (0x1 | (22 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_6_SEL_P15          (0x2 | (22 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_6_SEL_P31          (0x3 | (22 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_6_SEL_P32          (0x4 | (22 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_6_SEL_P41          (0x5 | (22 << EOSS3_PAD_SEL_IDX_SHIFT))

#define S_INTR_7_SEL_0            (0x0 | (23 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_7_SEL_P13          (0x1 | (23 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_7_SEL_P23          (0x2 | (23 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_7_SEL_P33          (0x3 | (23 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_7_SEL_P34          (0x4 | (23 << EOSS3_PAD_SEL_IDX_SHIFT))
#define S_INTR_7_SEL_P42          (0x5 | (23 << EOSS3_PAD_SEL_IDX_SHIFT))

#define NUARTCTS_SEL_0            (0x0 | (24 << EOSS3_PAD_SEL_IDX_SHIFT))
#define NUARTCTS_SEL_P17          (0x1 | (24 << EOSS3_PAD_SEL_IDX_SHIFT))
#define NUARTCTS_SEL_P22          (0x2 | (24 << EOSS3_PAD_SEL_IDX_SHIFT))

#define SW_CLK_SEL_P14_45         (0x0 | (29 << EOSS3_PAD_SEL_IDX_SHIFT))
#define SW_CLK_SEL_0              (0x1 | (29 << EOSS3_PAD_SEL_IDX_SHIFT))

#define SW_IO_SEL_P15_44          (0x0 | (30 << EOSS3_PAD_SEL_IDX_SHIFT))
#define SW_IO_SEL_0               (0x1 | (30 << EOSS3_PAD_SEL_IDX_SHIFT))

#define FBIO_SEL_1_0              (0x0 | (33 << EOSS3_PAD_SEL_IDX_SHIFT))
#define FBIO_SEL_1_PAD            (0x1 | (33 << EOSS3_PAD_SEL_IDX_SHIFT))

#define FBIO_SEL_2_0              (0x0 | (34 << EOSS3_PAD_SEL_IDX_SHIFT))
#define FBIO_SEL_2_PAD            (0x1 | (34 << EOSS3_PAD_SEL_IDX_SHIFT))

#define SPI_SENS_MISO_SEL_0       (0x0 | (37 << EOSS3_PAD_SEL_IDX_SHIFT))
#define SPI_SENS_MISO_SEL_P8      (0x1 | (37 << EOSS3_PAD_SEL_IDX_SHIFT))
#define SPI_SENS_MISO_SEL_P29     (0x2 | (37 << EOSS3_PAD_SEL_IDX_SHIFT))

#define SPI_SENS_MOSI_SEL_0       (0x0 | (38 << EOSS3_PAD_SEL_IDX_SHIFT))
#define SPI_SENS_MOSI_SEL_P6      (0x1 | (38 << EOSS3_PAD_SEL_IDX_SHIFT))
#define SPI_SENS_MOSI_SEL_P28     (0x2 | (38 << EOSS3_PAD_SEL_IDX_SHIFT))

#define I2S_WD_CLKIN_SEL_0        (0x0 | (41 << EOSS3_PAD_SEL_IDX_SHIFT))
#define I2S_WD_CLKIN_SEL_P23      (0x1 | (41 << EOSS3_PAD_SEL_IDX_SHIFT))

#define I2S_CLKIN_SEL_0           (0x0 | (42 << EOSS3_PAD_SEL_IDX_SHIFT))
#define I2S_CLKIN_SEL_P31         (0x1 | (42 << EOSS3_PAD_SEL_IDX_SHIFT))

#define PDM_STAT_IN_SEL_0         (0x0 | (43 << EOSS3_PAD_SEL_IDX_SHIFT))
#define PDM_STAT_IN_SEL_P9        (0x0 | (43 << EOSS3_PAD_SEL_IDX_SHIFT))
#define PDM_STAT_IN_SEL_P30       (0x0 | (43 << EOSS3_PAD_SEL_IDX_SHIFT))

#define PDM_CLKIN_SEL_0           (0x0 | (44 << EOSS3_PAD_SEL_IDX_SHIFT))
#define PDM_CLKIN_SEL_P38         (0x0 | (44 << EOSS3_PAD_SEL_IDX_SHIFT))

/* IO_REG Select Register */

#define IO_REG_P6                 (0)  /* Low Enable */
#define IO_REG_P24                (0)  /* High Enable */
#define IO_REG_P9                 (1)  /* Low Enable */
#define IO_REG_P26                (1)  /* High Enable */
#define IO_REG_P11                (2)  /* Low Enable */
#define IO_REG_P28                (2)  /* High Enable */
#define IO_REG_P14                (3)  /* Low Enable */
#define IO_REG_P30                (3)  /* High Enable */
#define IO_REG_P18                (4)  /* Low Enable */
#define IO_REG_P31                (4)  /* High Enable */
#define IO_REG_P21                (5)  /* Low Enable */
#define IO_REG_P36                (5)  /* High Enable */
#define IO_REG_P22                (6)  /* Low Enable */
#define IO_REG_P38                (6)  /* High Enable */
#define IO_REG_P23                (7)  /* Low Enable */
#define IO_REG_P45                (7)  /* High Enable */
#define IO_REG_MASK               (7)

#define IO_REG_START_HI_PAD       (24)  /* First pad where enable is high */

#endif /* __ARCH_ARM_SRC_EOSS3_HARDWARE_EOSS3_IOMUX_H */
