/****************************************************************************
 * arch/xtensa/src/esp32s2/hardware/esp32s2_spi.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S2_HARDWARE_ESP32S2_SPI_H
#define __ARCH_XTENSA_SRC_ESP32S2_HARDWARE_ESP32S2_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "esp32s2_soc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI_CMD_REG register */

#define SPI_CMD_REG(i) (REG_SPI_BASE(i) + 0x0)

/* SPI_USR : R/W; bitpos: [24]; default: 0;
 * User define command enable.  An operation will be triggered when the bit
 * is set. The bit will be cleared once the operation done.1: enable 0:
 * disable. Can not be changed by CONF_buf.
 */

#define SPI_USR    (BIT(24))
#define SPI_USR_M  (SPI_USR_V << SPI_USR_S)
#define SPI_USR_V  0x00000001
#define SPI_USR_S  24

/* SPI_CONF_BITLEN : R/W; bitpos: [22:0]; default: 104;
 * Define the spi_clk cycles of  SPI_CONF state.
 */

#define SPI_CONF_BITLEN    0x007FFFFF
#define SPI_CONF_BITLEN_M  (SPI_CONF_BITLEN_V << SPI_CONF_BITLEN_S)
#define SPI_CONF_BITLEN_V  0x007FFFFF
#define SPI_CONF_BITLEN_S  0

/* SPI_ADDR_REG register */

#define SPI_ADDR_REG(i)    (REG_SPI_BASE(i) + 0x4)

/* SPI_USR_ADDR_VALUE : R/W; bitpos: [31:0]; default: 0;
 * [31:8]:address to slave, [7:0]:Reserved.
 */

#define SPI_USR_ADDR_VALUE    0xFFFFFFFF
#define SPI_USR_ADDR_VALUE_M  (SPI_USR_ADDR_VALUE_V << SPI_USR_ADDR_VALUE_S)
#define SPI_USR_ADDR_VALUE_V  0xFFFFFFFF
#define SPI_USR_ADDR_VALUE_S  0

/* SPI_CTRL_REG register */

#define SPI_CTRL_REG(i)       (REG_SPI_BASE(i) + 0x8)

/* SPI_WR_BIT_ORDER : R/W; bitpos: [26]; default: 0;
 * In command address write-data (MOSI) phases 1: LSB firs 0: MSB first
 */

#define SPI_WR_BIT_ORDER    (BIT(26))
#define SPI_WR_BIT_ORDER_M  (SPI_WR_BIT_ORDER_V << SPI_WR_BIT_ORDER_S)
#define SPI_WR_BIT_ORDER_V  0x00000001
#define SPI_WR_BIT_ORDER_S  26

/* SPI_RD_BIT_ORDER : R/W; bitpos: [25]; default: 0;
 * In read-data (MISO) phase 1: LSB first 0: MSB first
 */

#define SPI_RD_BIT_ORDER    (BIT(25))
#define SPI_RD_BIT_ORDER_M  (SPI_RD_BIT_ORDER_V << SPI_RD_BIT_ORDER_S)
#define SPI_RD_BIT_ORDER_V  0x00000001
#define SPI_RD_BIT_ORDER_S  25

/* SPI_WP_REG : R/W; bitpos: [21]; default: 1;
 * Write protect signal output when SPI is idle.  1: output high, 0: output
 * low.
 */

#define SPI_WP_REG    (BIT(21))
#define SPI_WP_REG_M  (SPI_WP_REG_V << SPI_WP_REG_S)
#define SPI_WP_REG_V  0x00000001
#define SPI_WP_REG_S  21

/* SPI_D_POL : R/W; bitpos: [19]; default: 1;
 * The bit is used to set MOSI line polarity, 1: high 0, low
 */

#define SPI_D_POL    (BIT(19))
#define SPI_D_POL_M  (SPI_D_POL_V << SPI_D_POL_S)
#define SPI_D_POL_V  0x00000001
#define SPI_D_POL_S  19

/* SPI_Q_POL : R/W; bitpos: [18]; default: 1;
 * The bit is used to set MISO line polarity, 1: high 0, low
 */

#define SPI_Q_POL    (BIT(18))
#define SPI_Q_POL_M  (SPI_Q_POL_V << SPI_Q_POL_S)
#define SPI_Q_POL_V  0x00000001
#define SPI_Q_POL_S  18

/* SPI_FREAD_OCT : R/W; bitpos: [16]; default: 0;
 * In the read operations read-data phase apply 8 signals. 1: enable 0:
 * disable.
 */

#define SPI_FREAD_OCT    (BIT(16))
#define SPI_FREAD_OCT_M  (SPI_FREAD_OCT_V << SPI_FREAD_OCT_S)
#define SPI_FREAD_OCT_V  0x00000001
#define SPI_FREAD_OCT_S  16

/* SPI_FREAD_QUAD : R/W; bitpos: [15]; default: 0;
 * In the read operations read-data phase apply 4 signals. 1: enable 0:
 * disable.
 */

#define SPI_FREAD_QUAD    (BIT(15))
#define SPI_FREAD_QUAD_M  (SPI_FREAD_QUAD_V << SPI_FREAD_QUAD_S)
#define SPI_FREAD_QUAD_V  0x00000001
#define SPI_FREAD_QUAD_S  15

/* SPI_FREAD_DUAL : R/W; bitpos: [14]; default: 0;
 * In the read operations, read-data phase apply 2 signals. 1: enable 0:
 * disable.
 */

#define SPI_FREAD_DUAL    (BIT(14))
#define SPI_FREAD_DUAL_M  (SPI_FREAD_DUAL_V << SPI_FREAD_DUAL_S)
#define SPI_FREAD_DUAL_V  0x00000001
#define SPI_FREAD_DUAL_S  14

/* SPI_FCMD_OCT : R/W; bitpos: [10]; default: 0;
 * Apply 8 signals during command phase 1:enable 0: disable
 */

#define SPI_FCMD_OCT    (BIT(10))
#define SPI_FCMD_OCT_M  (SPI_FCMD_OCT_V << SPI_FCMD_OCT_S)
#define SPI_FCMD_OCT_V  0x00000001
#define SPI_FCMD_OCT_S  10

/* SPI_FCMD_QUAD : R/W; bitpos: [9]; default: 0;
 * Apply 4 signals during command phase 1:enable 0: disable
 */

#define SPI_FCMD_QUAD    (BIT(9))
#define SPI_FCMD_QUAD_M  (SPI_FCMD_QUAD_V << SPI_FCMD_QUAD_S)
#define SPI_FCMD_QUAD_V  0x00000001
#define SPI_FCMD_QUAD_S  9

/* SPI_FCMD_DUAL : R/W; bitpos: [8]; default: 0;
 * Apply 2 signals during command phase 1:enable 0: disable
 */

#define SPI_FCMD_DUAL    (BIT(8))
#define SPI_FCMD_DUAL_M  (SPI_FCMD_DUAL_V << SPI_FCMD_DUAL_S)
#define SPI_FCMD_DUAL_V  0x00000001
#define SPI_FCMD_DUAL_S  8

/* SPI_FADDR_OCT : R/W; bitpos: [7]; default: 0;
 * Apply 8 signals during addr phase 1:enable 0: disable
 */

#define SPI_FADDR_OCT    (BIT(7))
#define SPI_FADDR_OCT_M  (SPI_FADDR_OCT_V << SPI_FADDR_OCT_S)
#define SPI_FADDR_OCT_V  0x00000001
#define SPI_FADDR_OCT_S  7

/* SPI_FADDR_QUAD : R/W; bitpos: [6]; default: 0;
 * Apply 4 signals during addr phase 1:enable 0: disable
 */

#define SPI_FADDR_QUAD    (BIT(6))
#define SPI_FADDR_QUAD_M  (SPI_FADDR_QUAD_V << SPI_FADDR_QUAD_S)
#define SPI_FADDR_QUAD_V  0x00000001
#define SPI_FADDR_QUAD_S  6

/* SPI_FADDR_DUAL : R/W; bitpos: [5]; default: 0;
 * Apply 2 signals during addr phase 1:enable 0: disable
 */

#define SPI_FADDR_DUAL    (BIT(5))
#define SPI_FADDR_DUAL_M  (SPI_FADDR_DUAL_V << SPI_FADDR_DUAL_S)
#define SPI_FADDR_DUAL_V  0x00000001
#define SPI_FADDR_DUAL_S  5

/* SPI_DUMMY_OUT : R/W; bitpos: [3]; default: 0;
 * In the dummy phase the signal level of spi is output by the spi
 * controller.
 */

#define SPI_DUMMY_OUT    (BIT(3))
#define SPI_DUMMY_OUT_M  (SPI_DUMMY_OUT_V << SPI_DUMMY_OUT_S)
#define SPI_DUMMY_OUT_V  0x00000001
#define SPI_DUMMY_OUT_S  3

/* SPI_EXT_HOLD_EN : R/W; bitpos: [2]; default: 0;
 * Set the bit to hold spi. The bit is combined with
 * spi_usr_prep_hold,spi_usr_cmd_hold,spi_usr_addr_hold, spi_usr_dummy_hold,
 * spi_usr_din_hold,spi_usr_dout_hold and spi_usr_hold_pol.
 */

#define SPI_EXT_HOLD_EN    (BIT(2))
#define SPI_EXT_HOLD_EN_M  (SPI_EXT_HOLD_EN_V << SPI_EXT_HOLD_EN_S)
#define SPI_EXT_HOLD_EN_V  0x00000001
#define SPI_EXT_HOLD_EN_S  2

/* SPI_CTRL1_REG register */

#define SPI_CTRL1_REG(i)     (REG_SPI_BASE(i) + 0xc)

/* SPI_CS_HOLD_DELAY : R/W; bitpos: [19:14]; default: 1;
 * SPI cs signal is delayed by spi clock cycles.
 */

#define SPI_CS_HOLD_DELAY    0x0000003F
#define SPI_CS_HOLD_DELAY_M  (SPI_CS_HOLD_DELAY_V << SPI_CS_HOLD_DELAY_S)
#define SPI_CS_HOLD_DELAY_V  0x0000003F
#define SPI_CS_HOLD_DELAY_S  14

/* SPI_W16_17_WR_ENA : R/W; bitpos: [4]; default: 1;
 * 1:reg_buf[16] [17] can be written   0:reg_buf[16] [17] can not  be
 * written.
 */

#define SPI_W16_17_WR_ENA    (BIT(4))
#define SPI_W16_17_WR_ENA_M  (SPI_W16_17_WR_ENA_V << SPI_W16_17_WR_ENA_S)
#define SPI_W16_17_WR_ENA_V  0x00000001
#define SPI_W16_17_WR_ENA_S  4

/* SPI_RSCK_DATA_OUT : R/W; bitpos: [3]; default: 0;
 * It saves half a cycle when tsck is the same as rsck. 1: output data at
 * rsck posedge   0: output data at tsck posedge
 */

#define SPI_RSCK_DATA_OUT    (BIT(3))
#define SPI_RSCK_DATA_OUT_M  (SPI_RSCK_DATA_OUT_V << SPI_RSCK_DATA_OUT_S)
#define SPI_RSCK_DATA_OUT_V  0x00000001
#define SPI_RSCK_DATA_OUT_S  3

/* SPI_CLK_MODE_13 : R/W; bitpos: [2]; default: 0;
 * {CPOL, CPHA},1: support spi clk mode 1 and 3, first edge output data
 * B[0]/B[7].  0: support spi clk mode 0 and 2, first edge output data
 * B[1]/B[6].
 */

#define SPI_CLK_MODE_13    (BIT(2))
#define SPI_CLK_MODE_13_M  (SPI_CLK_MODE_13_V << SPI_CLK_MODE_13_S)
#define SPI_CLK_MODE_13_V  0x00000001
#define SPI_CLK_MODE_13_S  2

/* SPI_CLK_MODE : R/W; bitpos: [1:0]; default: 0;
 * SPI clock mode bits. 0: SPI clock is off when CS inactive 1: SPI clock is
 * delayed one cycle after CS inactive 2: SPI clock is delayed two cycles
 * after CS inactive 3: SPI clock is alwasy on.
 */

#define SPI_CLK_MODE    0x00000003
#define SPI_CLK_MODE_M  (SPI_CLK_MODE_V << SPI_CLK_MODE_S)
#define SPI_CLK_MODE_V  0x00000003
#define SPI_CLK_MODE_S  0

/* SPI_CTRL2_REG register */

#define SPI_CTRL2_REG(i)    (REG_SPI_BASE(i) + 0x10)

/* SPI_CS_DELAY_NUM : R/W; bitpos: [30:29]; default: 0;
 * spi_cs signal is delayed by system clock cycles
 */

#define SPI_CS_DELAY_NUM    0x00000003
#define SPI_CS_DELAY_NUM_M  (SPI_CS_DELAY_NUM_V << SPI_CS_DELAY_NUM_S)
#define SPI_CS_DELAY_NUM_V  0x00000003
#define SPI_CS_DELAY_NUM_S  29

/* SPI_CS_DELAY_MODE : R/W; bitpos: [28:26]; default: 0;
 * spi_cs signal is delayed by spi_clk . 0: zero 1: if spi_ck_out_edge or
 * spi_ck_i_edge is set 1 delayed by half cycle  else delayed by one cycle
 * 2: if spi_ck_out_edge or spi_ck_i_edge is set 1 delayed by one cycle,
 * else delayed by half cycle 3: delayed one cycle
 */

#define SPI_CS_DELAY_MODE    0x00000007
#define SPI_CS_DELAY_MODE_M  (SPI_CS_DELAY_MODE_V << SPI_CS_DELAY_MODE_S)
#define SPI_CS_DELAY_MODE_V  0x00000007
#define SPI_CS_DELAY_MODE_S  26

/* SPI_CS_HOLD_TIME : R/W; bitpos: [25:13]; default: 1;
 * delay cycles of cs pin by spi clock this bits are combined with
 * spi_cs_hold bit.
 */

#define SPI_CS_HOLD_TIME    0x00001FFF
#define SPI_CS_HOLD_TIME_M  (SPI_CS_HOLD_TIME_V << SPI_CS_HOLD_TIME_S)
#define SPI_CS_HOLD_TIME_V  0x00001FFF
#define SPI_CS_HOLD_TIME_S  13

/* SPI_CS_SETUP_TIME : R/W; bitpos: [12:0]; default: 1;
 * (cycles-1) of prepare phase by spi clock this bits are combined with
 * spi_cs_setup bit.
 */

#define SPI_CS_SETUP_TIME    0x00001FFF
#define SPI_CS_SETUP_TIME_M  (SPI_CS_SETUP_TIME_V << SPI_CS_SETUP_TIME_S)
#define SPI_CS_SETUP_TIME_V  0x00001FFF
#define SPI_CS_SETUP_TIME_S  0

/* SPI_CLOCK_REG register */

#define SPI_CLOCK_REG(i)      (REG_SPI_BASE(i) + 0x14)

/* SPI_CLK_EQU_SYSCLK : R/W; bitpos: [31]; default: 1;
 * In the master mode 1: spi_clk is eqaul to system 0: spi_clk is divided
 * from system clock.
 */

#define SPI_CLK_EQU_SYSCLK    (BIT(31))
#define SPI_CLK_EQU_SYSCLK_M  (SPI_CLK_EQU_SYSCLK_V << SPI_CLK_EQU_SYSCLK_S)
#define SPI_CLK_EQU_SYSCLK_V  0x00000001
#define SPI_CLK_EQU_SYSCLK_S  31

/* SPI_CLKDIV_PRE : R/W; bitpos: [30:18]; default: 0;
 * In the master mode it is pre-divider of spi_clk.
 */

#define SPI_CLKDIV_PRE    0x00001FFF
#define SPI_CLKDIV_PRE_M  (SPI_CLKDIV_PRE_V << SPI_CLKDIV_PRE_S)
#define SPI_CLKDIV_PRE_V  0x00001FFF
#define SPI_CLKDIV_PRE_S  18

/* SPI_CLKCNT_N : R/W; bitpos: [17:12]; default: 3;
 * In the master mode it is the divider of spi_clk. So spi_clk frequency is
 * system/(spi_clkdiv_pre+1)/(spi_clkcnt_N+1)
 */

#define SPI_CLKCNT_N    0x0000003F
#define SPI_CLKCNT_N_M  (SPI_CLKCNT_N_V << SPI_CLKCNT_N_S)
#define SPI_CLKCNT_N_V  0x0000003F
#define SPI_CLKCNT_N_S  12

/* SPI_CLKCNT_H : R/W; bitpos: [11:6]; default: 1;
 * In the master mode it must be floor((spi_clkcnt_N+1)/2-1). In the slave
 * mode it must be 0.
 */

#define SPI_CLKCNT_H    0x0000003F
#define SPI_CLKCNT_H_M  (SPI_CLKCNT_H_V << SPI_CLKCNT_H_S)
#define SPI_CLKCNT_H_V  0x0000003F
#define SPI_CLKCNT_H_S  6

/* SPI_CLKCNT_L : R/W; bitpos: [5:0]; default: 3;
 * In the master mode it must be equal to spi_clkcnt_N. In the slave mode it
 * must be 0.
 */

#define SPI_CLKCNT_L    0x0000003F
#define SPI_CLKCNT_L_M  (SPI_CLKCNT_L_V << SPI_CLKCNT_L_S)
#define SPI_CLKCNT_L_V  0x0000003F
#define SPI_CLKCNT_L_S  0

/* SPI_USER_REG register */

#define SPI_USER_REG(i)    (REG_SPI_BASE(i) + 0x18)

/* SPI_USR_COMMAND : R/W; bitpos: [31]; default: 1;
 * This bit enable the command phase of an operation.
 */

#define SPI_USR_COMMAND    (BIT(31))
#define SPI_USR_COMMAND_M  (SPI_USR_COMMAND_V << SPI_USR_COMMAND_S)
#define SPI_USR_COMMAND_V  0x00000001
#define SPI_USR_COMMAND_S  31

/* SPI_USR_ADDR : R/W; bitpos: [30]; default: 0;
 * This bit enable the address phase of an operation.
 */

#define SPI_USR_ADDR    (BIT(30))
#define SPI_USR_ADDR_M  (SPI_USR_ADDR_V << SPI_USR_ADDR_S)
#define SPI_USR_ADDR_V  0x00000001
#define SPI_USR_ADDR_S  30

/* SPI_USR_DUMMY : R/W; bitpos: [29]; default: 0;
 * This bit enable the dummy phase of an operation.
 */

#define SPI_USR_DUMMY    (BIT(29))
#define SPI_USR_DUMMY_M  (SPI_USR_DUMMY_V << SPI_USR_DUMMY_S)
#define SPI_USR_DUMMY_V  0x00000001
#define SPI_USR_DUMMY_S  29

/* SPI_USR_MISO : R/W; bitpos: [28]; default: 0;
 * This bit enable the read-data phase of an operation.
 */

#define SPI_USR_MISO    (BIT(28))
#define SPI_USR_MISO_M  (SPI_USR_MISO_V << SPI_USR_MISO_S)
#define SPI_USR_MISO_V  0x00000001
#define SPI_USR_MISO_S  28

/* SPI_USR_MOSI : R/W; bitpos: [27]; default: 0;
 * This bit enable the write-data phase of an operation.
 */

#define SPI_USR_MOSI    (BIT(27))
#define SPI_USR_MOSI_M  (SPI_USR_MOSI_V << SPI_USR_MOSI_S)
#define SPI_USR_MOSI_V  0x00000001
#define SPI_USR_MOSI_S  27

/* SPI_USR_DUMMY_IDLE : R/W; bitpos: [26]; default: 0;
 * spi clock is disable in dummy phase when the bit is enable.
 */

#define SPI_USR_DUMMY_IDLE    (BIT(26))
#define SPI_USR_DUMMY_IDLE_M  (SPI_USR_DUMMY_IDLE_V << SPI_USR_DUMMY_IDLE_S)
#define SPI_USR_DUMMY_IDLE_V  0x00000001
#define SPI_USR_DUMMY_IDLE_S  26

/* SPI_USR_MOSI_HIGHPART : R/W; bitpos: [25]; default: 0;
 * write-data phase only access to high-part of the buffer spi_w8~spi_w15.
 * 1: enable 0: disable.
 */

#define SPI_USR_MOSI_HIGHPART    (BIT(25))
#define SPI_USR_MOSI_HIGHPART_M  (SPI_USR_MOSI_HIGHPART_V << SPI_USR_MOSI_HIGHPART_S)
#define SPI_USR_MOSI_HIGHPART_V  0x00000001
#define SPI_USR_MOSI_HIGHPART_S  25

/* SPI_USR_MISO_HIGHPART : R/W; bitpos: [24]; default: 0;
 * read-data phase only access to high-part of the buffer spi_w8~spi_w15. 1:
 * enable 0: disable.
 */

#define SPI_USR_MISO_HIGHPART    (BIT(24))
#define SPI_USR_MISO_HIGHPART_M  (SPI_USR_MISO_HIGHPART_V << SPI_USR_MISO_HIGHPART_S)
#define SPI_USR_MISO_HIGHPART_V  0x00000001
#define SPI_USR_MISO_HIGHPART_S  24

/* SPI_USR_PREP_HOLD : R/W; bitpos: [23]; default: 0;
 * spi is hold at prepare state the bit are combined with spi_usr_hold_pol
 * bit.
 */

#define SPI_USR_PREP_HOLD    (BIT(23))
#define SPI_USR_PREP_HOLD_M  (SPI_USR_PREP_HOLD_V << SPI_USR_PREP_HOLD_S)
#define SPI_USR_PREP_HOLD_V  0x00000001
#define SPI_USR_PREP_HOLD_S  23

/* SPI_USR_CMD_HOLD : R/W; bitpos: [22]; default: 0;
 * spi is hold at command state the bit are combined with spi_usr_hold_pol
 * bit.
 */

#define SPI_USR_CMD_HOLD    (BIT(22))
#define SPI_USR_CMD_HOLD_M  (SPI_USR_CMD_HOLD_V << SPI_USR_CMD_HOLD_S)
#define SPI_USR_CMD_HOLD_V  0x00000001
#define SPI_USR_CMD_HOLD_S  22

/* SPI_USR_ADDR_HOLD : R/W; bitpos: [21]; default: 0;
 * spi is hold at address state the bit are combined with spi_usr_hold_pol
 * bit.
 */

#define SPI_USR_ADDR_HOLD    (BIT(21))
#define SPI_USR_ADDR_HOLD_M  (SPI_USR_ADDR_HOLD_V << SPI_USR_ADDR_HOLD_S)
#define SPI_USR_ADDR_HOLD_V  0x00000001
#define SPI_USR_ADDR_HOLD_S  21

/* SPI_USR_DUMMY_HOLD : R/W; bitpos: [20]; default: 0;
 * spi is hold at dummy state the bit are combined with spi_usr_hold_pol bit.
 */

#define SPI_USR_DUMMY_HOLD    (BIT(20))
#define SPI_USR_DUMMY_HOLD_M  (SPI_USR_DUMMY_HOLD_V << SPI_USR_DUMMY_HOLD_S)
#define SPI_USR_DUMMY_HOLD_V  0x00000001
#define SPI_USR_DUMMY_HOLD_S  20

/* SPI_USR_DIN_HOLD : R/W; bitpos: [19]; default: 0;
 * spi is hold at data in state the bit are combined with spi_usr_hold_pol
 * bit.
 */

#define SPI_USR_DIN_HOLD    (BIT(19))
#define SPI_USR_DIN_HOLD_M  (SPI_USR_DIN_HOLD_V << SPI_USR_DIN_HOLD_S)
#define SPI_USR_DIN_HOLD_V  0x00000001
#define SPI_USR_DIN_HOLD_S  19

/* SPI_USR_DOUT_HOLD : R/W; bitpos: [18]; default: 0;
 * spi is hold at data out state the bit are combined with spi_usr_hold_pol
 * bit.
 */

#define SPI_USR_DOUT_HOLD    (BIT(18))
#define SPI_USR_DOUT_HOLD_M  (SPI_USR_DOUT_HOLD_V << SPI_USR_DOUT_HOLD_S)
#define SPI_USR_DOUT_HOLD_V  0x00000001
#define SPI_USR_DOUT_HOLD_S  18

/* SPI_USR_HOLD_POL : R/W; bitpos: [17]; default: 0;
 * It is combined with hold bits to set the polarity of spi hold line 1: spi
 * will be held when spi hold line is high 0: spi will be held when spi hold
 * line is low
 */

#define SPI_USR_HOLD_POL    (BIT(17))
#define SPI_USR_HOLD_POL_M  (SPI_USR_HOLD_POL_V << SPI_USR_HOLD_POL_S)
#define SPI_USR_HOLD_POL_V  0x00000001
#define SPI_USR_HOLD_POL_S  17

/* SPI_SIO : R/W; bitpos: [16]; default: 0;
 * Set the bit to enable 3-line half duplex communication mosi and miso
 * signals share the same pin. 1: enable 0: disable.
 */

#define SPI_SIO    (BIT(16))
#define SPI_SIO_M  (SPI_SIO_V << SPI_SIO_S)
#define SPI_SIO_V  0x00000001
#define SPI_SIO_S  16

/* SPI_USR_CONF_NXT : R/W; bitpos: [15]; default: 0;
 * 1: Enable the DMA CONF phase of next seg-trans operation, which means
 * seg-trans will continue. 0: The seg-trans will end after the current SPI
 * seg-trans or this is not seg-trans mode.
 */

#define SPI_USR_CONF_NXT    (BIT(15))
#define SPI_USR_CONF_NXT_M  (SPI_USR_CONF_NXT_V << SPI_USR_CONF_NXT_S)
#define SPI_USR_CONF_NXT_V  0x00000001
#define SPI_USR_CONF_NXT_S  15

/* SPI_FWRITE_OCT : R/W; bitpos: [14]; default: 0;
 * In the write operations read-data phase apply 8 signals
 */

#define SPI_FWRITE_OCT    (BIT(14))
#define SPI_FWRITE_OCT_M  (SPI_FWRITE_OCT_V << SPI_FWRITE_OCT_S)
#define SPI_FWRITE_OCT_V  0x00000001
#define SPI_FWRITE_OCT_S  14

/* SPI_FWRITE_QUAD : R/W; bitpos: [13]; default: 0;
 * In the write operations read-data phase apply 4 signals
 */

#define SPI_FWRITE_QUAD    (BIT(13))
#define SPI_FWRITE_QUAD_M  (SPI_FWRITE_QUAD_V << SPI_FWRITE_QUAD_S)
#define SPI_FWRITE_QUAD_V  0x00000001
#define SPI_FWRITE_QUAD_S  13

/* SPI_FWRITE_DUAL : R/W; bitpos: [12]; default: 0;
 * In the write operations read-data phase apply 2 signals
 */

#define SPI_FWRITE_DUAL    (BIT(12))
#define SPI_FWRITE_DUAL_M  (SPI_FWRITE_DUAL_V << SPI_FWRITE_DUAL_S)
#define SPI_FWRITE_DUAL_V  0x00000001
#define SPI_FWRITE_DUAL_S  12

/* SPI_WR_BYTE_ORDER : R/W; bitpos: [11]; default: 0;
 * In command address write-data (MOSI) phases 1: big-endian 0: litte_endian
 */

#define SPI_WR_BYTE_ORDER    (BIT(11))
#define SPI_WR_BYTE_ORDER_M  (SPI_WR_BYTE_ORDER_V << SPI_WR_BYTE_ORDER_S)
#define SPI_WR_BYTE_ORDER_V  0x00000001
#define SPI_WR_BYTE_ORDER_S  11

/* SPI_RD_BYTE_ORDER : R/W; bitpos: [10]; default: 0;
 * In read-data (MISO) phase 1: big-endian 0: little_endian
 */

#define SPI_RD_BYTE_ORDER    (BIT(10))
#define SPI_RD_BYTE_ORDER_M  (SPI_RD_BYTE_ORDER_V << SPI_RD_BYTE_ORDER_S)
#define SPI_RD_BYTE_ORDER_V  0x00000001
#define SPI_RD_BYTE_ORDER_S  10

/* SPI_CK_OUT_EDGE : R/W; bitpos: [9]; default: 0;
 * the bit combined with spi_mosi_delay_mode bits to set mosi signal delay
 * mode.
 */

#define SPI_CK_OUT_EDGE    (BIT(9))
#define SPI_CK_OUT_EDGE_M  (SPI_CK_OUT_EDGE_V << SPI_CK_OUT_EDGE_S)
#define SPI_CK_OUT_EDGE_V  0x00000001
#define SPI_CK_OUT_EDGE_S  9

/* SPI_RSCK_I_EDGE : R/W; bitpos: [8]; default: 0;
 * In the slave mode, this bit can be used to change the polarity of rsck.
 * 0: rsck = !spi_ck_i. 1:rsck = spi_ck_i.
 */

#define SPI_RSCK_I_EDGE    (BIT(8))
#define SPI_RSCK_I_EDGE_M  (SPI_RSCK_I_EDGE_V << SPI_RSCK_I_EDGE_S)
#define SPI_RSCK_I_EDGE_V  0x00000001
#define SPI_RSCK_I_EDGE_S  8

/* SPI_CS_SETUP : R/W; bitpos: [7]; default: 0;
 * spi cs is enable when spi is in  prepare  phase. 1: enable 0: disable.
 */

#define SPI_CS_SETUP    (BIT(7))
#define SPI_CS_SETUP_M  (SPI_CS_SETUP_V << SPI_CS_SETUP_S)
#define SPI_CS_SETUP_V  0x00000001
#define SPI_CS_SETUP_S  7

/* SPI_CS_HOLD : R/W; bitpos: [6]; default: 0;
 * spi cs keep low when spi is in  done  phase. 1: enable 0: disable.
 */

#define SPI_CS_HOLD    (BIT(6))
#define SPI_CS_HOLD_M  (SPI_CS_HOLD_V << SPI_CS_HOLD_S)
#define SPI_CS_HOLD_V  0x00000001
#define SPI_CS_HOLD_S  6

/* SPI_TSCK_I_EDGE : R/W; bitpos: [5]; default: 0;
 * In the slave mode, this bit can be used to change the polarity of tsck.
 * 0: tsck = spi_ck_i. 1:tsck = !spi_ck_i.
 */

#define SPI_TSCK_I_EDGE    (BIT(5))
#define SPI_TSCK_I_EDGE_M  (SPI_TSCK_I_EDGE_V << SPI_TSCK_I_EDGE_S)
#define SPI_TSCK_I_EDGE_V  0x00000001
#define SPI_TSCK_I_EDGE_S  5

/* SPI_DOUTDIN : R/W; bitpos: [0]; default: 0;
 * Set the bit to enable full duplex communication. 1: enable 0: disable.
 */

#define SPI_DOUTDIN    (BIT(0))
#define SPI_DOUTDIN_M  (SPI_DOUTDIN_V << SPI_DOUTDIN_S)
#define SPI_DOUTDIN_V  0x00000001
#define SPI_DOUTDIN_S  0

/* SPI_USER1_REG register */

#define SPI_USER1_REG(i)       (REG_SPI_BASE(i) + 0x1c)

/* SPI_USR_ADDR_BITLEN : R/W; bitpos: [31:27]; default: 23;
 * The length in bits of address phase. The register value shall be
 * (bit_num-1).
 */

#define SPI_USR_ADDR_BITLEN    0x0000001F
#define SPI_USR_ADDR_BITLEN_M  (SPI_USR_ADDR_BITLEN_V << SPI_USR_ADDR_BITLEN_S)
#define SPI_USR_ADDR_BITLEN_V  0x0000001F
#define SPI_USR_ADDR_BITLEN_S  27

/* SPI_USR_DUMMY_CYCLELEN : R/W; bitpos: [7:0]; default: 7;
 * The length in spi_clk cycles of dummy phase. The register value shall be
 * (cycle_num-1).
 */

#define SPI_USR_DUMMY_CYCLELEN    0x000000FF
#define SPI_USR_DUMMY_CYCLELEN_M  (SPI_USR_DUMMY_CYCLELEN_V << SPI_USR_DUMMY_CYCLELEN_S)
#define SPI_USR_DUMMY_CYCLELEN_V  0x000000FF
#define SPI_USR_DUMMY_CYCLELEN_S  0

/* SPI_USER2_REG register */

#define SPI_USER2_REG(i)          (REG_SPI_BASE(i) + 0x20)

/* SPI_USR_COMMAND_BITLEN : R/W; bitpos: [31:28]; default: 7;
 * The length in bits of command phase. The register value shall be
 * (bit_num-1)
 */

#define SPI_USR_COMMAND_BITLEN    0x0000000F
#define SPI_USR_COMMAND_BITLEN_M  (SPI_USR_COMMAND_BITLEN_V << SPI_USR_COMMAND_BITLEN_S)
#define SPI_USR_COMMAND_BITLEN_V  0x0000000F
#define SPI_USR_COMMAND_BITLEN_S  28

/* SPI_USR_COMMAND_VALUE : R/W; bitpos: [15:0]; default: 0;
 * The value of  command.
 */

#define SPI_USR_COMMAND_VALUE    0x0000FFFF
#define SPI_USR_COMMAND_VALUE_M  (SPI_USR_COMMAND_VALUE_V << SPI_USR_COMMAND_VALUE_S)
#define SPI_USR_COMMAND_VALUE_V  0x0000FFFF
#define SPI_USR_COMMAND_VALUE_S  0

/* SPI_MOSI_DLEN_REG register */

#define SPI_MOSI_DLEN_REG(i)     (REG_SPI_BASE(i) + 0x24)

/* SPI_USR_MOSI_DBITLEN : R/W; bitpos: [22:0]; default: 0;
 * The length in bits of write-data. The register value shall be (bit_num-1).
 */

#define SPI_USR_MOSI_DBITLEN    0x007FFFFF
#define SPI_USR_MOSI_DBITLEN_M  (SPI_USR_MOSI_DBITLEN_V << SPI_USR_MOSI_DBITLEN_S)
#define SPI_USR_MOSI_DBITLEN_V  0x007FFFFF
#define SPI_USR_MOSI_DBITLEN_S  0

/* SPI_MISO_DLEN_REG register */

#define SPI_MISO_DLEN_REG(i)    (REG_SPI_BASE(i) + 0x28)

/* SPI_USR_MISO_DBITLEN : R/W; bitpos: [22:0]; default: 0;
 * The length in bits of  read-data. The register value shall be (bit_num-1).
 */

#define SPI_USR_MISO_DBITLEN    0x007FFFFF
#define SPI_USR_MISO_DBITLEN_M  (SPI_USR_MISO_DBITLEN_V << SPI_USR_MISO_DBITLEN_S)
#define SPI_USR_MISO_DBITLEN_V  0x007FFFFF
#define SPI_USR_MISO_DBITLEN_S  0

/* SPI_OPI_MODE : R/W; bitpos: [1]; default: 0;
 * Just for master mode. 1: spi controller is in OPI mode (all in 8-b-m). 0:
 * others.
 */

#define SPI_OPI_MODE    (BIT(1))
#define SPI_OPI_MODE_M  (SPI_OPI_MODE_V << SPI_OPI_MODE_S)
#define SPI_OPI_MODE_V  0x00000001
#define SPI_OPI_MODE_S  1

/* SPI_QPI_MODE : R/W; bitpos: [0]; default: 0;
 * Both for master mode and slave mode. 1: spi controller is in QPI mode. 0:
 * others.
 */

#define SPI_QPI_MODE    (BIT(0))
#define SPI_QPI_MODE_M  (SPI_QPI_MODE_V << SPI_QPI_MODE_S)
#define SPI_QPI_MODE_V  0x00000001
#define SPI_QPI_MODE_S  0

/* SPI_MISC_REG register */

#define SPI_MISC_REG(i)          (REG_SPI_BASE(i) + 0x2c)

/* SPI_QUAD_DIN_PIN_SWAP : R/W; bitpos: [31]; default: 0;
 * 1:  spi quad input swap enable  0:  spi quad input swap disable
 */

#define SPI_QUAD_DIN_PIN_SWAP    (BIT(31))
#define SPI_QUAD_DIN_PIN_SWAP_M  (SPI_QUAD_DIN_PIN_SWAP_V << SPI_QUAD_DIN_PIN_SWAP_S)
#define SPI_QUAD_DIN_PIN_SWAP_V  0x00000001
#define SPI_QUAD_DIN_PIN_SWAP_S  31

/* SPI_CS_KEEP_ACTIVE : R/W; bitpos: [30]; default: 0;
 * spi cs line keep low when the bit is set.
 */

#define SPI_CS_KEEP_ACTIVE    (BIT(30))
#define SPI_CS_KEEP_ACTIVE_M  (SPI_CS_KEEP_ACTIVE_V << SPI_CS_KEEP_ACTIVE_S)
#define SPI_CS_KEEP_ACTIVE_V  0x00000001
#define SPI_CS_KEEP_ACTIVE_S  30

/* SPI_CK_IDLE_EDGE : R/W; bitpos: [29]; default: 0;
 * 1: spi clk line is high when idle     0: spi clk line is low when idle
 */

#define SPI_CK_IDLE_EDGE    (BIT(29))
#define SPI_CK_IDLE_EDGE_M  (SPI_CK_IDLE_EDGE_V << SPI_CK_IDLE_EDGE_S)
#define SPI_CK_IDLE_EDGE_V  0x00000001
#define SPI_CK_IDLE_EDGE_S  29

/* SPI_CD_IDLE_EDGE : R/W; bitpos: [26]; default: 0;
 * The default value of spi_cd.
 */

#define SPI_CD_IDLE_EDGE    (BIT(26))
#define SPI_CD_IDLE_EDGE_M  (SPI_CD_IDLE_EDGE_V << SPI_CD_IDLE_EDGE_S)
#define SPI_CD_IDLE_EDGE_V  0x00000001
#define SPI_CD_IDLE_EDGE_S  26

/* SPI_CD_CMD_SET : R/W; bitpos: [25]; default: 0;
 * 1: spi_cd = !spi_cd_idle_edge when spi_st[3:0] is in SPI_SEND_CMD state.
 * 0: spi_cd = spi_cd_idle_edge.
 */

#define SPI_CD_CMD_SET    (BIT(25))
#define SPI_CD_CMD_SET_M  (SPI_CD_CMD_SET_V << SPI_CD_CMD_SET_S)
#define SPI_CD_CMD_SET_V  0x00000001
#define SPI_CD_CMD_SET_S  25

/* SPI_DQS_IDLE_EDGE : R/W; bitpos: [24]; default: 0;
 * The default value of spi_dqs.
 */

#define SPI_DQS_IDLE_EDGE    (BIT(24))
#define SPI_DQS_IDLE_EDGE_M  (SPI_DQS_IDLE_EDGE_V << SPI_DQS_IDLE_EDGE_S)
#define SPI_DQS_IDLE_EDGE_V  0x00000001
#define SPI_DQS_IDLE_EDGE_S  24

/* SPI_SLAVE_CS_POL : R/W; bitpos: [23]; default: 0;
 * spi slave input cs polarity select. 1: inv  0: not change
 */

#define SPI_SLAVE_CS_POL    (BIT(23))
#define SPI_SLAVE_CS_POL_M  (SPI_SLAVE_CS_POL_V << SPI_SLAVE_CS_POL_S)
#define SPI_SLAVE_CS_POL_V  0x00000001
#define SPI_SLAVE_CS_POL_S  23

/* SPI_CD_ADDR_SET : R/W; bitpos: [22]; default: 0;
 * 1: spi_cd = !spi_cd_idle_edge when spi_st[3:0] is in SPI_SEND_ADDR state.
 * 0: spi_cd = spi_cd_idle_edge.
 */

#define SPI_CD_ADDR_SET    (BIT(22))
#define SPI_CD_ADDR_SET_M  (SPI_CD_ADDR_SET_V << SPI_CD_ADDR_SET_S)
#define SPI_CD_ADDR_SET_V  0x00000001
#define SPI_CD_ADDR_SET_S  22

/* SPI_CD_DUMMY_SET : R/W; bitpos: [21]; default: 0;
 * 1: spi_cd = !spi_cd_idle_edge when spi_st[3:0] is in SPI_DUMMY state.  0:
 * spi_cd = spi_cd_idle_edge.
 */

#define SPI_CD_DUMMY_SET    (BIT(21))
#define SPI_CD_DUMMY_SET_M  (SPI_CD_DUMMY_SET_V << SPI_CD_DUMMY_SET_S)
#define SPI_CD_DUMMY_SET_V  0x00000001
#define SPI_CD_DUMMY_SET_S  21

/* SPI_CD_DATA_SET : R/W; bitpos: [20]; default: 0;
 * 1: spi_cd = !spi_cd_idle_edge when spi_st[3:0] is in SPI_DOUT or SPI_DIN
 * state.  0: spi_cd = spi_cd_idle_edge.
 */

#define SPI_CD_DATA_SET    (BIT(20))
#define SPI_CD_DATA_SET_M  (SPI_CD_DATA_SET_V << SPI_CD_DATA_SET_S)
#define SPI_CD_DATA_SET_V  0x00000001
#define SPI_CD_DATA_SET_S  20

/* SPI_MASTER_CS_POL : R/W; bitpos: [12:7]; default: 0;
 * In the master mode the bits are the polarity of spi cs line, the value is
 * equivalent to spi_cs ^ spi_master_cs_pol.
 */

#define SPI_MASTER_CS_POL    0x0000003F
#define SPI_MASTER_CS_POL_M  (SPI_MASTER_CS_POL_V << SPI_MASTER_CS_POL_S)
#define SPI_MASTER_CS_POL_V  0x0000003F
#define SPI_MASTER_CS_POL_S  7

/* SPI_CK_DIS : R/W; bitpos: [6]; default: 0;
 * 1: spi clk out disable,  0: spi clk out enable
 */

#define SPI_CK_DIS    (BIT(6))
#define SPI_CK_DIS_M  (SPI_CK_DIS_V << SPI_CK_DIS_S)
#define SPI_CK_DIS_V  0x00000001
#define SPI_CK_DIS_S  6

/* SPI_CS5_DIS : R/W; bitpos: [5]; default: 1;
 * SPI CS5 pin enable, 1: disable CS5, 0: SPI_CS5 signal is from/to CS5 pin
 */

#define SPI_CS5_DIS    (BIT(5))
#define SPI_CS5_DIS_M  (SPI_CS5_DIS_V << SPI_CS5_DIS_S)
#define SPI_CS5_DIS_V  0x00000001
#define SPI_CS5_DIS_S  5

/* SPI_CS4_DIS : R/W; bitpos: [4]; default: 1;
 * SPI CS4 pin enable, 1: disable CS4, 0: SPI_CS4 signal is from/to CS4 pin
 */

#define SPI_CS4_DIS    (BIT(4))
#define SPI_CS4_DIS_M  (SPI_CS4_DIS_V << SPI_CS4_DIS_S)
#define SPI_CS4_DIS_V  0x00000001
#define SPI_CS4_DIS_S  4

/* SPI_CS3_DIS : R/W; bitpos: [3]; default: 1;
 * SPI CS3 pin enable, 1: disable CS3, 0: SPI_CS3 signal is from/to CS3 pin
 */

#define SPI_CS3_DIS    (BIT(3))
#define SPI_CS3_DIS_M  (SPI_CS3_DIS_V << SPI_CS3_DIS_S)
#define SPI_CS3_DIS_V  0x00000001
#define SPI_CS3_DIS_S  3

/* SPI_CS2_DIS : R/W; bitpos: [2]; default: 1;
 * SPI CS2 pin enable, 1: disable CS2, 0: SPI_CS2 signal is from/to CS2 pin
 */

#define SPI_CS2_DIS    (BIT(2))
#define SPI_CS2_DIS_M  (SPI_CS2_DIS_V << SPI_CS2_DIS_S)
#define SPI_CS2_DIS_V  0x00000001
#define SPI_CS2_DIS_S  2

/* SPI_CS1_DIS : R/W; bitpos: [1]; default: 1;
 * SPI CS1 pin enable, 1: disable CS1, 0: SPI_CS1 signal is from/to CS1 pin
 */

#define SPI_CS1_DIS    (BIT(1))
#define SPI_CS1_DIS_M  (SPI_CS1_DIS_V << SPI_CS1_DIS_S)
#define SPI_CS1_DIS_V  0x00000001
#define SPI_CS1_DIS_S  1

/* SPI_CS0_DIS : R/W; bitpos: [0]; default: 1;
 * SPI CS0 pin enable, 1: disable CS0, 0: SPI_CS0 signal is from/to CS0 pin
 */

#define SPI_CS0_DIS    (BIT(0))
#define SPI_CS0_DIS_M  (SPI_CS0_DIS_V << SPI_CS0_DIS_S)
#define SPI_CS0_DIS_V  0x00000001
#define SPI_CS0_DIS_S  0

/* SPI_SLAVE_REG register */

#define SPI_SLAVE_REG(i)  (REG_SPI_BASE(i) + 0x30)

/* SPI_SOFT_RESET : R/W; bitpos: [31]; default: 0;
 * Software reset enable, reset the spi clock line cs line and data lines.
 */

#define SPI_SOFT_RESET    (BIT(31))
#define SPI_SOFT_RESET_M  (SPI_SOFT_RESET_V << SPI_SOFT_RESET_S)
#define SPI_SOFT_RESET_V  0x00000001
#define SPI_SOFT_RESET_S  31

/* SPI_SLAVE_MODE : R/W; bitpos: [30]; default: 0;
 * Set SPI work mode. 1: slave mode 0: master mode.
 */

#define SPI_SLAVE_MODE    (BIT(30))
#define SPI_SLAVE_MODE_M  (SPI_SLAVE_MODE_V << SPI_SLAVE_MODE_S)
#define SPI_SLAVE_MODE_V  0x00000001
#define SPI_SLAVE_MODE_S  30

/* SPI_TRANS_DONE_AUTO_CLR_EN : R/W; bitpos: [29]; default: 0;
 * spi_trans_done auto clear enable, clear it 3 apb cycles after the pos
 * edge of spi_trans_done.  0:disable. 1: enable.
 */

#define SPI_TRANS_DONE_AUTO_CLR_EN    (BIT(29))
#define SPI_TRANS_DONE_AUTO_CLR_EN_M  (SPI_TRANS_DONE_AUTO_CLR_EN_V << SPI_TRANS_DONE_AUTO_CLR_EN_S)
#define SPI_TRANS_DONE_AUTO_CLR_EN_V  0x00000001
#define SPI_TRANS_DONE_AUTO_CLR_EN_S  29

/* SPI_TRANS_CNT : RO; bitpos: [26:23]; default: 0;
 * The operations counter in both the master mode and the slave mode.
 */

#define SPI_TRANS_CNT    0x0000000F
#define SPI_TRANS_CNT_M  (SPI_TRANS_CNT_V << SPI_TRANS_CNT_S)
#define SPI_TRANS_CNT_V  0x0000000F
#define SPI_TRANS_CNT_S  23

/* SPI_SEG_MAGIC_ERR_INT_EN : R/W; bitpos: [11]; default: 0;
 * 1: Enable seg magic value error interrupt. 0: Others
 */

#define SPI_SEG_MAGIC_ERR_INT_EN    (BIT(11))
#define SPI_SEG_MAGIC_ERR_INT_EN_M  (SPI_SEG_MAGIC_ERR_INT_EN_V << SPI_SEG_MAGIC_ERR_INT_EN_S)
#define SPI_SEG_MAGIC_ERR_INT_EN_V  0x00000001
#define SPI_SEG_MAGIC_ERR_INT_EN_S  11

/* SPI_INT_DMA_SEG_TRANS_EN : R/W; bitpos: [10]; default: 0;
 * spi_dma_seg_trans_done Interrupt enable. 1: enable 0: disable
 */

#define SPI_INT_DMA_SEG_TRANS_EN    (BIT(10))
#define SPI_INT_DMA_SEG_TRANS_EN_M  (SPI_INT_DMA_SEG_TRANS_EN_V << SPI_INT_DMA_SEG_TRANS_EN_S)
#define SPI_INT_DMA_SEG_TRANS_EN_V  0x00000001
#define SPI_INT_DMA_SEG_TRANS_EN_S  10

/* SPI_INT_TRANS_DONE_EN : R/W; bitpos: [9]; default: 1;
 * spi_trans_done Interrupt enable. 1: enable 0: disable
 */

#define SPI_INT_TRANS_DONE_EN    (BIT(9))
#define SPI_INT_TRANS_DONE_EN_M  (SPI_INT_TRANS_DONE_EN_V << SPI_INT_TRANS_DONE_EN_S)
#define SPI_INT_TRANS_DONE_EN_V  0x00000001
#define SPI_INT_TRANS_DONE_EN_S  9

/* SPI_INT_WR_DMA_DONE_EN : R/W; bitpos: [8]; default: 0;
 * spi_slv_wr_dma Interrupt enable. 1: enable 0: disable
 */

#define SPI_INT_WR_DMA_DONE_EN    (BIT(8))
#define SPI_INT_WR_DMA_DONE_EN_M  (SPI_INT_WR_DMA_DONE_EN_V << SPI_INT_WR_DMA_DONE_EN_S)
#define SPI_INT_WR_DMA_DONE_EN_V  0x00000001
#define SPI_INT_WR_DMA_DONE_EN_S  8

/* SPI_INT_RD_DMA_DONE_EN : R/W; bitpos: [7]; default: 0;
 * spi_slv_rd_dma Interrupt enable. 1: enable 0: disable
 */

#define SPI_INT_RD_DMA_DONE_EN    (BIT(7))
#define SPI_INT_RD_DMA_DONE_EN_M  (SPI_INT_RD_DMA_DONE_EN_V << SPI_INT_RD_DMA_DONE_EN_S)
#define SPI_INT_RD_DMA_DONE_EN_V  0x00000001
#define SPI_INT_RD_DMA_DONE_EN_S  7

/* SPI_INT_WR_BUF_DONE_EN : R/W; bitpos: [6]; default: 0;
 * spi_slv_wr_buf Interrupt enable. 1: enable 0: disable
 */

#define SPI_INT_WR_BUF_DONE_EN    (BIT(6))
#define SPI_INT_WR_BUF_DONE_EN_M  (SPI_INT_WR_BUF_DONE_EN_V << SPI_INT_WR_BUF_DONE_EN_S)
#define SPI_INT_WR_BUF_DONE_EN_V  0x00000001
#define SPI_INT_WR_BUF_DONE_EN_S  6

/* SPI_INT_RD_BUF_DONE_EN : R/W; bitpos: [5]; default: 0;
 * spi_slv_rd_buf Interrupt enable. 1: enable 0: disable
 */

#define SPI_INT_RD_BUF_DONE_EN    (BIT(5))
#define SPI_INT_RD_BUF_DONE_EN_M  (SPI_INT_RD_BUF_DONE_EN_V << SPI_INT_RD_BUF_DONE_EN_S)
#define SPI_INT_RD_BUF_DONE_EN_V  0x00000001
#define SPI_INT_RD_BUF_DONE_EN_S  5

/* SPI_TRANS_DONE : R/W; bitpos: [4]; default: 0;
 * The interrupt raw bit for the completion of any operation in both the
 * master mode and the slave mode.  Can not be changed by CONF_buf.
 */

#define SPI_TRANS_DONE    (BIT(4))
#define SPI_TRANS_DONE_M  (SPI_TRANS_DONE_V << SPI_TRANS_DONE_S)
#define SPI_TRANS_DONE_V  0x00000001
#define SPI_TRANS_DONE_S  4

/* SPI_SLAVE1_REG register */

#define SPI_SLAVE1_REG(i)    (REG_SPI_BASE(i) + 0x34)

/* SPI_SLV_LAST_ADDR : R/W; bitpos: [31:24]; default: 0;
 * In the slave mode it is the value of address.
 */

#define SPI_SLV_LAST_ADDR    0x000000FF
#define SPI_SLV_LAST_ADDR_M  (SPI_SLV_LAST_ADDR_V << SPI_SLV_LAST_ADDR_S)
#define SPI_SLV_LAST_ADDR_V  0x000000FF
#define SPI_SLV_LAST_ADDR_S  24

/* SPI_SLV_LAST_COMMAND : R/W; bitpos: [23:16]; default: 0;
 * In the slave mode it is the value of command.
 */

#define SPI_SLV_LAST_COMMAND    0x000000FF
#define SPI_SLV_LAST_COMMAND_M  (SPI_SLV_LAST_COMMAND_V << SPI_SLV_LAST_COMMAND_S)
#define SPI_SLV_LAST_COMMAND_V  0x000000FF
#define SPI_SLV_LAST_COMMAND_S  16

/* SPI_SLV_WR_DMA_DONE : R/W; bitpos: [15]; default: 0;
 * The interrupt raw bit for the completion of dma write operation in the
 * slave mode.  Can not be changed by CONF_buf.
 */

#define SPI_SLV_WR_DMA_DONE    (BIT(15))
#define SPI_SLV_WR_DMA_DONE_M  (SPI_SLV_WR_DMA_DONE_V << SPI_SLV_WR_DMA_DONE_S)
#define SPI_SLV_WR_DMA_DONE_V  0x00000001
#define SPI_SLV_WR_DMA_DONE_S  15

/* SPI_SLV_CMD_ERR : R/W; bitpos: [14]; default: 0;
 * 1: The command value of the last SPI transfer is not supported by SPI
 * slave. 0: The command value is supported or no command value is received.
 */

#define SPI_SLV_CMD_ERR    (BIT(14))
#define SPI_SLV_CMD_ERR_M  (SPI_SLV_CMD_ERR_V << SPI_SLV_CMD_ERR_S)
#define SPI_SLV_CMD_ERR_V  0x00000001
#define SPI_SLV_CMD_ERR_S  14

/* SPI_SLV_ADDR_ERR : R/W; bitpos: [13]; default: 0;
 * 1: The address value of the last SPI transfer is not supported by SPI
 * slave. 0: The address value is supported or no address value is received.
 */

#define SPI_SLV_ADDR_ERR    (BIT(13))
#define SPI_SLV_ADDR_ERR_M  (SPI_SLV_ADDR_ERR_V << SPI_SLV_ADDR_ERR_S)
#define SPI_SLV_ADDR_ERR_V  0x00000001
#define SPI_SLV_ADDR_ERR_S  13

/* SPI_SLV_RD_DMA_DONE : R/W; bitpos: [8]; default: 0;
 * The interrupt raw bit for the completion of Rd-DMA operation in the slave
 * mode.  Can not be changed by CONF_buf.
 */

#define SPI_SLV_RD_DMA_DONE    (BIT(8))
#define SPI_SLV_RD_DMA_DONE_M  (SPI_SLV_RD_DMA_DONE_V << SPI_SLV_RD_DMA_DONE_S)
#define SPI_SLV_RD_DMA_DONE_V  0x00000001
#define SPI_SLV_RD_DMA_DONE_S  8

/* SPI_SLV_WRBUF_DLEN_REG register */

#define SPI_SLV_WRBUF_DLEN_REG(i)  (REG_SPI_BASE(i) + 0x38)

/* SPI_CONF_BASE_BITLEN : R/W; bitpos: [31:25]; default: 108;
 * The basic spi_clk cycles of CONF state. The real cycle length of CONF
 * state, if spi_usr_conf is enabled, is spi_conf_base_bitlen[6:0] +
 * spi_conf_bitlen[23:0].
 */

#define SPI_CONF_BASE_BITLEN    0x0000007F
#define SPI_CONF_BASE_BITLEN_M  (SPI_CONF_BASE_BITLEN_V << SPI_CONF_BASE_BITLEN_S)
#define SPI_CONF_BASE_BITLEN_V  0x0000007F
#define SPI_CONF_BASE_BITLEN_S  25

/* SPI_SLV_WR_BUF_DONE : R/W; bitpos: [24]; default: 0;
 * The interrupt raw bit for the completion of write-buffer operation in the
 * slave mode.  Can not be changed by CONF_buf.
 */

#define SPI_SLV_WR_BUF_DONE    (BIT(24))
#define SPI_SLV_WR_BUF_DONE_M  (SPI_SLV_WR_BUF_DONE_V << SPI_SLV_WR_BUF_DONE_S)
#define SPI_SLV_WR_BUF_DONE_V  0x00000001
#define SPI_SLV_WR_BUF_DONE_S  24

/* SPI_SLV_RDBUF_DLEN_REG register */

#define SPI_SLV_RDBUF_DLEN_REG(i)  (REG_SPI_BASE(i) + 0x3c)

/* SPI_SEG_MAGIC_ERR : R/W; bitpos: [25]; default: 0;
 * 1: The recent magic value in CONF buffer is not right in master DMA
 * seg-trans mode. 0: others.
 */

#define SPI_SEG_MAGIC_ERR    (BIT(25))
#define SPI_SEG_MAGIC_ERR_M  (SPI_SEG_MAGIC_ERR_V << SPI_SEG_MAGIC_ERR_S)
#define SPI_SEG_MAGIC_ERR_V  0x00000001
#define SPI_SEG_MAGIC_ERR_S  25

/* SPI_SLV_RD_BUF_DONE : R/W; bitpos: [24]; default: 0;
 * The interrupt raw bit for the completion of read-buffer operation in the
 * slave mode.  Can not be changed by CONF_buf.
 */

#define SPI_SLV_RD_BUF_DONE    (BIT(24))
#define SPI_SLV_RD_BUF_DONE_M  (SPI_SLV_RD_BUF_DONE_V << SPI_SLV_RD_BUF_DONE_S)
#define SPI_SLV_RD_BUF_DONE_V  0x00000001
#define SPI_SLV_RD_BUF_DONE_S  24

/* SPI_SLV_DMA_RD_BYTELEN : R/W; bitpos: [19:0]; default: 0;
 * In the slave mode it is the length in bytes for read operations. The
 * register value shall be byte_num.
 */

#define SPI_SLV_DMA_RD_BYTELEN    0x000FFFFF
#define SPI_SLV_DMA_RD_BYTELEN_M  (SPI_SLV_DMA_RD_BYTELEN_V << SPI_SLV_DMA_RD_BYTELEN_S)
#define SPI_SLV_DMA_RD_BYTELEN_V  0x000FFFFF
#define SPI_SLV_DMA_RD_BYTELEN_S  0

/* SPI_SLV_RD_BYTE_REG register */

#define SPI_SLV_RD_BYTE_REG(i)    (REG_SPI_BASE(i) + 0x40)

/* SPI_USR_CONF : R/W; bitpos: [31]; default: 0;
 * 1: Enable the DMA CONF phase of current seg-trans operation, which means
 * seg-trans will start. 0: This is not seg-trans mode.
 */

#define SPI_USR_CONF    (BIT(31))
#define SPI_USR_CONF_M  (SPI_USR_CONF_V << SPI_USR_CONF_S)
#define SPI_USR_CONF_V  0x00000001
#define SPI_USR_CONF_S  31

/* SPI_DMA_SEG_MAGIC_VALUE : R/W; bitpos: [29:24]; default: 23;
 * The magic value of BM table in master DMA seg-trans.
 */

#define SPI_DMA_SEG_MAGIC_VALUE    0x0000003F
#define SPI_DMA_SEG_MAGIC_VALUE_M  (SPI_DMA_SEG_MAGIC_VALUE_V << SPI_DMA_SEG_MAGIC_VALUE_S)
#define SPI_DMA_SEG_MAGIC_VALUE_V  0x0000003F
#define SPI_DMA_SEG_MAGIC_VALUE_S  24

/* SPI_SLV_WRBUF_BYTELEN_EN : R/W; bitpos: [23]; default: 0;
 * 1: spi_slv_data_bytelen stores data byte length of master-write-to-slave
 * data length in CPU controlled mode(Wr_BUF). 0: others
 */

#define SPI_SLV_WRBUF_BYTELEN_EN    (BIT(23))
#define SPI_SLV_WRBUF_BYTELEN_EN_M  (SPI_SLV_WRBUF_BYTELEN_EN_V << SPI_SLV_WRBUF_BYTELEN_EN_S)
#define SPI_SLV_WRBUF_BYTELEN_EN_V  0x00000001
#define SPI_SLV_WRBUF_BYTELEN_EN_S  23

/* SPI_SLV_RDBUF_BYTELEN_EN : R/W; bitpos: [22]; default: 0;
 * 1: spi_slv_data_bytelen stores data byte length of master-read-slave data
 * length in CPU controlled mode(Rd_BUF). 0: others
 */

#define SPI_SLV_RDBUF_BYTELEN_EN    (BIT(22))
#define SPI_SLV_RDBUF_BYTELEN_EN_M  (SPI_SLV_RDBUF_BYTELEN_EN_V << SPI_SLV_RDBUF_BYTELEN_EN_S)
#define SPI_SLV_RDBUF_BYTELEN_EN_V  0x00000001
#define SPI_SLV_RDBUF_BYTELEN_EN_S  22

/* SPI_SLV_WRDMA_BYTELEN_EN : R/W; bitpos: [21]; default: 0;
 * 1: spi_slv_data_bytelen stores data byte length of master-write-to-slave
 * data length in DMA controlled mode(Wr_DMA). 0: others
 */

#define SPI_SLV_WRDMA_BYTELEN_EN    (BIT(21))
#define SPI_SLV_WRDMA_BYTELEN_EN_M  (SPI_SLV_WRDMA_BYTELEN_EN_V << SPI_SLV_WRDMA_BYTELEN_EN_S)
#define SPI_SLV_WRDMA_BYTELEN_EN_V  0x00000001
#define SPI_SLV_WRDMA_BYTELEN_EN_S  21

/* SPI_SLV_RDDMA_BYTELEN_EN : R/W; bitpos: [20]; default: 0;
 * 1: spi_slv_data_bytelen stores data byte length of master-read-slave data
 * length in DMA controlled mode(Rd_DMA). 0: others
 */

#define SPI_SLV_RDDMA_BYTELEN_EN    (BIT(20))
#define SPI_SLV_RDDMA_BYTELEN_EN_M  (SPI_SLV_RDDMA_BYTELEN_EN_V << SPI_SLV_RDDMA_BYTELEN_EN_S)
#define SPI_SLV_RDDMA_BYTELEN_EN_V  0x00000001
#define SPI_SLV_RDDMA_BYTELEN_EN_S  20

/* SPI_SLV_DATA_BYTELEN : R/W; bitpos: [19:0]; default: 0;
 * The full-duplex or half-duplex data byte length of the last SPI transfer
 * in slave mode. In half-duplex mode, this value is controlled by bits
 * [23:20].
 */

#define SPI_SLV_DATA_BYTELEN    0x000FFFFF
#define SPI_SLV_DATA_BYTELEN_M  (SPI_SLV_DATA_BYTELEN_V << SPI_SLV_DATA_BYTELEN_S)
#define SPI_SLV_DATA_BYTELEN_V  0x000FFFFF
#define SPI_SLV_DATA_BYTELEN_S  0

/* SPI_FSM_REG register */

#define SPI_FSM_REG(i)          (REG_SPI_BASE(i) + 0x44)

/* SPI_MST_DMA_RD_BYTELEN : R/W; bitpos: [31:12]; default: 0;
 * Define the master DMA read byte length in non seg-trans or seg-trans
 * mode. Invalid when spi_rx_eof_en is 0.
 */

#define SPI_MST_DMA_RD_BYTELEN    0x000FFFFF
#define SPI_MST_DMA_RD_BYTELEN_M  (SPI_MST_DMA_RD_BYTELEN_V << SPI_MST_DMA_RD_BYTELEN_S)
#define SPI_MST_DMA_RD_BYTELEN_V  0x000FFFFF
#define SPI_MST_DMA_RD_BYTELEN_S  12

/* SPI_ST : RO; bitpos: [3:0]; default: 0;
 * The status of spi state machine. 0: idle state, 1: preparation state, 2:
 * send command state, 3: send data state, 4: red data state, 5:write data
 * state, 6: wait state, 7: done state.
 */

#define SPI_ST    0x0000000F
#define SPI_ST_M  (SPI_ST_V << SPI_ST_S)
#define SPI_ST_V  0x0000000F
#define SPI_ST_S  0

/* SPI_HOLD_REG register */

#define SPI_HOLD_REG(i)           (REG_SPI_BASE(i) + 0x48)

/* SPI_DMA_SEG_TRANS_DONE : R/W; bitpos: [7]; default: 0;
 * 1:  spi master DMA full-duplex/half-duplex seg-trans ends or slave
 * half-duplex seg-trans ends. And data has been pushed to corresponding
 * memory.  0:  seg-trans is not ended or not occurred.  Can not be changed
 * by CONF_buf.
 */

#define SPI_DMA_SEG_TRANS_DONE    (BIT(7))
#define SPI_DMA_SEG_TRANS_DONE_M  (SPI_DMA_SEG_TRANS_DONE_V << SPI_DMA_SEG_TRANS_DONE_S)
#define SPI_DMA_SEG_TRANS_DONE_V  0x00000001
#define SPI_DMA_SEG_TRANS_DONE_S  7

/* SPI_HOLD_OUT_TIME : R/W; bitpos: [6:4]; default: 0;
 * set the hold cycles of output spi_hold signal when spi_hold_out_en is
 * enable.
 */

#define SPI_HOLD_OUT_TIME    0x00000007
#define SPI_HOLD_OUT_TIME_M  (SPI_HOLD_OUT_TIME_V << SPI_HOLD_OUT_TIME_S)
#define SPI_HOLD_OUT_TIME_V  0x00000007
#define SPI_HOLD_OUT_TIME_S  4

/* SPI_HOLD_OUT_EN : R/W; bitpos: [3]; default: 0;
 * Enable set spi output hold value to spi_hold_reg. It can be used to hold
 * spi state machine with spi_ext_hold_en and other usr hold signals.
 */

#define SPI_HOLD_OUT_EN    (BIT(3))
#define SPI_HOLD_OUT_EN_M  (SPI_HOLD_OUT_EN_V << SPI_HOLD_OUT_EN_S)
#define SPI_HOLD_OUT_EN_V  0x00000001
#define SPI_HOLD_OUT_EN_S  3

/* SPI_HOLD_VAL_REG : R/W; bitpos: [2]; default: 0;
 * spi hold output value, which should be used with spi_hold_out_en.
 */

#define SPI_HOLD_VAL_REG    (BIT(2))
#define SPI_HOLD_VAL_REG_M  (SPI_HOLD_VAL_REG_V << SPI_HOLD_VAL_REG_S)
#define SPI_HOLD_VAL_REG_V  0x00000001
#define SPI_HOLD_VAL_REG_S  2

/* SPI_INT_HOLD_ENA : R/W; bitpos: [1:0]; default: 0;
 * This register is for two SPI masters to share the same cs clock and data
 * signals. The bits of one SPI are set, if the other SPI is busy, the SPI
 * will be hold. 1(3): hold at  idle  phase 2: hold at  prepare  phase.
 */

#define SPI_INT_HOLD_ENA    0x00000003
#define SPI_INT_HOLD_ENA_M  (SPI_INT_HOLD_ENA_V << SPI_INT_HOLD_ENA_S)
#define SPI_INT_HOLD_ENA_V  0x00000003
#define SPI_INT_HOLD_ENA_S  0

/* SPI_DMA_CONF_REG register */

#define SPI_DMA_CONF_REG(i)    (REG_SPI_BASE(i) + 0x4c)

/* SPI_EXT_MEM_BK_SIZE : R/W; bitpos: [27:26]; default: 0;
 * Select the external memory block size.
 */

#define SPI_EXT_MEM_BK_SIZE    0x00000003
#define SPI_EXT_MEM_BK_SIZE_M  (SPI_EXT_MEM_BK_SIZE_V << SPI_EXT_MEM_BK_SIZE_S)
#define SPI_EXT_MEM_BK_SIZE_V  0x00000003
#define SPI_EXT_MEM_BK_SIZE_S  26

/* SPI_DMA_OUTFIFO_EMPTY_ERR : R/W; bitpos: [25]; default: 0;
 * 1:spi_dma_outfifo_empty and spi_pop_data_prep are valid, which means that
 * there is no data to pop but pop is valid.  0: Others.  Can not be changed
 * by CONF_buf.
 */

#define SPI_DMA_OUTFIFO_EMPTY_ERR    (BIT(25))
#define SPI_DMA_OUTFIFO_EMPTY_ERR_M  (SPI_DMA_OUTFIFO_EMPTY_ERR_V << SPI_DMA_OUTFIFO_EMPTY_ERR_S)
#define SPI_DMA_OUTFIFO_EMPTY_ERR_V  0x00000001
#define SPI_DMA_OUTFIFO_EMPTY_ERR_S  25

/* SPI_DMA_INFIFO_FULL_ERR : R/W; bitpos: [24]; default: 0;
 * 1:spi_dma_infifo_full and spi_push_data_prep are valid, which means that
 * DMA Rx buffer is full but push is valid.  0: Others.  Can not be changed
 * by CONF_buf.
 */

#define SPI_DMA_INFIFO_FULL_ERR    (BIT(24))
#define SPI_DMA_INFIFO_FULL_ERR_M  (SPI_DMA_INFIFO_FULL_ERR_V << SPI_DMA_INFIFO_FULL_ERR_S)
#define SPI_DMA_INFIFO_FULL_ERR_V  0x00000001
#define SPI_DMA_INFIFO_FULL_ERR_S  24

/* SPI_DMA_OUTFIFO_EMPTY_CLR : R/W; bitpos: [23]; default: 0;
 * 1:Clear spi_dma_outfifo_empty_vld. 0: Do not control it.
 */

#define SPI_DMA_OUTFIFO_EMPTY_CLR    (BIT(23))
#define SPI_DMA_OUTFIFO_EMPTY_CLR_M  (SPI_DMA_OUTFIFO_EMPTY_CLR_V << SPI_DMA_OUTFIFO_EMPTY_CLR_S)
#define SPI_DMA_OUTFIFO_EMPTY_CLR_V  0x00000001
#define SPI_DMA_OUTFIFO_EMPTY_CLR_S  23

/* SPI_DMA_INFIFO_FULL_CLR : R/W; bitpos: [22]; default: 0;
 * 1:Clear spi_dma_infifo_full_vld. 0: Do not control it.
 */

#define SPI_DMA_INFIFO_FULL_CLR    (BIT(22))
#define SPI_DMA_INFIFO_FULL_CLR_M  (SPI_DMA_INFIFO_FULL_CLR_V << SPI_DMA_INFIFO_FULL_CLR_S)
#define SPI_DMA_INFIFO_FULL_CLR_V  0x00000001
#define SPI_DMA_INFIFO_FULL_CLR_S  22

/* SPI_REG_SPI_REG_RX_EOF_EN : R/W; bitpos: [21]; default: 0;
 * 1: spi_dma_inlink_eof is set when the number of dma pushed data bytes is
 * equal to the value of spi_slv/mst_dma_rd_bytelen[19:0] in spi dma
 * transition.  0: spi_dma_inlink_eof is set by spi_trans_done in
 * non-seg-trans or spi_dma_seg_trans_done in seg-trans.
 */

#define SPI_REG_SPI_REG_RX_EOF_EN    (BIT(21))
#define SPI_REG_SPI_REG_RX_EOF_EN_M  (SPI_REG_SPI_REG_RX_EOF_EN_V << SPI_REG_SPI_REG_RX_EOF_EN_S)
#define SPI_REG_SPI_REG_RX_EOF_EN_V  0x00000001
#define SPI_REG_SPI_REG_RX_EOF_EN_S  21

/* SPI_SLV_TX_SEG_TRANS_CLR_EN : R/W; bitpos: [20]; default: 0;
 * 1: spi_dma_outfifo_empty_vld is cleared by spi slave cmd 6. 0:
 * spi_dma_outfifo_empty_vld is cleared by spi_trans_done.
 */

#define SPI_SLV_TX_SEG_TRANS_CLR_EN    (BIT(20))
#define SPI_SLV_TX_SEG_TRANS_CLR_EN_M  (SPI_SLV_TX_SEG_TRANS_CLR_EN_V << SPI_SLV_TX_SEG_TRANS_CLR_EN_S)
#define SPI_SLV_TX_SEG_TRANS_CLR_EN_V  0x00000001
#define SPI_SLV_TX_SEG_TRANS_CLR_EN_S  20

/* SPI_SLV_RX_SEG_TRANS_CLR_EN : R/W; bitpos: [19]; default: 0;
 * 1: spi_dma_infifo_full_vld is cleared by spi slave cmd 5. 0:
 * spi_dma_infifo_full_vld is cleared by spi_trans_done.
 */

#define SPI_SLV_RX_SEG_TRANS_CLR_EN    (BIT(19))
#define SPI_SLV_RX_SEG_TRANS_CLR_EN_M  (SPI_SLV_RX_SEG_TRANS_CLR_EN_V << SPI_SLV_RX_SEG_TRANS_CLR_EN_S)
#define SPI_SLV_RX_SEG_TRANS_CLR_EN_V  0x00000001
#define SPI_SLV_RX_SEG_TRANS_CLR_EN_S  19

/* SPI_DMA_SLV_SEG_TRANS_EN : R/W; bitpos: [18]; default: 0;
 * Enable dma segment transfer in spi dma half slave mode. 1: enable. 0:
 * disable.
 */

#define SPI_DMA_SLV_SEG_TRANS_EN    (BIT(18))
#define SPI_DMA_SLV_SEG_TRANS_EN_M  (SPI_DMA_SLV_SEG_TRANS_EN_V << SPI_DMA_SLV_SEG_TRANS_EN_S)
#define SPI_DMA_SLV_SEG_TRANS_EN_V  0x00000001
#define SPI_DMA_SLV_SEG_TRANS_EN_S  18

/* SPI_SLV_LAST_SEG_POP_CLR : R/W; bitpos: [17]; default: 0;
 * 1: Clear spi_slv_seg_frt_pop_mask. 0 : others
 */

#define SPI_SLV_LAST_SEG_POP_CLR    (BIT(17))
#define SPI_SLV_LAST_SEG_POP_CLR_M  (SPI_SLV_LAST_SEG_POP_CLR_V << SPI_SLV_LAST_SEG_POP_CLR_S)
#define SPI_SLV_LAST_SEG_POP_CLR_V  0x00000001
#define SPI_SLV_LAST_SEG_POP_CLR_S  17

/* SPI_DMA_CONTINUE : R/W; bitpos: [16]; default: 0;
 * spi dma continue tx/rx data.
 */

#define SPI_DMA_CONTINUE    (BIT(16))
#define SPI_DMA_CONTINUE_M  (SPI_DMA_CONTINUE_V << SPI_DMA_CONTINUE_S)
#define SPI_DMA_CONTINUE_V  0x00000001
#define SPI_DMA_CONTINUE_S  16

/* SPI_DMA_TX_STOP : R/W; bitpos: [15]; default: 0;
 * spi dma write data stop when in continue tx/rx mode.
 */

#define SPI_DMA_TX_STOP    (BIT(15))
#define SPI_DMA_TX_STOP_M  (SPI_DMA_TX_STOP_V << SPI_DMA_TX_STOP_S)
#define SPI_DMA_TX_STOP_V  0x00000001
#define SPI_DMA_TX_STOP_S  15

/* SPI_REG_SPI_REG_DMA_RX_STOP : R/W; bitpos: [14]; default: 0;
 * spi dma read data stop  when in continue tx/rx mode.
 */

#define SPI_REG_SPI_REG_DMA_RX_STOP    (BIT(14))
#define SPI_REG_SPI_REG_DMA_RX_STOP_M  (SPI_REG_SPI_REG_DMA_RX_STOP_V << SPI_REG_SPI_REG_DMA_RX_STOP_S)
#define SPI_REG_SPI_REG_DMA_RX_STOP_V  0x00000001
#define SPI_REG_SPI_REG_DMA_RX_STOP_S  14

/* SPI_MEM_TRANS_EN : R/W; bitpos: [13]; default: 0;
 * DMA internal memory data transfer enable signal.
 */

#define SPI_MEM_TRANS_EN    (BIT(13))
#define SPI_MEM_TRANS_EN_M  (SPI_MEM_TRANS_EN_V << SPI_MEM_TRANS_EN_S)
#define SPI_MEM_TRANS_EN_V  0x00000001
#define SPI_MEM_TRANS_EN_S  13

/* SPI_OUT_DATA_BURST_EN : R/W; bitpos: [12]; default: 0;
 * spi dma read data from memory in burst mode.
 */

#define SPI_OUT_DATA_BURST_EN    (BIT(12))
#define SPI_OUT_DATA_BURST_EN_M  (SPI_OUT_DATA_BURST_EN_V << SPI_OUT_DATA_BURST_EN_S)
#define SPI_OUT_DATA_BURST_EN_V  0x00000001
#define SPI_OUT_DATA_BURST_EN_S  12

/* SPI_INDSCR_BURST_EN : R/W; bitpos: [11]; default: 0;
 * read descriptor use burst mode when write data to memory.
 */

#define SPI_INDSCR_BURST_EN    (BIT(11))
#define SPI_INDSCR_BURST_EN_M  (SPI_INDSCR_BURST_EN_V << SPI_INDSCR_BURST_EN_S)
#define SPI_INDSCR_BURST_EN_V  0x00000001
#define SPI_INDSCR_BURST_EN_S  11

/* SPI_OUTDSCR_BURST_EN : R/W; bitpos: [10]; default: 0;
 * read descriptor use burst mode when read data for memory.
 */

#define SPI_OUTDSCR_BURST_EN    (BIT(10))
#define SPI_OUTDSCR_BURST_EN_M  (SPI_OUTDSCR_BURST_EN_V << SPI_OUTDSCR_BURST_EN_S)
#define SPI_OUTDSCR_BURST_EN_V  0x00000001
#define SPI_OUTDSCR_BURST_EN_S  10

/* SPI_OUT_EOF_MODE : R/W; bitpos: [9]; default: 1;
 * out eof flag generation mode . 1: when dma pop all data from fifo  0:when
 * ahb push all data to fifo.
 */

#define SPI_OUT_EOF_MODE    (BIT(9))
#define SPI_OUT_EOF_MODE_M  (SPI_OUT_EOF_MODE_V << SPI_OUT_EOF_MODE_S)
#define SPI_OUT_EOF_MODE_V  0x00000001
#define SPI_OUT_EOF_MODE_S  9

/* SPI_OUT_AUTO_WRBACK : R/W; bitpos: [8]; default: 0;
 * when the bit is set, DMA continue to use the next inlink node when the
 * length of inlink is 0.
 */

#define SPI_OUT_AUTO_WRBACK    (BIT(8))
#define SPI_OUT_AUTO_WRBACK_M  (SPI_OUT_AUTO_WRBACK_V << SPI_OUT_AUTO_WRBACK_S)
#define SPI_OUT_AUTO_WRBACK_V  0x00000001
#define SPI_OUT_AUTO_WRBACK_S  8

/* SPI_OUT_LOOP_TEST : R/W; bitpos: [7]; default: 0;
 * Set bit to test out link.
 */

#define SPI_OUT_LOOP_TEST    (BIT(7))
#define SPI_OUT_LOOP_TEST_M  (SPI_OUT_LOOP_TEST_V << SPI_OUT_LOOP_TEST_S)
#define SPI_OUT_LOOP_TEST_V  0x00000001
#define SPI_OUT_LOOP_TEST_S  7

/* SPI_IN_LOOP_TEST : R/W; bitpos: [6]; default: 0;
 * Set bit to test in link.
 */

#define SPI_IN_LOOP_TEST    (BIT(6))
#define SPI_IN_LOOP_TEST_M  (SPI_IN_LOOP_TEST_V << SPI_IN_LOOP_TEST_S)
#define SPI_IN_LOOP_TEST_V  0x00000001
#define SPI_IN_LOOP_TEST_S  6

/* SPI_AHBM_RST : R/W; bitpos: [5]; default: 0;
 * Reset spi dma ahb master.
 */

#define SPI_AHBM_RST    (BIT(5))
#define SPI_AHBM_RST_M  (SPI_AHBM_RST_V << SPI_AHBM_RST_S)
#define SPI_AHBM_RST_V  0x00000001
#define SPI_AHBM_RST_S  5

/* SPI_AHBM_FIFO_RST : R/W; bitpos: [4]; default: 0;
 * Reset spi dma ahb master fifo pointer.
 */

#define SPI_AHBM_FIFO_RST    (BIT(4))
#define SPI_AHBM_FIFO_RST_M  (SPI_AHBM_FIFO_RST_V << SPI_AHBM_FIFO_RST_S)
#define SPI_AHBM_FIFO_RST_V  0x00000001
#define SPI_AHBM_FIFO_RST_S  4

/* SPI_OUT_RST : R/W; bitpos: [3]; default: 0;
 * The bit is used to reset out dma fsm and out data fifo pointer.
 */

#define SPI_OUT_RST    (BIT(3))
#define SPI_OUT_RST_M  (SPI_OUT_RST_V << SPI_OUT_RST_S)
#define SPI_OUT_RST_V  0x00000001
#define SPI_OUT_RST_S  3

/* SPI_IN_RST : R/W; bitpos: [2]; default: 0;
 * The bit is used to reset in dma fsm and in data fifo pointer.
 */

#define SPI_IN_RST    (BIT(2))
#define SPI_IN_RST_M  (SPI_IN_RST_V << SPI_IN_RST_S)
#define SPI_IN_RST_V  0x00000001
#define SPI_IN_RST_S  2

/* SPI_DMA_OUT_LINK_REG register */

#define SPI_DMA_OUT_LINK_REG(i)  (REG_SPI_BASE(i) + 0x50)

/* SPI_DMA_TX_ENA : R/W; bitpos: [31]; default: 0;
 * spi dma write data status bit.
 */

#define SPI_DMA_TX_ENA    (BIT(31))
#define SPI_DMA_TX_ENA_M  (SPI_DMA_TX_ENA_V << SPI_DMA_TX_ENA_S)
#define SPI_DMA_TX_ENA_V  0x00000001
#define SPI_DMA_TX_ENA_S  31

/* SPI_OUTLINK_RESTART : R/W; bitpos: [30]; default: 0;
 * Set the bit to mount on new outlink descriptors.
 */

#define SPI_OUTLINK_RESTART    (BIT(30))
#define SPI_OUTLINK_RESTART_M  (SPI_OUTLINK_RESTART_V << SPI_OUTLINK_RESTART_S)
#define SPI_OUTLINK_RESTART_V  0x00000001
#define SPI_OUTLINK_RESTART_S  30

/* SPI_OUTLINK_START : R/W; bitpos: [29]; default: 0;
 * Set the bit to start to use outlink descriptor.
 */

#define SPI_OUTLINK_START    (BIT(29))
#define SPI_OUTLINK_START_M  (SPI_OUTLINK_START_V << SPI_OUTLINK_START_S)
#define SPI_OUTLINK_START_V  0x00000001
#define SPI_OUTLINK_START_S  29

/* SPI_OUTLINK_STOP : R/W; bitpos: [28]; default: 0;
 * Set the bit to stop to use outlink descriptor.
 */

#define SPI_OUTLINK_STOP    (BIT(28))
#define SPI_OUTLINK_STOP_M  (SPI_OUTLINK_STOP_V << SPI_OUTLINK_STOP_S)
#define SPI_OUTLINK_STOP_V  0x00000001
#define SPI_OUTLINK_STOP_S  28

/* SPI_OUTLINK_ADDR : R/W; bitpos: [19:0]; default: 0;
 * The address of the first outlink descriptor.
 */

#define SPI_OUTLINK_ADDR    0x000FFFFF
#define SPI_OUTLINK_ADDR_M  (SPI_OUTLINK_ADDR_V << SPI_OUTLINK_ADDR_S)
#define SPI_OUTLINK_ADDR_V  0x000FFFFF
#define SPI_OUTLINK_ADDR_S  0

/* SPI_DMA_IN_LINK_REG register */

#define SPI_DMA_IN_LINK_REG(i)   (REG_SPI_BASE(i) + 0x54)

/* SPI_DMA_RX_ENA : R/W; bitpos: [31]; default: 0;
 * spi dma read data status bit.
 */

#define SPI_DMA_RX_ENA    (BIT(31))
#define SPI_DMA_RX_ENA_M  (SPI_DMA_RX_ENA_V << SPI_DMA_RX_ENA_S)
#define SPI_DMA_RX_ENA_V  0x00000001
#define SPI_DMA_RX_ENA_S  31

/* SPI_INLINK_RESTART : R/W; bitpos: [30]; default: 0;
 * Set the bit to mount on new inlink descriptors.
 */

#define SPI_INLINK_RESTART    (BIT(30))
#define SPI_INLINK_RESTART_M  (SPI_INLINK_RESTART_V << SPI_INLINK_RESTART_S)
#define SPI_INLINK_RESTART_V  0x00000001
#define SPI_INLINK_RESTART_S  30

/* SPI_INLINK_START : R/W; bitpos: [29]; default: 0;
 * Set the bit to start to use inlink descriptor.
 */

#define SPI_INLINK_START    (BIT(29))
#define SPI_INLINK_START_M  (SPI_INLINK_START_V << SPI_INLINK_START_S)
#define SPI_INLINK_START_V  0x00000001
#define SPI_INLINK_START_S  29

/* SPI_INLINK_STOP : R/W; bitpos: [28]; default: 0;
 * Set the bit to stop to use inlink descriptor.
 */

#define SPI_INLINK_STOP    (BIT(28))
#define SPI_INLINK_STOP_M  (SPI_INLINK_STOP_V << SPI_INLINK_STOP_S)
#define SPI_INLINK_STOP_V  0x00000001
#define SPI_INLINK_STOP_S  28

/* SPI_INLINK_AUTO_RET : R/W; bitpos: [20]; default: 0;
 * when the bit is set, the inlink descriptor returns to the first link node
 * when a packet is error.
 */

#define SPI_INLINK_AUTO_RET    (BIT(20))
#define SPI_INLINK_AUTO_RET_M  (SPI_INLINK_AUTO_RET_V << SPI_INLINK_AUTO_RET_S)
#define SPI_INLINK_AUTO_RET_V  0x00000001
#define SPI_INLINK_AUTO_RET_S  20

/* SPI_INLINK_ADDR : R/W; bitpos: [19:0]; default: 0;
 * The address of the first inlink descriptor.
 */

#define SPI_INLINK_ADDR    0x000FFFFF
#define SPI_INLINK_ADDR_M  (SPI_INLINK_ADDR_V << SPI_INLINK_ADDR_S)
#define SPI_INLINK_ADDR_V  0x000FFFFF
#define SPI_INLINK_ADDR_S  0

/* SPI_DMA_INT_ENA_REG register */

#define SPI_DMA_INT_ENA_REG(i)       (REG_SPI_BASE(i) + 0x58)

/* SPI_OUT_TOTAL_EOF_INT_ENA : R/W; bitpos: [8]; default: 0;
 * The enable bit for sending all the packets to host done.
 */

#define SPI_OUT_TOTAL_EOF_INT_ENA    (BIT(8))
#define SPI_OUT_TOTAL_EOF_INT_ENA_M  (SPI_OUT_TOTAL_EOF_INT_ENA_V << SPI_OUT_TOTAL_EOF_INT_ENA_S)
#define SPI_OUT_TOTAL_EOF_INT_ENA_V  0x00000001
#define SPI_OUT_TOTAL_EOF_INT_ENA_S  8

/* SPI_OUT_EOF_INT_ENA : R/W; bitpos: [7]; default: 0;
 * The enable bit for sending a packet to host done.
 */

#define SPI_OUT_EOF_INT_ENA    (BIT(7))
#define SPI_OUT_EOF_INT_ENA_M  (SPI_OUT_EOF_INT_ENA_V << SPI_OUT_EOF_INT_ENA_S)
#define SPI_OUT_EOF_INT_ENA_V  0x00000001
#define SPI_OUT_EOF_INT_ENA_S  7

/* SPI_OUT_DONE_INT_ENA : R/W; bitpos: [6]; default: 0;
 * The enable bit for completing usage of a outlink descriptor .
 */

#define SPI_OUT_DONE_INT_ENA    (BIT(6))
#define SPI_OUT_DONE_INT_ENA_M  (SPI_OUT_DONE_INT_ENA_V << SPI_OUT_DONE_INT_ENA_S)
#define SPI_OUT_DONE_INT_ENA_V  0x00000001
#define SPI_OUT_DONE_INT_ENA_S  6

/* SPI_IN_SUC_EOF_INT_ENA : R/W; bitpos: [5]; default: 0;
 * The enable bit for completing receiving all the packets from host.
 */

#define SPI_IN_SUC_EOF_INT_ENA    (BIT(5))
#define SPI_IN_SUC_EOF_INT_ENA_M  (SPI_IN_SUC_EOF_INT_ENA_V << SPI_IN_SUC_EOF_INT_ENA_S)
#define SPI_IN_SUC_EOF_INT_ENA_V  0x00000001
#define SPI_IN_SUC_EOF_INT_ENA_S  5

/* SPI_IN_ERR_EOF_INT_ENA : R/W; bitpos: [4]; default: 0;
 * The enable bit for receiving error.
 */

#define SPI_IN_ERR_EOF_INT_ENA    (BIT(4))
#define SPI_IN_ERR_EOF_INT_ENA_M  (SPI_IN_ERR_EOF_INT_ENA_V << SPI_IN_ERR_EOF_INT_ENA_S)
#define SPI_IN_ERR_EOF_INT_ENA_V  0x00000001
#define SPI_IN_ERR_EOF_INT_ENA_S  4

/* SPI_IN_DONE_INT_ENA : R/W; bitpos: [3]; default: 0;
 * The enable bit for completing usage of a inlink descriptor.
 */

#define SPI_IN_DONE_INT_ENA    (BIT(3))
#define SPI_IN_DONE_INT_ENA_M  (SPI_IN_DONE_INT_ENA_V << SPI_IN_DONE_INT_ENA_S)
#define SPI_IN_DONE_INT_ENA_V  0x00000001
#define SPI_IN_DONE_INT_ENA_S  3

/* SPI_INLINK_DSCR_ERROR_INT_ENA : R/W; bitpos: [2]; default: 0;
 * The enable bit for inlink descriptor error.
 */

#define SPI_INLINK_DSCR_ERROR_INT_ENA    (BIT(2))
#define SPI_INLINK_DSCR_ERROR_INT_ENA_M  (SPI_INLINK_DSCR_ERROR_INT_ENA_V << SPI_INLINK_DSCR_ERROR_INT_ENA_S)
#define SPI_INLINK_DSCR_ERROR_INT_ENA_V  0x00000001
#define SPI_INLINK_DSCR_ERROR_INT_ENA_S  2

/* SPI_OUTLINK_DSCR_ERROR_INT_ENA : R/W; bitpos: [1]; default: 0;
 * The enable bit for outlink descriptor error.
 */

#define SPI_OUTLINK_DSCR_ERROR_INT_ENA    (BIT(1))
#define SPI_OUTLINK_DSCR_ERROR_INT_ENA_M  (SPI_OUTLINK_DSCR_ERROR_INT_ENA_V << SPI_OUTLINK_DSCR_ERROR_INT_ENA_S)
#define SPI_OUTLINK_DSCR_ERROR_INT_ENA_V  0x00000001
#define SPI_OUTLINK_DSCR_ERROR_INT_ENA_S  1

/* SPI_INLINK_DSCR_EMPTY_INT_ENA : R/W; bitpos: [0]; default: 0;
 * The enable bit for lack of enough inlink descriptors.
 */

#define SPI_INLINK_DSCR_EMPTY_INT_ENA    (BIT(0))
#define SPI_INLINK_DSCR_EMPTY_INT_ENA_M  (SPI_INLINK_DSCR_EMPTY_INT_ENA_V << SPI_INLINK_DSCR_EMPTY_INT_ENA_S)
#define SPI_INLINK_DSCR_EMPTY_INT_ENA_V  0x00000001
#define SPI_INLINK_DSCR_EMPTY_INT_ENA_S  0

/* SPI_DMA_INT_RAW_REG register */

#define SPI_DMA_INT_RAW_REG(i)       (REG_SPI_BASE(i) + 0x5c)

/* SPI_OUT_TOTAL_EOF_INT_RAW : RO; bitpos: [8]; default: 0;
 * The raw bit for sending all the packets to host done.
 */

#define SPI_OUT_TOTAL_EOF_INT_RAW    (BIT(8))
#define SPI_OUT_TOTAL_EOF_INT_RAW_M  (SPI_OUT_TOTAL_EOF_INT_RAW_V << SPI_OUT_TOTAL_EOF_INT_RAW_S)
#define SPI_OUT_TOTAL_EOF_INT_RAW_V  0x00000001
#define SPI_OUT_TOTAL_EOF_INT_RAW_S  8

/* SPI_OUT_EOF_INT_RAW : RO; bitpos: [7]; default: 0;
 * The raw bit for sending a packet to host done.
 */

#define SPI_OUT_EOF_INT_RAW    (BIT(7))
#define SPI_OUT_EOF_INT_RAW_M  (SPI_OUT_EOF_INT_RAW_V << SPI_OUT_EOF_INT_RAW_S)
#define SPI_OUT_EOF_INT_RAW_V  0x00000001
#define SPI_OUT_EOF_INT_RAW_S  7

/* SPI_OUT_DONE_INT_RAW : RO; bitpos: [6]; default: 0;
 * The raw bit for completing usage of a outlink descriptor.
 */

#define SPI_OUT_DONE_INT_RAW    (BIT(6))
#define SPI_OUT_DONE_INT_RAW_M  (SPI_OUT_DONE_INT_RAW_V << SPI_OUT_DONE_INT_RAW_S)
#define SPI_OUT_DONE_INT_RAW_V  0x00000001
#define SPI_OUT_DONE_INT_RAW_S  6

/* SPI_IN_SUC_EOF_INT_RAW : RO; bitpos: [5]; default: 0;
 * The raw bit for completing receiving all the packets from host.
 */

#define SPI_IN_SUC_EOF_INT_RAW    (BIT(5))
#define SPI_IN_SUC_EOF_INT_RAW_M  (SPI_IN_SUC_EOF_INT_RAW_V << SPI_IN_SUC_EOF_INT_RAW_S)
#define SPI_IN_SUC_EOF_INT_RAW_V  0x00000001
#define SPI_IN_SUC_EOF_INT_RAW_S  5

/* SPI_IN_ERR_EOF_INT_RAW : RO; bitpos: [4]; default: 0;
 * The raw bit for receiving error.
 */

#define SPI_IN_ERR_EOF_INT_RAW    (BIT(4))
#define SPI_IN_ERR_EOF_INT_RAW_M  (SPI_IN_ERR_EOF_INT_RAW_V << SPI_IN_ERR_EOF_INT_RAW_S)
#define SPI_IN_ERR_EOF_INT_RAW_V  0x00000001
#define SPI_IN_ERR_EOF_INT_RAW_S  4

/* SPI_IN_DONE_INT_RAW : RO; bitpos: [3]; default: 0;
 * The raw bit for completing usage of a inlink descriptor.
 */

#define SPI_IN_DONE_INT_RAW    (BIT(3))
#define SPI_IN_DONE_INT_RAW_M  (SPI_IN_DONE_INT_RAW_V << SPI_IN_DONE_INT_RAW_S)
#define SPI_IN_DONE_INT_RAW_V  0x00000001
#define SPI_IN_DONE_INT_RAW_S  3

/* SPI_INLINK_DSCR_ERROR_INT_RAW : RO; bitpos: [2]; default: 0;
 * The raw bit for inlink descriptor error.
 */

#define SPI_INLINK_DSCR_ERROR_INT_RAW    (BIT(2))
#define SPI_INLINK_DSCR_ERROR_INT_RAW_M  (SPI_INLINK_DSCR_ERROR_INT_RAW_V << SPI_INLINK_DSCR_ERROR_INT_RAW_S)
#define SPI_INLINK_DSCR_ERROR_INT_RAW_V  0x00000001
#define SPI_INLINK_DSCR_ERROR_INT_RAW_S  2

/* SPI_OUTLINK_DSCR_ERROR_INT_RAW : RO; bitpos: [1]; default: 0;
 * The raw bit for outlink descriptor error.
 */

#define SPI_OUTLINK_DSCR_ERROR_INT_RAW    (BIT(1))
#define SPI_OUTLINK_DSCR_ERROR_INT_RAW_M  (SPI_OUTLINK_DSCR_ERROR_INT_RAW_V << SPI_OUTLINK_DSCR_ERROR_INT_RAW_S)
#define SPI_OUTLINK_DSCR_ERROR_INT_RAW_V  0x00000001
#define SPI_OUTLINK_DSCR_ERROR_INT_RAW_S  1

/* SPI_INLINK_DSCR_EMPTY_INT_RAW : RO; bitpos: [0]; default: 0;
 * The raw bit for lack of enough inlink descriptors.
 */

#define SPI_INLINK_DSCR_EMPTY_INT_RAW    (BIT(0))
#define SPI_INLINK_DSCR_EMPTY_INT_RAW_M  (SPI_INLINK_DSCR_EMPTY_INT_RAW_V << SPI_INLINK_DSCR_EMPTY_INT_RAW_S)
#define SPI_INLINK_DSCR_EMPTY_INT_RAW_V  0x00000001
#define SPI_INLINK_DSCR_EMPTY_INT_RAW_S  0

/* SPI_DMA_INT_ST_REG register */

#define SPI_DMA_INT_ST_REG(i)       (REG_SPI_BASE(i) + 0x60)

/* SPI_OUT_TOTAL_EOF_INT_ST : RO; bitpos: [8]; default: 0;
 * The status bit for sending all the packets to host done.
 */

#define SPI_OUT_TOTAL_EOF_INT_ST    (BIT(8))
#define SPI_OUT_TOTAL_EOF_INT_ST_M  (SPI_OUT_TOTAL_EOF_INT_ST_V << SPI_OUT_TOTAL_EOF_INT_ST_S)
#define SPI_OUT_TOTAL_EOF_INT_ST_V  0x00000001
#define SPI_OUT_TOTAL_EOF_INT_ST_S  8

/* SPI_OUT_EOF_INT_ST : RO; bitpos: [7]; default: 0;
 * The status bit for sending a packet to host done.
 */

#define SPI_OUT_EOF_INT_ST    (BIT(7))
#define SPI_OUT_EOF_INT_ST_M  (SPI_OUT_EOF_INT_ST_V << SPI_OUT_EOF_INT_ST_S)
#define SPI_OUT_EOF_INT_ST_V  0x00000001
#define SPI_OUT_EOF_INT_ST_S  7

/* SPI_OUT_DONE_INT_ST : RO; bitpos: [6]; default: 0;
 * The status bit for completing usage of a outlink descriptor.
 */

#define SPI_OUT_DONE_INT_ST    (BIT(6))
#define SPI_OUT_DONE_INT_ST_M  (SPI_OUT_DONE_INT_ST_V << SPI_OUT_DONE_INT_ST_S)
#define SPI_OUT_DONE_INT_ST_V  0x00000001
#define SPI_OUT_DONE_INT_ST_S  6

/* SPI_IN_SUC_EOF_INT_ST : RO; bitpos: [5]; default: 0;
 * The status bit for completing receiving all the packets from host.
 */

#define SPI_IN_SUC_EOF_INT_ST    (BIT(5))
#define SPI_IN_SUC_EOF_INT_ST_M  (SPI_IN_SUC_EOF_INT_ST_V << SPI_IN_SUC_EOF_INT_ST_S)
#define SPI_IN_SUC_EOF_INT_ST_V  0x00000001
#define SPI_IN_SUC_EOF_INT_ST_S  5

/* SPI_IN_ERR_EOF_INT_ST : RO; bitpos: [4]; default: 0;
 * The status bit for receiving error.
 */

#define SPI_IN_ERR_EOF_INT_ST    (BIT(4))
#define SPI_IN_ERR_EOF_INT_ST_M  (SPI_IN_ERR_EOF_INT_ST_V << SPI_IN_ERR_EOF_INT_ST_S)
#define SPI_IN_ERR_EOF_INT_ST_V  0x00000001
#define SPI_IN_ERR_EOF_INT_ST_S  4

/* SPI_IN_DONE_INT_ST : RO; bitpos: [3]; default: 0;
 * The status bit for completing usage of a inlink descriptor.
 */

#define SPI_IN_DONE_INT_ST    (BIT(3))
#define SPI_IN_DONE_INT_ST_M  (SPI_IN_DONE_INT_ST_V << SPI_IN_DONE_INT_ST_S)
#define SPI_IN_DONE_INT_ST_V  0x00000001
#define SPI_IN_DONE_INT_ST_S  3

/* SPI_INLINK_DSCR_ERROR_INT_ST : RO; bitpos: [2]; default: 0;
 * The status bit for inlink descriptor error.
 */

#define SPI_INLINK_DSCR_ERROR_INT_ST    (BIT(2))
#define SPI_INLINK_DSCR_ERROR_INT_ST_M  (SPI_INLINK_DSCR_ERROR_INT_ST_V << SPI_INLINK_DSCR_ERROR_INT_ST_S)
#define SPI_INLINK_DSCR_ERROR_INT_ST_V  0x00000001
#define SPI_INLINK_DSCR_ERROR_INT_ST_S  2

/* SPI_OUTLINK_DSCR_ERROR_INT_ST : RO; bitpos: [1]; default: 0;
 * The status bit for outlink descriptor error.
 */

#define SPI_OUTLINK_DSCR_ERROR_INT_ST    (BIT(1))
#define SPI_OUTLINK_DSCR_ERROR_INT_ST_M  (SPI_OUTLINK_DSCR_ERROR_INT_ST_V << SPI_OUTLINK_DSCR_ERROR_INT_ST_S)
#define SPI_OUTLINK_DSCR_ERROR_INT_ST_V  0x00000001
#define SPI_OUTLINK_DSCR_ERROR_INT_ST_S  1

/* SPI_INLINK_DSCR_EMPTY_INT_ST : RO; bitpos: [0]; default: 0;
 * The status bit for lack of enough inlink descriptors.
 */

#define SPI_INLINK_DSCR_EMPTY_INT_ST    (BIT(0))
#define SPI_INLINK_DSCR_EMPTY_INT_ST_M  (SPI_INLINK_DSCR_EMPTY_INT_ST_V << SPI_INLINK_DSCR_EMPTY_INT_ST_S)
#define SPI_INLINK_DSCR_EMPTY_INT_ST_V  0x00000001
#define SPI_INLINK_DSCR_EMPTY_INT_ST_S  0

/* SPI_DMA_INT_CLR_REG register */

#define SPI_DMA_INT_CLR_REG(i)       (REG_SPI_BASE(i) + 0x64)

/* SPI_OUT_TOTAL_EOF_INT_CLR : R/W; bitpos: [8]; default: 0;
 * The clear bit for sending all the packets to host done.
 */

#define SPI_OUT_TOTAL_EOF_INT_CLR    (BIT(8))
#define SPI_OUT_TOTAL_EOF_INT_CLR_M  (SPI_OUT_TOTAL_EOF_INT_CLR_V << SPI_OUT_TOTAL_EOF_INT_CLR_S)
#define SPI_OUT_TOTAL_EOF_INT_CLR_V  0x00000001
#define SPI_OUT_TOTAL_EOF_INT_CLR_S  8

/* SPI_OUT_EOF_INT_CLR : R/W; bitpos: [7]; default: 0;
 * The clear bit for sending a packet to host done.
 */

#define SPI_OUT_EOF_INT_CLR    (BIT(7))
#define SPI_OUT_EOF_INT_CLR_M  (SPI_OUT_EOF_INT_CLR_V << SPI_OUT_EOF_INT_CLR_S)
#define SPI_OUT_EOF_INT_CLR_V  0x00000001
#define SPI_OUT_EOF_INT_CLR_S  7

/* SPI_OUT_DONE_INT_CLR : R/W; bitpos: [6]; default: 0;
 * The clear bit for completing usage of a outlink descriptor.
 */

#define SPI_OUT_DONE_INT_CLR    (BIT(6))
#define SPI_OUT_DONE_INT_CLR_M  (SPI_OUT_DONE_INT_CLR_V << SPI_OUT_DONE_INT_CLR_S)
#define SPI_OUT_DONE_INT_CLR_V  0x00000001
#define SPI_OUT_DONE_INT_CLR_S  6

/* SPI_IN_SUC_EOF_INT_CLR : R/W; bitpos: [5]; default: 0;
 * The clear bit for completing receiving all the packets from host.
 */

#define SPI_IN_SUC_EOF_INT_CLR    (BIT(5))
#define SPI_IN_SUC_EOF_INT_CLR_M  (SPI_IN_SUC_EOF_INT_CLR_V << SPI_IN_SUC_EOF_INT_CLR_S)
#define SPI_IN_SUC_EOF_INT_CLR_V  0x00000001
#define SPI_IN_SUC_EOF_INT_CLR_S  5

/* SPI_IN_ERR_EOF_INT_CLR : R/W; bitpos: [4]; default: 0;
 * The clear bit for receiving error.
 */

#define SPI_IN_ERR_EOF_INT_CLR    (BIT(4))
#define SPI_IN_ERR_EOF_INT_CLR_M  (SPI_IN_ERR_EOF_INT_CLR_V << SPI_IN_ERR_EOF_INT_CLR_S)
#define SPI_IN_ERR_EOF_INT_CLR_V  0x00000001
#define SPI_IN_ERR_EOF_INT_CLR_S  4

/* SPI_IN_DONE_INT_CLR : R/W; bitpos: [3]; default: 0;
 * The clear bit for completing usage of a inlink descriptor.
 */

#define SPI_IN_DONE_INT_CLR    (BIT(3))
#define SPI_IN_DONE_INT_CLR_M  (SPI_IN_DONE_INT_CLR_V << SPI_IN_DONE_INT_CLR_S)
#define SPI_IN_DONE_INT_CLR_V  0x00000001
#define SPI_IN_DONE_INT_CLR_S  3

/* SPI_INLINK_DSCR_ERROR_INT_CLR : R/W; bitpos: [2]; default: 0;
 * The clear bit for inlink descriptor error.
 */

#define SPI_INLINK_DSCR_ERROR_INT_CLR    (BIT(2))
#define SPI_INLINK_DSCR_ERROR_INT_CLR_M  (SPI_INLINK_DSCR_ERROR_INT_CLR_V << SPI_INLINK_DSCR_ERROR_INT_CLR_S)
#define SPI_INLINK_DSCR_ERROR_INT_CLR_V  0x00000001
#define SPI_INLINK_DSCR_ERROR_INT_CLR_S  2

/* SPI_OUTLINK_DSCR_ERROR_INT_CLR : R/W; bitpos: [1]; default: 0;
 * The clear bit for outlink descriptor error.
 */

#define SPI_OUTLINK_DSCR_ERROR_INT_CLR    (BIT(1))
#define SPI_OUTLINK_DSCR_ERROR_INT_CLR_M  (SPI_OUTLINK_DSCR_ERROR_INT_CLR_V << SPI_OUTLINK_DSCR_ERROR_INT_CLR_S)
#define SPI_OUTLINK_DSCR_ERROR_INT_CLR_V  0x00000001
#define SPI_OUTLINK_DSCR_ERROR_INT_CLR_S  1

/* SPI_INLINK_DSCR_EMPTY_INT_CLR : R/W; bitpos: [0]; default: 0;
 * The clear bit for lack of enough inlink descriptors.
 */

#define SPI_INLINK_DSCR_EMPTY_INT_CLR    (BIT(0))
#define SPI_INLINK_DSCR_EMPTY_INT_CLR_M  (SPI_INLINK_DSCR_EMPTY_INT_CLR_V << SPI_INLINK_DSCR_EMPTY_INT_CLR_S)
#define SPI_INLINK_DSCR_EMPTY_INT_CLR_V  0x00000001
#define SPI_INLINK_DSCR_EMPTY_INT_CLR_S  0

/* SPI_IN_ERR_EOF_DES_ADDR_REG register */

#define SPI_IN_ERR_EOF_DES_ADDR_REG(i)   (REG_SPI_BASE(i) + 0x68)

/* SPI_DMA_IN_ERR_EOF_DES_ADDR : RO; bitpos: [31:0]; default: 0;
 * The inlink descriptor address when spi dma produce receiving error.
 */

#define SPI_DMA_IN_ERR_EOF_DES_ADDR    0xFFFFFFFF
#define SPI_DMA_IN_ERR_EOF_DES_ADDR_M  (SPI_DMA_IN_ERR_EOF_DES_ADDR_V << SPI_DMA_IN_ERR_EOF_DES_ADDR_S)
#define SPI_DMA_IN_ERR_EOF_DES_ADDR_V  0xFFFFFFFF
#define SPI_DMA_IN_ERR_EOF_DES_ADDR_S  0

/* SPI_IN_SUC_EOF_DES_ADDR_REG register */

#define SPI_IN_SUC_EOF_DES_ADDR_REG(i) (REG_SPI_BASE(i) + 0x6c)

/* SPI_DMA_IN_SUC_EOF_DES_ADDR : RO; bitpos: [31:0]; default: 0;
 * The last inlink descriptor address when spi dma produce from_suc_eof.
 */

#define SPI_DMA_IN_SUC_EOF_DES_ADDR    0xFFFFFFFF
#define SPI_DMA_IN_SUC_EOF_DES_ADDR_M  (SPI_DMA_IN_SUC_EOF_DES_ADDR_V << SPI_DMA_IN_SUC_EOF_DES_ADDR_S)
#define SPI_DMA_IN_SUC_EOF_DES_ADDR_V  0xFFFFFFFF
#define SPI_DMA_IN_SUC_EOF_DES_ADDR_S  0

/* SPI_INLINK_DSCR_REG register */

#define SPI_INLINK_DSCR_REG(i)         (REG_SPI_BASE(i) + 0x70)

/* SPI_DMA_INLINK_DSCR : RO; bitpos: [31:0]; default: 0;
 * The content of current in descriptor pointer.
 */

#define SPI_DMA_INLINK_DSCR    0xFFFFFFFF
#define SPI_DMA_INLINK_DSCR_M  (SPI_DMA_INLINK_DSCR_V << SPI_DMA_INLINK_DSCR_S)
#define SPI_DMA_INLINK_DSCR_V  0xFFFFFFFF
#define SPI_DMA_INLINK_DSCR_S  0

/* SPI_INLINK_DSCR_BF0_REG register */

#define SPI_INLINK_DSCR_BF0_REG(i)       (REG_SPI_BASE(i) + 0x74)

/* SPI_DMA_INLINK_DSCR_BF0 : RO; bitpos: [31:0]; default: 0;
 * The content of next in descriptor pointer.
 */

#define SPI_DMA_INLINK_DSCR_BF0    0xFFFFFFFF
#define SPI_DMA_INLINK_DSCR_BF0_M  (SPI_DMA_INLINK_DSCR_BF0_V << SPI_DMA_INLINK_DSCR_BF0_S)
#define SPI_DMA_INLINK_DSCR_BF0_V  0xFFFFFFFF
#define SPI_DMA_INLINK_DSCR_BF0_S  0

/* SPI_INLINK_DSCR_BF1_REG register */

#define SPI_INLINK_DSCR_BF1_REG(i)       (REG_SPI_BASE(i) + 0x84)

/* SPI_DMA_INLINK_DSCR_BF1 : RO; bitpos: [31:0]; default: 0;
 * The content of current in descriptor data buffer pointer.
 */

#define SPI_DMA_INLINK_DSCR_BF1    0xFFFFFFFF
#define SPI_DMA_INLINK_DSCR_BF1_M  (SPI_DMA_INLINK_DSCR_BF1_V << SPI_DMA_INLINK_DSCR_BF1_S)
#define SPI_DMA_INLINK_DSCR_BF1_V  0xFFFFFFFF
#define SPI_DMA_INLINK_DSCR_BF1_S  0

/* SPI_OUT_EOF_BFR_DES_ADDR_REG register */

#define SPI_OUT_EOF_BFR_DES_ADDR_REG(i)    (REG_SPI_BASE(i) + 0x7c)

/* SPI_DMA_OUT_EOF_BFR_DES_ADDR : RO; bitpos: [31:0]; default: 0;
 * The address of buffer relative to the outlink descriptor that produce eof.
 */

#define SPI_DMA_OUT_EOF_BFR_DES_ADDR    0xFFFFFFFF
#define SPI_DMA_OUT_EOF_BFR_DES_ADDR_M  (SPI_DMA_OUT_EOF_BFR_DES_ADDR_V << SPI_DMA_OUT_EOF_BFR_DES_ADDR_S)
#define SPI_DMA_OUT_EOF_BFR_DES_ADDR_V  0xFFFFFFFF
#define SPI_DMA_OUT_EOF_BFR_DES_ADDR_S  0

/* SPI_OUT_EOF_DES_ADDR_REG register */

#define SPI_OUT_EOF_DES_ADDR_REG(i)     (REG_SPI_BASE(i) + 0x80)

/* SPI_DMA_OUT_EOF_DES_ADDR : RO; bitpos: [31:0]; default: 0;
 * The last outlink descriptor address when spi dma produce to_eof.
 */

#define SPI_DMA_OUT_EOF_DES_ADDR    0xFFFFFFFF
#define SPI_DMA_OUT_EOF_DES_ADDR_M  (SPI_DMA_OUT_EOF_DES_ADDR_V << SPI_DMA_OUT_EOF_DES_ADDR_S)
#define SPI_DMA_OUT_EOF_DES_ADDR_V  0xFFFFFFFF
#define SPI_DMA_OUT_EOF_DES_ADDR_S  0

/* SPI_OUTLINK_DSCR_REG register */

#define SPI_OUTLINK_DSCR_REG(i)     (REG_SPI_BASE(i) + 0x84)

/* SPI_DMA_OUTLINK_DSCR : RO; bitpos: [31:0]; default: 0;
 * The content of current out descriptor pointer.
 */

#define SPI_DMA_OUTLINK_DSCR    0xFFFFFFFF
#define SPI_DMA_OUTLINK_DSCR_M  (SPI_DMA_OUTLINK_DSCR_V << SPI_DMA_OUTLINK_DSCR_S)
#define SPI_DMA_OUTLINK_DSCR_V  0xFFFFFFFF
#define SPI_DMA_OUTLINK_DSCR_S  0

/* SPI_OUTLINK_DSCR_BF0_REG register */

#define SPI_OUTLINK_DSCR_BF0_REG(i)       (REG_SPI_BASE(i) + 0x88)

/* SPI_DMA_OUTLINK_DSCR_BF0 : RO; bitpos: [31:0]; default: 0;
 * The content of next out descriptor pointer.
 */

#define SPI_DMA_OUTLINK_DSCR_BF0    0xFFFFFFFF
#define SPI_DMA_OUTLINK_DSCR_BF0_M  (SPI_DMA_OUTLINK_DSCR_BF0_V << SPI_DMA_OUTLINK_DSCR_BF0_S)
#define SPI_DMA_OUTLINK_DSCR_BF0_V  0xFFFFFFFF
#define SPI_DMA_OUTLINK_DSCR_BF0_S  0

/* SPI_OUTLINK_DSCR_BF1_REG register */

#define SPI_OUTLINK_DSCR_BF1_REG(i)       (REG_SPI_BASE(i) + 0x8c)

/* SPI_DMA_OUTLINK_DSCR_BF1 : RO; bitpos: [31:0]; default: 0;
 * The content of current out descriptor data buffer pointer.
 */

#define SPI_DMA_OUTLINK_DSCR_BF1    0xFFFFFFFF
#define SPI_DMA_OUTLINK_DSCR_BF1_M  (SPI_DMA_OUTLINK_DSCR_BF1_V << SPI_DMA_OUTLINK_DSCR_BF1_S)
#define SPI_DMA_OUTLINK_DSCR_BF1_V  0xFFFFFFFF
#define SPI_DMA_OUTLINK_DSCR_BF1_S  0

/* SPI_DMA_OUTSTATUS_REG register */

#define SPI_DMA_OUTSTATUS_REG(i)       (REG_SPI_BASE(i) + 0x90)

/* SPI_DMA_OUTFIFO_EMPTY : RO; bitpos: [31]; default: 1;
 * SPI dma outfifo is empty.
 */

#define SPI_DMA_OUTFIFO_EMPTY    (BIT(31))
#define SPI_DMA_OUTFIFO_EMPTY_M  (SPI_DMA_OUTFIFO_EMPTY_V << SPI_DMA_OUTFIFO_EMPTY_S)
#define SPI_DMA_OUTFIFO_EMPTY_V  0x00000001
#define SPI_DMA_OUTFIFO_EMPTY_S  31

/* SPI_DMA_OUTFIFO_FULL : RO; bitpos: [30]; default: 0;
 * SPI dma outfifo is full.
 */

#define SPI_DMA_OUTFIFO_FULL    (BIT(30))
#define SPI_DMA_OUTFIFO_FULL_M  (SPI_DMA_OUTFIFO_FULL_V << SPI_DMA_OUTFIFO_FULL_S)
#define SPI_DMA_OUTFIFO_FULL_V  0x00000001
#define SPI_DMA_OUTFIFO_FULL_S  30

/* SPI_DMA_OUTFIFO_CNT : RO; bitpos: [29:23]; default: 0;
 * The remains of SPI dma outfifo data.
 */

#define SPI_DMA_OUTFIFO_CNT    0x0000007F
#define SPI_DMA_OUTFIFO_CNT_M  (SPI_DMA_OUTFIFO_CNT_V << SPI_DMA_OUTFIFO_CNT_S)
#define SPI_DMA_OUTFIFO_CNT_V  0x0000007F
#define SPI_DMA_OUTFIFO_CNT_S  23

/* SPI_DMA_OUT_STATE : RO; bitpos: [22:20]; default: 0;
 * SPI dma out data state.
 */

#define SPI_DMA_OUT_STATE    0x00000007
#define SPI_DMA_OUT_STATE_M  (SPI_DMA_OUT_STATE_V << SPI_DMA_OUT_STATE_S)
#define SPI_DMA_OUT_STATE_V  0x00000007
#define SPI_DMA_OUT_STATE_S  20

/* SPI_DMA_OUTDSCR_STATE : RO; bitpos: [19:18]; default: 0;
 * SPI dma out descriptor state.
 */

#define SPI_DMA_OUTDSCR_STATE    0x00000003
#define SPI_DMA_OUTDSCR_STATE_M  (SPI_DMA_OUTDSCR_STATE_V << SPI_DMA_OUTDSCR_STATE_S)
#define SPI_DMA_OUTDSCR_STATE_V  0x00000003
#define SPI_DMA_OUTDSCR_STATE_S  18

/* SPI_DMA_OUTDSCR_ADDR : RO; bitpos: [17:0]; default: 0;
 * SPI dma out descriptor address.
 */

#define SPI_DMA_OUTDSCR_ADDR    0x0003FFFF
#define SPI_DMA_OUTDSCR_ADDR_M  (SPI_DMA_OUTDSCR_ADDR_V << SPI_DMA_OUTDSCR_ADDR_S)
#define SPI_DMA_OUTDSCR_ADDR_V  0x0003FFFF
#define SPI_DMA_OUTDSCR_ADDR_S  0

/* SPI_DMA_INSTATUS_REG register */

#define SPI_DMA_INSTATUS_REG(i)       (REG_SPI_BASE(i) + 0x94)

/* SPI_DMA_INFIFO_EMPTY : RO; bitpos: [31]; default: 1;
 * SPI dma infifo is empty.
 */

#define SPI_DMA_INFIFO_EMPTY    (BIT(31))
#define SPI_DMA_INFIFO_EMPTY_M  (SPI_DMA_INFIFO_EMPTY_V << SPI_DMA_INFIFO_EMPTY_S)
#define SPI_DMA_INFIFO_EMPTY_V  0x00000001
#define SPI_DMA_INFIFO_EMPTY_S  31

/* SPI_DMA_INFIFO_FULL : RO; bitpos: [30]; default: 0;
 * SPI dma infifo is full.
 */

#define SPI_DMA_INFIFO_FULL    (BIT(30))
#define SPI_DMA_INFIFO_FULL_M  (SPI_DMA_INFIFO_FULL_V << SPI_DMA_INFIFO_FULL_S)
#define SPI_DMA_INFIFO_FULL_V  0x00000001
#define SPI_DMA_INFIFO_FULL_S  30

/* SPI_DMA_INFIFO_CNT : RO; bitpos: [29:23]; default: 0;
 * The remains of SPI dma infifo data.
 */

#define SPI_DMA_INFIFO_CNT    0x0000007F
#define SPI_DMA_INFIFO_CNT_M  (SPI_DMA_INFIFO_CNT_V << SPI_DMA_INFIFO_CNT_S)
#define SPI_DMA_INFIFO_CNT_V  0x0000007F
#define SPI_DMA_INFIFO_CNT_S  23

/* SPI_DMA_IN_STATE : RO; bitpos: [22:20]; default: 0;
 * SPI dma in data state.
 */

#define SPI_DMA_IN_STATE    0x00000007
#define SPI_DMA_IN_STATE_M  (SPI_DMA_IN_STATE_V << SPI_DMA_IN_STATE_S)
#define SPI_DMA_IN_STATE_V  0x00000007
#define SPI_DMA_IN_STATE_S  20

/* SPI_DMA_INDSCR_STATE : RO; bitpos: [19:18]; default: 0;
 * SPI dma in descriptor state.
 */

#define SPI_DMA_INDSCR_STATE    0x00000003
#define SPI_DMA_INDSCR_STATE_M  (SPI_DMA_INDSCR_STATE_V << SPI_DMA_INDSCR_STATE_S)
#define SPI_DMA_INDSCR_STATE_V  0x00000003
#define SPI_DMA_INDSCR_STATE_S  18

/* SPI_DMA_INDSCR_ADDR : RO; bitpos: [17:0]; default: 0;
 * SPI dma in descriptor address.
 */

#define SPI_DMA_INDSCR_ADDR    0x0003FFFF
#define SPI_DMA_INDSCR_ADDR_M  (SPI_DMA_INDSCR_ADDR_V << SPI_DMA_INDSCR_ADDR_S)
#define SPI_DMA_INDSCR_ADDR_V  0x0003FFFF
#define SPI_DMA_INDSCR_ADDR_S  0

/* SPI_W0_REG register */

#define SPI_W0_REG(i)      (REG_SPI_BASE(i) + 0x98)

/* SPI_BUF0 : R/W; bitpos: [31:0]; default: 0;
 * data buffer
 */

#define SPI_BUF0    0xFFFFFFFF
#define SPI_BUF0_M  (SPI_BUF0_V << SPI_BUF0_S)
#define SPI_BUF0_V  0xFFFFFFFF
#define SPI_BUF0_S  0

/* SPI_W1_REG register */

#define SPI_W1_REG(i)      (REG_SPI_BASE(i) + 0x9c)

/* SPI_BUF1 : R/W; bitpos: [31:0]; default: 0;
 * data buffer
 */

#define SPI_BUF1    0xFFFFFFFF
#define SPI_BUF1_M  (SPI_BUF1_V << SPI_BUF1_S)
#define SPI_BUF1_V  0xFFFFFFFF
#define SPI_BUF1_S  0

/* SPI_W2_REG register */

#define SPI_W2_REG(i)      (REG_SPI_BASE(i) + 0xa0)

/* SPI_BUF2 : R/W; bitpos: [31:0]; default: 0;
 * data buffer
 */

#define SPI_BUF2    0xFFFFFFFF
#define SPI_BUF2_M  (SPI_BUF2_V << SPI_BUF2_S)
#define SPI_BUF2_V  0xFFFFFFFF
#define SPI_BUF2_S  0

/* SPI_W3_REG register */

#define SPI_W3_REG(i)      (REG_SPI_BASE(i) + 0xa4)

/* SPI_BUF3 : R/W; bitpos: [31:0]; default: 0;
 * data buffer
 */

#define SPI_BUF3    0xFFFFFFFF
#define SPI_BUF3_M  (SPI_BUF3_V << SPI_BUF3_S)
#define SPI_BUF3_V  0xFFFFFFFF
#define SPI_BUF3_S  0

/* SPI_W4_REG register */

#define SPI_W4_REG(i)      (REG_SPI_BASE(i) + 0xa8)

/* SPI_BUF4 : R/W; bitpos: [31:0]; default: 0;
 * data buffer
 */

#define SPI_BUF4    0xFFFFFFFF
#define SPI_BUF4_M  (SPI_BUF4_V << SPI_BUF4_S)
#define SPI_BUF4_V  0xFFFFFFFF
#define SPI_BUF4_S  0

/* SPI_W5_REG register */

#define SPI_W5_REG(i)      (REG_SPI_BASE(i) + 0xac)

/* SPI_BUF5 : R/W; bitpos: [31:0]; default: 0;
 * data buffer
 */

#define SPI_BUF5    0xFFFFFFFF
#define SPI_BUF5_M  (SPI_BUF5_V << SPI_BUF5_S)
#define SPI_BUF5_V  0xFFFFFFFF
#define SPI_BUF5_S  0

/* SPI_W6_REG register */

#define SPI_W6_REG(i)      (REG_SPI_BASE(i) + 0xb0)

/* SPI_BUF6 : R/W; bitpos: [31:0]; default: 0;
 * data buffer
 */

#define SPI_BUF6    0xFFFFFFFF
#define SPI_BUF6_M  (SPI_BUF6_V << SPI_BUF6_S)
#define SPI_BUF6_V  0xFFFFFFFF
#define SPI_BUF6_S  0

/* SPI_W7_REG register */

#define SPI_W7_REG(i)      (REG_SPI_BASE(i) + 0xb4)

/* SPI_BUF7 : R/W; bitpos: [31:0]; default: 0;
 * data buffer
 */

#define SPI_BUF7    0xFFFFFFFF
#define SPI_BUF7_M  (SPI_BUF7_V << SPI_BUF7_S)
#define SPI_BUF7_V  0xFFFFFFFF
#define SPI_BUF7_S  0

/* SPI_W8_REG register */

#define SPI_W8_REG(i)      (REG_SPI_BASE(i) + 0xb8)

/* SPI_BUF8 : R/W; bitpos: [31:0]; default: 0;
 * data buffer
 */

#define SPI_BUF8    0xFFFFFFFF
#define SPI_BUF8_M  (SPI_BUF8_V << SPI_BUF8_S)
#define SPI_BUF8_V  0xFFFFFFFF
#define SPI_BUF8_S  0

/* SPI_W9_REG register */

#define SPI_W9_REG(i)      (REG_SPI_BASE(i) + 0xbc)

/* SPI_BUF9 : R/W; bitpos: [31:0]; default: 0;
 * data buffer
 */

#define SPI_BUF9    0xFFFFFFFF
#define SPI_BUF9_M  (SPI_BUF9_V << SPI_BUF9_S)
#define SPI_BUF9_V  0xFFFFFFFF
#define SPI_BUF9_S  0

/* SPI_W10_REG register */

#define SPI_W10_REG(i)      (REG_SPI_BASE(i) + 0xc0)

/* SPI_BUF10 : R/W; bitpos: [31:0]; default: 0;
 * data buffer
 */

#define SPI_BUF10    0xFFFFFFFF
#define SPI_BUF10_M  (SPI_BUF10_V << SPI_BUF10_S)
#define SPI_BUF10_V  0xFFFFFFFF
#define SPI_BUF10_S  0

/* SPI_W11_REG register */

#define SPI_W11_REG(i)      (REG_SPI_BASE(i) + 0xc4)

/* SPI_BUF11 : R/W; bitpos: [31:0]; default: 0;
 * data buffer
 */

#define SPI_BUF11    0xFFFFFFFF
#define SPI_BUF11_M  (SPI_BUF11_V << SPI_BUF11_S)
#define SPI_BUF11_V  0xFFFFFFFF
#define SPI_BUF11_S  0

/* SPI_W12_REG register */

#define SPI_W12_REG(i)      (REG_SPI_BASE(i) + 0xc8)

/* SPI_BUF12 : R/W; bitpos: [31:0]; default: 0;
 * data buffer
 */

#define SPI_BUF12    0xFFFFFFFF
#define SPI_BUF12_M  (SPI_BUF12_V << SPI_BUF12_S)
#define SPI_BUF12_V  0xFFFFFFFF
#define SPI_BUF12_S  0

/* SPI_W13_REG register */

#define SPI_W13_REG(i)      (REG_SPI_BASE(i) + 0xcc)

/* SPI_BUF13 : R/W; bitpos: [31:0]; default: 0;
 * data buffer
 */

#define SPI_BUF13    0xFFFFFFFF
#define SPI_BUF13_M  (SPI_BUF13_V << SPI_BUF13_S)
#define SPI_BUF13_V  0xFFFFFFFF
#define SPI_BUF13_S  0

/* SPI_W14_REG register */

#define SPI_W14_REG(i)      (REG_SPI_BASE(i) + 0xd0)

/* SPI_BUF14 : R/W; bitpos: [31:0]; default: 0;
 * data buffer
 */

#define SPI_BUF14    0xFFFFFFFF
#define SPI_BUF14_M  (SPI_BUF14_V << SPI_BUF14_S)
#define SPI_BUF14_V  0xFFFFFFFF
#define SPI_BUF14_S  0

/* SPI_W15_REG register */

#define SPI_W15_REG(i)      (REG_SPI_BASE(i) + 0xd4)

/* SPI_BUF15 : R/W; bitpos: [31:0]; default: 0;
 * data buffer
 */

#define SPI_BUF15    0xFFFFFFFF
#define SPI_BUF15_M  (SPI_BUF15_V << SPI_BUF15_S)
#define SPI_BUF15_V  0xFFFFFFFF
#define SPI_BUF15_S  0

/* SPI_W16_REG register */

#define SPI_W16_REG(i)      (REG_SPI_BASE(i) + 0xd8)

/* SPI_BUF16 : R/W; bitpos: [31:0]; default: 0;
 * data buffer
 */

#define SPI_BUF16    0xFFFFFFFF
#define SPI_BUF16_M  (SPI_BUF16_V << SPI_BUF16_S)
#define SPI_BUF16_V  0xFFFFFFFF
#define SPI_BUF16_S  0

/* SPI_W17_REG register */

#define SPI_W17_REG(i)      (REG_SPI_BASE(i) + 0xdc)

/* SPI_BUF17 : R/W; bitpos: [31:0]; default: 0;
 * data buffer
 */

#define SPI_BUF17    0xFFFFFFFF
#define SPI_BUF17_M  (SPI_BUF17_V << SPI_BUF17_S)
#define SPI_BUF17_V  0xFFFFFFFF
#define SPI_BUF17_S  0

/* SPI_DIN_MODE_REG register */

#define SPI_DIN_MODE_REG(i)          (REG_SPI_BASE(i) + 0xe0)

/* SPI_TIMING_CLK_ENA : R/W; bitpos: [24]; default: 0;
 * 1:enable hclk in spi_timing.v.  0: disable it.
 */

#define SPI_TIMING_CLK_ENA    (BIT(24))
#define SPI_TIMING_CLK_ENA_M  (SPI_TIMING_CLK_ENA_V << SPI_TIMING_CLK_ENA_S)
#define SPI_TIMING_CLK_ENA_V  0x00000001
#define SPI_TIMING_CLK_ENA_S  24

/* SPI_DIN7_MODE : R/W; bitpos: [23:21]; default: 0;
 * Configure the input signal delay mode. 0: without delayed, 1: with the
 * posedge of clk_apb, 2: with the negedge of clk_apb, 3: with the posedge
 * of hclk, whose frequency is the double of the clk_apb frequency, 4: with
 * the negedge of hclk,  5: with the posedge of spi_clk, 6: with the negedge
 * of spi_clk.
 */

#define SPI_DIN7_MODE    0x00000007
#define SPI_DIN7_MODE_M  (SPI_DIN7_MODE_V << SPI_DIN7_MODE_S)
#define SPI_DIN7_MODE_V  0x00000007
#define SPI_DIN7_MODE_S  21

/* SPI_DIN6_MODE : R/W; bitpos: [20:18]; default: 0;
 * Configure the input signal delay mode. 0: without delayed, 1: with the
 * posedge of clk_apb, 2: with the negedge of clk_apb, 3: with the posedge
 * of hclk, whose frequency is the double of the clk_apb frequency, 4: with
 * the negedge of hclk,  5: with the posedge of spi_clk, 6: with the negedge
 * of spi_clk.
 */

#define SPI_DIN6_MODE    0x00000007
#define SPI_DIN6_MODE_M  (SPI_DIN6_MODE_V << SPI_DIN6_MODE_S)
#define SPI_DIN6_MODE_V  0x00000007
#define SPI_DIN6_MODE_S  18

/* SPI_DIN5_MODE : R/W; bitpos: [17:15]; default: 0;
 * Configure the input signal delay mode. 0: without delayed, 1: with the
 * posedge of clk_apb, 2: with the negedge of clk_apb, 3: with the posedge
 * of hclk, whose frequency is the double of the clk_apb frequency, 4: with
 * the negedge of hclk,  5: with the posedge of spi_clk, 6: with the negedge
 * of spi_clk.
 */

#define SPI_DIN5_MODE    0x00000007
#define SPI_DIN5_MODE_M  (SPI_DIN5_MODE_V << SPI_DIN5_MODE_S)
#define SPI_DIN5_MODE_V  0x00000007
#define SPI_DIN5_MODE_S  15

/* SPI_DIN4_MODE : R/W; bitpos: [14:12]; default: 0;
 * Configure the input signal delay mode. 0: without delayed, 1: with the
 * posedge of clk_apb, 2: with the negedge of clk_apb, 3: with the posedge
 * of hclk, whose frequency is the double of the clk_apb frequency, 4: with
 * the negedge of hclk,  5: with the posedge of spi_clk, 6: with the negedge
 * of spi_clk.
 */

#define SPI_DIN4_MODE    0x00000007
#define SPI_DIN4_MODE_M  (SPI_DIN4_MODE_V << SPI_DIN4_MODE_S)
#define SPI_DIN4_MODE_V  0x00000007
#define SPI_DIN4_MODE_S  12

/* SPI_DIN3_MODE : R/W; bitpos: [11:9]; default: 0;
 * Configure the input signal delay mode. 0: without delayed, 1: with the
 * posedge of clk_apb, 2: with the negedge of clk_apb, 3: with the posedge
 * of hclk, whose frequency is the double of the clk_apb frequency, 4: with
 * the negedge of hclk,  5: with the posedge of spi_clk, 6: with the negedge
 * of spi_clk.
 */

#define SPI_DIN3_MODE    0x00000007
#define SPI_DIN3_MODE_M  (SPI_DIN3_MODE_V << SPI_DIN3_MODE_S)
#define SPI_DIN3_MODE_V  0x00000007
#define SPI_DIN3_MODE_S  9

/* SPI_DIN2_MODE : R/W; bitpos: [8:6]; default: 0;
 * Configure the input signal delay mode. 0: without delayed, 1: with the
 * posedge of clk_apb, 2: with the negedge of clk_apb, 3: with the posedge
 * of hclk, whose frequency is the double of the clk_apb frequency, 4: with
 * the negedge of hclk,  5: with the posedge of spi_clk, 6: with the negedge
 * of spi_clk.
 */

#define SPI_DIN2_MODE    0x00000007
#define SPI_DIN2_MODE_M  (SPI_DIN2_MODE_V << SPI_DIN2_MODE_S)
#define SPI_DIN2_MODE_V  0x00000007
#define SPI_DIN2_MODE_S  6

/* SPI_DIN1_MODE : R/W; bitpos: [5:3]; default: 0;
 * Configure the input signal delay mode. 0: without delayed, 1: with the
 * posedge of clk_apb, 2: with the negedge of clk_apb, 3: with the posedge
 * of hclk, whose frequency is the double of the clk_apb frequency, 4: with
 * the negedge of hclk,  5: with the posedge of spi_clk, 6: with the negedge
 * of spi_clk.
 */

#define SPI_DIN1_MODE    0x00000007
#define SPI_DIN1_MODE_M  (SPI_DIN1_MODE_V << SPI_DIN1_MODE_S)
#define SPI_DIN1_MODE_V  0x00000007
#define SPI_DIN1_MODE_S  3

/* SPI_DIN0_MODE : R/W; bitpos: [2:0]; default: 0;
 * Configure the input signal delay mode. 0: without delayed, 1: with the
 * posedge of clk_apb, 2: with the negedge of clk_apb, 3: with the posedge
 * of hclk, whose frequency is the double of the clk_apb frequency, 4: with
 * the negedge of hclk,  5: with the posedge of spi_clk, 6: with the negedge
 * of spi_clk.
 */

#define SPI_DIN0_MODE    0x00000007
#define SPI_DIN0_MODE_M  (SPI_DIN0_MODE_V << SPI_DIN0_MODE_S)
#define SPI_DIN0_MODE_V  0x00000007
#define SPI_DIN0_MODE_S  0

/* SPI_DIN_NUM_REG register */

#define SPI_DIN_NUM_REG(i)     (REG_SPI_BASE(i) + 0xe4)

/* SPI_DIN7_NUM : R/W; bitpos: [15:14]; default: 0;
 * the input signals are delayed by system clock cycles, 0: delayed by 1
 * cycle, 1: delayed by 2 cycles,...
 */

#define SPI_DIN7_NUM    0x00000003
#define SPI_DIN7_NUM_M  (SPI_DIN7_NUM_V << SPI_DIN7_NUM_S)
#define SPI_DIN7_NUM_V  0x00000003
#define SPI_DIN7_NUM_S  14

/* SPI_DIN6_NUM : R/W; bitpos: [13:12]; default: 0;
 * the input signals are delayed by system clock cycles, 0: delayed by 1
 * cycle, 1: delayed by 2 cycles,...
 */

#define SPI_DIN6_NUM    0x00000003
#define SPI_DIN6_NUM_M  (SPI_DIN6_NUM_V << SPI_DIN6_NUM_S)
#define SPI_DIN6_NUM_V  0x00000003
#define SPI_DIN6_NUM_S  12

/* SPI_DIN5_NUM : R/W; bitpos: [11:10]; default: 0;
 * the input signals are delayed by system clock cycles, 0: delayed by 1
 * cycle, 1: delayed by 2 cycles,...
 */

#define SPI_DIN5_NUM    0x00000003
#define SPI_DIN5_NUM_M  (SPI_DIN5_NUM_V << SPI_DIN5_NUM_S)
#define SPI_DIN5_NUM_V  0x00000003
#define SPI_DIN5_NUM_S  10

/* SPI_DIN4_NUM : R/W; bitpos: [9:8]; default: 0;
 * the input signals are delayed by system clock cycles, 0: delayed by 1
 * cycle, 1: delayed by 2 cycles,...
 */

#define SPI_DIN4_NUM    0x00000003
#define SPI_DIN4_NUM_M  (SPI_DIN4_NUM_V << SPI_DIN4_NUM_S)
#define SPI_DIN4_NUM_V  0x00000003
#define SPI_DIN4_NUM_S  8

/* SPI_DIN3_NUM : R/W; bitpos: [7:6]; default: 0;
 * the input signals are delayed by system clock cycles, 0: delayed by 1
 * cycle, 1: delayed by 2 cycles,...
 */

#define SPI_DIN3_NUM    0x00000003
#define SPI_DIN3_NUM_M  (SPI_DIN3_NUM_V << SPI_DIN3_NUM_S)
#define SPI_DIN3_NUM_V  0x00000003
#define SPI_DIN3_NUM_S  6

/* SPI_DIN2_NUM : R/W; bitpos: [5:4]; default: 0;
 * the input signals are delayed by system clock cycles, 0: delayed by 1
 * cycle, 1: delayed by 2 cycles,...
 */

#define SPI_DIN2_NUM    0x00000003
#define SPI_DIN2_NUM_M  (SPI_DIN2_NUM_V << SPI_DIN2_NUM_S)
#define SPI_DIN2_NUM_V  0x00000003
#define SPI_DIN2_NUM_S  4

/* SPI_DIN1_NUM : R/W; bitpos: [3:2]; default: 0;
 * the input signals are delayed by system clock cycles, 0: delayed by 1
 * cycle, 1: delayed by 2 cycles,...
 */

#define SPI_DIN1_NUM    0x00000003
#define SPI_DIN1_NUM_M  (SPI_DIN1_NUM_V << SPI_DIN1_NUM_S)
#define SPI_DIN1_NUM_V  0x00000003
#define SPI_DIN1_NUM_S  2

/* SPI_DIN0_NUM : R/W; bitpos: [1:0]; default: 0;
 * the input signals are delayed by system clock cycles, 0: delayed by 1
 * cycle, 1: delayed by 2 cycles,...
 */

#define SPI_DIN0_NUM    0x00000003
#define SPI_DIN0_NUM_M  (SPI_DIN0_NUM_V << SPI_DIN0_NUM_S)
#define SPI_DIN0_NUM_V  0x00000003
#define SPI_DIN0_NUM_S  0

/* SPI_DOUT_MODE_REG register */

#define SPI_DOUT_MODE_REG(i)     (REG_SPI_BASE(i) + 0xe8)

/* SPI_DOUT7_MODE : R/W; bitpos: [23:21]; default: 0;
 * Configure the output signal delay mode. 0: without delayed, 1: with the
 * posedge of clk_apb, 2: with the negedge of clk_apb, 3: with the posedge
 * of hclk, whose frequency is the double of the clk_apb frequency, 4: with
 * the negedge of hclk,  5: with the posedge of spi_clk, 6: with the negedge
 * of spi_clk.
 */

#define SPI_DOUT7_MODE    0x00000007
#define SPI_DOUT7_MODE_M  (SPI_DOUT7_MODE_V << SPI_DOUT7_MODE_S)
#define SPI_DOUT7_MODE_V  0x00000007
#define SPI_DOUT7_MODE_S  21

/* SPI_DOUT6_MODE : R/W; bitpos: [20:18]; default: 0;
 * Configure the output signal delay mode. 0: without delayed, 1: with the
 * posedge of clk_apb, 2: with the negedge of clk_apb, 3: with the posedge
 * of hclk, whose frequency is the double of the clk_apb frequency, 4: with
 * the negedge of hclk,  5: with the posedge of spi_clk, 6: with the negedge
 * of spi_clk.
 */

#define SPI_DOUT6_MODE    0x00000007
#define SPI_DOUT6_MODE_M  (SPI_DOUT6_MODE_V << SPI_DOUT6_MODE_S)
#define SPI_DOUT6_MODE_V  0x00000007
#define SPI_DOUT6_MODE_S  18

/* SPI_DOUT5_MODE : R/W; bitpos: [17:15]; default: 0;
 * Configure the output signal delay mode. 0: without delayed, 1: with the
 * posedge of clk_apb, 2: with the negedge of clk_apb, 3: with the posedge
 * of hclk, whose frequency is the double of the clk_apb frequency, 4: with
 * the negedge of hclk,  5: with the posedge of spi_clk, 6: with the negedge
 * of spi_clk.
 */

#define SPI_DOUT5_MODE    0x00000007
#define SPI_DOUT5_MODE_M  (SPI_DOUT5_MODE_V << SPI_DOUT5_MODE_S)
#define SPI_DOUT5_MODE_V  0x00000007
#define SPI_DOUT5_MODE_S  15

/* SPI_DOUT4_MODE : R/W; bitpos: [14:12]; default: 0;
 * Configure the output signal delay mode. 0: without delayed, 1: with the
 * posedge of clk_apb, 2: with the negedge of clk_apb, 3: with the posedge
 * of hclk, whose frequency is the double of the clk_apb frequency, 4: with
 * the negedge of hclk,  5: with the posedge of spi_clk, 6: with the negedge
 * of spi_clk.
 */

#define SPI_DOUT4_MODE    0x00000007
#define SPI_DOUT4_MODE_M  (SPI_DOUT4_MODE_V << SPI_DOUT4_MODE_S)
#define SPI_DOUT4_MODE_V  0x00000007
#define SPI_DOUT4_MODE_S  12

/* SPI_DOUT3_MODE : R/W; bitpos: [11:9]; default: 0;
 * Configure the output signal delay mode. 0: without delayed, 1: with the
 * posedge of clk_apb, 2: with the negedge of clk_apb, 3: with the posedge
 * of hclk, whose frequency is the double of the clk_apb frequency, 4: with
 * the negedge of hclk,  5: with the posedge of spi_clk, 6: with the negedge
 * of spi_clk.
 */

#define SPI_DOUT3_MODE    0x00000007
#define SPI_DOUT3_MODE_M  (SPI_DOUT3_MODE_V << SPI_DOUT3_MODE_S)
#define SPI_DOUT3_MODE_V  0x00000007
#define SPI_DOUT3_MODE_S  9

/* SPI_DOUT2_MODE : R/W; bitpos: [8:6]; default: 0;
 * Configure the output signal delay mode. 0: without delayed, 1: with the
 * posedge of clk_apb, 2: with the negedge of clk_apb, 3: with the posedge
 * of hclk, whose frequency is the double of the clk_apb frequency, 4: with
 * the negedge of hclk,  5: with the posedge of spi_clk, 6: with the negedge
 * of spi_clk.
 */

#define SPI_DOUT2_MODE    0x00000007
#define SPI_DOUT2_MODE_M  (SPI_DOUT2_MODE_V << SPI_DOUT2_MODE_S)
#define SPI_DOUT2_MODE_V  0x00000007
#define SPI_DOUT2_MODE_S  6

/* SPI_DOUT1_MODE : R/W; bitpos: [5:3]; default: 0;
 * Configure the output signal delay mode. 0: without delayed, 1: with the
 * posedge of clk_apb, 2: with the negedge of clk_apb, 3: with the posedge
 * of hclk, whose frequency is the double of the clk_apb frequency, 4: with
 * the negedge of hclk,  5: with the posedge of spi_clk, 6: with the negedge
 * of spi_clk.
 */

#define SPI_DOUT1_MODE    0x00000007
#define SPI_DOUT1_MODE_M  (SPI_DOUT1_MODE_V << SPI_DOUT1_MODE_S)
#define SPI_DOUT1_MODE_V  0x00000007
#define SPI_DOUT1_MODE_S  3

/* SPI_DOUT0_MODE : R/W; bitpos: [2:0]; default: 0;
 * Configure the output signal delay mode. 0: without delayed, 1: with the
 * posedge of clk_apb, 2: with the negedge of clk_apb, 3: with the posedge
 * of hclk, whose frequency is the double of the clk_apb frequency, 4: with
 * the negedge of hclk,  5: with the posedge of spi_clk, 6: with the negedge
 * of spi_clk.
 */

#define SPI_DOUT0_MODE    0x00000007
#define SPI_DOUT0_MODE_M  (SPI_DOUT0_MODE_V << SPI_DOUT0_MODE_S)
#define SPI_DOUT0_MODE_V  0x00000007
#define SPI_DOUT0_MODE_S  0

/* SPI_DOUT_NUM_REG register */

#define SPI_DOUT_NUM_REG(i)     (REG_SPI_BASE(i) + 0xec)

/* SPI_DOUT7_NUM : R/W; bitpos: [15:14]; default: 0;
 * the output signals are delayed by system clock cycles, 0: delayed by 1
 * cycle, 1: delayed by 2 cycles,...
 */

#define SPI_DOUT7_NUM    0x00000003
#define SPI_DOUT7_NUM_M  (SPI_DOUT7_NUM_V << SPI_DOUT7_NUM_S)
#define SPI_DOUT7_NUM_V  0x00000003
#define SPI_DOUT7_NUM_S  14

/* SPI_DOUT6_NUM : R/W; bitpos: [13:12]; default: 0;
 * the output signals are delayed by system clock cycles, 0: delayed by 1
 * cycle, 1: delayed by 2 cycles,...
 */

#define SPI_DOUT6_NUM    0x00000003
#define SPI_DOUT6_NUM_M  (SPI_DOUT6_NUM_V << SPI_DOUT6_NUM_S)
#define SPI_DOUT6_NUM_V  0x00000003
#define SPI_DOUT6_NUM_S  12

/* SPI_DOUT5_NUM : R/W; bitpos: [11:10]; default: 0;
 * the output signals are delayed by system clock cycles, 0: delayed by 1
 * cycle, 1: delayed by 2 cycles,...
 */

#define SPI_DOUT5_NUM    0x00000003
#define SPI_DOUT5_NUM_M  (SPI_DOUT5_NUM_V << SPI_DOUT5_NUM_S)
#define SPI_DOUT5_NUM_V  0x00000003
#define SPI_DOUT5_NUM_S  10

/* SPI_DOUT4_NUM : R/W; bitpos: [9:8]; default: 0;
 * the output signals are delayed by system clock cycles, 0: delayed by 1
 * cycle, 1: delayed by 2 cycles,...
 */

#define SPI_DOUT4_NUM    0x00000003
#define SPI_DOUT4_NUM_M  (SPI_DOUT4_NUM_V << SPI_DOUT4_NUM_S)
#define SPI_DOUT4_NUM_V  0x00000003
#define SPI_DOUT4_NUM_S  8

/* SPI_DOUT3_NUM : R/W; bitpos: [7:6]; default: 0;
 * the output signals are delayed by system clock cycles, 0: delayed by 1
 * cycle, 1: delayed by 2 cycles,...
 */

#define SPI_DOUT3_NUM    0x00000003
#define SPI_DOUT3_NUM_M  (SPI_DOUT3_NUM_V << SPI_DOUT3_NUM_S)
#define SPI_DOUT3_NUM_V  0x00000003
#define SPI_DOUT3_NUM_S  6

/* SPI_DOUT2_NUM : R/W; bitpos: [5:4]; default: 0;
 * the output signals are delayed by system clock cycles, 0: delayed by 1
 * cycle, 1: delayed by 2 cycles,...
 */

#define SPI_DOUT2_NUM    0x00000003
#define SPI_DOUT2_NUM_M  (SPI_DOUT2_NUM_V << SPI_DOUT2_NUM_S)
#define SPI_DOUT2_NUM_V  0x00000003
#define SPI_DOUT2_NUM_S  4

/* SPI_DOUT1_NUM : R/W; bitpos: [3:2]; default: 0;
 * the output signals are delayed by system clock cycles, 0: delayed by 1
 * cycle, 1: delayed by 2 cycles,...
 */

#define SPI_DOUT1_NUM    0x00000003
#define SPI_DOUT1_NUM_M  (SPI_DOUT1_NUM_V << SPI_DOUT1_NUM_S)
#define SPI_DOUT1_NUM_V  0x00000003
#define SPI_DOUT1_NUM_S  2

/* SPI_DOUT0_NUM : R/W; bitpos: [1:0]; default: 0;
 * the output signals are delayed by system clock cycles, 0: delayed by 1
 * cycle, 1: delayed by 2 cycles,...
 */

#define SPI_DOUT0_NUM    0x00000003
#define SPI_DOUT0_NUM_M  (SPI_DOUT0_NUM_V << SPI_DOUT0_NUM_S)
#define SPI_DOUT0_NUM_V  0x00000003
#define SPI_DOUT0_NUM_S  0

/* SPI_LCD_CTRL_REG register */

#define SPI_LCD_CTRL_REG(i)     (REG_SPI_BASE(i) + 0xf0)

/* SPI_LCD_SRGB_MODE_EN : R/W; bitpos: [31]; default: 0;
 * 1: Enable LCD mode output vsync, hsync, de. 0: Disable.
 */

#define SPI_LCD_SRGB_MODE_EN    (BIT(31))
#define SPI_LCD_SRGB_MODE_EN_M  (SPI_LCD_SRGB_MODE_EN_V << SPI_LCD_SRGB_MODE_EN_S)
#define SPI_LCD_SRGB_MODE_EN_V  0x00000001
#define SPI_LCD_SRGB_MODE_EN_S  31

/* SPI_LCD_VT_HEIGHT : R/W; bitpos: [30:21]; default: 0;
 * It is the vertical total height of a frame.
 */

#define SPI_LCD_VT_HEIGHT    0x000003FF
#define SPI_LCD_VT_HEIGHT_M  (SPI_LCD_VT_HEIGHT_V << SPI_LCD_VT_HEIGHT_S)
#define SPI_LCD_VT_HEIGHT_V  0x000003FF
#define SPI_LCD_VT_HEIGHT_S  21

/* SPI_LCD_VA_HEIGHT : R/W; bitpos: [20:11]; default: 0;
 * It is the vertical active height of a frame.
 */

#define SPI_LCD_VA_HEIGHT    0x000003FF
#define SPI_LCD_VA_HEIGHT_M  (SPI_LCD_VA_HEIGHT_V << SPI_LCD_VA_HEIGHT_S)
#define SPI_LCD_VA_HEIGHT_V  0x000003FF
#define SPI_LCD_VA_HEIGHT_S  11

/* SPI_LCD_HB_FRONT : R/W; bitpos: [10:0]; default: 0;
 * It is the horizontal blank front porch of a frame.
 */

#define SPI_LCD_HB_FRONT    0x000007FF
#define SPI_LCD_HB_FRONT_M  (SPI_LCD_HB_FRONT_V << SPI_LCD_HB_FRONT_S)
#define SPI_LCD_HB_FRONT_V  0x000007FF
#define SPI_LCD_HB_FRONT_S  0

/* SPI_LCD_CTRL1_REG register */

#define SPI_LCD_CTRL1_REG(i)       (REG_SPI_BASE(i) + 0xf4)

/* SPI_LCD_HT_WIDTH : R/W; bitpos: [31:20]; default: 0;
 * It is the horizontal total width of a frame.
 */

#define SPI_LCD_HT_WIDTH    0x00000FFF
#define SPI_LCD_HT_WIDTH_M  (SPI_LCD_HT_WIDTH_V << SPI_LCD_HT_WIDTH_S)
#define SPI_LCD_HT_WIDTH_V  0x00000FFF
#define SPI_LCD_HT_WIDTH_S  20

/* SPI_LCD_HA_WIDTH : R/W; bitpos: [19:8]; default: 0;
 * It is the horizontal active width of a frame.
 */

#define SPI_LCD_HA_WIDTH    0x00000FFF
#define SPI_LCD_HA_WIDTH_M  (SPI_LCD_HA_WIDTH_V << SPI_LCD_HA_WIDTH_S)
#define SPI_LCD_HA_WIDTH_V  0x00000FFF
#define SPI_LCD_HA_WIDTH_S  8

/* SPI_LCD_VB_FRONT : R/W; bitpos: [7:0]; default: 0;
 * It is the vertical blank front porch of a frame.
 */

#define SPI_LCD_VB_FRONT    0x000000FF
#define SPI_LCD_VB_FRONT_M  (SPI_LCD_VB_FRONT_V << SPI_LCD_VB_FRONT_S)
#define SPI_LCD_VB_FRONT_V  0x000000FF
#define SPI_LCD_VB_FRONT_S  0

/* SPI_LCD_CTRL2_REG register */

#define SPI_LCD_CTRL2_REG(i)      (REG_SPI_BASE(i) + 0xf8)

/* SPI_LCD_HSYNC_POSITION : R/W; bitpos: [31:24]; default: 0;
 * It is the position of spi_hsync_out active pulse in a line.
 */

#define SPI_LCD_HSYNC_POSITION    0x000000FF
#define SPI_LCD_HSYNC_POSITION_M  (SPI_LCD_HSYNC_POSITION_V << SPI_LCD_HSYNC_POSITION_S)
#define SPI_LCD_HSYNC_POSITION_V  0x000000FF
#define SPI_LCD_HSYNC_POSITION_S  24

/* SPI_HSYNC_IDLE_POL : R/W; bitpos: [23]; default: 0;
 * It is the idle value of spi_hsync_out.
 */

#define SPI_HSYNC_IDLE_POL    (BIT(23))
#define SPI_HSYNC_IDLE_POL_M  (SPI_HSYNC_IDLE_POL_V << SPI_HSYNC_IDLE_POL_S)
#define SPI_HSYNC_IDLE_POL_V  0x00000001
#define SPI_HSYNC_IDLE_POL_S  23

/* SPI_LCD_HSYNC_WIDTH : R/W; bitpos: [22:16]; default: 0;
 * It is the position of spi_hsync_out active pulse in a line.
 */

#define SPI_LCD_HSYNC_WIDTH    0x0000007F
#define SPI_LCD_HSYNC_WIDTH_M  (SPI_LCD_HSYNC_WIDTH_V << SPI_LCD_HSYNC_WIDTH_S)
#define SPI_LCD_HSYNC_WIDTH_V  0x0000007F
#define SPI_LCD_HSYNC_WIDTH_S  16

/* SPI_LCD_VSYNC_POSITION : R/W; bitpos: [15:8]; default: 0;
 * It is the position of spi_vsync_out active pulse in a line.
 */

#define SPI_LCD_VSYNC_POSITION    0x000000FF
#define SPI_LCD_VSYNC_POSITION_M  (SPI_LCD_VSYNC_POSITION_V << SPI_LCD_VSYNC_POSITION_S)
#define SPI_LCD_VSYNC_POSITION_V  0x000000FF
#define SPI_LCD_VSYNC_POSITION_S  8

/* SPI_VSYNC_IDLE_POL : R/W; bitpos: [7]; default: 0;
 * It is the idle value of spi_vsync_out.
 */

#define SPI_VSYNC_IDLE_POL    (BIT(7))
#define SPI_VSYNC_IDLE_POL_M  (SPI_VSYNC_IDLE_POL_V << SPI_VSYNC_IDLE_POL_S)
#define SPI_VSYNC_IDLE_POL_V  0x00000001
#define SPI_VSYNC_IDLE_POL_S  7

/* SPI_LCD_VSYNC_WIDTH : R/W; bitpos: [6:0]; default: 0;
 * It is the position of spi_vsync_out active pulse in a line.
 */

#define SPI_LCD_VSYNC_WIDTH    0x0000007F
#define SPI_LCD_VSYNC_WIDTH_M  (SPI_LCD_VSYNC_WIDTH_V << SPI_LCD_VSYNC_WIDTH_S)
#define SPI_LCD_VSYNC_WIDTH_V  0x0000007F
#define SPI_LCD_VSYNC_WIDTH_S  0

/* SPI_LCD_D_MODE_REG register */

#define SPI_LCD_D_MODE_REG(i)      (REG_SPI_BASE(i) + 0xfc)

/* SPI_D_VSYNC_MODE : R/W; bitpos: [14:12]; default: 0;
 * Configure the output  spi_vsync delay mode. 0: without delayed, 1: with
 * the posedge of clk_apb, 2: with the negedge of clk_apb, 3: with the
 * posedge of hclk, whose frequency is the double of the clk_apb frequency,
 * 4: with the negedge of hclk,  5: with the posedge of spi_clk, 6: with the
 * negedge of spi_clk.
 */

#define SPI_D_VSYNC_MODE    0x00000007
#define SPI_D_VSYNC_MODE_M  (SPI_D_VSYNC_MODE_V << SPI_D_VSYNC_MODE_S)
#define SPI_D_VSYNC_MODE_V  0x00000007
#define SPI_D_VSYNC_MODE_S  12

/* SPI_D_HSYNC_MODE : R/W; bitpos: [11:9]; default: 0;
 * Configure the output  spi_hsync delay mode. 0: without delayed, 1: with
 * the posedge of clk_apb, 2: with the negedge of clk_apb, 3: with the
 * posedge of hclk, whose frequency is the double of the clk_apb frequency,
 * 4: with the negedge of hclk,  5: with the posedge of spi_clk, 6: with the
 * negedge of spi_clk.
 */

#define SPI_D_HSYNC_MODE    0x00000007
#define SPI_D_HSYNC_MODE_M  (SPI_D_HSYNC_MODE_V << SPI_D_HSYNC_MODE_S)
#define SPI_D_HSYNC_MODE_V  0x00000007
#define SPI_D_HSYNC_MODE_S  9

/* SPI_D_DE_MODE : R/W; bitpos: [8:6]; default: 0;
 * Configure the output  spi_de delay mode. 0: without delayed, 1: with the
 * posedge of clk_apb, 2: with the negedge of clk_apb, 3: with the posedge
 * of hclk, whose frequency is the double of the clk_apb frequency, 4: with
 * the negedge of hclk,  5: with the posedge of spi_clk, 6: with the negedge
 * of spi_clk.
 */

#define SPI_D_DE_MODE    0x00000007
#define SPI_D_DE_MODE_M  (SPI_D_DE_MODE_V << SPI_D_DE_MODE_S)
#define SPI_D_DE_MODE_V  0x00000007
#define SPI_D_DE_MODE_S  6

/* SPI_D_CD_MODE : R/W; bitpos: [5:3]; default: 0;
 * Configure the output  spi_cd delay mode. 0: without delayed, 1: with the
 * posedge of clk_apb, 2: with the negedge of clk_apb, 3: with the posedge
 * of hclk, whose frequency is the double of the clk_apb frequency, 4: with
 * the negedge of hclk,  5: with the posedge of spi_clk, 6: with the negedge
 * of spi_clk.
 */

#define SPI_D_CD_MODE    0x00000007
#define SPI_D_CD_MODE_M  (SPI_D_CD_MODE_V << SPI_D_CD_MODE_S)
#define SPI_D_CD_MODE_V  0x00000007
#define SPI_D_CD_MODE_S  3

/* SPI_D_DQS_MODE : R/W; bitpos: [2:0]; default: 0;
 * Configure the output  spi_dqs delay mode. 0: without delayed, 1: with the
 * posedge of clk_apb, 2: with the negedge of clk_apb, 3: with the posedge
 * of hclk, whose frequency is the double of the clk_apb frequency, 4: with
 * the negedge of hclk,  5: with the posedge of spi_clk, 6: with the negedge
 * of spi_clk.
 */

#define SPI_D_DQS_MODE    0x00000007
#define SPI_D_DQS_MODE_M  (SPI_D_DQS_MODE_V << SPI_D_DQS_MODE_S)
#define SPI_D_DQS_MODE_V  0x00000007
#define SPI_D_DQS_MODE_S  0

/* SPI_LCD_D_NUM_REG register */

#define SPI_LCD_D_NUM_REG(i)      (REG_SPI_BASE(i) + 0x100)

/* SPI_D_VSYNC_NUM : R/W; bitpos: [9:8]; default: 0;
 * the output spi_vsync is delayed by system clock cycles, 0: delayed by 1
 * cycle, 1: delayed by 2 cycles,...
 */

#define SPI_D_VSYNC_NUM    0x00000003
#define SPI_D_VSYNC_NUM_M  (SPI_D_VSYNC_NUM_V << SPI_D_VSYNC_NUM_S)
#define SPI_D_VSYNC_NUM_V  0x00000003
#define SPI_D_VSYNC_NUM_S  8

/* SPI_D_HSYNC_NUM : R/W; bitpos: [7:6]; default: 0;
 * the output spi_hsync is delayed by system clock cycles, 0: delayed by 1
 * cycle, 1: delayed by 2 cycles,...
 */

#define SPI_D_HSYNC_NUM    0x00000003
#define SPI_D_HSYNC_NUM_M  (SPI_D_HSYNC_NUM_V << SPI_D_HSYNC_NUM_S)
#define SPI_D_HSYNC_NUM_V  0x00000003
#define SPI_D_HSYNC_NUM_S  6

/* SPI_D_DE_NUM : R/W; bitpos: [5:4]; default: 0;
 * the output spi_de is delayed by system clock cycles, 0: delayed by 1
 * cycle, 1: delayed by 2 cycles,...
 */

#define SPI_D_DE_NUM    0x00000003
#define SPI_D_DE_NUM_M  (SPI_D_DE_NUM_V << SPI_D_DE_NUM_S)
#define SPI_D_DE_NUM_V  0x00000003
#define SPI_D_DE_NUM_S  4

/* SPI_D_CD_NUM : R/W; bitpos: [3:2]; default: 0;
 * the output spi_cd is delayed by system clock cycles, 0: delayed by 1
 * cycle, 1: delayed by 2 cycles,...
 */

#define SPI_D_CD_NUM    0x00000003
#define SPI_D_CD_NUM_M  (SPI_D_CD_NUM_V << SPI_D_CD_NUM_S)
#define SPI_D_CD_NUM_V  0x00000003
#define SPI_D_CD_NUM_S  2

/* SPI_D_DQS_NUM : R/W; bitpos: [1:0]; default: 0;
 * the output spi_dqs is delayed by system clock cycles, 0: delayed by 1
 * cycle, 1: delayed by 2 cycles,...
 */

#define SPI_D_DQS_NUM    0x00000003
#define SPI_D_DQS_NUM_M  (SPI_D_DQS_NUM_V << SPI_D_DQS_NUM_S)
#define SPI_D_DQS_NUM_V  0x00000003
#define SPI_D_DQS_NUM_S  0

/* SPI_REG_DATE_REG register */

#define SPI_REG_DATE_REG(i) (REG_SPI_BASE(i) + 0x3fc)

/* SPI_DATE : RW; bitpos: [27:0]; default: 26222993;
 * SPI register version.
 */

#define SPI_DATE    0x0FFFFFFF
#define SPI_DATE_M  (SPI_DATE_V << SPI_DATE_S)
#define SPI_DATE_V  0x0FFFFFFF
#define SPI_DATE_S  0

#endif /* __ARCH_XTENSA_SRC_ESP32S2_HARDWARE_ESP32S2_SPI_H */
