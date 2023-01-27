/****************************************************************************
 * arch/xtensa/src/esp32/hardware/esp32_i2s.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_I2S_H
#define __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_I2S_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define REG_I2S_BASE(i) (DR_REG_I2S_BASE + ((i)*0x1e000))

/* I2S_CONF_REG register
 * I2S Configure register
 */

#define I2S_CONF_REG(i) (REG_I2S_BASE(i)  + 0x8)

/* I2S_SIG_LOOPBACK : R/W; bitpos: [18]; default: 0;
 * Enable signal loopback mode with transmitter module and receiver module
 * sharing the same WS and BCK signals.
 */

#define I2S_SIG_LOOPBACK    (BIT(18))
#define I2S_SIG_LOOPBACK_M  (I2S_SIG_LOOPBACK_V << I2S_SIG_LOOPBACK_S)
#define I2S_SIG_LOOPBACK_V  0x00000001
#define I2S_SIG_LOOPBACK_S  18

/* I2S_RX_MSB_RIGHT : R/W; bitpos: [17]; default: 0;
 * Set this bit to place right channel data at the MSB in the receive FIFO.
 */

#define I2S_RX_MSB_RIGHT    (BIT(17))
#define I2S_RX_MSB_RIGHT_M  (I2S_RX_MSB_RIGHT_V << I2S_RX_MSB_RIGHT_S)
#define I2S_RX_MSB_RIGHT_V  0x00000001
#define I2S_RX_MSB_RIGHT_S  17

/* I2S_TX_MSB_RIGHT : R/W; bitpos: [16]; default: 0;
 * Set this bit to place right channel data at the MSB in the transmit FIFO.
 */

#define I2S_TX_MSB_RIGHT    (BIT(16))
#define I2S_TX_MSB_RIGHT_M  (I2S_TX_MSB_RIGHT_V << I2S_TX_MSB_RIGHT_S)
#define I2S_TX_MSB_RIGHT_V  0x00000001
#define I2S_TX_MSB_RIGHT_S  16

/* I2S_RX_MONO : R/W; bitpos: [15]; default: 0;
 * Set this bit to enable receiver  in mono mode
 */

#define I2S_RX_MONO    (BIT(15))
#define I2S_RX_MONO_M  (I2S_RX_MONO_V << I2S_RX_MONO_S)
#define I2S_RX_MONO_V  0x00000001
#define I2S_RX_MONO_S  15

/* I2S_TX_MONO : R/W; bitpos: [14]; default: 0;
 * Set this bit to enable transmitter in mono mode
 */

#define I2S_TX_MONO    (BIT(14))
#define I2S_TX_MONO_M  (I2S_TX_MONO_V << I2S_TX_MONO_S)
#define I2S_TX_MONO_V  0x00000001
#define I2S_TX_MONO_S  14

/* I2S_RX_SHORT_SYNC : R/W; bitpos: [13]; default: 0;
 * Set this bit to enable receiver in PCM standard mode
 */

#define I2S_RX_SHORT_SYNC    (BIT(13))
#define I2S_RX_SHORT_SYNC_M  (I2S_RX_SHORT_SYNC_V << I2S_RX_SHORT_SYNC_S)
#define I2S_RX_SHORT_SYNC_V  0x00000001
#define I2S_RX_SHORT_SYNC_S  13

/* I2S_TX_SHORT_SYNC : R/W; bitpos: [12]; default: 0;
 * Set this bit to enable transmitter in PCM standard mode
 */

#define I2S_TX_SHORT_SYNC    (BIT(12))
#define I2S_TX_SHORT_SYNC_M  (I2S_TX_SHORT_SYNC_V << I2S_TX_SHORT_SYNC_S)
#define I2S_TX_SHORT_SYNC_V  0x00000001
#define I2S_TX_SHORT_SYNC_S  12

/* I2S_RX_MSB_SHIFT : R/W; bitpos: [11]; default: 0;
 * Set this bit to enable receiver in Phillips standard mode
 */

#define I2S_RX_MSB_SHIFT    (BIT(11))
#define I2S_RX_MSB_SHIFT_M  (I2S_RX_MSB_SHIFT_V << I2S_RX_MSB_SHIFT_S)
#define I2S_RX_MSB_SHIFT_V  0x00000001
#define I2S_RX_MSB_SHIFT_S  11

/* I2S_TX_MSB_SHIFT : R/W; bitpos: [10]; default: 0;
 * Set this bit to enable transmitter in Phillips standard mode
 */

#define I2S_TX_MSB_SHIFT    (BIT(10))
#define I2S_TX_MSB_SHIFT_M  (I2S_TX_MSB_SHIFT_V << I2S_TX_MSB_SHIFT_S)
#define I2S_TX_MSB_SHIFT_V  0x00000001
#define I2S_TX_MSB_SHIFT_S  10

/* I2S_RX_RIGHT_FIRST : R/W; bitpos: [9]; default: 1;
 * Set this bit to receive right channel data first
 */

#define I2S_RX_RIGHT_FIRST    (BIT(9))
#define I2S_RX_RIGHT_FIRST_M  (I2S_RX_RIGHT_FIRST_V << I2S_RX_RIGHT_FIRST_S)
#define I2S_RX_RIGHT_FIRST_V  0x00000001
#define I2S_RX_RIGHT_FIRST_S  9

/* I2S_TX_RIGHT_FIRST : R/W; bitpos: [8]; default: 1;
 * Set this bit to transmit right channel data first
 */

#define I2S_TX_RIGHT_FIRST    (BIT(8))
#define I2S_TX_RIGHT_FIRST_M  (I2S_TX_RIGHT_FIRST_V << I2S_TX_RIGHT_FIRST_S)
#define I2S_TX_RIGHT_FIRST_V  0x00000001
#define I2S_TX_RIGHT_FIRST_S  8

/* I2S_RX_SLAVE_MOD : R/W; bitpos: [7]; default: 0;
 * Set this bit to enable slave receiver mode
 */

#define I2S_RX_SLAVE_MOD    (BIT(7))
#define I2S_RX_SLAVE_MOD_M  (I2S_RX_SLAVE_MOD_V << I2S_RX_SLAVE_MOD_S)
#define I2S_RX_SLAVE_MOD_V  0x00000001
#define I2S_RX_SLAVE_MOD_S  7

/* I2S_TX_SLAVE_MOD : R/W; bitpos: [6]; default: 0;
 * Set this bit to enable slave transmitter mode
 */

#define I2S_TX_SLAVE_MOD    (BIT(6))
#define I2S_TX_SLAVE_MOD_M  (I2S_TX_SLAVE_MOD_V << I2S_TX_SLAVE_MOD_S)
#define I2S_TX_SLAVE_MOD_V  0x00000001
#define I2S_TX_SLAVE_MOD_S  6

/* I2S_RX_START : R/W; bitpos: [5]; default: 0;
 * Set this bit to start receiving data
 */

#define I2S_RX_START    (BIT(5))
#define I2S_RX_START_M  (I2S_RX_START_V << I2S_RX_START_S)
#define I2S_RX_START_V  0x00000001
#define I2S_RX_START_S  5

/* I2S_TX_START : R/W; bitpos: [4]; default: 0;
 * Set this bit to start transmitting data
 */

#define I2S_TX_START    (BIT(4))
#define I2S_TX_START_M  (I2S_TX_START_V << I2S_TX_START_S)
#define I2S_TX_START_V  0x00000001
#define I2S_TX_START_S  4

/* I2S_RX_FIFO_RESET : WO; bitpos: [3]; default: 0;
 * Set this bit to reset rxFIFO
 */

#define I2S_RX_FIFO_RESET    (BIT(3))
#define I2S_RX_FIFO_RESET_M  (I2S_RX_FIFO_RESET_V << I2S_RX_FIFO_RESET_S)
#define I2S_RX_FIFO_RESET_V  0x00000001
#define I2S_RX_FIFO_RESET_S  3

/* I2S_TX_FIFO_RESET : WO; bitpos: [2]; default: 0;
 * Set this bit to reset txFIFO
 */

#define I2S_TX_FIFO_RESET    (BIT(2))
#define I2S_TX_FIFO_RESET_M  (I2S_TX_FIFO_RESET_V << I2S_TX_FIFO_RESET_S)
#define I2S_TX_FIFO_RESET_V  0x00000001
#define I2S_TX_FIFO_RESET_S  2

/* I2S_RX_RESET : WO; bitpos: [1]; default: 0;
 * Set this bit to reset receiver
 */

#define I2S_RX_RESET    (BIT(1))
#define I2S_RX_RESET_M  (I2S_RX_RESET_V << I2S_RX_RESET_S)
#define I2S_RX_RESET_V  0x00000001
#define I2S_RX_RESET_S  1

/* I2S_TX_RESET : WO; bitpos: [0]; default: 0;
 * Set this bit to reset transmitter
 */

#define I2S_TX_RESET    (BIT(0))
#define I2S_TX_RESET_M  (I2S_TX_RESET_V << I2S_TX_RESET_S)
#define I2S_TX_RESET_V  0x00000001
#define I2S_TX_RESET_S  0

/* I2S_INT_RAW_REG register
 * Raw interrupt status
 */

#define I2S_INT_RAW_REG(i) (REG_I2S_BASE(i) + 0xc)

/* I2S_OUT_TOTAL_EOF_INT_RAW : RO; bitpos: [16]; default: 0;
 * The raw interrupt status bit  for the i2s_out_total_eof_int interrupt
 */

#define I2S_OUT_TOTAL_EOF_INT_RAW    (BIT(16))
#define I2S_OUT_TOTAL_EOF_INT_RAW_M  (I2S_OUT_TOTAL_EOF_INT_RAW_V << I2S_OUT_TOTAL_EOF_INT_RAW_S)
#define I2S_OUT_TOTAL_EOF_INT_RAW_V  0x00000001
#define I2S_OUT_TOTAL_EOF_INT_RAW_S  16

/* I2S_IN_DSCR_EMPTY_INT_RAW : RO; bitpos: [15]; default: 0;
 * The raw interrupt status bit  for the i2s_in_dscr_empty_int interrupt
 */

#define I2S_IN_DSCR_EMPTY_INT_RAW    (BIT(15))
#define I2S_IN_DSCR_EMPTY_INT_RAW_M  (I2S_IN_DSCR_EMPTY_INT_RAW_V << I2S_IN_DSCR_EMPTY_INT_RAW_S)
#define I2S_IN_DSCR_EMPTY_INT_RAW_V  0x00000001
#define I2S_IN_DSCR_EMPTY_INT_RAW_S  15

/* I2S_OUT_DSCR_ERR_INT_RAW : RO; bitpos: [14]; default: 0;
 * The raw interrupt status bit  for the i2s_out_dscr_err_int interrupt
 */

#define I2S_OUT_DSCR_ERR_INT_RAW    (BIT(14))
#define I2S_OUT_DSCR_ERR_INT_RAW_M  (I2S_OUT_DSCR_ERR_INT_RAW_V << I2S_OUT_DSCR_ERR_INT_RAW_S)
#define I2S_OUT_DSCR_ERR_INT_RAW_V  0x00000001
#define I2S_OUT_DSCR_ERR_INT_RAW_S  14

/* I2S_IN_DSCR_ERR_INT_RAW : RO; bitpos: [13]; default: 0;
 * The raw interrupt status bit  for the i2s_in_dscr_err_int interrupt
 */

#define I2S_IN_DSCR_ERR_INT_RAW    (BIT(13))
#define I2S_IN_DSCR_ERR_INT_RAW_M  (I2S_IN_DSCR_ERR_INT_RAW_V << I2S_IN_DSCR_ERR_INT_RAW_S)
#define I2S_IN_DSCR_ERR_INT_RAW_V  0x00000001
#define I2S_IN_DSCR_ERR_INT_RAW_S  13

/* I2S_OUT_EOF_INT_RAW : RO; bitpos: [12]; default: 0;
 * The raw interrupt status bit  for the i2s_out_eof_int interrupt
 */

#define I2S_OUT_EOF_INT_RAW    (BIT(12))
#define I2S_OUT_EOF_INT_RAW_M  (I2S_OUT_EOF_INT_RAW_V << I2S_OUT_EOF_INT_RAW_S)
#define I2S_OUT_EOF_INT_RAW_V  0x00000001
#define I2S_OUT_EOF_INT_RAW_S  12

/* I2S_OUT_DONE_INT_RAW : RO; bitpos: [11]; default: 0;
 * The raw interrupt status bit  for the i2s_out_done_int interrupt
 */

#define I2S_OUT_DONE_INT_RAW    (BIT(11))
#define I2S_OUT_DONE_INT_RAW_M  (I2S_OUT_DONE_INT_RAW_V << I2S_OUT_DONE_INT_RAW_S)
#define I2S_OUT_DONE_INT_RAW_V  0x00000001
#define I2S_OUT_DONE_INT_RAW_S  11

/* I2S_IN_ERR_EOF_INT_RAW : RO; bitpos: [10]; default: 0;
 * don't use
 */

#define I2S_IN_ERR_EOF_INT_RAW    (BIT(10))
#define I2S_IN_ERR_EOF_INT_RAW_M  (I2S_IN_ERR_EOF_INT_RAW_V << I2S_IN_ERR_EOF_INT_RAW_S)
#define I2S_IN_ERR_EOF_INT_RAW_V  0x00000001
#define I2S_IN_ERR_EOF_INT_RAW_S  10

/* I2S_IN_SUC_EOF_INT_RAW : RO; bitpos: [9]; default: 0;
 * The raw interrupt status bit  for the i2s_in_suc_eof_int interrupt
 */

#define I2S_IN_SUC_EOF_INT_RAW    (BIT(9))
#define I2S_IN_SUC_EOF_INT_RAW_M  (I2S_IN_SUC_EOF_INT_RAW_V << I2S_IN_SUC_EOF_INT_RAW_S)
#define I2S_IN_SUC_EOF_INT_RAW_V  0x00000001
#define I2S_IN_SUC_EOF_INT_RAW_S  9

/* I2S_IN_DONE_INT_RAW : RO; bitpos: [8]; default: 0;
 * The raw interrupt status bit  for the i2s_in_done_int interrupt
 */

#define I2S_IN_DONE_INT_RAW    (BIT(8))
#define I2S_IN_DONE_INT_RAW_M  (I2S_IN_DONE_INT_RAW_V << I2S_IN_DONE_INT_RAW_S)
#define I2S_IN_DONE_INT_RAW_V  0x00000001
#define I2S_IN_DONE_INT_RAW_S  8

/* I2S_TX_HUNG_INT_RAW : RO; bitpos: [7]; default: 0;
 * The raw interrupt status bit  for the i2s_tx_hung_int interrupt
 */

#define I2S_TX_HUNG_INT_RAW    (BIT(7))
#define I2S_TX_HUNG_INT_RAW_M  (I2S_TX_HUNG_INT_RAW_V << I2S_TX_HUNG_INT_RAW_S)
#define I2S_TX_HUNG_INT_RAW_V  0x00000001
#define I2S_TX_HUNG_INT_RAW_S  7

/* I2S_RX_HUNG_INT_RAW : RO; bitpos: [6]; default: 0;
 * The raw interrupt status bit  for the i2s_rx_hung_int interrupt
 */

#define I2S_RX_HUNG_INT_RAW    (BIT(6))
#define I2S_RX_HUNG_INT_RAW_M  (I2S_RX_HUNG_INT_RAW_V << I2S_RX_HUNG_INT_RAW_S)
#define I2S_RX_HUNG_INT_RAW_V  0x00000001
#define I2S_RX_HUNG_INT_RAW_S  6

/* I2S_TX_REMPTY_INT_RAW : RO; bitpos: [5]; default: 0;
 * The raw interrupt status bit  for the i2s_tx_rempty_int interrupt
 */

#define I2S_TX_REMPTY_INT_RAW    (BIT(5))
#define I2S_TX_REMPTY_INT_RAW_M  (I2S_TX_REMPTY_INT_RAW_V << I2S_TX_REMPTY_INT_RAW_S)
#define I2S_TX_REMPTY_INT_RAW_V  0x00000001
#define I2S_TX_REMPTY_INT_RAW_S  5

/* I2S_TX_WFULL_INT_RAW : RO; bitpos: [4]; default: 0;
 * The raw interrupt status bit  for the i2s_tx_wfull_int interrupt
 */

#define I2S_TX_WFULL_INT_RAW    (BIT(4))
#define I2S_TX_WFULL_INT_RAW_M  (I2S_TX_WFULL_INT_RAW_V << I2S_TX_WFULL_INT_RAW_S)
#define I2S_TX_WFULL_INT_RAW_V  0x00000001
#define I2S_TX_WFULL_INT_RAW_S  4

/* I2S_RX_REMPTY_INT_RAW : RO; bitpos: [3]; default: 0;
 * The raw interrupt status bit  for the i2s_rx_rempty_int interrupt
 */

#define I2S_RX_REMPTY_INT_RAW    (BIT(3))
#define I2S_RX_REMPTY_INT_RAW_M  (I2S_RX_REMPTY_INT_RAW_V << I2S_RX_REMPTY_INT_RAW_S)
#define I2S_RX_REMPTY_INT_RAW_V  0x00000001
#define I2S_RX_REMPTY_INT_RAW_S  3

/* I2S_RX_WFULL_INT_RAW : RO; bitpos: [2]; default: 0;
 * The raw interrupt status bit  for the i2s_rx_wfull_int interrupt
 */

#define I2S_RX_WFULL_INT_RAW    (BIT(2))
#define I2S_RX_WFULL_INT_RAW_M  (I2S_RX_WFULL_INT_RAW_V << I2S_RX_WFULL_INT_RAW_S)
#define I2S_RX_WFULL_INT_RAW_V  0x00000001
#define I2S_RX_WFULL_INT_RAW_S  2

/* I2S_TX_PUT_DATA_INT_RAW : RO; bitpos: [1]; default: 0;
 * The raw interrupt status bit  for the i2s_tx_put_data_int interrupt
 */

#define I2S_TX_PUT_DATA_INT_RAW    (BIT(1))
#define I2S_TX_PUT_DATA_INT_RAW_M  (I2S_TX_PUT_DATA_INT_RAW_V << I2S_TX_PUT_DATA_INT_RAW_S)
#define I2S_TX_PUT_DATA_INT_RAW_V  0x00000001
#define I2S_TX_PUT_DATA_INT_RAW_S  1

/* I2S_RX_TAKE_DATA_INT_RAW : RO; bitpos: [0]; default: 0;
 * The raw interrupt status bit  for the i2s_rx_take_data_int interrupt
 */

#define I2S_RX_TAKE_DATA_INT_RAW    (BIT(0))
#define I2S_RX_TAKE_DATA_INT_RAW_M  (I2S_RX_TAKE_DATA_INT_RAW_V << I2S_RX_TAKE_DATA_INT_RAW_S)
#define I2S_RX_TAKE_DATA_INT_RAW_V  0x00000001
#define I2S_RX_TAKE_DATA_INT_RAW_S  0

/* I2S_INT_ST_REG register
 * Masked interrupt status
 */

#define I2S_INT_ST_REG(i) (REG_I2S_BASE(i) + 0x10)

/* I2S_OUT_TOTAL_EOF_INT_ST : RO; bitpos: [16]; default: 0;
 * The masked interrupt status bit  for the i2s_out_total_eof_int interrupt
 */

#define I2S_OUT_TOTAL_EOF_INT_ST    (BIT(16))
#define I2S_OUT_TOTAL_EOF_INT_ST_M  (I2S_OUT_TOTAL_EOF_INT_ST_V << I2S_OUT_TOTAL_EOF_INT_ST_S)
#define I2S_OUT_TOTAL_EOF_INT_ST_V  0x00000001
#define I2S_OUT_TOTAL_EOF_INT_ST_S  16

/* I2S_IN_DSCR_EMPTY_INT_ST : RO; bitpos: [15]; default: 0;
 * The masked interrupt status bit  for the i2s_in_dscr_empty_int interrupt
 */

#define I2S_IN_DSCR_EMPTY_INT_ST    (BIT(15))
#define I2S_IN_DSCR_EMPTY_INT_ST_M  (I2S_IN_DSCR_EMPTY_INT_ST_V << I2S_IN_DSCR_EMPTY_INT_ST_S)
#define I2S_IN_DSCR_EMPTY_INT_ST_V  0x00000001
#define I2S_IN_DSCR_EMPTY_INT_ST_S  15

/* I2S_OUT_DSCR_ERR_INT_ST : RO; bitpos: [14]; default: 0;
 * The masked interrupt status bit  for the i2s_out_dscr_err_int interrupt
 */

#define I2S_OUT_DSCR_ERR_INT_ST    (BIT(14))
#define I2S_OUT_DSCR_ERR_INT_ST_M  (I2S_OUT_DSCR_ERR_INT_ST_V << I2S_OUT_DSCR_ERR_INT_ST_S)
#define I2S_OUT_DSCR_ERR_INT_ST_V  0x00000001
#define I2S_OUT_DSCR_ERR_INT_ST_S  14

/* I2S_IN_DSCR_ERR_INT_ST : RO; bitpos: [13]; default: 0;
 * The masked interrupt status bit  for the i2s_in_dscr_err_int interrupt
 */

#define I2S_IN_DSCR_ERR_INT_ST    (BIT(13))
#define I2S_IN_DSCR_ERR_INT_ST_M  (I2S_IN_DSCR_ERR_INT_ST_V << I2S_IN_DSCR_ERR_INT_ST_S)
#define I2S_IN_DSCR_ERR_INT_ST_V  0x00000001
#define I2S_IN_DSCR_ERR_INT_ST_S  13

/* I2S_OUT_EOF_INT_ST : RO; bitpos: [12]; default: 0;
 * The masked interrupt status bit  for the i2s_out_eof_int interrupt
 */

#define I2S_OUT_EOF_INT_ST    (BIT(12))
#define I2S_OUT_EOF_INT_ST_M  (I2S_OUT_EOF_INT_ST_V << I2S_OUT_EOF_INT_ST_S)
#define I2S_OUT_EOF_INT_ST_V  0x00000001
#define I2S_OUT_EOF_INT_ST_S  12

/* I2S_OUT_DONE_INT_ST : RO; bitpos: [11]; default: 0;
 * The masked interrupt status bit  for the i2s_out_done_int interrupt
 */

#define I2S_OUT_DONE_INT_ST    (BIT(11))
#define I2S_OUT_DONE_INT_ST_M  (I2S_OUT_DONE_INT_ST_V << I2S_OUT_DONE_INT_ST_S)
#define I2S_OUT_DONE_INT_ST_V  0x00000001
#define I2S_OUT_DONE_INT_ST_S  11

/* I2S_IN_ERR_EOF_INT_ST : RO; bitpos: [10]; default: 0;
 * don't use
 */

#define I2S_IN_ERR_EOF_INT_ST    (BIT(10))
#define I2S_IN_ERR_EOF_INT_ST_M  (I2S_IN_ERR_EOF_INT_ST_V << I2S_IN_ERR_EOF_INT_ST_S)
#define I2S_IN_ERR_EOF_INT_ST_V  0x00000001
#define I2S_IN_ERR_EOF_INT_ST_S  10

/* I2S_IN_SUC_EOF_INT_ST : RO; bitpos: [9]; default: 0;
 * The masked interrupt status bit  for the i2s_in_suc_eof_int interrupt
 */

#define I2S_IN_SUC_EOF_INT_ST    (BIT(9))
#define I2S_IN_SUC_EOF_INT_ST_M  (I2S_IN_SUC_EOF_INT_ST_V << I2S_IN_SUC_EOF_INT_ST_S)
#define I2S_IN_SUC_EOF_INT_ST_V  0x00000001
#define I2S_IN_SUC_EOF_INT_ST_S  9

/* I2S_IN_DONE_INT_ST : RO; bitpos: [8]; default: 0;
 * The masked interrupt status bit  for the i2s_in_done_int interrupt
 */

#define I2S_IN_DONE_INT_ST    (BIT(8))
#define I2S_IN_DONE_INT_ST_M  (I2S_IN_DONE_INT_ST_V << I2S_IN_DONE_INT_ST_S)
#define I2S_IN_DONE_INT_ST_V  0x00000001
#define I2S_IN_DONE_INT_ST_S  8

/* I2S_TX_HUNG_INT_ST : RO; bitpos: [7]; default: 0;
 * The masked interrupt status bit  for the i2s_tx_hung_int interrupt
 */

#define I2S_TX_HUNG_INT_ST    (BIT(7))
#define I2S_TX_HUNG_INT_ST_M  (I2S_TX_HUNG_INT_ST_V << I2S_TX_HUNG_INT_ST_S)
#define I2S_TX_HUNG_INT_ST_V  0x00000001
#define I2S_TX_HUNG_INT_ST_S  7

/* I2S_RX_HUNG_INT_ST : RO; bitpos: [6]; default: 0;
 * The masked interrupt status bit  for the i2s_rx_hung_int interrupt
 */

#define I2S_RX_HUNG_INT_ST    (BIT(6))
#define I2S_RX_HUNG_INT_ST_M  (I2S_RX_HUNG_INT_ST_V << I2S_RX_HUNG_INT_ST_S)
#define I2S_RX_HUNG_INT_ST_V  0x00000001
#define I2S_RX_HUNG_INT_ST_S  6

/* I2S_TX_REMPTY_INT_ST : RO; bitpos: [5]; default: 0;
 * The masked interrupt status bit  for the i2s_tx_rempty_int interrupt
 */

#define I2S_TX_REMPTY_INT_ST    (BIT(5))
#define I2S_TX_REMPTY_INT_ST_M  (I2S_TX_REMPTY_INT_ST_V << I2S_TX_REMPTY_INT_ST_S)
#define I2S_TX_REMPTY_INT_ST_V  0x00000001
#define I2S_TX_REMPTY_INT_ST_S  5

/* I2S_TX_WFULL_INT_ST : RO; bitpos: [4]; default: 0;
 * The masked interrupt status bit  for the i2s_tx_wfull_int interrupt
 */

#define I2S_TX_WFULL_INT_ST    (BIT(4))
#define I2S_TX_WFULL_INT_ST_M  (I2S_TX_WFULL_INT_ST_V << I2S_TX_WFULL_INT_ST_S)
#define I2S_TX_WFULL_INT_ST_V  0x00000001
#define I2S_TX_WFULL_INT_ST_S  4

/* I2S_RX_REMPTY_INT_ST : RO; bitpos: [3]; default: 0;
 * The masked interrupt status bit  for the i2s_rx_rempty_int interrupt
 */

#define I2S_RX_REMPTY_INT_ST    (BIT(3))
#define I2S_RX_REMPTY_INT_ST_M  (I2S_RX_REMPTY_INT_ST_V << I2S_RX_REMPTY_INT_ST_S)
#define I2S_RX_REMPTY_INT_ST_V  0x00000001
#define I2S_RX_REMPTY_INT_ST_S  3

/* I2S_RX_WFULL_INT_ST : RO; bitpos: [2]; default: 0;
 * The masked interrupt status bit  for the i2s_rx_wfull_int interrupt
 */

#define I2S_RX_WFULL_INT_ST    (BIT(2))
#define I2S_RX_WFULL_INT_ST_M  (I2S_RX_WFULL_INT_ST_V << I2S_RX_WFULL_INT_ST_S)
#define I2S_RX_WFULL_INT_ST_V  0x00000001
#define I2S_RX_WFULL_INT_ST_S  2

/* I2S_TX_PUT_DATA_INT_ST : RO; bitpos: [1]; default: 0;
 * The masked interrupt status bit  for the i2s_tx_put_data_int interrupt
 */

#define I2S_TX_PUT_DATA_INT_ST    (BIT(1))
#define I2S_TX_PUT_DATA_INT_ST_M  (I2S_TX_PUT_DATA_INT_ST_V << I2S_TX_PUT_DATA_INT_ST_S)
#define I2S_TX_PUT_DATA_INT_ST_V  0x00000001
#define I2S_TX_PUT_DATA_INT_ST_S  1

/* I2S_RX_TAKE_DATA_INT_ST : RO; bitpos: [0]; default: 0;
 * The masked interrupt status bit  for the i2s_rx_take_data_int interrupt
 */

#define I2S_RX_TAKE_DATA_INT_ST    (BIT(0))
#define I2S_RX_TAKE_DATA_INT_ST_M  (I2S_RX_TAKE_DATA_INT_ST_V << I2S_RX_TAKE_DATA_INT_ST_S)
#define I2S_RX_TAKE_DATA_INT_ST_V  0x00000001
#define I2S_RX_TAKE_DATA_INT_ST_S  0

/* I2S_INT_ENA_REG register
 * Interrupt enable bits
 */

#define I2S_INT_ENA_REG(i) (REG_I2S_BASE(i) + 0x14)

/* I2S_OUT_TOTAL_EOF_INT_ENA : R/W; bitpos: [16]; default: 0;
 * The interrupt enable bit  for the i2s_out_total_eof_int interrupt
 */

#define I2S_OUT_TOTAL_EOF_INT_ENA    (BIT(16))
#define I2S_OUT_TOTAL_EOF_INT_ENA_M  (I2S_OUT_TOTAL_EOF_INT_ENA_V << I2S_OUT_TOTAL_EOF_INT_ENA_S)
#define I2S_OUT_TOTAL_EOF_INT_ENA_V  0x00000001
#define I2S_OUT_TOTAL_EOF_INT_ENA_S  16

/* I2S_IN_DSCR_EMPTY_INT_ENA : R/W; bitpos: [15]; default: 0;
 * The interrupt enable bit  for the i2s_in_dscr_empty_int interrupt
 */

#define I2S_IN_DSCR_EMPTY_INT_ENA    (BIT(15))
#define I2S_IN_DSCR_EMPTY_INT_ENA_M  (I2S_IN_DSCR_EMPTY_INT_ENA_V << I2S_IN_DSCR_EMPTY_INT_ENA_S)
#define I2S_IN_DSCR_EMPTY_INT_ENA_V  0x00000001
#define I2S_IN_DSCR_EMPTY_INT_ENA_S  15

/* I2S_OUT_DSCR_ERR_INT_ENA : R/W; bitpos: [14]; default: 0;
 * The interrupt enable bit  for the i2s_out_dscr_err_int interrupt
 */

#define I2S_OUT_DSCR_ERR_INT_ENA    (BIT(14))
#define I2S_OUT_DSCR_ERR_INT_ENA_M  (I2S_OUT_DSCR_ERR_INT_ENA_V << I2S_OUT_DSCR_ERR_INT_ENA_S)
#define I2S_OUT_DSCR_ERR_INT_ENA_V  0x00000001
#define I2S_OUT_DSCR_ERR_INT_ENA_S  14

/* I2S_IN_DSCR_ERR_INT_ENA : R/W; bitpos: [13]; default: 0;
 * The interrupt enable bit  for the i2s_in_dscr_err_int interrupt
 */

#define I2S_IN_DSCR_ERR_INT_ENA    (BIT(13))
#define I2S_IN_DSCR_ERR_INT_ENA_M  (I2S_IN_DSCR_ERR_INT_ENA_V << I2S_IN_DSCR_ERR_INT_ENA_S)
#define I2S_IN_DSCR_ERR_INT_ENA_V  0x00000001
#define I2S_IN_DSCR_ERR_INT_ENA_S  13

/* I2S_OUT_EOF_INT_ENA : R/W; bitpos: [12]; default: 0;
 * The interrupt enable bit  for the i2s_out_eof_int interrupt
 */

#define I2S_OUT_EOF_INT_ENA    (BIT(12))
#define I2S_OUT_EOF_INT_ENA_M  (I2S_OUT_EOF_INT_ENA_V << I2S_OUT_EOF_INT_ENA_S)
#define I2S_OUT_EOF_INT_ENA_V  0x00000001
#define I2S_OUT_EOF_INT_ENA_S  12

/* I2S_OUT_DONE_INT_ENA : R/W; bitpos: [11]; default: 0;
 * The interrupt enable bit  for the i2s_out_done_int interrupt
 */

#define I2S_OUT_DONE_INT_ENA    (BIT(11))
#define I2S_OUT_DONE_INT_ENA_M  (I2S_OUT_DONE_INT_ENA_V << I2S_OUT_DONE_INT_ENA_S)
#define I2S_OUT_DONE_INT_ENA_V  0x00000001
#define I2S_OUT_DONE_INT_ENA_S  11

/* I2S_IN_ERR_EOF_INT_ENA : R/W; bitpos: [10]; default: 0;
 * don't use
 */

#define I2S_IN_ERR_EOF_INT_ENA    (BIT(10))
#define I2S_IN_ERR_EOF_INT_ENA_M  (I2S_IN_ERR_EOF_INT_ENA_V << I2S_IN_ERR_EOF_INT_ENA_S)
#define I2S_IN_ERR_EOF_INT_ENA_V  0x00000001
#define I2S_IN_ERR_EOF_INT_ENA_S  10

/* I2S_IN_SUC_EOF_INT_ENA : R/W; bitpos: [9]; default: 0;
 * The interrupt enable bit  for the i2s_in_suc_eof_int interrupt
 */

#define I2S_IN_SUC_EOF_INT_ENA    (BIT(9))
#define I2S_IN_SUC_EOF_INT_ENA_M  (I2S_IN_SUC_EOF_INT_ENA_V << I2S_IN_SUC_EOF_INT_ENA_S)
#define I2S_IN_SUC_EOF_INT_ENA_V  0x00000001
#define I2S_IN_SUC_EOF_INT_ENA_S  9

/* I2S_IN_DONE_INT_ENA : R/W; bitpos: [8]; default: 0;
 * The interrupt enable bit  for the i2s_in_done_int interrupt
 */

#define I2S_IN_DONE_INT_ENA    (BIT(8))
#define I2S_IN_DONE_INT_ENA_M  (I2S_IN_DONE_INT_ENA_V << I2S_IN_DONE_INT_ENA_S)
#define I2S_IN_DONE_INT_ENA_V  0x00000001
#define I2S_IN_DONE_INT_ENA_S  8

/* I2S_TX_HUNG_INT_ENA : R/W; bitpos: [7]; default: 0;
 * The interrupt enable bit  for the i2s_tx_hung_int interrupt
 */

#define I2S_TX_HUNG_INT_ENA    (BIT(7))
#define I2S_TX_HUNG_INT_ENA_M  (I2S_TX_HUNG_INT_ENA_V << I2S_TX_HUNG_INT_ENA_S)
#define I2S_TX_HUNG_INT_ENA_V  0x00000001
#define I2S_TX_HUNG_INT_ENA_S  7

/* I2S_RX_HUNG_INT_ENA : R/W; bitpos: [6]; default: 0;
 * The interrupt enable bit  for the i2s_rx_hung_int interrupt
 */

#define I2S_RX_HUNG_INT_ENA    (BIT(6))
#define I2S_RX_HUNG_INT_ENA_M  (I2S_RX_HUNG_INT_ENA_V << I2S_RX_HUNG_INT_ENA_S)
#define I2S_RX_HUNG_INT_ENA_V  0x00000001
#define I2S_RX_HUNG_INT_ENA_S  6

/* I2S_TX_REMPTY_INT_ENA : R/W; bitpos: [5]; default: 0;
 * The interrupt enable bit  for the i2s_tx_rempty_int interrupt
 */

#define I2S_TX_REMPTY_INT_ENA    (BIT(5))
#define I2S_TX_REMPTY_INT_ENA_M  (I2S_TX_REMPTY_INT_ENA_V << I2S_TX_REMPTY_INT_ENA_S)
#define I2S_TX_REMPTY_INT_ENA_V  0x00000001
#define I2S_TX_REMPTY_INT_ENA_S  5

/* I2S_TX_WFULL_INT_ENA : R/W; bitpos: [4]; default: 0;
 * The interrupt enable bit  for the i2s_tx_wfull_int interrupt
 */

#define I2S_TX_WFULL_INT_ENA    (BIT(4))
#define I2S_TX_WFULL_INT_ENA_M  (I2S_TX_WFULL_INT_ENA_V << I2S_TX_WFULL_INT_ENA_S)
#define I2S_TX_WFULL_INT_ENA_V  0x00000001
#define I2S_TX_WFULL_INT_ENA_S  4

/* I2S_RX_REMPTY_INT_ENA : R/W; bitpos: [3]; default: 0;
 * The interrupt enable bit  for the i2s_rx_rempty_int interrupt
 */

#define I2S_RX_REMPTY_INT_ENA    (BIT(3))
#define I2S_RX_REMPTY_INT_ENA_M  (I2S_RX_REMPTY_INT_ENA_V << I2S_RX_REMPTY_INT_ENA_S)
#define I2S_RX_REMPTY_INT_ENA_V  0x00000001
#define I2S_RX_REMPTY_INT_ENA_S  3

/* I2S_RX_WFULL_INT_ENA : R/W; bitpos: [2]; default: 0;
 * The interrupt enable bit  for the i2s_rx_wfull_int interrupt
 */

#define I2S_RX_WFULL_INT_ENA    (BIT(2))
#define I2S_RX_WFULL_INT_ENA_M  (I2S_RX_WFULL_INT_ENA_V << I2S_RX_WFULL_INT_ENA_S)
#define I2S_RX_WFULL_INT_ENA_V  0x00000001
#define I2S_RX_WFULL_INT_ENA_S  2

/* I2S_TX_PUT_DATA_INT_ENA : R/W; bitpos: [1]; default: 0;
 * The interrupt enable bit  for the i2s_tx_put_data_int interrupt
 */

#define I2S_TX_PUT_DATA_INT_ENA    (BIT(1))
#define I2S_TX_PUT_DATA_INT_ENA_M  (I2S_TX_PUT_DATA_INT_ENA_V << I2S_TX_PUT_DATA_INT_ENA_S)
#define I2S_TX_PUT_DATA_INT_ENA_V  0x00000001
#define I2S_TX_PUT_DATA_INT_ENA_S  1

/* I2S_RX_TAKE_DATA_INT_ENA : R/W; bitpos: [0]; default: 0;
 * The interrupt enable bit  for the i2s_rx_take_data_int interrupt
 */

#define I2S_RX_TAKE_DATA_INT_ENA    (BIT(0))
#define I2S_RX_TAKE_DATA_INT_ENA_M  (I2S_RX_TAKE_DATA_INT_ENA_V << I2S_RX_TAKE_DATA_INT_ENA_S)
#define I2S_RX_TAKE_DATA_INT_ENA_V  0x00000001
#define I2S_RX_TAKE_DATA_INT_ENA_S  0

/* I2S_INT_CLR_REG register
 * Interrupt clear bits
 */

#define I2S_INT_CLR_REG(i) (REG_I2S_BASE(i) + 0x18)

/* I2S_OUT_TOTAL_EOF_INT_CLR : WO; bitpos: [16]; default: 0;
 * Set this bit to clear the i2s_out_total_eof_int interrupt
 */

#define I2S_OUT_TOTAL_EOF_INT_CLR    (BIT(16))
#define I2S_OUT_TOTAL_EOF_INT_CLR_M  (I2S_OUT_TOTAL_EOF_INT_CLR_V << I2S_OUT_TOTAL_EOF_INT_CLR_S)
#define I2S_OUT_TOTAL_EOF_INT_CLR_V  0x00000001
#define I2S_OUT_TOTAL_EOF_INT_CLR_S  16

/* I2S_IN_DSCR_EMPTY_INT_CLR : WO; bitpos: [15]; default: 0;
 * Set this bit to clear the i2s_in_dscr_empty_int interrupt
 */

#define I2S_IN_DSCR_EMPTY_INT_CLR    (BIT(15))
#define I2S_IN_DSCR_EMPTY_INT_CLR_M  (I2S_IN_DSCR_EMPTY_INT_CLR_V << I2S_IN_DSCR_EMPTY_INT_CLR_S)
#define I2S_IN_DSCR_EMPTY_INT_CLR_V  0x00000001
#define I2S_IN_DSCR_EMPTY_INT_CLR_S  15

/* I2S_OUT_DSCR_ERR_INT_CLR : WO; bitpos: [14]; default: 0;
 * Set this bit to clear the i2s_out_dscr_err_int interrupt
 */

#define I2S_OUT_DSCR_ERR_INT_CLR    (BIT(14))
#define I2S_OUT_DSCR_ERR_INT_CLR_M  (I2S_OUT_DSCR_ERR_INT_CLR_V << I2S_OUT_DSCR_ERR_INT_CLR_S)
#define I2S_OUT_DSCR_ERR_INT_CLR_V  0x00000001
#define I2S_OUT_DSCR_ERR_INT_CLR_S  14

/* I2S_IN_DSCR_ERR_INT_CLR : WO; bitpos: [13]; default: 0;
 * Set this bit to clear the i2s_in_dscr_err_int interrupt
 */

#define I2S_IN_DSCR_ERR_INT_CLR    (BIT(13))
#define I2S_IN_DSCR_ERR_INT_CLR_M  (I2S_IN_DSCR_ERR_INT_CLR_V << I2S_IN_DSCR_ERR_INT_CLR_S)
#define I2S_IN_DSCR_ERR_INT_CLR_V  0x00000001
#define I2S_IN_DSCR_ERR_INT_CLR_S  13

/* I2S_OUT_EOF_INT_CLR : WO; bitpos: [12]; default: 0;
 * Set this bit to clear the i2s_out_eof_int interrupt
 */

#define I2S_OUT_EOF_INT_CLR    (BIT(12))
#define I2S_OUT_EOF_INT_CLR_M  (I2S_OUT_EOF_INT_CLR_V << I2S_OUT_EOF_INT_CLR_S)
#define I2S_OUT_EOF_INT_CLR_V  0x00000001
#define I2S_OUT_EOF_INT_CLR_S  12

/* I2S_OUT_DONE_INT_CLR : WO; bitpos: [11]; default: 0;
 * Set this bit to clear the i2s_out_done_int interrupt
 */

#define I2S_OUT_DONE_INT_CLR    (BIT(11))
#define I2S_OUT_DONE_INT_CLR_M  (I2S_OUT_DONE_INT_CLR_V << I2S_OUT_DONE_INT_CLR_S)
#define I2S_OUT_DONE_INT_CLR_V  0x00000001
#define I2S_OUT_DONE_INT_CLR_S  11

/* I2S_IN_ERR_EOF_INT_CLR : WO; bitpos: [10]; default: 0;
 * don't use
 */

#define I2S_IN_ERR_EOF_INT_CLR    (BIT(10))
#define I2S_IN_ERR_EOF_INT_CLR_M  (I2S_IN_ERR_EOF_INT_CLR_V << I2S_IN_ERR_EOF_INT_CLR_S)
#define I2S_IN_ERR_EOF_INT_CLR_V  0x00000001
#define I2S_IN_ERR_EOF_INT_CLR_S  10

/* I2S_IN_SUC_EOF_INT_CLR : WO; bitpos: [9]; default: 0;
 * Set this bit to clear the i2s_in_suc_eof_int interrupt
 */

#define I2S_IN_SUC_EOF_INT_CLR    (BIT(9))
#define I2S_IN_SUC_EOF_INT_CLR_M  (I2S_IN_SUC_EOF_INT_CLR_V << I2S_IN_SUC_EOF_INT_CLR_S)
#define I2S_IN_SUC_EOF_INT_CLR_V  0x00000001
#define I2S_IN_SUC_EOF_INT_CLR_S  9

/* I2S_IN_DONE_INT_CLR : WO; bitpos: [8]; default: 0;
 * Set this bit to clear the i2s_in_done_int interrupt
 */

#define I2S_IN_DONE_INT_CLR    (BIT(8))
#define I2S_IN_DONE_INT_CLR_M  (I2S_IN_DONE_INT_CLR_V << I2S_IN_DONE_INT_CLR_S)
#define I2S_IN_DONE_INT_CLR_V  0x00000001
#define I2S_IN_DONE_INT_CLR_S  8

/* I2S_TX_HUNG_INT_CLR : WO; bitpos: [7]; default: 0;
 * Set this bit to clear the i2s_tx_hung_int interrupt
 */

#define I2S_TX_HUNG_INT_CLR    (BIT(7))
#define I2S_TX_HUNG_INT_CLR_M  (I2S_TX_HUNG_INT_CLR_V << I2S_TX_HUNG_INT_CLR_S)
#define I2S_TX_HUNG_INT_CLR_V  0x00000001
#define I2S_TX_HUNG_INT_CLR_S  7

/* I2S_RX_HUNG_INT_CLR : WO; bitpos: [6]; default: 0;
 * Set this bit to clear the i2s_rx_hung_int interrupt
 */

#define I2S_RX_HUNG_INT_CLR    (BIT(6))
#define I2S_RX_HUNG_INT_CLR_M  (I2S_RX_HUNG_INT_CLR_V << I2S_RX_HUNG_INT_CLR_S)
#define I2S_RX_HUNG_INT_CLR_V  0x00000001
#define I2S_RX_HUNG_INT_CLR_S  6

/* I2S_TX_REMPTY_INT_CLR : WO; bitpos: [5]; default: 0;
 * Set this bit to clear the i2s_tx_rempty_int interrupt
 */

#define I2S_TX_REMPTY_INT_CLR    (BIT(5))
#define I2S_TX_REMPTY_INT_CLR_M  (I2S_TX_REMPTY_INT_CLR_V << I2S_TX_REMPTY_INT_CLR_S)
#define I2S_TX_REMPTY_INT_CLR_V  0x00000001
#define I2S_TX_REMPTY_INT_CLR_S  5

/* I2S_TX_WFULL_INT_CLR : WO; bitpos: [4]; default: 0;
 * Set this bit to clear the i2s_tx_wfull_int interrupt
 */

#define I2S_TX_WFULL_INT_CLR    (BIT(4))
#define I2S_TX_WFULL_INT_CLR_M  (I2S_TX_WFULL_INT_CLR_V << I2S_TX_WFULL_INT_CLR_S)
#define I2S_TX_WFULL_INT_CLR_V  0x00000001
#define I2S_TX_WFULL_INT_CLR_S  4

/* I2S_RX_REMPTY_INT_CLR : WO; bitpos: [3]; default: 0;
 * Set this bit to clear the i2s_rx_rempty_int interrupt
 */

#define I2S_RX_REMPTY_INT_CLR    (BIT(3))
#define I2S_RX_REMPTY_INT_CLR_M  (I2S_RX_REMPTY_INT_CLR_V << I2S_RX_REMPTY_INT_CLR_S)
#define I2S_RX_REMPTY_INT_CLR_V  0x00000001
#define I2S_RX_REMPTY_INT_CLR_S  3

/* I2S_RX_WFULL_INT_CLR : WO; bitpos: [2]; default: 0;
 * Set this bit to clear the i2s_rx_wfull_int interrupt
 */

#define I2S_RX_WFULL_INT_CLR    (BIT(2))
#define I2S_RX_WFULL_INT_CLR_M  (I2S_RX_WFULL_INT_CLR_V << I2S_RX_WFULL_INT_CLR_S)
#define I2S_RX_WFULL_INT_CLR_V  0x00000001
#define I2S_RX_WFULL_INT_CLR_S  2

/* I2S_TX_PUT_DATA_INT_CLR : WO; bitpos: [1]; default: 0;
 * Set this bit to clear the i2s_tx_put_data_int interrupt
 */

#define I2S_TX_PUT_DATA_INT_CLR     (BIT(1))
#define I2S_TX_PUT_DATA_INT_CLR_M   (I2S_TX_PUT_DATA_INT_CLR_V << I2S_TX_PUT_DATA_INT_CLR_S)
#define I2S_TX_PUT_DATA_INT_CLR_V   0x00000001
#define I2S_TX_PUT_DATA_INT_CLR_S   1

/* I2S_RX_TAKE_DATA_INT_CLR : WO; bitpos: [0]; default: 0;
 * Set this bit to clear the i2s_rx_take_data_int interrupt
 */

#define I2S_RX_TAKE_DATA_INT_CLR    (BIT(0))
#define I2S_RX_TAKE_DATA_INT_CLR_M  (I2S_RX_TAKE_DATA_INT_CLR_V << I2S_RX_TAKE_DATA_INT_CLR_S)
#define I2S_RX_TAKE_DATA_INT_CLR_V  0x00000001
#define I2S_RX_TAKE_DATA_INT_CLR_S  0

/* I2S_TIMING_REG register
 * I2S timing register
 */

#define I2S_TIMING_REG(i) (REG_I2S_BASE(i) + 0x1c)

/* I2S_TX_BCK_IN_INV : R/W; bitpos: [24]; default: 0;
 * Set this bit to invert BCK signal input to the slave transmitter
 */

#define I2S_TX_BCK_IN_INV    (BIT(24))
#define I2S_TX_BCK_IN_INV_M  (I2S_TX_BCK_IN_INV_V << I2S_TX_BCK_IN_INV_S)
#define I2S_TX_BCK_IN_INV_V  0x00000001
#define I2S_TX_BCK_IN_INV_S  24

/* I2S_DATA_ENABLE_DELAY : R/W; bitpos: [23:22]; default: 0;
 * Number of delay cycles for data valid flag.
 */

#define I2S_DATA_ENABLE_DELAY    0x00000003
#define I2S_DATA_ENABLE_DELAY_M  (I2S_DATA_ENABLE_DELAY_V << I2S_DATA_ENABLE_DELAY_S)
#define I2S_DATA_ENABLE_DELAY_V  0x00000003
#define I2S_DATA_ENABLE_DELAY_S  22

/* I2S_RX_DSYNC_SW : R/W; bitpos: [21]; default: 0;
 * Set this bit to synchronize signals with the double sync method into the
 * receiver
 */

#define I2S_RX_DSYNC_SW    (BIT(21))
#define I2S_RX_DSYNC_SW_M  (I2S_RX_DSYNC_SW_V << I2S_RX_DSYNC_SW_S)
#define I2S_RX_DSYNC_SW_V  0x00000001
#define I2S_RX_DSYNC_SW_S  21

/* I2S_TX_DSYNC_SW : R/W; bitpos: [20]; default: 0;
 * Set this bit to synchronize signals with the double sync method into the
 * transmitter
 */

#define I2S_TX_DSYNC_SW    (BIT(20))
#define I2S_TX_DSYNC_SW_M  (I2S_TX_DSYNC_SW_V << I2S_TX_DSYNC_SW_S)
#define I2S_TX_DSYNC_SW_V  0x00000001
#define I2S_TX_DSYNC_SW_S  20

/* I2S_RX_BCK_OUT_DELAY : R/W; bitpos: [19:18]; default: 0;
 * Number of delay cycles for BCK out of the receiver
 */

#define I2S_RX_BCK_OUT_DELAY    0x00000003
#define I2S_RX_BCK_OUT_DELAY_M  (I2S_RX_BCK_OUT_DELAY_V << I2S_RX_BCK_OUT_DELAY_S)
#define I2S_RX_BCK_OUT_DELAY_V  0x00000003
#define I2S_RX_BCK_OUT_DELAY_S  18

/* I2S_RX_WS_OUT_DELAY : R/W; bitpos: [17:16]; default: 0;
 * Number of delay cycles for WS out of the receiver
 */

#define I2S_RX_WS_OUT_DELAY    0x00000003
#define I2S_RX_WS_OUT_DELAY_M  (I2S_RX_WS_OUT_DELAY_V << I2S_RX_WS_OUT_DELAY_S)
#define I2S_RX_WS_OUT_DELAY_V  0x00000003
#define I2S_RX_WS_OUT_DELAY_S  16

/* I2S_TX_SD_OUT_DELAY : R/W; bitpos: [15:14]; default: 0;
 * Number of delay cycles for SD out of the transmitter
 */

#define I2S_TX_SD_OUT_DELAY    0x00000003
#define I2S_TX_SD_OUT_DELAY_M  (I2S_TX_SD_OUT_DELAY_V << I2S_TX_SD_OUT_DELAY_S)
#define I2S_TX_SD_OUT_DELAY_V  0x00000003
#define I2S_TX_SD_OUT_DELAY_S  14

/* I2S_TX_WS_OUT_DELAY : R/W; bitpos: [13:12]; default: 0;
 * Number of delay cycles for WS out of the transmitter
 */

#define I2S_TX_WS_OUT_DELAY    0x00000003
#define I2S_TX_WS_OUT_DELAY_M  (I2S_TX_WS_OUT_DELAY_V << I2S_TX_WS_OUT_DELAY_S)
#define I2S_TX_WS_OUT_DELAY_V  0x00000003
#define I2S_TX_WS_OUT_DELAY_S  12

/* I2S_TX_BCK_OUT_DELAY : R/W; bitpos: [11:10]; default: 0;
 * Number of delay cycles for BCK out of the transmitter
 */

#define I2S_TX_BCK_OUT_DELAY    0x00000003
#define I2S_TX_BCK_OUT_DELAY_M  (I2S_TX_BCK_OUT_DELAY_V << I2S_TX_BCK_OUT_DELAY_S)
#define I2S_TX_BCK_OUT_DELAY_V  0x00000003
#define I2S_TX_BCK_OUT_DELAY_S  10

/* I2S_RX_SD_IN_DELAY : R/W; bitpos: [9:8]; default: 0;
 * Number of delay cycles for SD into the receiver
 */

#define I2S_RX_SD_IN_DELAY    0x00000003
#define I2S_RX_SD_IN_DELAY_M  (I2S_RX_SD_IN_DELAY_V << I2S_RX_SD_IN_DELAY_S)
#define I2S_RX_SD_IN_DELAY_V  0x00000003
#define I2S_RX_SD_IN_DELAY_S  8

/* I2S_RX_WS_IN_DELAY : R/W; bitpos: [7:6]; default: 0;
 * Number of delay cycles for WS into the receiver
 */

#define I2S_RX_WS_IN_DELAY    0x00000003
#define I2S_RX_WS_IN_DELAY_M  (I2S_RX_WS_IN_DELAY_V << I2S_RX_WS_IN_DELAY_S)
#define I2S_RX_WS_IN_DELAY_V  0x00000003
#define I2S_RX_WS_IN_DELAY_S  6

/* I2S_RX_BCK_IN_DELAY : R/W; bitpos: [5:4]; default: 0;
 * Number of delay cycles for BCK into the receiver
 */

#define I2S_RX_BCK_IN_DELAY    0x00000003
#define I2S_RX_BCK_IN_DELAY_M  (I2S_RX_BCK_IN_DELAY_V << I2S_RX_BCK_IN_DELAY_S)
#define I2S_RX_BCK_IN_DELAY_V  0x00000003
#define I2S_RX_BCK_IN_DELAY_S  4

/* I2S_TX_WS_IN_DELAY : R/W; bitpos: [3:2]; default: 0;
 * Number of delay cycles for WS into the transmitter
 */

#define I2S_TX_WS_IN_DELAY    0x00000003
#define I2S_TX_WS_IN_DELAY_M  (I2S_TX_WS_IN_DELAY_V << I2S_TX_WS_IN_DELAY_S)
#define I2S_TX_WS_IN_DELAY_V  0x00000003
#define I2S_TX_WS_IN_DELAY_S  2

/* I2S_TX_BCK_IN_DELAY : R/W; bitpos: [1:0]; default: 0;
 * Number of delay cycles for BCK into the transmitter
 */

#define I2S_TX_BCK_IN_DELAY    0x00000003
#define I2S_TX_BCK_IN_DELAY_M  (I2S_TX_BCK_IN_DELAY_V << I2S_TX_BCK_IN_DELAY_S)
#define I2S_TX_BCK_IN_DELAY_V  0x00000003
#define I2S_TX_BCK_IN_DELAY_S  0

/* I2S_FIFO_CONF_REG register
 * I2S FIFO configure register
 */

#define I2S_FIFO_CONF_REG(i) (REG_I2S_BASE(i) + 0x20)

/* I2S_RX_FIFO_MOD_FORCE_EN : R/W; bitpos: [20]; default: 0;
 * The bit should always be set to 1
 */

#define I2S_RX_FIFO_MOD_FORCE_EN    (BIT(20))
#define I2S_RX_FIFO_MOD_FORCE_EN_M  (I2S_RX_FIFO_MOD_FORCE_EN_V << I2S_RX_FIFO_MOD_FORCE_EN_S)
#define I2S_RX_FIFO_MOD_FORCE_EN_V  0x00000001
#define I2S_RX_FIFO_MOD_FORCE_EN_S  20

/* I2S_TX_FIFO_MOD_FORCE_EN : R/W; bitpos: [19]; default: 0;
 * The bit should always be set to 1
 */

#define I2S_TX_FIFO_MOD_FORCE_EN    (BIT(19))
#define I2S_TX_FIFO_MOD_FORCE_EN_M  (I2S_TX_FIFO_MOD_FORCE_EN_V << I2S_TX_FIFO_MOD_FORCE_EN_S)
#define I2S_TX_FIFO_MOD_FORCE_EN_V  0x00000001
#define I2S_TX_FIFO_MOD_FORCE_EN_S  19

/* I2S_RX_FIFO_MOD : R/W; bitpos: [18:16]; default: 0;
 * Receiver FIFO mode configuration bits
 */

#define I2S_RX_FIFO_MOD    0x00000007
#define I2S_RX_FIFO_MOD_M  (I2S_RX_FIFO_MOD_V << I2S_RX_FIFO_MOD_S)
#define I2S_RX_FIFO_MOD_V  0x00000007
#define I2S_RX_FIFO_MOD_S  16

/* I2S_TX_FIFO_MOD : R/W; bitpos: [15:13]; default: 0;
 * Transmitter FIFO mode configuration bits
 */

#define I2S_TX_FIFO_MOD    0x00000007
#define I2S_TX_FIFO_MOD_M  (I2S_TX_FIFO_MOD_V << I2S_TX_FIFO_MOD_S)
#define I2S_TX_FIFO_MOD_V  0x00000007
#define I2S_TX_FIFO_MOD_S  13

/* I2S_DSCR_EN : R/W; bitpos: [12]; default: 1;
 * Set this bit to enable I2S DMA mode
 */

#define I2S_DSCR_EN    (BIT(12))
#define I2S_DSCR_EN_M  (I2S_DSCR_EN_V << I2S_DSCR_EN_S)
#define I2S_DSCR_EN_V  0x00000001
#define I2S_DSCR_EN_S  12

/* I2S_TX_DATA_NUM : R/W; bitpos: [11:6]; default: 32;
 * Threshold of data length in transmitter FIFO
 */

#define I2S_TX_DATA_NUM    0x0000003F
#define I2S_TX_DATA_NUM_M  (I2S_TX_DATA_NUM_V << I2S_TX_DATA_NUM_S)
#define I2S_TX_DATA_NUM_V  0x0000003F
#define I2S_TX_DATA_NUM_S  6

/* I2S_RX_DATA_NUM : R/W; bitpos: [5:0]; default: 32;
 * Threshold of data length in receiver FIFO
 */

#define I2S_RX_DATA_NUM    0x0000003F
#define I2S_RX_DATA_NUM_M  (I2S_RX_DATA_NUM_V << I2S_RX_DATA_NUM_S)
#define I2S_RX_DATA_NUM_V  0x0000003F
#define I2S_RX_DATA_NUM_S  0

/* I2S_RXEOF_NUM_REG register
 * I2S DMA RX EOF data length
 */

#define I2S_RXEOF_NUM_REG(i) (REG_I2S_BASE(i) + 0x24)

/* I2S_RX_EOF_NUM : R/W; bitpos: [31:0]; default: 64;
 * the length of data to be received. It will trigger i2s_in_suc_eof_int.
 */

#define I2S_RX_EOF_NUM    0xFFFFFFFF
#define I2S_RX_EOF_NUM_M  (I2S_RX_EOF_NUM_V << I2S_RX_EOF_NUM_S)
#define I2S_RX_EOF_NUM_V  0xFFFFFFFF
#define I2S_RX_EOF_NUM_S  0

/* I2S_CONF_SIGLE_DATA_REG register
 * Constant single channel data
 */

#define I2S_CONF_SINGLE_DATA_REG(i) (REG_I2S_BASE(i) + 0x28)

/* I2S_SIGLE_DATA : R/W; bitpos: [31:0]; default: 0;
 * the right channel or left channel put out constant value stored in this
 * register according to tx_chan_mod and reg_tx_msb_right
 */

#define I2S_SIGLE_DATA    0xFFFFFFFF
#define I2S_SIGLE_DATA_M  (I2S_SIGLE_DATA_V << I2S_SIGLE_DATA_S)
#define I2S_SIGLE_DATA_V  0xFFFFFFFF
#define I2S_SIGLE_DATA_S  0

/* I2S_CONF_CHAN_REG register
 * I2S channel configure register
 */

#define I2S_CONF_CHAN_REG(i) (REG_I2S_BASE(i) + 0x2c)

/* I2S_RX_CHAN_MOD : R/W; bitpos: [4:3]; default: 0;
 * I2S receiver channel mode configuration bits.
 */

#define I2S_RX_CHAN_MOD    0x00000003
#define I2S_RX_CHAN_MOD_M  (I2S_RX_CHAN_MOD_V << I2S_RX_CHAN_MOD_S)
#define I2S_RX_CHAN_MOD_V  0x00000003
#define I2S_RX_CHAN_MOD_S  3

/* I2S_TX_CHAN_MOD : R/W; bitpos: [2:0]; default: 0;
 * I2S transmitter channel mode configuration bits.
 */

#define I2S_TX_CHAN_MOD    0x00000007
#define I2S_TX_CHAN_MOD_M  (I2S_TX_CHAN_MOD_V << I2S_TX_CHAN_MOD_S)
#define I2S_TX_CHAN_MOD_V  0x00000007
#define I2S_TX_CHAN_MOD_S  0

/* I2S_OUT_LINK_REG register
 * I2S DMA TX configure register
 */

#define I2S_OUT_LINK_REG(i) (REG_I2S_BASE(i) + 0x30)

/* I2S_OUTLINK_RESTART : R/W; bitpos: [30]; default: 0;
 * Set this bit to restart outlink descriptor
 */

#define I2S_OUTLINK_RESTART    (BIT(30))
#define I2S_OUTLINK_RESTART_M  (I2S_OUTLINK_RESTART_V << I2S_OUTLINK_RESTART_S)
#define I2S_OUTLINK_RESTART_V  0x00000001
#define I2S_OUTLINK_RESTART_S  30

/* I2S_OUTLINK_START : R/W; bitpos: [29]; default: 0;
 * Set this bit to start outlink descriptor
 */

#define I2S_OUTLINK_START    (BIT(29))
#define I2S_OUTLINK_START_M  (I2S_OUTLINK_START_V << I2S_OUTLINK_START_S)
#define I2S_OUTLINK_START_V  0x00000001
#define I2S_OUTLINK_START_S  29

/* I2S_OUTLINK_STOP : R/W; bitpos: [28]; default: 0;
 * Set this bit to stop outlink descriptor
 */

#define I2S_OUTLINK_STOP    (BIT(28))
#define I2S_OUTLINK_STOP_M  (I2S_OUTLINK_STOP_V << I2S_OUTLINK_STOP_S)
#define I2S_OUTLINK_STOP_V  0x00000001
#define I2S_OUTLINK_STOP_S  28

/* I2S_OUTLINK_ADDR : R/W; bitpos: [19:0]; default: 0;
 * The address of first outlink descriptor
 */

#define I2S_OUTLINK_ADDR    0x000FFFFF
#define I2S_OUTLINK_ADDR_M  (I2S_OUTLINK_ADDR_V << I2S_OUTLINK_ADDR_S)
#define I2S_OUTLINK_ADDR_V  0x000FFFFF
#define I2S_OUTLINK_ADDR_S  0

/* I2S_IN_LINK_REG register
 * I2S DMA RX configure register
 */

#define I2S_IN_LINK_REG(i) (REG_I2S_BASE(i) + 0x34)

/* I2S_INLINK_RESTART : R/W; bitpos: [30]; default: 0;
 * Set this bit to restart inlink descriptor
 */

#define I2S_INLINK_RESTART    (BIT(30))
#define I2S_INLINK_RESTART_M  (I2S_INLINK_RESTART_V << I2S_INLINK_RESTART_S)
#define I2S_INLINK_RESTART_V  0x00000001
#define I2S_INLINK_RESTART_S  30

/* I2S_INLINK_START : R/W; bitpos: [29]; default: 0;
 * Set this bit to start inlink descriptor
 */

#define I2S_INLINK_START    (BIT(29))
#define I2S_INLINK_START_M  (I2S_INLINK_START_V << I2S_INLINK_START_S)
#define I2S_INLINK_START_V  0x00000001
#define I2S_INLINK_START_S  29

/* I2S_INLINK_STOP : R/W; bitpos: [28]; default: 0;
 * Set this bit to stop inlink descriptor
 */

#define I2S_INLINK_STOP    (BIT(28))
#define I2S_INLINK_STOP_M  (I2S_INLINK_STOP_V << I2S_INLINK_STOP_S)
#define I2S_INLINK_STOP_V  0x00000001
#define I2S_INLINK_STOP_S  28

/* I2S_INLINK_ADDR : R/W; bitpos: [19:0]; default: 0;
 * The address of first inlink descriptor
 */

#define I2S_INLINK_ADDR    0x000FFFFF
#define I2S_INLINK_ADDR_M  (I2S_INLINK_ADDR_V << I2S_INLINK_ADDR_S)
#define I2S_INLINK_ADDR_V  0x000FFFFF
#define I2S_INLINK_ADDR_S  0

/* I2S_OUT_EOF_DES_ADDR_REG register
 * The address of outlink descriptor that produces EOF
 */

#define I2S_OUT_EOF_DES_ADDR_REG(i) (REG_I2S_BASE(i) + 0x38)

/* I2S_OUT_EOF_DES_ADDR : RO; bitpos: [31:0]; default: 0;
 * The address of outlink descriptor that produces EOF
 */

#define I2S_OUT_EOF_DES_ADDR    0xFFFFFFFF
#define I2S_OUT_EOF_DES_ADDR_M  (I2S_OUT_EOF_DES_ADDR_V << I2S_OUT_EOF_DES_ADDR_S)
#define I2S_OUT_EOF_DES_ADDR_V  0xFFFFFFFF
#define I2S_OUT_EOF_DES_ADDR_S  0

/* I2S_IN_EOF_DES_ADDR_REG register
 * The address of inlink descriptor that produces EOF
 */

#define I2S_IN_EOF_DES_ADDR_REG(i) (REG_I2S_BASE(i) + 0x3c)

/* I2S_IN_EOF_DES_ADDR_REG : RO; bitpos: [31:0]; default: 0;
 * The address of inlink descriptor that produces EOF
 */

#define I2S_IN_EOF_DES_ADDR     0xFFFFFFFF
#define I2S_IN_EOF_DES_ADDR_M   (I2S_IN_EOF_DES_ADDR_V << I2S_IN_EOF_DES_ADDR_S)
#define I2S_IN_EOF_DES_ADDR_V   0xFFFFFFFF
#define I2S_IN_EOF_DES_ADDR_S   0

/* I2S_OUT_EOF_BFR_DES_ADDR_REG register
 * The address of buffer relative to the outlink descriptor that produces EOF
 */

#define I2S_OUT_EOF_BFR_DES_ADDR_REG(i) (REG_I2S_BASE(i) + 0x40)

/* I2S_OUT_EOF_BFR_DES_ADDR : RO; bitpos: [31:0]; default: 0;
 * The address of buffer relative to the outlink descriptor that produces EOF
 */

#define I2S_OUT_EOF_BFR_DES_ADDR    0xFFFFFFFF
#define I2S_OUT_EOF_BFR_DES_ADDR_M  (I2S_OUT_EOF_BFR_DES_ADDR_V << I2S_OUT_EOF_BFR_DES_ADDR_S)
#define I2S_OUT_EOF_BFR_DES_ADDR_V  0xFFFFFFFF
#define I2S_OUT_EOF_BFR_DES_ADDR_S  0

/* I2S_INLINK_DSCR_REG register
 * The address of current inlink descriptor
 */

#define I2S_INLINK_DSCR_REG(i) (REG_I2S_BASE(i) + 0x48)

/* I2S_INLINK_DSCR : RO; bitpos: [31:0]; default: 0;
 * The address of current inlink descriptor
 */

#define I2S_INLINK_DSCR    0xFFFFFFFF
#define I2S_INLINK_DSCR_M  (I2S_INLINK_DSCR_V << I2S_INLINK_DSCR_S)
#define I2S_INLINK_DSCR_V  0xFFFFFFFF
#define I2S_INLINK_DSCR_S  0

/* I2S_INLINK_DSCR_BF0_REG register
 * The address of next inlink descriptor
 */

#define I2S_INLINK_DSCR_BF0_REG(i) (REG_I2S_BASE(i) + 0x4c)

/* I2S_INLINK_DSCR_BF0 : RO; bitpos: [31:0]; default: 0;
 * The address of next inlink descriptor
 */

#define I2S_INLINK_DSCR_BF0    0xFFFFFFFF
#define I2S_INLINK_DSCR_BF0_M  (I2S_INLINK_DSCR_BF0_V << I2S_INLINK_DSCR_BF0_S)
#define I2S_INLINK_DSCR_BF0_V  0xFFFFFFFF
#define I2S_INLINK_DSCR_BF0_S  0

/* I2S_INLINK_DSCR_BF1_REG register
 * The address of next inlink data buffer
 */

#define I2S_INLINK_DSCR_BF1_REG(i) (REG_I2S_BASE(i) + 0x50)

/* I2S_INLINK_DSCR_BF1 : RO; bitpos: [31:0]; default: 0;
 * The address of next inlink data buffer
 */

#define I2S_INLINK_DSCR_BF1    0xFFFFFFFF
#define I2S_INLINK_DSCR_BF1_M  (I2S_INLINK_DSCR_BF1_V << I2S_INLINK_DSCR_BF1_S)
#define I2S_INLINK_DSCR_BF1_V  0xFFFFFFFF
#define I2S_INLINK_DSCR_BF1_S  0

/* I2S_OUTLINK_DSCR_REG register
 * The address of current outlink descriptor
 */

#define I2S_OUTLINK_DSCR_REG(i) (REG_I2S_BASE(i) + 0x54)

/* I2S_OUTLINK_DSCR : RO; bitpos: [31:0]; default: 0;
 * The address of current outlink descriptor
 */

#define I2S_OUTLINK_DSCR    0xFFFFFFFF
#define I2S_OUTLINK_DSCR_M  (I2S_OUTLINK_DSCR_V << I2S_OUTLINK_DSCR_S)
#define I2S_OUTLINK_DSCR_V  0xFFFFFFFF
#define I2S_OUTLINK_DSCR_S  0

/* I2S_OUTLINK_DSCR_BF0_REG register
 * The address of next outlink descriptor
 */

#define I2S_OUTLINK_DSCR_BF0_REG(i) (REG_I2S_BASE(i) + 0x58)

/* I2S_OUTLINK_DSCR_BF0 : RO; bitpos: [31:0]; default: 0;
 * The address of next outlink descriptor
 */

#define I2S_OUTLINK_DSCR_BF0    0xFFFFFFFF
#define I2S_OUTLINK_DSCR_BF0_M  (I2S_OUTLINK_DSCR_BF0_V << I2S_OUTLINK_DSCR_BF0_S)
#define I2S_OUTLINK_DSCR_BF0_V  0xFFFFFFFF
#define I2S_OUTLINK_DSCR_BF0_S  0

/* I2S_OUTLINK_DSCR_BF1_REG register
 * The address of next outlink data buffer
 */

#define I2S_OUTLINK_DSCR_BF1_REG(i) (REG_I2S_BASE(i) + 0x5c)

/* I2S_OUTLINK_DSCR_BF1 : RO; bitpos: [31:0]; default: 0;
 * The address of next outlink data buffer
 */

#define I2S_OUTLINK_DSCR_BF1    0xFFFFFFFF
#define I2S_OUTLINK_DSCR_BF1_M  (I2S_OUTLINK_DSCR_BF1_V << I2S_OUTLINK_DSCR_BF1_S)
#define I2S_OUTLINK_DSCR_BF1_V  0xFFFFFFFF
#define I2S_OUTLINK_DSCR_BF1_S  0

/* I2S_LC_CONF_REG register
 * I2S DMA configuration register
 */

#define I2S_LC_CONF_REG(i) (REG_I2S_BASE(i) + 0x60)

/* I2S_EXT_MEM_BK_SIZE : R/W; bitpos: [15:14]; default: 0;
 * DMA access external memory block size. 0: 16 bytes      1: 32 bytes
 * 2:64 bytes      3:reserved
 */

#define I2S_EXT_MEM_BK_SIZE    0x00000003
#define I2S_EXT_MEM_BK_SIZE_M  (I2S_EXT_MEM_BK_SIZE_V << I2S_EXT_MEM_BK_SIZE_S)
#define I2S_EXT_MEM_BK_SIZE_V  0x00000003
#define I2S_EXT_MEM_BK_SIZE_S  14

/* I2S_MEM_TRANS_EN : R/W; bitpos: [13]; default: 0;
 * don't use
 */

#define I2S_MEM_TRANS_EN    (BIT(13))
#define I2S_MEM_TRANS_EN_M  (I2S_MEM_TRANS_EN_V << I2S_MEM_TRANS_EN_S)
#define I2S_MEM_TRANS_EN_V  0x00000001
#define I2S_MEM_TRANS_EN_S  13

/* I2S_CHECK_OWNER : R/W; bitpos: [12]; default: 0;
 * Set this bit to enable check owner bit by hardware
 */

#define I2S_CHECK_OWNER    (BIT(12))
#define I2S_CHECK_OWNER_M  (I2S_CHECK_OWNER_V << I2S_CHECK_OWNER_S)
#define I2S_CHECK_OWNER_V  0x00000001
#define I2S_CHECK_OWNER_S  12

/* I2S_OUT_DATA_BURST_EN : R/W; bitpos: [11]; default: 0;
 * Transmitter data transfer mode configuration bit. 1:  to prepare out data
 * with burst mode      0: to prepare out data with byte mode
 */

#define I2S_OUT_DATA_BURST_EN    (BIT(11))
#define I2S_OUT_DATA_BURST_EN_M  (I2S_OUT_DATA_BURST_EN_V << I2S_OUT_DATA_BURST_EN_S)
#define I2S_OUT_DATA_BURST_EN_V  0x00000001
#define I2S_OUT_DATA_BURST_EN_S  11

/* I2S_INDSCR_BURST_EN : R/W; bitpos: [10]; default: 0;
 * DMA inlink descriptor transfer mode configuration bit. 1:  to prepare
 * inlink descriptor with burst mode    0: to prepare inlink descriptor with
 * byte mode
 */

#define I2S_INDSCR_BURST_EN    (BIT(10))
#define I2S_INDSCR_BURST_EN_M  (I2S_INDSCR_BURST_EN_V << I2S_INDSCR_BURST_EN_S)
#define I2S_INDSCR_BURST_EN_V  0x00000001
#define I2S_INDSCR_BURST_EN_S  10

/* I2S_OUTDSCR_BURST_EN : R/W; bitpos: [9]; default: 0;
 * DMA outlink descriptor transfer mode configuration bit. 1:  to prepare
 * outlink descriptor with burst mode    0: to prepare outlink descriptor
 * with byte mode
 */

#define I2S_OUTDSCR_BURST_EN    (BIT(9))
#define I2S_OUTDSCR_BURST_EN_M  (I2S_OUTDSCR_BURST_EN_V << I2S_OUTDSCR_BURST_EN_S)
#define I2S_OUTDSCR_BURST_EN_V  0x00000001
#define I2S_OUTDSCR_BURST_EN_S  9

/* I2S_OUT_EOF_MODE : R/W; bitpos: [8]; default: 1;
 * DMA out EOF flag generation mode . 1: when dma has popped all data from
 * the FIFO  0:when ahb has pushed all data to the FIFO
 */

#define I2S_OUT_EOF_MODE    (BIT(8))
#define I2S_OUT_EOF_MODE_M  (I2S_OUT_EOF_MODE_V << I2S_OUT_EOF_MODE_S)
#define I2S_OUT_EOF_MODE_V  0x00000001
#define I2S_OUT_EOF_MODE_S  8

/* I2S_OUT_NO_RESTART_CLR : R/W; bitpos: [7]; default: 0;
 * don't use
 */

#define I2S_OUT_NO_RESTART_CLR    (BIT(7))
#define I2S_OUT_NO_RESTART_CLR_M  (I2S_OUT_NO_RESTART_CLR_V << I2S_OUT_NO_RESTART_CLR_S)
#define I2S_OUT_NO_RESTART_CLR_V  0x00000001
#define I2S_OUT_NO_RESTART_CLR_S  7

/* I2S_OUT_AUTO_WRBACK : R/W; bitpos: [6]; default: 0;
 * Set this bit to enable outlink-written-back automatically when out buffer
 * is transmitted done.
 */

#define I2S_OUT_AUTO_WRBACK    (BIT(6))
#define I2S_OUT_AUTO_WRBACK_M  (I2S_OUT_AUTO_WRBACK_V << I2S_OUT_AUTO_WRBACK_S)
#define I2S_OUT_AUTO_WRBACK_V  0x00000001
#define I2S_OUT_AUTO_WRBACK_S  6

/* I2S_OUT_LOOP_TEST : R/W; bitpos: [5]; default: 0;
 * Set this bit to loop test outlink
 */

#define I2S_OUT_LOOP_TEST    (BIT(5))
#define I2S_OUT_LOOP_TEST_M  (I2S_OUT_LOOP_TEST_V << I2S_OUT_LOOP_TEST_S)
#define I2S_OUT_LOOP_TEST_V  0x00000001
#define I2S_OUT_LOOP_TEST_S  5

/* I2S_IN_LOOP_TEST : R/W; bitpos: [4]; default: 0;
 * Set this bit to loop test inlink
 */

#define I2S_IN_LOOP_TEST    (BIT(4))
#define I2S_IN_LOOP_TEST_M  (I2S_IN_LOOP_TEST_V << I2S_IN_LOOP_TEST_S)
#define I2S_IN_LOOP_TEST_V  0x00000001
#define I2S_IN_LOOP_TEST_S  4

/* I2S_AHBM_RST : R/W; bitpos: [3]; default: 0;
 * Set this bit to reset ahb interface of DMA
 */

#define I2S_AHBM_RST    (BIT(3))
#define I2S_AHBM_RST_M  (I2S_AHBM_RST_V << I2S_AHBM_RST_S)
#define I2S_AHBM_RST_V  0x00000001
#define I2S_AHBM_RST_S  3

/* I2S_AHBM_FIFO_RST : R/W; bitpos: [2]; default: 0;
 * Set this bit to reset ahb interface cmdFIFO of DMA
 */

#define I2S_AHBM_FIFO_RST    (BIT(2))
#define I2S_AHBM_FIFO_RST_M  (I2S_AHBM_FIFO_RST_V << I2S_AHBM_FIFO_RST_S)
#define I2S_AHBM_FIFO_RST_V  0x00000001
#define I2S_AHBM_FIFO_RST_S  2

/* I2S_OUT_RST : R/W; bitpos: [1]; default: 0;
 * Set this bit to reset out dma FSM
 */

#define I2S_OUT_RST    (BIT(1))
#define I2S_OUT_RST_M  (I2S_OUT_RST_V << I2S_OUT_RST_S)
#define I2S_OUT_RST_V  0x00000001
#define I2S_OUT_RST_S  1

/* I2S_IN_RST : R/W; bitpos: [0]; default: 0;
 * Set this bit to reset in dma FSM
 */

#define I2S_IN_RST    (BIT(0))
#define I2S_IN_RST_M  (I2S_IN_RST_V << I2S_IN_RST_S)
#define I2S_IN_RST_V  0x00000001
#define I2S_IN_RST_S  0

/* I2S_LC_STATE0_REG register
 * I2S DMA TX status
 */

#define I2S_LC_STATE0_REG(i) (REG_I2S_BASE(i) + 0x6c)

/* I2S_OUT_EMPTY : RO; bitpos: [31]; default: 0;
 * I2S DMA outfifo is empty.
 */

#define I2S_OUT_EMPTY    (BIT(31))
#define I2S_OUT_EMPTY_M  (I2S_OUT_EMPTY_V << I2S_OUT_EMPTY_S)
#define I2S_OUT_EMPTY_V  0x00000001
#define I2S_OUT_EMPTY_S  31

/* I2S_OUT_FULL : RO; bitpos: [30]; default: 0;
 * I2S DMA outfifo is full.
 */

#define I2S_OUT_FULL    (BIT(30))
#define I2S_OUT_FULL_M  (I2S_OUT_FULL_V << I2S_OUT_FULL_S)
#define I2S_OUT_FULL_V  0x00000001
#define I2S_OUT_FULL_S  30

/* I2S_OUTFIFO_CNT : RO; bitpos: [29:23]; default: 0;
 * The remains of I2S DMA outfifo data.
 */

#define I2S_OUTFIFO_CNT    0x0000007F
#define I2S_OUTFIFO_CNT_M  (I2S_OUTFIFO_CNT_V << I2S_OUTFIFO_CNT_S)
#define I2S_OUTFIFO_CNT_V  0x0000007F
#define I2S_OUTFIFO_CNT_S  23

/* I2S_OUT_STATE : RO; bitpos: [22:20]; default: 0;
 * I2S DMA out data state.
 */

#define I2S_OUT_STATE    0x00000007
#define I2S_OUT_STATE_M  (I2S_OUT_STATE_V << I2S_OUT_STATE_S)
#define I2S_OUT_STATE_V  0x00000007
#define I2S_OUT_STATE_S  20

/* I2S_OUT_DSCR_STATE : RO; bitpos: [19:18]; default: 0;
 * I2S DMA out descriptor state.
 */

#define I2S_OUT_DSCR_STATE    0x00000003
#define I2S_OUT_DSCR_STATE_M  (I2S_OUT_DSCR_STATE_V << I2S_OUT_DSCR_STATE_S)
#define I2S_OUT_DSCR_STATE_V  0x00000003
#define I2S_OUT_DSCR_STATE_S  18

/* I2S_OUTLINK_DSCR_ADDR : RO; bitpos: [17:0]; default: 0;
 * I2S DMA out descriptor address.
 */

#define I2S_OUTLINK_DSCR_ADDR    0x0003FFFF
#define I2S_OUTLINK_DSCR_ADDR_M  (I2S_OUTLINK_DSCR_ADDR_V << I2S_OUTLINK_DSCR_ADDR_S)
#define I2S_OUTLINK_DSCR_ADDR_V  0x0003FFFF
#define I2S_OUTLINK_DSCR_ADDR_S  0

/* I2S_LC_STATE1_REG register
 * I2S DMA RX status
 */

#define I2S_LC_STATE1_REG(i) (REG_I2S_BASE(i) + 0x70)

/* I2S_IN_EMPTY : RO; bitpos: [31]; default: 0;
 * I2S DMA infifo is empty.
 */

#define I2S_IN_EMPTY    (BIT(31))
#define I2S_IN_EMPTY_M  (I2S_IN_EMPTY_V << I2S_IN_EMPTY_S)
#define I2S_IN_EMPTY_V  0x00000001
#define I2S_IN_EMPTY_S  31

/* I2S_IN_FULL : RO; bitpos: [30]; default: 0;
 * I2S DMA infifo is full.
 */

#define I2S_IN_FULL    (BIT(30))
#define I2S_IN_FULL_M  (I2S_IN_FULL_V << I2S_IN_FULL_S)
#define I2S_IN_FULL_V  0x00000001
#define I2S_IN_FULL_S  30

/* I2S_INFIFO_CNT_DEBUG : RO; bitpos: [29:23]; default: 0;
 * The remains of I2S DMA infifo data.
 */

#define I2S_INFIFO_CNT_DEBUG    0x0000007F
#define I2S_INFIFO_CNT_DEBUG_M  (I2S_INFIFO_CNT_DEBUG_V << I2S_INFIFO_CNT_DEBUG_S)
#define I2S_INFIFO_CNT_DEBUG_V  0x0000007F
#define I2S_INFIFO_CNT_DEBUG_S  23

/* I2S_IN_STATE : RO; bitpos: [22:20]; default: 0;
 * I2S DMA in data state.
 */

#define I2S_IN_STATE    0x00000007
#define I2S_IN_STATE_M  (I2S_IN_STATE_V << I2S_IN_STATE_S)
#define I2S_IN_STATE_V  0x00000007
#define I2S_IN_STATE_S  20

/* I2S_IN_DSCR_STATE : RO; bitpos: [19:18]; default: 0;
 * I2S DMA in descriptor state.
 */

#define I2S_IN_DSCR_STATE    0x00000003
#define I2S_IN_DSCR_STATE_M  (I2S_IN_DSCR_STATE_V << I2S_IN_DSCR_STATE_S)
#define I2S_IN_DSCR_STATE_V  0x00000003
#define I2S_IN_DSCR_STATE_S  18

/* I2S_INLINK_DSCR_ADDR : RO; bitpos: [17:0]; default: 0;
 * I2S DMA in descriptor address.
 */

#define I2S_INLINK_DSCR_ADDR    0x0003FFFF
#define I2S_INLINK_DSCR_ADDR_M  (I2S_INLINK_DSCR_ADDR_V << I2S_INLINK_DSCR_ADDR_S)
#define I2S_INLINK_DSCR_ADDR_V  0x0003FFFF
#define I2S_INLINK_DSCR_ADDR_S  0

/* I2S_LC_HUNG_CONF_REG register
 * I2S Hung configure register
 */

#define I2S_LC_HUNG_CONF_REG(i) (REG_I2S_BASE(i) + 0x74)

/* I2S_LC_FIFO_TIMEOUT_ENA : R/W; bitpos: [11]; default: 1;
 * The enable bit for FIFO timeout
 */

#define I2S_LC_FIFO_TIMEOUT_ENA    (BIT(11))
#define I2S_LC_FIFO_TIMEOUT_ENA_M  (I2S_LC_FIFO_TIMEOUT_ENA_V << I2S_LC_FIFO_TIMEOUT_ENA_S)
#define I2S_LC_FIFO_TIMEOUT_ENA_V  0x00000001
#define I2S_LC_FIFO_TIMEOUT_ENA_S  11

/* I2S_LC_FIFO_TIMEOUT_SHIFT : R/W; bitpos: [10:8]; default: 0;
 * The bits are used to scale tick counter threshold. The tick counter is
 * reset when counter value >= 88000/2^i2s_lc_fifo_timeout_shift
 */

#define I2S_LC_FIFO_TIMEOUT_SHIFT    0x00000007
#define I2S_LC_FIFO_TIMEOUT_SHIFT_M  (I2S_LC_FIFO_TIMEOUT_SHIFT_V << I2S_LC_FIFO_TIMEOUT_SHIFT_S)
#define I2S_LC_FIFO_TIMEOUT_SHIFT_V  0x00000007
#define I2S_LC_FIFO_TIMEOUT_SHIFT_S  8

/* I2S_LC_FIFO_TIMEOUT : R/W; bitpos: [7:0]; default: 16;
 * the i2s_tx_hung_int interrupt or the i2s_rx_hung_int interrupt will be
 * triggered when fifo hung counter is equal to this value
 */

#define I2S_LC_FIFO_TIMEOUT    0x000000FF
#define I2S_LC_FIFO_TIMEOUT_M  (I2S_LC_FIFO_TIMEOUT_V << I2S_LC_FIFO_TIMEOUT_S)
#define I2S_LC_FIFO_TIMEOUT_V  0x000000FF
#define I2S_LC_FIFO_TIMEOUT_S  0

/* I2S_CONF1_REG register
 * I2S configure1 register
 */

#define I2S_CONF1_REG(i) (REG_I2S_BASE(i) + 0xa0)

/* I2S_TX_ZEROS_RM_EN : R/W; bitpos: [9]; default: 0;
 * don't use
 */

#define I2S_TX_ZEROS_RM_EN    (BIT(9))
#define I2S_TX_ZEROS_RM_EN_M  (I2S_TX_ZEROS_RM_EN_V << I2S_TX_ZEROS_RM_EN_S)
#define I2S_TX_ZEROS_RM_EN_V  0x00000001
#define I2S_TX_ZEROS_RM_EN_S  9

/* I2S_TX_STOP_EN : R/W; bitpos: [8]; default: 0;
 * Set this bit to stop disable output BCK signal and WS signal when tx FIFO
 * is emtpy
 */

#define I2S_TX_STOP_EN    (BIT(8))
#define I2S_TX_STOP_EN_M  (I2S_TX_STOP_EN_V << I2S_TX_STOP_EN_S)
#define I2S_TX_STOP_EN_V  0x00000001
#define I2S_TX_STOP_EN_S  8

/* I2S_RX_PCM_BYPASS : R/W; bitpos: [7]; default: 1;
 * Set this bit to bypass Compress/Decompress module for received data.
 */

#define I2S_RX_PCM_BYPASS    (BIT(7))
#define I2S_RX_PCM_BYPASS_M  (I2S_RX_PCM_BYPASS_V << I2S_RX_PCM_BYPASS_S)
#define I2S_RX_PCM_BYPASS_V  0x00000001
#define I2S_RX_PCM_BYPASS_S  7

/* I2S_RX_PCM_CONF : R/W; bitpos: [6:4]; default: 0;
 * Compress/Decompress module configuration bits. 0: decompress received
 * data  1:compress received data
 */

#define I2S_RX_PCM_CONF    0x00000007
#define I2S_RX_PCM_CONF_M  (I2S_RX_PCM_CONF_V << I2S_RX_PCM_CONF_S)
#define I2S_RX_PCM_CONF_V  0x00000007
#define I2S_RX_PCM_CONF_S  4

/* I2S_TX_PCM_BYPASS : R/W; bitpos: [3]; default: 1;
 * Set this bit to bypass  Compress/Decompress module for transmitted data.
 */

#define I2S_TX_PCM_BYPASS    (BIT(3))
#define I2S_TX_PCM_BYPASS_M  (I2S_TX_PCM_BYPASS_V << I2S_TX_PCM_BYPASS_S)
#define I2S_TX_PCM_BYPASS_V  0x00000001
#define I2S_TX_PCM_BYPASS_S  3

/* I2S_TX_PCM_CONF : R/W; bitpos: [2:0]; default: 1;
 * Compress/Decompress module configuration bits. 0: decompress transmitted
 * data  1:compress transmitted data
 */

#define I2S_TX_PCM_CONF    0x00000007
#define I2S_TX_PCM_CONF_M  (I2S_TX_PCM_CONF_V << I2S_TX_PCM_CONF_S)
#define I2S_TX_PCM_CONF_V  0x00000007
#define I2S_TX_PCM_CONF_S  0

/* I2S_PD_CONF_REG register
 * I2S power down configure register
 */

#define I2S_PD_CONF_REG(i) (REG_I2S_BASE(i) + 0xa4)

/* I2S_PLC_MEM_FORCE_PU : R/W; bitpos: [3]; default: 1;
 * Force I2S memory power-up
 */

#define I2S_PLC_MEM_FORCE_PU    (BIT(3))
#define I2S_PLC_MEM_FORCE_PU_M  (I2S_PLC_MEM_FORCE_PU_V << I2S_PLC_MEM_FORCE_PU_S)
#define I2S_PLC_MEM_FORCE_PU_V  0x00000001
#define I2S_PLC_MEM_FORCE_PU_S  3

/* I2S_PLC_MEM_FORCE_PD : R/W; bitpos: [2]; default: 0;
 * Force I2S memory power-down
 */

#define I2S_PLC_MEM_FORCE_PD    (BIT(2))
#define I2S_PLC_MEM_FORCE_PD_M  (I2S_PLC_MEM_FORCE_PD_V << I2S_PLC_MEM_FORCE_PD_S)
#define I2S_PLC_MEM_FORCE_PD_V  0x00000001
#define I2S_PLC_MEM_FORCE_PD_S  2

/* I2S_FIFO_FORCE_PU : R/W; bitpos: [1]; default: 1;
 * Force FIFO power-up
 */

#define I2S_FIFO_FORCE_PU    (BIT(1))
#define I2S_FIFO_FORCE_PU_M  (I2S_FIFO_FORCE_PU_V << I2S_FIFO_FORCE_PU_S)
#define I2S_FIFO_FORCE_PU_V  0x00000001
#define I2S_FIFO_FORCE_PU_S  1

/* I2S_FIFO_FORCE_PD : R/W; bitpos: [0]; default: 0;
 * Force FIFO power-down
 */

#define I2S_FIFO_FORCE_PD    (BIT(0))
#define I2S_FIFO_FORCE_PD_M  (I2S_FIFO_FORCE_PD_V << I2S_FIFO_FORCE_PD_S)
#define I2S_FIFO_FORCE_PD_V  0x00000001
#define I2S_FIFO_FORCE_PD_S  0

/* I2S_CONF2_REG register
 * I2S configure2 register
 */

#define I2S_CONF2_REG(i) (REG_I2S_BASE(i) + 0xa8)

/* I2S_INTER_VALID_EN : R/W; bitpos: [7]; default: 0;
 * Set this bit to enable camera internal valid
 */

#define I2S_INTER_VALID_EN    (BIT(7))
#define I2S_INTER_VALID_EN_M  (I2S_INTER_VALID_EN_V << I2S_INTER_VALID_EN_S)
#define I2S_INTER_VALID_EN_V  0x00000001
#define I2S_INTER_VALID_EN_S  7

/* I2S_EXT_ADC_START_EN : R/W; bitpos: [6]; default: 0;
 * Set this bit to enable the function that ADC mode is triggered by
 * external signal.
 */

#define I2S_EXT_ADC_START_EN    (BIT(6))
#define I2S_EXT_ADC_START_EN_M  (I2S_EXT_ADC_START_EN_V << I2S_EXT_ADC_START_EN_S)
#define I2S_EXT_ADC_START_EN_V  0x00000001
#define I2S_EXT_ADC_START_EN_S  6

/* I2S_LCD_EN : R/W; bitpos: [5]; default: 0;
 * Set this bit to enable LCD mode
 */

#define I2S_LCD_EN    (BIT(5))
#define I2S_LCD_EN_M  (I2S_LCD_EN_V << I2S_LCD_EN_S)
#define I2S_LCD_EN_V  0x00000001
#define I2S_LCD_EN_S  5

/* I2S_LCD_TX_SDX2_EN : R/W; bitpos: [2]; default: 0;
 * Set this bit to duplicate data pairs (Frame Form 2) in LCD mode.
 */

#define I2S_LCD_TX_SDX2_EN    (BIT(2))
#define I2S_LCD_TX_SDX2_EN_M  (I2S_LCD_TX_SDX2_EN_V << I2S_LCD_TX_SDX2_EN_S)
#define I2S_LCD_TX_SDX2_EN_V  0x00000001
#define I2S_LCD_TX_SDX2_EN_S  2

/* I2S_LCD_TX_WRX2_EN : R/W; bitpos: [1]; default: 0;
 * LCD WR double for one datum.
 */

#define I2S_LCD_TX_WRX2_EN    (BIT(1))
#define I2S_LCD_TX_WRX2_EN_M  (I2S_LCD_TX_WRX2_EN_V << I2S_LCD_TX_WRX2_EN_S)
#define I2S_LCD_TX_WRX2_EN_V  0x00000001
#define I2S_LCD_TX_WRX2_EN_S  1

/* I2S_CAMERA_EN : R/W; bitpos: [0]; default: 0;
 * Set this bit to enable camera mode
 */

#define I2S_CAMERA_EN    (BIT(0))
#define I2S_CAMERA_EN_M  (I2S_CAMERA_EN_V << I2S_CAMERA_EN_S)
#define I2S_CAMERA_EN_V  0x00000001
#define I2S_CAMERA_EN_S  0

/* I2S_CLKM_CONF_REG register
 * I2S module clock configure register
 */

#define I2S_CLKM_CONF_REG(i) (REG_I2S_BASE(i) + 0xac)

/* I2S_CLKA_ENA : R/W; bitpos: [21]; default: 0;
 * Set this bit to enable clk_apll.
 */

#define I2S_CLKA_ENA    (BIT(21))
#define I2S_CLKA_ENA_M  (I2S_CLKA_ENA_V << I2S_CLKA_ENA_S)
#define I2S_CLKA_ENA_V  0x00000001
#define I2S_CLKA_ENA_S  21

/* I2S_CLK_ENA : R/W; bitpos: [21]; default: 0;
 * Set this bit to enable clk_apll.
 */

#define I2S_CLK_ENA    (BIT(20))
#define I2S_CLK_ENA_M  (I2S_CLK_ENA_V << I2S_CLK_ENA_S)
#define I2S_CLK_ENA_V  0x00000001
#define I2S_CLK_ENA_S  20

/* I2S_CLKM_DIV_A : R/W; bitpos: [19:14]; default: 0;
 * Fractional clock divider denominator value
 */

#define I2S_CLKM_DIV_A    0x0000003F
#define I2S_CLKM_DIV_A_M  (I2S_CLKM_DIV_A_V << I2S_CLKM_DIV_A_S)
#define I2S_CLKM_DIV_A_V  0x0000003F
#define I2S_CLKM_DIV_A_S  14

/* I2S_CLKM_DIV_B : R/W; bitpos: [13:8]; default: 0;
 * Fractional clock divider numerator value
 */

#define I2S_CLKM_DIV_B    0x0000003F
#define I2S_CLKM_DIV_B_M  (I2S_CLKM_DIV_B_V << I2S_CLKM_DIV_B_S)
#define I2S_CLKM_DIV_B_V  0x0000003F
#define I2S_CLKM_DIV_B_S  8

/* I2S_CLKM_DIV_NUM : R/W; bitpos: [7:0]; default: 4;
 * Integral I2S clock divider value
 */

#define I2S_CLKM_DIV_NUM    0x000000FF
#define I2S_CLKM_DIV_NUM_M  (I2S_CLKM_DIV_NUM_V << I2S_CLKM_DIV_NUM_S)
#define I2S_CLKM_DIV_NUM_V  0x000000FF
#define I2S_CLKM_DIV_NUM_S  0

/* I2S_SAMPLE_RATE_CONF_REG register
 * I2S sample rate register
 */

#define I2S_SAMPLE_RATE_CONF_REG(i) (REG_I2S_BASE(i) + 0xb0)

/* I2S_RX_BITS_MOD : R/W; bitpos: [23:18]; default: 16;
 * Set the bits to configure bit length of I2S receiver channel.
 */

#define I2S_RX_BITS_MOD    0x0000003F
#define I2S_RX_BITS_MOD_M  (I2S_RX_BITS_MOD_V << I2S_RX_BITS_MOD_S)
#define I2S_RX_BITS_MOD_V  0x0000003F
#define I2S_RX_BITS_MOD_S  18

/* I2S_TX_BITS_MOD : R/W; bitpos: [17:12]; default: 16;
 * Set the bits to configure bit length of I2S transmitter channel.
 */

#define I2S_TX_BITS_MOD    0x0000003F
#define I2S_TX_BITS_MOD_M  (I2S_TX_BITS_MOD_V << I2S_TX_BITS_MOD_S)
#define I2S_TX_BITS_MOD_V  0x0000003F
#define I2S_TX_BITS_MOD_S  12

/* I2S_RX_BCK_DIV_NUM : R/W; bitpos: [11:6]; default: 6;
 * Bit clock configuration bits in receiver mode.
 */

#define I2S_RX_BCK_DIV_NUM    0x0000003F
#define I2S_RX_BCK_DIV_NUM_M  (I2S_RX_BCK_DIV_NUM_V << I2S_RX_BCK_DIV_NUM_S)
#define I2S_RX_BCK_DIV_NUM_V  0x0000003F
#define I2S_RX_BCK_DIV_NUM_S  6

/* I2S_TX_BCK_DIV_NUM : R/W; bitpos: [5:0]; default: 6;
 * Bit clock configuration bits in transmitter mode.
 */

#define I2S_TX_BCK_DIV_NUM    0x0000003F
#define I2S_TX_BCK_DIV_NUM_M  (I2S_TX_BCK_DIV_NUM_V << I2S_TX_BCK_DIV_NUM_S)
#define I2S_TX_BCK_DIV_NUM_V  0x0000003F
#define I2S_TX_BCK_DIV_NUM_S  0

/* I2S_PDM_CONF_REG register
 * I2S PDM configuration register
 */

#define I2S_PDM_CONF_REG(i) (REG_I2S_BASE(i) + 0xb4)

/* I2S_TX_PDM_HP_BYPASS : R/W; bitpos: [25]; default: 0;
 * Set this bit to bypass the transmitter's PDM HP filter.
 */

#define I2S_TX_PDM_HP_BYPASS    BIT(25)
#define I2S_TX_PDM_HP_BYPASS_M  (I2S_TX_PDM_HP_BYPASS_V << I2S_TX_PDM_HP_BYPASS_S)
#define I2S_TX_PDM_HP_BYPASS_V  0x00000001
#define I2S_TX_PDM_HP_BYPASS_S  25

/* I2S_RX_PDM_SINC_DSR_16_EN : R/W; bitpos: [24]; default: 1;
 * PDM downsampling rate for filter group 1 in receiver mode.
 * 1: downsampling rate = 128; 0: downsampling rate = 64
 */

#define I2S_RX_PDM_SINC_DSR_16_EN   BIT(24)
#define I2S_RX_PDM_SINC_DSR_16_EN_M (I2S_RX_PDM_SINC_DSR_16_EN_V << I2S_RX_PDM_SINC_DSR_16_EN_S)
#define I2S_RX_PDM_SINC_DSR_16_EN_V 0x00000001
#define I2S_RX_PDM_SINC_DSR_16_EN_S 24

/* I2S_TX_PDM_SIGMADELTA_IN_SHIFT : R/W; bitpos: [23:22]; default: 1;
 * Adjust the size of the input signal into filter module.
 * 0: divided by 2; 1: multiplied by 1;
 * 2: multiplied by 2; 3: multiplied by 4.
 */

#define I2S_TX_PDM_SIGMADELTA_IN_SHIFT      0x00000003
#define I2S_TX_PDM_SIGMADELTA_IN_SHIFT_M    (I2S_TX_PDM_SIGMADELTA_IN_SHIFT_V << I2S_TX_PDM_SIGMADELTA_IN_SHIFT_S)
#define I2S_TX_PDM_SIGMADELTA_IN_SHIFT_V    0x00000003
#define I2S_TX_PDM_SIGMADELTA_IN_SHIFT_S    22

/* I2S_TX_PDM_SINC_IN_SHIFT : R/W; bitpos: [21:20]; default: 1;
 * Adjust the size of the input signal into filter module.
 * 0: divided by 2; 1: multiplied by 1;
 * 2: multiplied by 2; 3: multiplied by 4.
 */

#define I2S_TX_PDM_SINC_IN_SHIFT    0x00000003
#define I2S_TX_PDM_SINC_IN_SHIFT_M  (I2S_TX_PDM_SINC_IN_SHIFT_V << I2S_TX_PDM_SINC_IN_SHIFT_S)
#define I2S_TX_PDM_SINC_IN_SHIFT_V  0x00000003
#define I2S_TX_PDM_SINC_IN_SHIFT_S  20

/* I2S_TX_PDM_LP_IN_SHIFT : R/W; bitpos: [19:18]; default: 1;
 * Adjust the size of the input signal into filter module.
 * 0: divided by 2; 1: multiplied by 1;
 * 2: multiplied by 2; 3: multiplied by 4.
 */

#define I2S_TX_PDM_LP_IN_SHIFT      0x00000003
#define I2S_TX_PDM_LP_IN_SHIFT_M    (I2S_TX_PDM_LP_IN_SHIFT_V << I2S_TX_PDM_LP_IN_SHIFT_S)
#define I2S_TX_PDM_LP_IN_SHIFT_V    0x00000003
#define I2S_TX_PDM_LP_IN_SHIFT_S    18

/* I2S_TX_PDM_HP_IN_SHIFT : R/W; bitpos: [17:16]; default: 1;
 * Adjust the size of the input signal into filter module.
 * 0: divided by 2; 1: multiplied by 1;
 * 2: multiplied by 2; 3: multiplied by 4.
 */

#define I2S_TX_PDM_HP_IN_SHIFT      0x00000003
#define I2S_TX_PDM_HP_IN_SHIFT_M    (I2S_TX_PDM_HP_IN_SHIFT_V << I2S_TX_PDM_HP_IN_SHIFT_S)
#define I2S_TX_PDM_HP_IN_SHIFT_V    0x00000003
#define I2S_TX_PDM_HP_IN_SHIFT_S    16

/* I2S_TX_PDM_SINC_OSR2 : R/W; bitpos: [7:4]; default: 2;
 * Upsampling rate = 64×i2s_tx_pdm_sinc_osr2
 */

#define I2S_TX_PDM_SINC_OSR2    0x0000000F
#define I2S_TX_PDM_SINC_OSR2_M  (I2S_TX_PDM_SINC_OSR2_V << I2S_TX_PDM_SINC_OSR2_S)
#define I2S_TX_PDM_SINC_OSR2_V  0x0000000F
#define I2S_TX_PDM_SINC_OSR2_S  4

/* I2S_PDM2PCM_CONV_EN : R/W; bitpos: [3]; default: 1;
 * Set this bit to enable PDM-to-PCM converter.
 */

#define I2S_PDM2PCM_CONV_EN     BIT(3)
#define I2S_PDM2PCM_CONV_EN_M   (I2S_PDM2PCM_CONV_EN_V << I2S_PDM2PCM_CONV_EN_S)
#define I2S_PDM2PCM_CONV_EN_V   0x00000001
#define I2S_PDM2PCM_CONV_EN_S   3

/* I2S_PCM2PDM_CONV_EN : R/W; bitpos: [2]; default: 1;
 * Set this bit to enable PCM-to-PDM converter.
 */

#define I2S_PCM2PDM_CONV_EN     BIT(2)
#define I2S_PCM2PDM_CONV_EN_M   (I2S_PCM2PDM_CONV_EN_V << I2S_PCM2PDM_CONV_EN_S)
#define I2S_PCM2PDM_CONV_EN_V   0x00000001
#define I2S_PCM2PDM_CONV_EN_S   2

/* I2S_RX_PDM_EN : R/W; bitpos: [1]; default: 0;
 * Set this bit to enable receiver’s PDM mode.
 */

#define I2S_RX_PDM_EN       BIT(1)
#define I2S_RX_PDM_EN_M     (I2S_RX_PDM_EN_V << I2S_RX_PDM_EN_S)
#define I2S_RX_PDM_EN_V     0x00000001
#define I2S_RX_PDM_EN_S     1

/* I2S_TX_PDM_EN : R/W; bitpos: [0]; default: 0;
 * Set this bit to enable transmitter’s PDM mode.
 */

#define I2S_TX_PDM_EN       BIT(0)
#define I2S_TX_PDM_EN_M     (I2S_TX_PDM_EN_V << I2S_TX_PDM_EN_S)
#define I2S_TX_PDM_EN_V     0x00000001
#define I2S_TX_PDM_EN_S     0

/* I2S_PDM_FREQ_CONF_REG register
 * I2S PDM frequencies register
 */

#define I2S_PDM_FREQ_CONF_REG(i) (REG_I2S_BASE(i) + 0xb8)

/* I2S_TX_PDM_FP : R/W; bitpos: [19:10]; default: 960;
 * PCM-to-PDM converter’s PDM frequency parameter.
 */

#define I2S_TX_PDM_FP       0x000003FF
#define I2S_TX_PDM_FP_M     (I2S_TX_PDM_FP_V << I2S_TX_PDM_FP_S)
#define I2S_TX_PDM_FP_V     0x000003FF
#define I2S_TX_PDM_FP_S     10

/* I2S_TX_PDM_FS : R/W; bitpos: [9:0]; default: 441;
 * PCM-to-PDM converter’s PCM frequency parameter.
 */

#define I2S_TX_PDM_FS       0x000003FF
#define I2S_TX_PDM_FS_M     (I2S_TX_PDM_FS_V << I2S_TX_PDM_FS_S)
#define I2S_TX_PDM_FS_V     0x000003FF
#define I2S_TX_PDM_FS_S     0

/* I2S_STATE_REG register
 * I2S TX status register
 */

#define I2S_STATE_REG(i) (REG_I2S_BASE(i) + 0xbc)

/* I2S_RX_FIFO_RESET_BACK : RO; bitpos: [2]; default: 0;
 * This bit is used to confirm if the Rx FIFO reset is done.
 * 1: reset is not ready; 0: reset is ready.
 */

#define I2S_RX_FIFO_RESET_BACK      (BIT(2))
#define I2S_RX_FIFO_RESET_BACK_M    (I2S_RX_FIFO_RESET_BACK_V << I2S_RX_FIFO_RESET_BACK_S)
#define I2S_RX_FIFO_RESET_BACK_V    0x00000001
#define I2S_RX_FIFO_RESET_BACK_S    2

/* I2S_TX_FIFO_RESET_BACK : RO; bitpos: [1]; default: 0;
 * 1: I2S TX is in idle state. 0: I2S TX is busy.
 */

#define I2S_TX_FIFO_RESET_BACK      (BIT(1))
#define I2S_TX_FIFO_RESET_BACK_M    (I2S_TX_FIFO_RESET_BACK_V << I2S_TX_FIFO_RESET_BACK_S)
#define I2S_TX_FIFO_RESET_BACK_V    0x00000001
#define I2S_TX_FIFO_RESET_BACK_S    1

/* I2S_TX_IDLE : RO; bitpos: [0]; default: 1;
 * 1: I2S TX is in idle state. 0: I2S TX is busy.
 */

#define I2S_TX_IDLE    (BIT(0))
#define I2S_TX_IDLE_M  (I2S_TX_IDLE_V << I2S_TX_IDLE_S)
#define I2S_TX_IDLE_V  0x00000001
#define I2S_TX_IDLE_S  0

#endif /* __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_I2S_H */
