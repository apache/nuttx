/****************************************************************************
 * drivers/can/sja1000.h
 *
 * SJA1000 CAN driver based on esp32c3_twai.h
 *
 * License header retained from original source.
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

#ifndef __DRIVERS_CAN_SJA1000_H
#define __DRIVERS_CAN_SJA1000_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default interrupts - Exclude data overrun (bit[3]) and brp_div (bit[4])
 */

#define SJA1000_DEFAULT_INTERRUPTS 0xe7

/* Number of standard/extended filters */

#define SJA1000_NFILTERS           1

/* SJA1000_MODE_REG register
 * Mode Register
 */

#define SJA1000_MODE_REG         (0x00)

/* SJA1000_RX_FILTER_MODE : R/W; bitpos: [3]; default: 0;
 * This bit is used to configure the filter mode. 0: Dual filter mode; 1:
 * Single filter mode.
 */

#define SJA1000_RX_FILTER_MODE   (BIT(3))
#define SJA1000_RX_FILTER_MODE_M                                           \
  (SJA1000_RX_FILTER_MODE_V << SJA1000_RX_FILTER_MODE_S)
#define SJA1000_RX_FILTER_MODE_V 0x00000001
#define SJA1000_RX_FILTER_MODE_S 3

/* SJA1000_SELF_TEST_MODE : R/W; bitpos: [2]; default: 0;
 * 1: Self test mode. In this mode the TX nodes can perform a successful
 * transmission without receiving the acknowledge signal. This mode is often
 * used to test a single node with the self reception request command.
 */

#define SJA1000_SELF_TEST_MODE   (BIT(2))
#define SJA1000_SELF_TEST_MODE_M                                           \
  (SJA1000_SELF_TEST_MODE_V << SJA1000_SELF_TEST_MODE_S)
#define SJA1000_SELF_TEST_MODE_V 0x00000001
#define SJA1000_SELF_TEST_MODE_S 2

/* SJA1000_LISTEN_ONLY_MODE : R/W; bitpos: [1]; default: 0;
 * 1: Listen only mode. In this mode the nodes will only receive messages
 * from the bus, without generating the acknowledge signal nor updating the
 * RX error counter.
 */

#define SJA1000_LISTEN_ONLY_MODE (BIT(1))
#define SJA1000_LISTEN_ONLY_MODE_M                                         \
  (SJA1000_LISTEN_ONLY_MODE_V << SJA1000_LISTEN_ONLY_MODE_S)
#define SJA1000_LISTEN_ONLY_MODE_V 0x00000001
#define SJA1000_LISTEN_ONLY_MODE_S 1

/* SJA1000_RESET_MODE : R/W; bitpos: [0]; default: 1;
 * This bit is used to configure the operating mode of the SJA1000
 * Controller. 1: Reset mode; 0: Operating mode.
 */

#define SJA1000_RESET_MODE         (BIT(0))
#define SJA1000_RESET_MODE_M       (SJA1000_RESET_MODE_V << SJA1000_RESET_MODE_S)
#define SJA1000_RESET_MODE_V       0x00000001
#define SJA1000_RESET_MODE_S       0

/* SJA1000_CMD_REG register
 * Command Register
 */

#define SJA1000_CMD_REG          (0x01)

/* SJA1000_SELF_RX_REQ : WO; bitpos: [4]; default: 0;
 * Self reception request command. Set the bit to 1 to allow a message be
 * transmitted and received simultaneously.
 */

#define SJA1000_SELF_RX_REQ      (BIT(4))
#define SJA1000_SELF_RX_REQ_M                                              \
  (SJA1000_SELF_RX_REQ_V << SJA1000_SELF_RX_REQ_S)
#define SJA1000_SELF_RX_REQ_V 0x00000001
#define SJA1000_SELF_RX_REQ_S 4

/* SJA1000_CLR_OVERRUN : WO; bitpos: [3]; default: 0;
 * Set the bit to 1 to clear the data overrun status bit.
 */

#define SJA1000_CLR_OVERRUN      (BIT(3))
#define SJA1000_CLR_OVERRUN_M                                              \
  (SJA1000_CLR_OVERRUN_V << SJA1000_CLR_OVERRUN_S)
#define SJA1000_CLR_OVERRUN_V 0x00000001
#define SJA1000_CLR_OVERRUN_S 3

/* SJA1000_RELEASE_BUF : WO; bitpos: [2]; default: 0;
 * Set the bit to 1 to release the RX buffer.
 */

#define SJA1000_RELEASE_BUF      (BIT(2))
#define SJA1000_RELEASE_BUF_M                                              \
  (SJA1000_RELEASE_BUF_V << SJA1000_RELEASE_BUF_S)
#define SJA1000_RELEASE_BUF_V        0x00000001
#define SJA1000_RELEASE_BUF_S        2

/* SJA1000_ABORT_TX : WO; bitpos: [1]; default: 0;
 * Set the bit to 1 to cancel a pending transmission request.
 */

#define SJA1000_ABORT_TX             (BIT(1))
#define SJA1000_ABORT_TX_M           (SJA1000_ABORT_TX_V << SJA1000_ABORT_TX_S)
#define SJA1000_ABORT_TX_V           0x00000001
#define SJA1000_ABORT_TX_S           1

/* SJA1000_TX_REQ : WO; bitpos: [0]; default: 0;
 * Set the bit to 1 to allow the driving nodes start transmission.
 */

#define SJA1000_TX_REQ               (BIT(0))
#define SJA1000_TX_REQ_M             (SJA1000_TX_REQ_V << SJA1000_TX_REQ_S)
#define SJA1000_TX_REQ_V             0x00000001
#define SJA1000_TX_REQ_S             0

/* SJA1000_STATUS_REG register
 * Status register
 */

#define SJA1000_STATUS_REG       (0x02)

/* SJA1000_MISS_ST : RO; bitpos: [8]; default: 0;
 * This bit reflects whether the data packet in the RX FIFO is complete. 1:
 * The current packet is missing; 0: The current packet is complete
 */

#define SJA1000_MISS_ST              (BIT(8))
#define SJA1000_MISS_ST_M            (SJA1000_MISS_ST_V << SJA1000_MISS_ST_S)
#define SJA1000_MISS_ST_V            0x00000001
#define SJA1000_MISS_ST_S            8

/* SJA1000_BUS_OFF_ST : RO; bitpos: [7]; default: 0;
 * 1: In bus-off status, the SJA1000 Controller is no longer involved in bus
 * activities.
 */

#define SJA1000_BUS_OFF_ST           (BIT(7))
#define SJA1000_BUS_OFF_ST_M         (SJA1000_BUS_OFF_ST_V << SJA1000_BUS_OFF_ST_S)
#define SJA1000_BUS_OFF_ST_V         0x00000001
#define SJA1000_BUS_OFF_ST_S         7

/* SJA1000_ERR_ST : RO; bitpos: [6]; default: 0;
 * 1: At least one of the RX/TX error counter has reached or exceeded the
 * value set in register SJA1000_ERR_WARNING_LIMIT_REG.
 */

#define SJA1000_ERR_ST               (BIT(6))
#define SJA1000_ERR_ST_M             (SJA1000_ERR_ST_V << SJA1000_ERR_ST_S)
#define SJA1000_ERR_ST_V             0x00000001
#define SJA1000_ERR_ST_S             6

/* SJA1000_TX_ST : RO; bitpos: [5]; default: 0;
 * 1: The SJA1000 Controller is transmitting a message to the bus.
 */

#define SJA1000_TX_ST                (BIT(5))
#define SJA1000_TX_ST_M              (SJA1000_TX_ST_V << SJA1000_TX_ST_S)
#define SJA1000_TX_ST_V              0x00000001
#define SJA1000_TX_ST_S              5

/* SJA1000_RX_ST : RO; bitpos: [4]; default: 0;
 * 1: The SJA1000 Controller is receiving a message from the bus.
 */

#define SJA1000_RX_ST                (BIT(4))
#define SJA1000_RX_ST_M              (SJA1000_RX_ST_V << SJA1000_RX_ST_S)
#define SJA1000_RX_ST_V              0x00000001
#define SJA1000_RX_ST_S              4

/* SJA1000_TX_COMPLETE : RO; bitpos: [3]; default: 0;
 * 1: The SJA1000 controller has successfully received a packet from the
 * bus.
 */

#define SJA1000_TX_COMPLETE          (BIT(3))
#define SJA1000_TX_COMPLETE_M                                              \
  (SJA1000_TX_COMPLETE_V << SJA1000_TX_COMPLETE_S)
#define SJA1000_TX_COMPLETE_V         0x00000001
#define SJA1000_TX_COMPLETE_S         3

/* SJA1000_TX_BUF_ST : RO; bitpos: [2]; default: 0;
 * 1: The TX buffer is empty, the CPU may write a message into it.
 */

#define SJA1000_TX_BUF_ST             (BIT(2))
#define SJA1000_TX_BUF_ST_M           (SJA1000_TX_BUF_ST_V << SJA1000_TX_BUF_ST_S)
#define SJA1000_TX_BUF_ST_V           0x00000001
#define SJA1000_TX_BUF_ST_S           2

/* SJA1000_OVERRUN_ST : RO; bitpos: [1]; default: 0;
 * 1: The RX FIFO is full and data overrun has occurred.
 */

#define SJA1000_OVERRUN_ST            (BIT(1))
#define SJA1000_OVERRUN_ST_M          (SJA1000_OVERRUN_ST_V << SJA1000_OVERRUN_ST_S)
#define SJA1000_OVERRUN_ST_V          0x00000001
#define SJA1000_OVERRUN_ST_S          1

/* SJA1000_RX_BUF_ST : RO; bitpos: [0]; default: 0;
 * 1: The data in the RX buffer is not empty, with at least one received
 * data packet.
 */

#define SJA1000_RX_BUF_ST             (BIT(0))
#define SJA1000_RX_BUF_ST_M           (SJA1000_RX_BUF_ST_V << SJA1000_RX_BUF_ST_S)
#define SJA1000_RX_BUF_ST_V           0x00000001
#define SJA1000_RX_BUF_ST_S           0

/* SJA1000_INT_RAW_REG register
 * Interrupt Register
 */

#define SJA1000_INT_RAW_REG      (0x03)

/* SJA1000_BUS_ERR_INT_ST : RO; bitpos: [7]; default: 0;
 * Error interrupt. If this bit is set to 1, it indicates an error is
 * detected on the bus.
 */

#define SJA1000_BUS_ERR_INT_ST        (BIT(7))
#define SJA1000_BUS_ERR_INT_ST_M                                           \
  (SJA1000_BUS_ERR_INT_ST_V << SJA1000_BUS_ERR_INT_ST_S)
#define SJA1000_BUS_ERR_INT_ST_V 0x00000001
#define SJA1000_BUS_ERR_INT_ST_S 7

/* SJA1000_ARB_LOST_INT_ST : RO; bitpos: [6]; default: 0;
 * Arbitration lost interrupt. If this bit is set to 1, it indicates an
 * arbitration lost interrupt is generated.
 */

#define SJA1000_ARB_LOST_INT_ST  (BIT(6))
#define SJA1000_ARB_LOST_INT_ST_M                                          \
  (SJA1000_ARB_LOST_INT_ST_V << SJA1000_ARB_LOST_INT_ST_S)
#define SJA1000_ARB_LOST_INT_ST_V  0x00000001
#define SJA1000_ARB_LOST_INT_ST_S  6

/* SJA1000_ERR_PASSIVE_INT_ST : RO; bitpos: [5]; default: 0;
 * Error passive interrupt. If this bit is set to 1, it indicates the
 * SJA1000 Controller is switched between error active status and error
 * passive status due to the change of error counters.
 */

#define SJA1000_ERR_PASSIVE_INT_ST (BIT(5))
#define SJA1000_ERR_PASSIVE_INT_ST_M                                       \
  (SJA1000_ERR_PASSIVE_INT_ST_V << SJA1000_ERR_PASSIVE_INT_ST_S)
#define SJA1000_ERR_PASSIVE_INT_ST_V 0x00000001
#define SJA1000_ERR_PASSIVE_INT_ST_S 5

/* SJA1000_OVERRUN_INT_ST : RO; bitpos: [3]; default: 0;
 * Data overrun interrupt. If this bit is set to 1, it indicates a data
 * overrun interrupt is generated in the RX FIFO.
 */

#define SJA1000_OVERRUN_INT_ST       (BIT(3))
#define SJA1000_OVERRUN_INT_ST_M                                           \
  (SJA1000_OVERRUN_INT_ST_V << SJA1000_OVERRUN_INT_ST_S)
#define SJA1000_OVERRUN_INT_ST_V 0x00000001
#define SJA1000_OVERRUN_INT_ST_S 3

/* SJA1000_ERR_WARN_INT_ST : RO; bitpos: [2]; default: 0;
 * Error warning interrupt. If this bit is set to 1, it indicates the error
 * status signal and the bus-off status signal of Status register have
 * changed (e.g., switched from 0 to 1 or from 1 to 0).
 */

#define SJA1000_ERR_WARN_INT_ST  (BIT(2))
#define SJA1000_ERR_WARN_INT_ST_M                                          \
  (SJA1000_ERR_WARN_INT_ST_V << SJA1000_ERR_WARN_INT_ST_S)
#define SJA1000_ERR_WARN_INT_ST_V     0x00000001
#define SJA1000_ERR_WARN_INT_ST_S     2

/* SJA1000_TX_INT_ST : RO; bitpos: [1]; default: 0;
 * Transmit interrupt. If this bit is set to 1, it indicates the message
 * transmitting mis- sion is finished and a new transmission is able to
 * execute.
 */

#define SJA1000_TX_INT_ST             (BIT(1))
#define SJA1000_TX_INT_ST_M           (SJA1000_TX_INT_ST_V << SJA1000_TX_INT_ST_S)
#define SJA1000_TX_INT_ST_V           0x00000001
#define SJA1000_TX_INT_ST_S           1

/* SJA1000_RX_INT_ST : RO; bitpos: [0]; default: 0;
 * Receive interrupt. If this bit is set to 1, it indicates there are
 * messages to be handled in the RX FIFO.
 */

#define SJA1000_RX_INT_ST             (BIT(0))
#define SJA1000_RX_INT_ST_M           (SJA1000_RX_INT_ST_V << SJA1000_RX_INT_ST_S)
#define SJA1000_RX_INT_ST_V           0x00000001
#define SJA1000_RX_INT_ST_S           0

/* SJA1000_INT_ENA_REG register
 * Interrupt Enable Register
 */

#define SJA1000_INT_ENA_REG           (0x04)

/* SJA1000_BUS_ERR_INT_ENA : R/W; bitpos: [7]; default: 0;
 * Set this bit to 1 to enable error interrupt.
 */

#define SJA1000_BUS_ERR_INT_ENA       (BIT(7))
#define SJA1000_BUS_ERR_INT_ENA_M                                          \
  (SJA1000_BUS_ERR_INT_ENA_V << SJA1000_BUS_ERR_INT_ENA_S)
#define SJA1000_BUS_ERR_INT_ENA_V 0x00000001
#define SJA1000_BUS_ERR_INT_ENA_S 7

/* SJA1000_ARB_LOST_INT_ENA : R/W; bitpos: [6]; default: 0;
 * Set this bit to 1 to enable arbitration lost interrupt.
 */

#define SJA1000_ARB_LOST_INT_ENA  (BIT(6))
#define SJA1000_ARB_LOST_INT_ENA_M                                         \
  (SJA1000_ARB_LOST_INT_ENA_V << SJA1000_ARB_LOST_INT_ENA_S)
#define SJA1000_ARB_LOST_INT_ENA_V  0x00000001
#define SJA1000_ARB_LOST_INT_ENA_S  6

/* SJA1000_ERR_PASSIVE_INT_ENA : R/W; bitpos: [5]; default: 0;
 * Set this bit to 1 to enable error passive interrupt.
 */

#define SJA1000_ERR_PASSIVE_INT_ENA (BIT(5))
#define SJA1000_ERR_PASSIVE_INT_ENA_M                                      \
  (SJA1000_ERR_PASSIVE_INT_ENA_V << SJA1000_ERR_PASSIVE_INT_ENA_S)
#define SJA1000_ERR_PASSIVE_INT_ENA_V 0x00000001
#define SJA1000_ERR_PASSIVE_INT_ENA_S 5

/* SJA1000_OVERRUN_INT_ENA : R/W; bitpos: [3]; default: 0;
 * Set this bit to 1 to enable data overrun interrupt.
 */

#define SJA1000_OVERRUN_INT_ENA       (BIT(3))
#define SJA1000_OVERRUN_INT_ENA_M                                          \
  (SJA1000_OVERRUN_INT_ENA_V << SJA1000_OVERRUN_INT_ENA_S)
#define SJA1000_OVERRUN_INT_ENA_V 0x00000001
#define SJA1000_OVERRUN_INT_ENA_S 3

/* SJA1000_ERR_WARN_INT_ENA : R/W; bitpos: [2]; default: 0;
 * Set this bit to 1 to enable error warning interrupt.
 */

#define SJA1000_ERR_WARN_INT_ENA  (BIT(2))
#define SJA1000_ERR_WARN_INT_ENA_M                                         \
  (SJA1000_ERR_WARN_INT_ENA_V << SJA1000_ERR_WARN_INT_ENA_S)
#define SJA1000_ERR_WARN_INT_ENA_V         0x00000001
#define SJA1000_ERR_WARN_INT_ENA_S         2

/* SJA1000_TX_INT_ENA : R/W; bitpos: [1]; default: 0;
 * Set this bit to 1 to enable transmit interrupt.
 */

#define SJA1000_TX_INT_ENA                 (BIT(1))
#define SJA1000_TX_INT_ENA_M               (SJA1000_TX_INT_ENA_V << SJA1000_TX_INT_ENA_S)
#define SJA1000_TX_INT_ENA_V               0x00000001
#define SJA1000_TX_INT_ENA_S               1

/* SJA1000_RX_INT_ENA : R/W; bitpos: [0]; default: 0;
 * Set this bit to 1 to enable receive interrupt.
 */

#define SJA1000_RX_INT_ENA                 (BIT(0))
#define SJA1000_RX_INT_ENA_M               (SJA1000_RX_INT_ENA_V << SJA1000_RX_INT_ENA_S)
#define SJA1000_RX_INT_ENA_V               0x00000001
#define SJA1000_RX_INT_ENA_S               0

/* SJA1000_BUS_TIMING_0_REG register
 * Bus Timing Register 0
 */

#define SJA1000_BUS_TIMING_0_REG           (0x06)

/* SJA1000_SYNC_JUMP_WIDTH : RO | R/W; bitpos: [7:6]; default: 0;
 * Synchronization Jump Width (SJW), 1 \verb+~+ 14 Tq wide.
 */

#define SJA1000_SYNC_JUMP_WIDTH            0x00000003
#define SJA1000_SYNC_JUMP_WIDTH_M                                          \
  (SJA1000_SYNC_JUMP_WIDTH_V << SJA1000_SYNC_JUMP_WIDTH_S)
#define SJA1000_SYNC_JUMP_WIDTH_V          0x00000003
#define SJA1000_SYNC_JUMP_WIDTH_S          6

/* SJA1000_BAUD_PRESC : RO | R/W; bitpos: [5:0]; default: 0;
 * Baud Rate Prescaler, determines the frequency dividing ratio.
 */

#define SJA1000_BAUD_PRESC                 0x0000003F
#define SJA1000_BAUD_PRESC_M               (SJA1000_BAUD_PRESC_V << SJA1000_BAUD_PRESC_S)
#define SJA1000_BAUD_PRESC_V               0x0000003F
#define SJA1000_BAUD_PRESC_S               0

/* SJA1000_BUS_TIMING_1_REG register
 * Bus Timing Register 1
 */

#define SJA1000_BUS_TIMING_1_REG           (0x07)

/* SJA1000_TIME_SAMP : RO | R/W; bitpos: [7]; default: 0;
 * The number of sample points. 0: the bus is sampled once; 1: the bus is
 * sampled three times
 */

#define SJA1000_TIME_SAMP                  (BIT(7))
#define SJA1000_TIME_SAMP_M                (SJA1000_TIME_SAMP_V << SJA1000_TIME_SAMP_S)
#define SJA1000_TIME_SAMP_V                0x00000001
#define SJA1000_TIME_SAMP_S                7

/* SJA1000_TIME_SEG2 : RO | R/W; bitpos: [6:4]; default: 0;
 * The width of PBS2.
 */

#define SJA1000_TIME_SEG2                  0x00000007
#define SJA1000_TIME_SEG2_M                (SJA1000_TIME_SEG2_V << SJA1000_TIME_SEG2_S)
#define SJA1000_TIME_SEG2_V                0x00000007
#define SJA1000_TIME_SEG2_S                4

/* SJA1000_TIME_SEG1 : RO | R/W; bitpos: [3:0]; default: 0;
 * The width of PBS1.
 */

#define SJA1000_TIME_SEG1                  0x0000000F
#define SJA1000_TIME_SEG1_M                (SJA1000_TIME_SEG1_V << SJA1000_TIME_SEG1_S)
#define SJA1000_TIME_SEG1_V                0x0000000F
#define SJA1000_TIME_SEG1_S                0

/* SJA1000_ARB_LOST_CAP_REG register
 * Arbitration Lost Capture Register
 */

#define SJA1000_ARB_LOST_CAP_REG           (0x0B)

/* SJA1000_ARB_LOST_CAP : RO; bitpos: [4:0]; default: 0;
 * This register contains information about the bit position of lost
 * arbitration.
 */

#define SJA1000_ARB_LOST_CAP               0x0000001F
#define SJA1000_ARB_LOST_CAP_M                                             \
  (SJA1000_ARB_LOST_CAP_V << SJA1000_ARB_LOST_CAP_S)
#define SJA1000_ARB_LOST_CAP_V             0x0000001F
#define SJA1000_ARB_LOST_CAP_S             0

/* SJA1000_ERR_CODE_CAP_REG register
 * Error Code Capture Register
 */

#define SJA1000_ERR_CODE_CAP_REG           (0x0C)

/* SJA1000_ECC_TYPE : RO; bitpos: [7:6]; default: 0;
 * This register contains information about error types: 00: bit error; 01:
 * form error; 10: stuff error; 11: other type of error
 */

#define SJA1000_ECC_TYPE                   0x00000003
#define SJA1000_ECC_TYPE_M                 (SJA1000_ECC_TYPE_V << SJA1000_ECC_TYPE_S)
#define SJA1000_ECC_TYPE_V                 0x00000003
#define SJA1000_ECC_TYPE_S                 6

/* SJA1000_ECC_DIRECTION : RO; bitpos: [5]; default: 0;
 * This register contains information about transmission direction of the
 * node when error occurs. 1: Error occurs when receiving a message; 0:
 * Error occurs when transmitting a message
 */

#define SJA1000_ECC_DIRECTION              (BIT(5))
#define SJA1000_ECC_DIRECTION_M                                            \
  (SJA1000_ECC_DIRECTION_V << SJA1000_ECC_DIRECTION_S)
#define SJA1000_ECC_DIRECTION_V 0x00000001
#define SJA1000_ECC_DIRECTION_S 5

/* SJA1000_ECC_SEGMENT : RO; bitpos: [4:0]; default: 0;
 * This register contains information about the location of errors, see
 * Table 181 for details.
 */

#define SJA1000_ECC_SEGMENT     0x0000001F
#define SJA1000_ECC_SEGMENT_M                                              \
  (SJA1000_ECC_SEGMENT_V << SJA1000_ECC_SEGMENT_S)
#define SJA1000_ECC_SEGMENT_V                   0x0000001F
#define SJA1000_ECC_SEGMENT_S                   0

/* SJA1000_ERR_WARNING_LIMIT_REG register
 * Error Warning Limit Register
 */

#define SJA1000_ERR_WARNING_LIMIT_REG           (0x0D)

/* SJA1000_ERR_WARNING_LIMIT : RO | R/W; bitpos: [7:0]; default: 96;
 * Error warning threshold. In the case when any of a error counter value
 * exceeds the threshold, or all the error counter values are below the
 * threshold, an error warning interrupt will be triggered (given the enable
 * signal is valid).
 */

#define SJA1000_ERR_WARNING_LIMIT               0x000000FF
#define SJA1000_ERR_WARNING_LIMIT_M                                        \
  (SJA1000_ERR_WARNING_LIMIT_V << SJA1000_ERR_WARNING_LIMIT_S)
#define SJA1000_ERR_WARNING_LIMIT_V      0x000000FF
#define SJA1000_ERR_WARNING_LIMIT_S      0

/* SJA1000_RX_ERR_CNT_REG register
 * Receive Error Counter Register
 */

#define SJA1000_RX_ERR_CNT_REG           (0x0E)

/* SJA1000_RX_ERR_CNT : RO | R/W; bitpos: [7:0]; default: 0;
 * The RX error counter register, reflects value changes under reception
 * status.
 */

#define SJA1000_RX_ERR_CNT               0x000000FF
#define SJA1000_RX_ERR_CNT_M             (SJA1000_RX_ERR_CNT_V << SJA1000_RX_ERR_CNT_S)
#define SJA1000_RX_ERR_CNT_V             0x000000FF
#define SJA1000_RX_ERR_CNT_S             0

/* SJA1000_TX_ERR_CNT_REG register
 * Transmit Error Counter Register
 */

#define SJA1000_TX_ERR_CNT_REG           (0x0F)

/* SJA1000_TX_ERR_CNT : RO | R/W; bitpos: [7:0]; default: 0;
 * The TX error counter register, reflects value changes under transmission
 * status.
 */

#define SJA1000_TX_ERR_CNT               0x000000FF
#define SJA1000_TX_ERR_CNT_M             (SJA1000_TX_ERR_CNT_V << SJA1000_TX_ERR_CNT_S)
#define SJA1000_TX_ERR_CNT_V             0x000000FF
#define SJA1000_TX_ERR_CNT_S             0

/* SJA1000_DATA_0_REG register
 * Data register 0
 */

#define SJA1000_DATA_0_REG           (0x10)

/* SJA1000_TX_BYTE_0 : WO; bitpos: [7:0]; default: 0;
 * Stored the 0th byte information of the data to be transmitted under
 * operating mode.
 */

#define SJA1000_TX_BYTE_0                0x000000FF
#define SJA1000_TX_BYTE_0_M              (SJA1000_TX_BYTE_0_V << SJA1000_TX_BYTE_0_S)
#define SJA1000_TX_BYTE_0_V              0x000000FF
#define SJA1000_TX_BYTE_0_S              0

/* SJA1000_ACCEPTANCE_CODE_0 : WO; bitpos: [7:0]; default: 0;
 * Stored the 0th byte of the filter code in reset mode.
 */

#define SJA1000_ACCEPTANCE_CODE_0        0x000000FF
#define SJA1000_ACCEPTANCE_CODE_0_M                                        \
  (SJA1000_ACCEPTANCE_CODE_0_V << SJA1000_ACCEPTANCE_CODE_0_S)
#define SJA1000_ACCEPTANCE_CODE_0_V  0x000000FF
#define SJA1000_ACCEPTANCE_CODE_0_S  0

/* SJA1000_DATA_1_REG register
 * Data register 1
 */

#define SJA1000_DATA_1_REG           (0x11)

/* SJA1000_TX_BYTE_1 : WO; bitpos: [7:0]; default: 0;
 * Stored the 1st byte information of the data to be transmitted under
 * operating mode.
 */

#define SJA1000_TX_BYTE_1            0x000000FF
#define SJA1000_TX_BYTE_1_M          (SJA1000_TX_BYTE_1_V << SJA1000_TX_BYTE_1_S)
#define SJA1000_TX_BYTE_1_V          0x000000FF
#define SJA1000_TX_BYTE_1_S          0

/* SJA1000_ACCEPTANCE_CODE_1 : WO; bitpos: [7:0]; default: 0;
 * Stored the 1st byte of the filter code in reset mode.
 */

#define SJA1000_ACCEPTANCE_CODE_1    0x000000FF
#define SJA1000_ACCEPTANCE_CODE_1_M                                        \
  (SJA1000_ACCEPTANCE_CODE_1_V << SJA1000_ACCEPTANCE_CODE_1_S)
#define SJA1000_ACCEPTANCE_CODE_1_V  0x000000FF
#define SJA1000_ACCEPTANCE_CODE_1_S  0

/* SJA1000_DATA_2_REG register
 * Data register 2
 */

#define SJA1000_DATA_2_REG           (0x12)

/* SJA1000_TX_BYTE_2 : WO; bitpos: [7:0]; default: 0;
 * Stored the 2nd byte information of the data to be transmitted under
 * operating mode.
 */

#define SJA1000_TX_BYTE_2            0x000000FF
#define SJA1000_TX_BYTE_2_M          (SJA1000_TX_BYTE_2_V << SJA1000_TX_BYTE_2_S)
#define SJA1000_TX_BYTE_2_V          0x000000FF
#define SJA1000_TX_BYTE_2_S          0

/* SJA1000_ACCEPTANCE_CODE_2 : WO; bitpos: [7:0]; default: 0;
 * Stored the 2nd byte of the filter code in reset mode.
 */

#define SJA1000_ACCEPTANCE_CODE_2    0x000000FF
#define SJA1000_ACCEPTANCE_CODE_2_M                                        \
  (SJA1000_ACCEPTANCE_CODE_2_V << SJA1000_ACCEPTANCE_CODE_2_S)
#define SJA1000_ACCEPTANCE_CODE_2_V  0x000000FF
#define SJA1000_ACCEPTANCE_CODE_2_S  0

/* SJA1000_DATA_3_REG register
 * Data register 3
 */

#define SJA1000_DATA_3_REG           (0x13)

/* SJA1000_TX_BYTE_3 : WO; bitpos: [7:0]; default: 0;
 * Stored the 3rd byte information of the data to be transmitted under
 * operating mode.
 */

#define SJA1000_TX_BYTE_3            0x000000FF
#define SJA1000_TX_BYTE_3_M          (SJA1000_TX_BYTE_3_V << SJA1000_TX_BYTE_3_S)
#define SJA1000_TX_BYTE_3_V          0x000000FF
#define SJA1000_TX_BYTE_3_S          0

/* SJA1000_ACCEPTANCE_CODE_3 : WO; bitpos: [7:0]; default: 0;
 * Stored the 3rd byte of the filter code in reset mode.
 */

#define SJA1000_ACCEPTANCE_CODE_3    0x000000FF
#define SJA1000_ACCEPTANCE_CODE_3_M                                        \
  (SJA1000_ACCEPTANCE_CODE_3_V << SJA1000_ACCEPTANCE_CODE_3_S)
#define SJA1000_ACCEPTANCE_CODE_3_V  0x000000FF
#define SJA1000_ACCEPTANCE_CODE_3_S  0

/* SJA1000_DATA_4_REG register
 * Data register 4
 */

#define SJA1000_DATA_4_REG           (0x14)

/* SJA1000_TX_BYTE_4 : WO; bitpos: [7:0]; default: 0;
 * Stored the 4th byte information of the data to be transmitted under
 * operating mode.
 */

#define SJA1000_TX_BYTE_4            0x000000FF
#define SJA1000_TX_BYTE_4_M          (SJA1000_TX_BYTE_4_V << SJA1000_TX_BYTE_4_S)
#define SJA1000_TX_BYTE_4_V          0x000000FF
#define SJA1000_TX_BYTE_4_S          0

/* SJA1000_ACCEPTANCE_MASK_0 : WO; bitpos: [7:0]; default: 0;
 * Stored the 0th byte of the filter code in reset mode.
 */

#define SJA1000_ACCEPTANCE_MASK_0    0x000000FF
#define SJA1000_ACCEPTANCE_MASK_0_M                                        \
  (SJA1000_ACCEPTANCE_MASK_0_V << SJA1000_ACCEPTANCE_MASK_0_S)
#define SJA1000_ACCEPTANCE_MASK_0_V  0x000000FF
#define SJA1000_ACCEPTANCE_MASK_0_S  0

/* SJA1000_DATA_5_REG register
 * Data register 5
 */

#define SJA1000_DATA_5_REG           (0x15)

/* SJA1000_TX_BYTE_5 : WO; bitpos: [7:0]; default: 0;
 * Stored the 5th byte information of the data to be transmitted under
 * operating mode.
 */

#define SJA1000_TX_BYTE_5            0x000000FF
#define SJA1000_TX_BYTE_5_M          (SJA1000_TX_BYTE_5_V << SJA1000_TX_BYTE_5_S)
#define SJA1000_TX_BYTE_5_V          0x000000FF
#define SJA1000_TX_BYTE_5_S          0

/* SJA1000_ACCEPTANCE_MASK_1 : WO; bitpos: [7:0]; default: 0;
 * Stored the 1st byte of the filter code in reset mode.
 */

#define SJA1000_ACCEPTANCE_MASK_1    0x000000FF
#define SJA1000_ACCEPTANCE_MASK_1_M                                        \
  (SJA1000_ACCEPTANCE_MASK_1_V << SJA1000_ACCEPTANCE_MASK_1_S)
#define SJA1000_ACCEPTANCE_MASK_1_V  0x000000FF
#define SJA1000_ACCEPTANCE_MASK_1_S  0

/* SJA1000_DATA_6_REG register
 * Data register 6
 */

#define SJA1000_DATA_6_REG           (0x16)

/* SJA1000_TX_BYTE_6 : WO; bitpos: [7:0]; default: 0;
 * Stored the 6th byte information of the data to be transmitted under
 * operating mode.
 */

#define SJA1000_TX_BYTE_6            0x000000FF
#define SJA1000_TX_BYTE_6_M          (SJA1000_TX_BYTE_6_V << SJA1000_TX_BYTE_6_S)
#define SJA1000_TX_BYTE_6_V          0x000000FF
#define SJA1000_TX_BYTE_6_S          0

/* SJA1000_ACCEPTANCE_MASK_2 : WO; bitpos: [7:0]; default: 0;
 * Stored the 2nd byte of the filter code in reset mode.
 */

#define SJA1000_ACCEPTANCE_MASK_2    0x000000FF
#define SJA1000_ACCEPTANCE_MASK_2_M                                        \
  (SJA1000_ACCEPTANCE_MASK_2_V << SJA1000_ACCEPTANCE_MASK_2_S)
#define SJA1000_ACCEPTANCE_MASK_2_V  0x000000FF
#define SJA1000_ACCEPTANCE_MASK_2_S  0

/* SJA1000_DATA_7_REG register
 * Data register 7
 */

#define SJA1000_DATA_7_REG           (0x17)

/* SJA1000_TX_BYTE_7 : WO; bitpos: [7:0]; default: 0;
 * Stored the 7th byte information of the data to be transmitted under
 * operating mode.
 */

#define SJA1000_TX_BYTE_7            0x000000FF
#define SJA1000_TX_BYTE_7_M          (SJA1000_TX_BYTE_7_V << SJA1000_TX_BYTE_7_S)
#define SJA1000_TX_BYTE_7_V          0x000000FF
#define SJA1000_TX_BYTE_7_S          0

/* SJA1000_ACCEPTANCE_MASK_3 : WO; bitpos: [7:0]; default: 0;
 * Stored the 3th byte of the filter code in reset mode.
 */

#define SJA1000_ACCEPTANCE_MASK_3    0x000000FF
#define SJA1000_ACCEPTANCE_MASK_3_M                                        \
  (SJA1000_ACCEPTANCE_MASK_3_V << SJA1000_ACCEPTANCE_MASK_3_S)
#define SJA1000_ACCEPTANCE_MASK_3_V          0x000000FF
#define SJA1000_ACCEPTANCE_MASK_3_S          0

/* SJA1000_DATA_8_REG register
 * Data register 8
 */

#define SJA1000_DATA_8_REG           (0x18)

/* SJA1000_TX_BYTE_8 : WO; bitpos: [7:0]; default: 0;
 * Stored the 8th byte information of the data to be transmitted under
 * operating mode.
 */

#define SJA1000_TX_BYTE_8                    0x000000FF
#define SJA1000_TX_BYTE_8_M                  (SJA1000_TX_BYTE_8_V << SJA1000_TX_BYTE_8_S)
#define SJA1000_TX_BYTE_8_V                  0x000000FF
#define SJA1000_TX_BYTE_8_S                  0

/* SJA1000_DATA_9_REG register
 * Data register 9
 */

#define SJA1000_DATA_9_REG           (0x19)

/* SJA1000_TX_BYTE_9 : WO; bitpos: [7:0]; default: 0;
 * Stored the 9th byte information of the data to be transmitted under
 * operating mode.
 */

#define SJA1000_TX_BYTE_9                    0x000000FF
#define SJA1000_TX_BYTE_9_M                  (SJA1000_TX_BYTE_9_V << SJA1000_TX_BYTE_9_S)
#define SJA1000_TX_BYTE_9_V                  0x000000FF
#define SJA1000_TX_BYTE_9_S                  0

/* SJA1000_DATA_10_REG register
 * Data register 10
 */

#define SJA1000_DATA_10_REG           (0x1A)

/* SJA1000_TX_BYTE_10 : WO; bitpos: [7:0]; default: 0;
 * Stored the 10th byte information of the data to be transmitted under
 * operating mode.
 */

#define SJA1000_TX_BYTE_10                   0x000000FF
#define SJA1000_TX_BYTE_10_M                 (SJA1000_TX_BYTE_10_V << SJA1000_TX_BYTE_10_S)
#define SJA1000_TX_BYTE_10_V                 0x000000FF
#define SJA1000_TX_BYTE_10_S                 0

/* SJA1000_DATA_11_REG register
 * Data register 11
 */

#define SJA1000_DATA_11_REG           (0x1B)

/* SJA1000_TX_BYTE_11 : WO; bitpos: [7:0]; default: 0;
 * Stored the 11th byte information of the data to be transmitted under
 * operating mode.
 */

#define SJA1000_TX_BYTE_11                   0x000000FF
#define SJA1000_TX_BYTE_11_M                 (SJA1000_TX_BYTE_11_V << SJA1000_TX_BYTE_11_S)
#define SJA1000_TX_BYTE_11_V                 0x000000FF
#define SJA1000_TX_BYTE_11_S                 0

/* SJA1000_DATA_12_REG register
 * Data register 12
 */

#define SJA1000_DATA_12_REG           (0x1C)

/* SJA1000_TX_BYTE_12 : WO; bitpos: [7:0]; default: 0;
 * Stored the 12th byte information of the data to be transmitted under
 * operating mode.
 */

#define SJA1000_TX_BYTE_12                   0x000000FF
#define SJA1000_TX_BYTE_12_M                 (SJA1000_TX_BYTE_12_V << SJA1000_TX_BYTE_12_S)
#define SJA1000_TX_BYTE_12_V                 0x000000FF
#define SJA1000_TX_BYTE_12_S                 0

/* SJA1000_RX_MESSAGE_CNT_REG register
 * Receive Message Counter Register
 */

#define SJA1000_RX_MESSAGE_CNT_REG           (0x1D)

/* SJA1000_RX_MESSAGE_COUNTER : RO; bitpos: [6:0]; default: 0;
 * This register reflects the number of messages available within the RX
 * FIFO.
 */

#define SJA1000_RX_MESSAGE_COUNTER           0x0000007F
#define SJA1000_RX_MESSAGE_COUNTER_M                                       \
  (SJA1000_RX_MESSAGE_COUNTER_V << SJA1000_RX_MESSAGE_COUNTER_S)
#define SJA1000_RX_MESSAGE_COUNTER_V        0x0000007F
#define SJA1000_RX_MESSAGE_COUNTER_S        0

/* SJA1000_CLOCK_DIVIDER_REG register
 * Clock Divider register
 */

#define SJA1000_CLOCK_DIVIDER_REG           (0x1F)

/* SJA1000_CLOCK_OFF : RO | R/W; bitpos: [3]; default: 0;
 * This bit can be configured under reset mode. 1: Disable the external
 * CLKOUT pin; 0: Enable the external CLKOUT pin
 */

#define SJA1000_CLOCK_OFF                   (BIT(3))
#define SJA1000_CLOCK_OFF_M                 (SJA1000_CLOCK_OFF_V << SJA1000_CLOCK_OFF_S)
#define SJA1000_CLOCK_OFF_V                 0x00000001
#define SJA1000_CLOCK_OFF_S                 3

/* SJA1000_EXT_MODE : RO | R/W; bitpos: [7]; default: 0;
 * This bit can be configured under reset mode. 1: Extended mode, compatible
 * with CAN2.0B; 0: Basic mode
 */

#define SJA1000_EXT_MODE                    (BIT(7))
#define SJA1000_EXT_MODE_M                  (SJA1000_EXT_MODE_V << SJA1000_EXT_MODE_S)
#define SJA1000_EXT_MODE_V                  0x00000001
#define SJA1000_EXT_MODE_S                  7

/* SJA1000_CD : R/W; bitpos: [7:0]; default: 0;
 * These bits are used to configure frequency dividing coefficients of the
 * external CLKOUT pin.
 */

#define SJA1000_CD                          0x00000007
#define SJA1000_CD_M                        (SJA1000_CD_V << SJA1000_CD_S)
#define SJA1000_CD_V                        0x00000007
#define SJA1000_CD_S                        0

#endif /* __DRIVERS_CAN_SJA1000_H */
