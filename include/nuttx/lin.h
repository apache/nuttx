/****************************************************************************
 * include/nuttx/lin.h
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

#ifndef __INCLUDE_NUTTX_LIN_H
#define __INCLUDE_NUTTX_LIN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/can.h>

#ifdef CONFIG_NET_CAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LIN_ID_BITS               6
#define LIN_ID_MASK               ((1 << LIN_ID_BITS) - 1)
#define LIN_ID_MAX                LIN_ID_MASK

/* Describe control information (such as write etc.) */

#define LIN_CTRL_FLAG             CAN_EFF_FLAG

/* Describe the direction of sending and receiving */

#define LIN_RTR_FLAG              CAN_RTR_FLAG

/* The flag indicate  this is LIN err_frame */

#define LIN_ERR_FLAG              CAN_ERR_FLAG

/* Lower_half report state switch event frame */

#define LIN_EVT_FLAG              CAN_EVT_FLAG

/* When slave response to master, slave node should send frame immediately
 * which already be cached in last transmission in case of response interval
 * over time
 */

#define LIN_CACHE_RESPONSE        (1 << (LIN_ID_BITS))

/* LIN checksum have two types, default type will be classic checksum */

#define LIN_CHECKSUM_EXTENDED     (1 << (LIN_ID_BITS + 1))

/* Cache LIN frame only work once. then will be clear */

#define LIN_SINGLE_RESPONSE       (1 << (LIN_ID_BITS + 2))

/* TxConfirmation frame. Lower_half use the flag to confirm frame-transmit */

#define LIN_TCF_FLAG              (1 << (LIN_ID_BITS + 3))

/* LIN Error Indications ****************************************************/

/* LIN_ERR_FLAG: Used to distinguish it from ordinary frames.
 * The error frame consists of err_flag（LIN_ERR_FLAG)、err_class
 * (defined in data[0]) and err_reason(defined in data[1] to data[4]).
 * The error frame is described using the following structure:
 * struct can_frame {
 *    canid_t can_id;
 *    uint8_t can_dlc;
 *    uint8_t __pad;
 *    uint8_t __res0;
 *    uint8_t __res1;
 *    uint8_t data[CAN_MAX_DLEN] __attribute__((aligned(8)));
 *  };
 *
 * Error frame description format：
 *
 * |   can_id     | xxx | data[0] | data[1] | data[2] | data[3] | data[4] |
 *       |                   |                        |
 *      \|/                 \|/                      \|/
 *  ERR_FLAG(bit29）     ERR_CLASS               ERR_REASON
 *
 * ERR_FLAG   : Indicates that the frame is an error frame,
 *              define in bit 29 of can_id(CAN_ERR_FLAG) in can_frame.
 * ERR_CLASS  : Indicate at what stage or where the error occurred,
 *              define in data[0] in can_frame.
 * ERR_REASON ：Indicate the error reason (timeout, error in the frame,
 *              etc.), define in data[0] ~ data[4] in can_frame.
 */

/* ERR_CLASS in  data[0] */

#define LIN_ERR_UNSPEC            0x00     /* Unspecified error */
#define LIN_ERR_TX                (1 << 0) /* Bit 0: TX error (see LIN_ERR_TX_* definitions) */
#define LIN_ERR_RX                (1 << 1) /* Bit 1: RX error (see LIN_ERR_RX_* definitions) */
#define LIN_ERR_BUS               (1 << 2) /* Bit 2: Bus reasons make frame error (see LIN_ERR_BUS_* definitions) */
#define LIN_ERR_CRTL              (1 << 3) /* Bit 3: Controller error (see LIN_ERR_Controller_* definitions) */

/* ERR_REASON in data[1]...DATA[4] */

/* Data[1] tx error */

#define LIN_ERR_TX_UNSPEC         0x00     /* Unspecified error */
#define LIN_ERR_TX_BREAK_TMO      (1 << 0) /* Bit 0: Master send break field, but detect break event timeout */
#define LIN_ERR_TX_SYNC_TMO       (1 << 1) /* Bit 1: Master send sync timeout (receive back timeout) */
#define LIN_ERR_TX_PID_TMO        (1 << 2) /* Bit 2: Master send pid timeout (receive back timeout) */
#define LIN_ERR_TX_DATA_TMO       (1 << 3) /* Bit 3: Master/slave send data timeout (receive back timeout) */
#define LIN_ERR_TX_CHECKSUM_TMO   (1 << 4) /* Bit 4: Master/slave send checksum timeout(receive back timeout) */

/* Data[2] rx error */

#define LIN_ERR_RX_UNSPEC         0x00     /* Unspecified error */
#define LIN_ERR_RX_NO_RESPONSE    (1 << 0) /* Bit 0: Prepared to receive response, but no response received */
#define LIN_ERR_RX_RESPONSE       (1 << 1) /* Bit 1: Receive incomplete response */
#define LIN_ERR_RX_CKSUM_TMO      (1 << 2) /* Bit 2: Response data received, receive checksum timeout */
#define LIN_ERR_RX_CKSUM          (1 << 3) /* Bit 3: Received error checksum */
#define LIN_ERR_RX_SYNC_TMO       (1 << 4) /* Bit 4: Slave receive sync timeout */
#define LIN_ERR_RX_SYNC           (1 << 5) /* Bit 5: Received error sync byte */
#define LIN_ERR_RX_PID_TMO        (1 << 6) /* Bit 6: Receive pid timeout after a sync field */
#define LIN_ERR_RX_PID_PARITY     (1 << 7) /* Bit 7: Pid parity error */

/* Data[3] bus reasons make frame error */

#define LIN_ERR_BUS_UNSPEC        0x00     /* Unspecified error */
#define LIN_ERR_BUS_PID           (1 << 0) /* Bit 0: Pid received back is not equal to the pid sent */
#define LIN_ERR_BUS_TXCKSUM       (1 << 1) /* Bit 1: Checksum received back is not equal to checksum sent */
#define LIN_ERR_BUS_SYNC          (1 << 2) /* Bit 2: Master send sync, but receive back sync is not 0x55 */
#define LIN_ERR_BUS_DATA          (1 << 3) /* Bit 3: Data received back is not equal to the data sent */

/* Data[4] error status of LIN-controller */

#define LIN_ERR_CRTL_UNSPEC       0x00     /* Unspecified error */
#define LIN_ERR_CRTL_RXOVERFLOW   (1 << 0) /* Hardware controller receive overflow */
#define LIN_ERR_CTRL_FRAMEERROR   (1 << 1) /* Hardware controller frame error */
#define LIN_ERR_CTRL_NOISE        (1 << 2) /* Hardware controller noise error */

/* LIN States Indications ***************************************************/

/* lower_half State switch events are defined in  data[0] */

#define LIN_EVT_UNSPEC           0x00     /* Unspecified state event */
#define LIN_EVT_WAKEUP           (1 << 0) /* Already send a wake-up command to lin_bus */
#define LIN_EVT_WAKEUP_PASSIVE   (1 << 1) /* Wake up when detected low level signal of bus a for a period of time (such as 250us-5ms) */
#define LIN_EVT_SLEEP            (1 << 2) /* Send a sleep command(0x3c) to the bus, only Master */
#define LIN_EVT_SLEEP_PASSIVE    (1 << 3) /* Receive a sleep command from bus */
#define LIN_EVT_SLEEP_IDLE       (1 << 4) /* Go to sleep when bus keep in the inactive for a period of time(such as 4s) */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_CAN */
#endif /* __INCLUDE_NUTTX_LIN_H */
