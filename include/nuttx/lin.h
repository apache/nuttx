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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LIN_ID_BITS               6
#define LIN_ID_MASK               ((1 << LIN_ID_BITS) - 1)
#define LIN_ID_MAX                LIN_ID_MASK

#define LIN_CTRL_FRAME            CAN_EFF_FLAG

/* When slave response to master, slave node should send  frame immediately
 * which already be cached in last transmission in case of response interval
 * over time;
 */

#define LIN_CACHE_RESPONSE        (1 << (LIN_ID_BITS))

/* LIN checksum have two types, default type will be classic checksum */

#define LIN_CHECKSUM_EXTENDED     (1 << (LIN_ID_BITS + 1))

/* Cache LIN frame only work once. then will be clear */

#define LIN_SINGLE_RESPONSE       (1 << (LIN_ID_BITS + 2))

/* LIN Error Indications ****************************************************/

/* LIN_ERR_FLAG: Used to distinguish it from ordinary frames.
 * The error frame consists of err_flag、err_class(defined in data[0])
 * and err_reason(defined in data[1] to data[4]).
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
 * ERR_REASON ：Indicate the error reson(timeout,error in the frame, etc.),
 *              define in data[0] ~ data[4] in can_frame.
 */

#define LIN_ERR_FLAG              CAN_ERR_FLAG  /* The flag indicate  this is LIN err_frame */

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

#endif /* __INCLUDE_NUTTX_LIN_H */
