/****************************************************************************
 * include/nuttx/lin.h
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

#define LIN_ID_BITS             6
#define LIN_ID_MASK             ((1 << LIN_ID_BITS) - 1)
#define LIN_ID_MAX              LIN_ID_MASK

#define LIN_CTRL_FRAME          CAN_EFF_FLAG

/* When slave response to master, slave node should send  frame immediately
 * which already be cached in last transmission in case of response interval
 * over time;
 */

#define LIN_CACHE_RESPONSE      (1 << (LIN_ID_BITS))

/* LIN checksum have two types, default type will be classic checksum */

#define LIN_CHECKSUM_EXTENDED   (1 << (LIN_ID_BITS + 1))

/* Cache LIN frame only work once. then will be clear */

#define LIN_SINGLE_RESPONSE     (1 << (LIN_ID_BITS + 2))

/* Error flags */

#define LIN_ERR_RX_TIMEOUT      (1 << (LIN_ID_BITS + 3))
#define LIN_ERR_CHECKSUM        (1 << (LIN_ID_BITS + 4))
#define LIN_ERR_FRAMING         (1 << (LIN_ID_BITS + 5))

/* TxConfirmation flag */

#define LIN_TCF_FRAMING         (1 << (LIN_ID_BITS + 6))

/* Event flag */

#define LIN_EVT_FRAMING         (1 << (LIN_ID_BITS + 7))

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
