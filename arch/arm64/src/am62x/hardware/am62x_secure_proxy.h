/****************************************************************************
 * arch/arm64/src/am62x/hardware/am62x_secure_proxy.h
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

/* AM62x Secure Proxy register layout.
 *
 * The secure proxy is the mailbox transport used to exchange TISCI messages
 * between the A53 (running NuttX) and the DM/TIFS system firmware on the R5.
 * Each "thread" is a unidirectional message queue.  A thread occupies a
 * 0x1000-stride window in each of two regions used here:
 *
 *   - "target_data" : the message payload, 15 data words per thread
 *                     (registers 0..14 at byte offsets 0x4..0x3c).  Writing
 *                     the last word (offset 0x3c) commits the message;
 *                     an RX thread consumes its message by reading it.
 *   - "rt"          : per-thread status; bits[7:0] hold the credit/message
 *                     count and bit31 flags a thread error.
 *
 * The "scfg" region carries per-thread direction config and is not needed
 * for fixed-direction synchronous request/response operation, so it is
 * omitted.
 *
 * Source: Linux drivers/mailbox/ti-msgmgr.c (k3_secure_proxy_desc and the
 *         SPROXY_THREAD_* macros) and U-Boot drivers/mailbox/k3-sec-proxy.c.
 */

#ifndef __ARCH_ARM64_SRC_AM62X_HARDWARE_AM62X_SECURE_PROXY_H
#define __ARCH_ARM64_SRC_AM62X_HARDWARE_AM62X_SECURE_PROXY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/am62x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per-thread window stride within each region */

#define AM62X_SEC_PROXY_THREAD_STRIDE  0x1000ul

/* target_data region: 15 message words (registers 0..14).
 * Word "reg" of thread "tid" lives at:
 *   AM62X_SEC_PROXY_DATA_BASE + tid*0x1000 + 0x4 + reg*4
 * giving the data window 0x4 .. 0x3c (60 bytes maximum per message).
 */

#define AM62X_SEC_PROXY_MSG_WORDS      15
#define AM62X_SEC_PROXY_MAX_MSG_SIZE   60
#define AM62X_SEC_PROXY_DATA_START     0x4ul   /* offset of word 0          */
#define AM62X_SEC_PROXY_DATA_END       0x3cul  /* offset of word 14 (commit)*/

/* rt region: per-thread status register at the thread base */

#define AM62X_SEC_PROXY_RT_STATUS      0x0ul
#define AM62X_SEC_PROXY_RT_CNT_MASK    0xfful       /* [7:0]  credit count   */
#define AM62X_SEC_PROXY_RT_ERR         (1ul << 31)  /* [31]   thread error   */

/* Region base for a given thread id */

#define AM62X_SEC_PROXY_DATA(tid)    \
  (AM62X_SEC_PROXY_DATA_BASE + (uintptr_t)(tid) * AM62X_SEC_PROXY_THREAD_STRIDE)
#define AM62X_SEC_PROXY_RT(tid)      \
  (AM62X_SEC_PROXY_RT_BASE + (uintptr_t)(tid) * AM62X_SEC_PROXY_THREAD_STRIDE)

#endif /* __ARCH_ARM64_SRC_AM62X_HARDWARE_AM62X_SECURE_PROXY_H */
