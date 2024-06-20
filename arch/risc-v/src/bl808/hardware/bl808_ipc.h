/****************************************************************************
 * arch/risc-v/src/bl808/hardware/bl808_ipc.h
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

#ifndef __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_IPC_H
#define __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_IPC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IPC_MSG_SEND_OFFSET 0x0
#define IPC0_MSG_SEND       (IPC0_BASE + IPC_MSG_SEND_OFFSET)

#define IPC_MSG_READ_OFFSET 0x24
#define IPC2_MSG_READ       (IPC2_BASE + IPC_MSG_READ_OFFSET)

#define IPC_MSG_ACK_OFFSET  0x28
#define IPC2_MSG_ACK        (IPC2_BASE + IPC_MSG_ACK_OFFSET)

#define IPC_INT_UNMASK_OFFSET 0x2c
#define IPC2_INT_UNMASK       (IPC2_BASE + IPC_INT_UNMASK_OFFSET)

#endif /* __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_IPC_H */
