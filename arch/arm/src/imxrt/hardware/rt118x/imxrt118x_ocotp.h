/****************************************************************************
 * arch/arm/src/imxrt/hardware/rt118x/imxrt118x_ocotp.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_OCOTP_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_OCOTP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* On i.MX RT chips fronted by the EdgeLock Enclave (RT1180 and later),
 * the OCOTP fuse controller is owned by the ELE (IMXRT1180RM Ch. 7.1.2).
 * The SoC exposes a read-only window of the non-security-related OCOTP
 * shadow registers at:
 *
 *   IMXRT_OCOTP_SHADOW_BASE + 0x4 * <fuse word index>
 *
 * (IMXRT1180RM Ch. 26.1 "Fusemap").
 *
 * Write and reload paths must go through the ELE messaging protocol.
 * That is not implemented in this port; imxrt_ocotp_write() and
 * imxrt_ocotp_reload() therefore return -ENOSYS on chips with
 * CONFIG_IMXRT_ELE=y.
 */

#define IMXRT_OCOTP_SHADOW_BASE   (0x47518000ul)

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_OCOTP_H */
