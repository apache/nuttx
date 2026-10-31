/****************************************************************************
 * arch/rh850/src/u2c/hardware/u2c_rlin3.h
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

#ifndef __ARCH_RH850_SRC_U2C_HARDWARE_U2C_RLIN3_H
#define __ARCH_RH850_SRC_U2C_HARDWARE_U2C_RLIN3_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RLIN3 unit base addresses */

#define U2C_RLIN311_BASE        0xff89bb00

/* RLIN3 register offsets, 8-bit unless noted */

#define RLIN3_LWBR_OFFSET       0x01
#define RLIN3_LBRP01_OFFSET     0x02        /* 16-bit */
#define RLIN3_LMD_OFFSET        0x08
#define RLIN3_LBFC_OFFSET       0x09
#define RLIN3_LEDE_OFFSET       0x0d
#define RLIN3_LCUC_OFFSET       0x0e
#define RLIN3_LMST_OFFSET       0x11
#define RLIN3_LST_OFFSET        0x12
#define RLIN3_LUOER_OFFSET      0x20
#define RLIN3_LUOR1_OFFSET      0x21
#define RLIN3_LUTDR_OFFSET      0x24        /* 16-bit */
#define RLIN3_LURDR_OFFSET      0x26        /* 16-bit */

/* Register bits */

#define RLIN3_LMD_UART          0x01        /* UART mode */
#define RLIN3_LCUC_OM0          (1 << 0)    /* Leave LIN reset mode */
#define RLIN3_LMST_OMM0         (1 << 0)    /* Not in LIN reset mode */
#define RLIN3_LST_UTS           (1 << 4)    /* Transmission in progress */
#define RLIN3_LUOER_UTOE        (1 << 0)    /* Transmission enable */
#define RLIN3_LUOER_UROE        (1 << 1)    /* Reception enable */

/* LWBR.NSPB = 0: 16 samples per bit, prescaler 1/1 */

#define RLIN3_SAMPLES           16

#endif /* __ARCH_RH850_SRC_U2C_HARDWARE_U2C_RLIN3_H */
