/****************************************************************************
 * arch/rh850/src/u2c/hardware/u2c_ostm.h
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

#ifndef __ARCH_RH850_SRC_U2C_HARDWARE_U2C_OSTM_H
#define __ARCH_RH850_SRC_U2C_HARDWARE_U2C_OSTM_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define U2C_OSTM0_BASE          0xffbf0000

/* Register offsets */

#define OSTM_CMP_OFFSET         0x00        /* 32-bit */
#define OSTM_CNT_OFFSET         0x04        /* 32-bit */
#define OSTM_TS_OFFSET          0x14        /* 8-bit */
#define OSTM_TT_OFFSET          0x18        /* 8-bit */
#define OSTM_CTL_OFFSET         0x20        /* 8-bit */

/* Register bits */

#define OSTM_TS_START           (1 << 0)
#define OSTM_TT_STOP            (1 << 0)
#define OSTM_CTL_MD1            (1 << 1)    /* Free-run compare mode */
#define OSTM_CTL_IE             (1 << 7)    /* TINT enabled */

#endif /* __ARCH_RH850_SRC_U2C_HARDWARE_U2C_OSTM_H */
