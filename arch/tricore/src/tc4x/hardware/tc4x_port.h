/****************************************************************************
 * arch/tricore/src/tc4x/hardware/tc4x_port.h
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

#ifndef __ARCH_TRICORE_SRC_TC4X_HARDWARE_TC4X_PORT_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define __ARCH_TRICORE_SRC_TC4X_HARDWARE_TC4X_PORT_H

#define AURIX_PORT_BASE         0xf003a000
#define AURIX_PORT_STRIDE       0x400
#define AURIX_PORT_ADDR(n)      (AURIX_PORT_BASE + \
                                 ((uint32_t)(n) * AURIX_PORT_STRIDE))

#define PORT_ID_OFFSET          0x00
#define PORT_RST_OFFSET         0x04
#define PORT_WKEN_OFFSET        0x10
#define PORT_WKSTS_OFFSET       0x14
#define PORT_WKENSTS_OFFSET     0x18
#define PORT_WKENSTSCLR_OFFSET  0x1c
#define PORT_OUT_OFFSET         0x20
#define PORT_IN_OFFSET          0x24
#define PORT_HWSELSTAT_OFFSET   0x28
#define PORT_PDISC_OFFSET       0x2c
#define PORT_PROTSE_OFFSET      0x30
#define PORT_PCSRSEL_OFFSET     0x34
#define PORT_BGTRIM_OFFSET      0x38
#define PORT_OMR_OFFSET         0x3c
#define PORT_OMCR_OFFSET        0x40
#define PORT_OMSR_OFFSET        0x44

#define PORT_PADCFG_BASE        0x0300
#define PORT_PADCFG_STRIDE      0x0010
#define PORT_PADCFG_GPIO(pin)   (PORT_PADCFG_BASE + \
                                 ((pin) * PORT_PADCFG_STRIDE) + 0x0)
#define PORT_PADCFG_DRVCFG(pin) (PORT_PADCFG_BASE + \
                                 ((pin) * PORT_PADCFG_STRIDE) + 0x4)

#define PORT_ACCGRP_BASE        0x90
#define PORT_ACCGRP_STRIDE      0x20
#define PORT_ACCGRP_PROTE(grp)  (PORT_ACCGRP_BASE + \
                                 ((grp) * PORT_ACCGRP_STRIDE) + 0x1c)

#define PORT_DRVCFG_DIR         BIT(0)
#define PORT_DRVCFG_OD          BIT(1)
#define PORT_DRVCFG_MODE_SHIFT  4
#define PORT_DRVCFG_MODE_MASK   GENMASK(7, 4)
#define PORT_DRVCFG_PD_SHIFT    8
#define PORT_DRVCFG_PD_MASK     GENMASK(10, 8)
#define PORT_DRVCFG_PL_SHIFT    12
#define PORT_DRVCFG_PL_MASK     GENMASK(14, 12)

#define PORT_OMR_SET(pin)       BIT(pin)
#define PORT_OMR_CLR(pin)       BIT((pin) + 16)

#endif /* __ARCH_TRICORE_SRC_TC4X_HARDWARE_TC4X_PORT_H */
