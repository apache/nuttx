/****************************************************************************
 * arch/tricore/src/tc4xx/hardware/tc4xx_vmt.h
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

#ifndef __ARCH_TRICORE_SRC_TC4XX_HARDWARE_TC4XX_VMT_H
#define __ARCH_TRICORE_SRC_TC4XX_HARDWARE_TC4XX_VMT_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Volatile memory test (VMT) module and its MBIST controllers */

#define TC4XX_VMT_BASE             0xf0400000
#define TC4XX_VMT_SIZE             0x20000
#define TC4XX_VMT_CLC              0x0
#define TC4XX_VMT_CLC_DISS         (1 << 1)
#define TC4XX_VMT_MEMTEST          0x80
#define TC4XX_VMT_MEMSTAT          0x88
#define TC4XX_VMT_MEMDONE          0x8c
#define TC4XX_VMT_MC(n)            (0x1000 + (n) * 0x200)
#define TC4XX_VMT_MC_CONFIG0       0x00
#define TC4XX_VMT_MC_CONFIG1       0x04
#define TC4XX_VMT_MC_MCONTROL      0x0c
#define TC4XX_VMT_MC_RANGE         0x14
#define TC4XX_VMT_MC_ALMSRCS       0x1d0

#define TC4XX_ALMSRCS_ALARMS       ((1 << 5) | (1 << 4))  /* MISCE, OPENE */

#endif /* __ARCH_TRICORE_SRC_TC4XX_HARDWARE_TC4XX_VMT_H */
