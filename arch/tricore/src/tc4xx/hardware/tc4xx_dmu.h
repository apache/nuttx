/****************************************************************************
 * arch/tricore/src/tc4xx/hardware/tc4xx_dmu.h
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

#ifndef __ARCH_TRICORE_SRC_TC4XX_HARDWARE_TC4XX_DMU_H
#define __ARCH_TRICORE_SRC_TC4XX_HARDWARE_TC4XX_DMU_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Data memory unit (DMU) flash wait state registers */

#define TC4XX_DMU_HOST_DFWAIT      0xf8040100
#define TC4XX_DMU_HOST_PFWAIT      0xf804011c

#define TC4XX_DMU_RFLASH_MASK      0x1f

#endif /* __ARCH_TRICORE_SRC_TC4XX_HARDWARE_TC4XX_DMU_H */
