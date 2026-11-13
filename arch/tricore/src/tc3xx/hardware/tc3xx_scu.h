/****************************************************************************
 * arch/tricore/src/tc3xx/hardware/tc3xx_scu.h
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

#ifndef __ARCH_TRICORE_SRC_TC3XX_HARDWARE_TC3XX_SCU_H
#define __ARCH_TRICORE_SRC_TC3XX_HARDWARE_TC3XX_SCU_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SCU reset status and control.  A power-on reset sets one of the
 * TC3XX_RSTSTAT_POWERON bits, an application reset has the RSTCON
 * reaction 2 for its RSTSTAT bit, CB3 or a CPU0 kernel reset.
 */

#define TC3XX_SCU_RSTSTAT          0xf0036050
#define TC3XX_SCU_RSTCON           0xf0036058

#define TC3XX_RSTSTAT_POWERON      0x138d0000
#define TC3XX_RSTSTAT_RESET        0x000007fb
#define TC3XX_RSTSTAT_CB3          (1 << 20)
#define TC3XX_RSTCON_APPLICATION   2

#endif /* __ARCH_TRICORE_SRC_TC3XX_HARDWARE_TC3XX_SCU_H */
