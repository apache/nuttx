/****************************************************************************
 * arch/tricore/src/tc3xx/hardware/tc3xx_smu.h
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

#ifndef __ARCH_TRICORE_SRC_TC3XX_HARDWARE_TC3XX_SMU_H
#define __ARCH_TRICORE_SRC_TC3XX_HARDWARE_TC3XX_SMU_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Safety management unit (SMU) registers */

#define TC3XX_SMU_CMD              0xf0036820
#define TC3XX_SMU_KEYS             0xf0036834
#define TC3XX_SMU_AGCF8(n)         (0xf0036960 + (n) * 4)
#define TC3XX_SMU_AG8              0xf00369e0

#define TC3XX_SMU_KEYS_CFG         0xbc
#define TC3XX_SMU_CMD_ALARM_CLR    0x5

#endif /* __ARCH_TRICORE_SRC_TC3XX_HARDWARE_TC3XX_SMU_H */
