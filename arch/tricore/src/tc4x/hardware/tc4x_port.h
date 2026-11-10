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
#define __ARCH_TRICORE_SRC_TC4X_HARDWARE_TC4X_PORT_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Port pad configuration: one DRVCFG register per pin */

#define TC4X_PORT_DRVCFG(p, n)     ((p) + 0x304 + (n) * 0x10)
#define TC4X_PORT_DRVCFG_MODE_MASK 0xf3        /* MODE, OD and DIR */
#define TC4X_PORT_DRVCFG_PAD_MASK   0x7700     /* PD and PL */

#define TC4X_PORT_IN_PULLUP        0x20        /* Input with pull-up */
#define TC4X_PORT_OUT_ALT2         0x21        /* Push-pull, alternate 2 */
#define TC4X_PORT_PAD_SPEED1       0           /* CMOS automotive speed 1 */

#endif /* __ARCH_TRICORE_SRC_TC4X_HARDWARE_TC4X_PORT_H */
