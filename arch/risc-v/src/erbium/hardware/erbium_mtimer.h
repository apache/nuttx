/****************************************************************************
 * arch/risc-v/src/erbium/hardware/erbium_mtimer.h
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

#ifndef __ARCH_RISCV_SRC_ERBIUM_HARDWARE_ERBIUM_MTIMER_H
#define __ARCH_RISCV_SRC_ERBIUM_HARDWARE_ERBIUM_MTIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/erbium_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ERBIUM_MTIMER_FREQ      CONFIG_ERBIUM_MTIMER_FREQ
#define ERBIUM_MTIMER_MTIME     ERBIUM_MTIME_BASE
#define ERBIUM_MTIMER_MTIMECMP  ERBIUM_MTIMECMP_BASE

#endif /* __ARCH_RISCV_SRC_ERBIUM_HARDWARE_ERBIUM_MTIMER_H */
