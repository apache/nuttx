/****************************************************************************
 * arch/risc-v/src/bl602/hardware/bl602_clint.h
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

#ifndef __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_CLINT_H
#define __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_CLINT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BL602_CLINT_BASE (0x02000000)

#define BL602_MTIMER_HIGH ((uint32_t *)(BL602_CLINT_BASE + 0xBFFC))
#define BL602_MTIMER_LOW  ((uint32_t *)(BL602_CLINT_BASE + 0xBFF8))
#define BL602_MTIMER_CMP  ((uint64_t *)(BL602_CLINT_BASE + 0x4000))

#endif /* __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_CLINT_H */
