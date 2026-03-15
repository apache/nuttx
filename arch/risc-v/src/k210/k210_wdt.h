/****************************************************************************
 * arch/risc-v/src/k210/k210_wdt.h
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

#ifndef __ARCH_RISCV_SRC_K210_K210_WDT_H
#define __ARCH_RISCV_SRC_K210_K210_WDT_H

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef enum
{
  K210_WDT_DEVICE0 = 0,
  K210_WDT_DEVICE1,
  K210_WDT_DEVICE_MAX
} k210_wdt_id_t;

#ifndef __ASSEMBLY__

#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int k210_wdt_initialize(const char *devpath, k210_wdt_id_t id);

#undef EXTERN

#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_K210_K210_WDT_H */
