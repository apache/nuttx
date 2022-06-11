/****************************************************************************
 * arch/risc-v/src/common/riscv_mtimer.h
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

#ifndef __ARCH_RISCV_SRC_COMMON_RISCV_MTIMER_H
#define __ARCH_RISCV_SRC_COMMON_RISCV_MTIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/timers/oneshot.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

struct oneshot_lowerhalf_s *
riscv_mtimer_initialize(uintptr_t mtime, uintptr_t mtimecmp,
                        int irq, uint64_t freq);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_RISCV_SRC_COMMON_RISCV_MTIMER_H */
