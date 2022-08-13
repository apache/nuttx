/****************************************************************************
 * arch/arm64/src/common/arm64_fpu.h
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

#ifndef __ARCH_ARM64_SRC_COMMON_ARM64_FPU_H
#define __ARCH_ARM64_SRC_COMMON_ARM64_FPU_H

#ifndef __ASSEMBLY__

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <debug.h>
#include <assert.h>

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

struct arm64_cpu_fpu_context
{
  /* owner of current CPU's FPU */

  struct tcb_s *fpu_owner;

  struct tcb_s *idle_thread;

  /* for statistic propose */

  int save_count;
  int restore_count;
  int switch_count;
  int exe_depth_count;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void arm64_init_fpu(struct tcb_s *tcb);
void arm64_destory_fpu(struct tcb_s *tcb);

void arm64_fpu_disable(void);
void arm64_fpu_enable(void);

void arm64_fpu_save(struct fpu_reg *saved_fp_context);
void arm64_fpu_restore(struct fpu_reg *saved_fp_context);

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM64_SRC_COMMON_ARM64_FPU_H */
