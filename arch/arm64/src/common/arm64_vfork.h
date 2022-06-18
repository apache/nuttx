/****************************************************************************
 * arch/arm64/src/common/arm64_vfork.h
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

#ifndef __ARCH_ARM_SRC_COMMON_ARM_VFORK_H
#define __ARCH_ARM_SRC_COMMON_ARM_VFORK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VFORK_REG_X0          (0)
#define VFORK_REG_X1          (1)
#define VFORK_REG_X2          (2)
#define VFORK_REG_X3          (3)
#define VFORK_REG_X4          (4)
#define VFORK_REG_X5          (5)
#define VFORK_REG_X6          (6)
#define VFORK_REG_X7          (7)
#define VFORK_REG_X8          (8)
#define VFORK_REG_X9          (9)
#define VFORK_REG_X10         (10)
#define VFORK_REG_X11         (11)
#define VFORK_REG_X12         (12)
#define VFORK_REG_X13         (13)
#define VFORK_REG_X14         (14)
#define VFORK_REG_X15         (15)
#define VFORK_REG_X16         (16)
#define VFORK_REG_X17         (17)
#define VFORK_REG_X18         (18)
#define VFORK_REG_X19         (19)
#define VFORK_REG_X20         (20)
#define VFORK_REG_X21         (21)
#define VFORK_REG_X22         (22)
#define VFORK_REG_X23         (23)
#define VFORK_REG_X24         (24)
#define VFORK_REG_X25         (25)
#define VFORK_REG_X26         (26)
#define VFORK_REG_X27         (27)
#define VFORK_REG_X28         (28)
#define VFORK_REG_FP          (29) /* Frame pointer*/
#define VFORK_REG_LR          (30) /* Return address*/
#define VFORK_REG_SP          (31) /* Stack pointer*/

#ifdef CONFIG_ARCH_FPU
#define VFORK_REGS_SIZE       (32 + XCPTCONTEXT_FPU_REGS)
#else
#define VFORK_REGS_SIZE       (32)
#endif

#ifndef __ASSEMBLY__

struct vfork_s
{
  uint64_t regs[29];
  uint64_t fp;
  uint64_t lr;
  uint64_t sp;
#ifdef CONFIG_ARCH_FPU
  struct fpu_reg fpu;
#endif
};

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_COMMON_ARM_VFORK_H */
